import 'dart:async';

import 'package:balancebot/screens/error_screen.dart';
import 'package:collection/collection.dart';
import 'package:flutter/material.dart';
import 'package:flutter_hooks/flutter_hooks.dart';
import 'package:provider/provider.dart';
import 'package:flutter_reactive_ble/flutter_reactive_ble.dart';

import '../src/ble/ble_device_connector.dart';
import '../src/ble/ble_device_interactor.dart';
import '../src/ble/ble_scanner.dart';
import 'balancebot_screen.dart';

class DeviceDetailsScreen extends HookWidget {
  const DeviceDetailsScreen({Key? key, required this.deviceId}) : super(key: key);

  final String deviceId;

  @override
  Widget build(BuildContext context) {
    return Consumer4<BleDeviceConnector, BleDeviceInteractor, BleScannerState?, ConnectionStateUpdate>(builder: (
      _,
      bleDeviceConnector,
      bleDeviceInteractor,
      maybeBleScannerState,
      connectionStateUpdate,
      __,
    ) {
      final bleScannerState = maybeBleScannerState ??
          const BleScannerState(
            discoveredDevices: [],
            scanIsInProgress: false,
          );
      final device = bleScannerState.discoveredDevices.firstWhereOrNull((device) => device.id == deviceId);

      if (device == null) {
        return ErrorScreen(error: "Device with id $deviceId could not be found");
      }

      final isConnected = connectionStateUpdate.deviceId == device.id &&
          connectionStateUpdate.connectionState == DeviceConnectionState.connected;

      return WillPopScope(
        onWillPop: () async {
          debugPrint("Navigating away from device details screen, disconnecting");

          bleDeviceConnector.disconnect(device.id);

          return true;
        },
        child: Scaffold(
          appBar: AppBar(
            title: Text("Device ${device.name.isNotEmpty ? device.name : device.id}"),
            actions: [
              Padding(
                padding: const EdgeInsets.symmetric(horizontal: 16, vertical: 12),
                child: GestureDetector(
                  onTap: () {
                    debugPrint("Attempting to connect again to device ${device.id}");

                    bleDeviceConnector.connect(device.id);
                  },
                  child: isConnected
                      ? const Icon(
                          Icons.bluetooth_connected,
                          size: 26.0,
                        )
                      : const AspectRatio(
                          aspectRatio: 1.0,
                          child: CircularProgressIndicator(
                            color: Colors.white,
                            strokeWidth: 3,
                          ),
                        ),
                ),
              ),
            ],
          ),
          body: DeviceDetails(
            device: device,
            bleDeviceConnector: bleDeviceConnector,
            bleDeviceInteractor: bleDeviceInteractor,
            bleScannerState: bleScannerState,
            connectionStateUpdate: connectionStateUpdate,
          ),
        ),
      );
    });
  }
}

class DeviceDetails extends HookWidget {
  const DeviceDetails({
    Key? key,
    required this.device,
    required this.bleDeviceConnector,
    required this.bleDeviceInteractor,
    required this.bleScannerState,
    required this.connectionStateUpdate,
  }) : super(key: key);

  // known service identifiers
  static final statusServiceUuid = Uuid.parse("19B10000-E8F2-537E-4F6C-D104768A1214");

  final DiscoveredDevice device;
  final BleDeviceConnector bleDeviceConnector;
  final BleDeviceInteractor bleDeviceInteractor;
  final BleScannerState bleScannerState;
  final ConnectionStateUpdate connectionStateUpdate;

  @override
  Widget build(BuildContext context) {
    final discoveredServices = useState<List<DiscoveredService>>([]);
    final characteristicValues = useState<Map<Uuid, List<int>>>({});

    // automatically attempt to connect to the device
    useEffect(() {
      // disconnect any existing connected device
      if (connectionStateUpdate.deviceId != device.id &&
          connectionStateUpdate.connectionState == DeviceConnectionState.connected) {
        debugPrint("Disconnecting from device ${device.id} (currently ${connectionStateUpdate.connectionState})");

        bleDeviceConnector.disconnect(connectionStateUpdate.deviceId);
      }

      // attempt to connect to the selected device
      if (connectionStateUpdate.deviceId != device.id ||
          connectionStateUpdate.connectionState == DeviceConnectionState.disconnected) {
        debugPrint("Attempting to connect to device ${device.id}");

        bleDeviceConnector.connect(device.id);
      }

      return () {};
    }, []);

    // discover services
    useEffect(() {
      if (connectionStateUpdate.connectionState == DeviceConnectionState.connected) {
        debugPrint(
          'Connected to bluetooth device "${connectionStateUpdate.deviceId}"',
        );

        // discover services of connected device
        bleDeviceInteractor
            .discoverServices(connectionStateUpdate.deviceId)
            .then((services) => discoveredServices.value = services);
      } else {
        debugPrint(
          'Status for bluetooth device "${connectionStateUpdate.deviceId}" changed to ${connectionStateUpdate.connectionState}',
        );
      }

      return () {};
    }, [connectionStateUpdate.connectionState]);

    // subscribe to characteristics once connected
    useEffect(() {
      // keep track of stream subscriptions
      List<StreamSubscription<List<int>>> characteristicSubscriptions = [];

      for (final discoveredService in discoveredServices.value) {
        debugPrint(
          "Discovered service: ${discoveredService.serviceId} with ${discoveredService.characteristics.length} characteristics",
        );

        // handle discovered characteristics
        for (final discoveredCharacteristic in discoveredService.characteristics) {
          final characteristic = QualifiedCharacteristic(
            deviceId: device.id,
            serviceId: discoveredCharacteristic.serviceId,
            characteristicId: discoveredCharacteristic.characteristicId,
          );

          // subscribe to characteristic if it's notifiable
          if (discoveredCharacteristic.isNotifiable) {
            debugPrint(
              "Subscribing to characteristic ${characteristic.characteristicId} on service ${characteristic.serviceId}",
            );

            // subscribe and update characteristic values
            final characteristicSubscription =
                bleDeviceInteractor.subScribeToCharacteristic(characteristic).listen((value) {
              //  debugPrint(
              //     "Got characteristic ${getCharacteristicName(characteristic.characteristicId)} update: ${value.join(",")}",
              //   );

              characteristicValues.value[characteristic.characteristicId] = value;
            });

            characteristicSubscriptions.add(characteristicSubscription);
          }

          // attempt to read initial characteristic value if readable
          if (discoveredCharacteristic.isReadable) {
            try {
              bleDeviceInteractor.readCharacteristic(characteristic).then((value) {
                debugPrint("Read characteristic ${characteristic.characteristicId} value: ${value.toString()}");

                characteristicValues.value[characteristic.characteristicId] = value;
              });
            } on Exception catch (e) {
              debugPrint("Reading characteristic ${characteristic.characteristicId} failed ($e)");
            }
          }
        }
      }

      return () {
        // cancel all subscriptions
        for (final characteristicSubscription in characteristicSubscriptions) {
          debugPrint("Cancelling subscription");

          characteristicSubscription.cancel();
        }
      };
    }, [discoveredServices.value]);

    final balancebotStatusService =
        discoveredServices.value.firstWhereOrNull((service) => service.serviceId == statusServiceUuid);
    final isBalancebot = balancebotStatusService != null;

    // render device details
    return Column(
      crossAxisAlignment: CrossAxisAlignment.stretch,
      children: [
        Expanded(
          child: ListView(
            children: [
              ListTile(
                title: const Text("Id"),
                subtitle: Text(device.id),
              ),
              ListTile(
                title: const Text("Name"),
                subtitle: Text(device.name.isNotEmpty ? device.name : "n/a"),
              ),
              ListTile(
                title: const Text("RSSI"),
                subtitle: Text(device.rssi.toString()),
              ),
              ListTile(
                title: const Text("Advertised service UUIDs"),
                subtitle: Text(device.serviceUuids.map((uuid) => uuid.toString()).join(", ")),
              ),
              ListTile(
                title: const Text("Manufacturer data"),
                subtitle: Text(device.manufacturerData.isNotEmpty ? device.manufacturerData.join(":") : "n/a"),
              ),
              ListTile(
                title: const Text("Service data"),
                subtitle: Text(device.serviceData.isNotEmpty ? device.serviceData.toString() : "n/a"),
              ),
              ListTile(
                title: const Text("Connection status"),
                subtitle: Text(connectionStateUpdate.connectionState.toString()),
              ),
              ...discoveredServices.value.map(
                (discoveredService) => ListTile(
                  title: Text("Service ${discoveredService.serviceId.toString()}"),
                  subtitle: Column(
                    crossAxisAlignment: CrossAxisAlignment.start,
                    children: discoveredService.characteristicIds
                        .map((characteristicId) => Text(characteristicId.toString()))
                        .toList(),
                  ),
                ),
              ),
              ...characteristicValues.value.entries.map(
                (entry) => ListTile(
                  title: Text(entry.key.toString()),
                  subtitle: Text(entry.value.toString()),
                ),
              ),
            ],
          ),
        ),
        isBalancebot
            ? Padding(
                padding: const EdgeInsets.symmetric(horizontal: 16, vertical: 8),
                child: ElevatedButton(
                    onPressed: () => Navigator.push<void>(
                          context,
                          MaterialPageRoute(
                            builder: (_) => BalancebotScreen(
                              deviceId: device.id,
                            ),
                          ),
                        ),
                    child: const Text("Control as Balancebot")),
              )
            : const SizedBox.shrink()
      ],
    );
  }
}
