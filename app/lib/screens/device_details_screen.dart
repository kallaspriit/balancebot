import 'package:balancebot/screens/error_screen.dart';
import 'package:collection/collection.dart';
import 'package:flutter/material.dart';
import 'package:flutter_hooks/flutter_hooks.dart';
import 'package:provider/provider.dart';
import 'package:flutter_reactive_ble/flutter_reactive_ble.dart';

import '../src/ble/ble_device_connector.dart';
import '../src/ble/ble_device_interactor.dart';
import '../src/ble/ble_scanner.dart';

class DeviceDetailsScreen extends HookWidget {
  const DeviceDetailsScreen({Key? key, required this.deviceId}) : super(key: key);

  final String deviceId;

  @override
  Widget build(BuildContext context) {
    return Consumer5<BleScanner, BleDeviceConnector, BleDeviceInteractor, BleScannerState?, ConnectionStateUpdate>(
        builder: (
      _,
      bleScanner,
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

      return Scaffold(
        appBar: AppBar(
          title: const Text("Device"),
        ),
        body: DeviceDetails(
          device: device,
          bleScanner: bleScanner,
          bleDeviceConnector: bleDeviceConnector,
          bleDeviceInteractor: bleDeviceInteractor,
          bleScannerState: bleScannerState,
          connectionStateUpdate: connectionStateUpdate,
        ),
      );
    });
  }
}

class DeviceDetails extends HookWidget {
  const DeviceDetails({
    Key? key,
    required this.device,
    required this.bleScanner,
    required this.bleDeviceConnector,
    required this.bleDeviceInteractor,
    required this.bleScannerState,
    required this.connectionStateUpdate,
  }) : super(key: key);

  final DiscoveredDevice device;
  final BleScanner bleScanner;
  final BleDeviceConnector bleDeviceConnector;
  final BleDeviceInteractor bleDeviceInteractor;
  final BleScannerState bleScannerState;
  final ConnectionStateUpdate connectionStateUpdate;

  @override
  Widget build(BuildContext context) {
    final discoveredServices = useState<List<DiscoveredService>>([]);

    // automatically attempt to connect to the device
    useEffect(() {
      bleDeviceConnector.connect(device.id);

      return () {};
    }, [bleScanner]);

    // open device screen once connected
    useEffect(() {
      if (connectionStateUpdate.connectionState == DeviceConnectionState.connected) {
        debugPrint(
          'Connected to bluetooth device "${connectionStateUpdate.deviceId}"',
        );

        // discover services of connected device
        bleDeviceInteractor
            .discoverServices(connectionStateUpdate.deviceId)
            .then((services) => discoveredServices.value = services);

        // TODO: open device details view
      } else {
        debugPrint(
          'Status for bluetooth device "${connectionStateUpdate.deviceId}" changed to ${connectionStateUpdate.connectionState}',
        );
      }

      return () {};
    }, [connectionStateUpdate.connectionState]);

    // list discovered services
    useEffect(() {
      for (final discoveredService in discoveredServices.value) {
        debugPrint(
          "Discovered service: ${discoveredService.serviceId} with ${discoveredService.characteristics.length} characteristics",
        );
      }

      return () {};
    }, [discoveredServices.value]);

    // render device details
    return ListView(children: [
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
      ListTile(
        title: const Text("Discovered services"),
        subtitle: Text(discoveredServices.value.isNotEmpty ? discoveredServices.value.length.toString() : "none"),
      ),
      ...discoveredServices.value.map(
        (discoveredService) => ListTile(
          title: const Text("Discovered service"),
          subtitle: Text(discoveredService.serviceId.toString()),
        ),
      )
    ]);
  }
}
