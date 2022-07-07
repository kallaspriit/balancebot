import 'dart:async';

import 'package:balancebot/screens/error_screen.dart';
import 'package:balancebot/services/parse_characteristic_float.dart';
import 'package:collection/collection.dart';
import 'package:flutter/material.dart';
import 'package:flutter_hooks/flutter_hooks.dart';
import 'package:provider/provider.dart';
import 'package:flutter_reactive_ble/flutter_reactive_ble.dart';
import 'package:flutter_joystick/flutter_joystick.dart';

import '../services/build_characteristic_bool.dart';
import '../services/build_characteristic_int32.dart';
import '../services/parse_characteristic_bool.dart';
import '../src/ble/ble_device_connector.dart';
import '../src/ble/ble_device_interactor.dart';
import '../src/ble/ble_scanner.dart';

class BalancebotScreen extends HookWidget {
  const BalancebotScreen({Key? key, required this.deviceId}) : super(key: key);

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

      return Scaffold(
        appBar: AppBar(
          title: const Text("Balancebot"),
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
        body: Balancebot(
          device: device,
          bleDeviceConnector: bleDeviceConnector,
          bleDeviceInteractor: bleDeviceInteractor,
          bleScannerState: bleScannerState,
          connectionStateUpdate: connectionStateUpdate,
        ),
      );
    });
  }
}

class Balancebot extends HookWidget {
  const Balancebot({
    Key? key,
    required this.device,
    required this.bleDeviceConnector,
    required this.bleDeviceInteractor,
    required this.bleScannerState,
    required this.connectionStateUpdate,
  }) : super(key: key);

  // known service identifiers
  static final statusServiceUuid = Uuid.parse("19B10000-E8F2-537E-4F6C-D104768A1214");
  static final controlServiceUuid = Uuid.parse("40760000-8912-48af-9146-b057c465d970");
  static final angleCharacteristicUuid = Uuid.parse("19B10001-E8F2-537E-4F6C-D104768A1214");
  static final targetSpeedCharacteristicUuid = Uuid.parse("40760001-8912-48af-9146-b057c465d970");
  static final targetRotationCharacteristicUuid = Uuid.parse("40760002-8912-48af-9146-b057c465d970");
  static final usePositionHoldCharacteristicUuid = Uuid.parse("40760003-8912-48af-9146-b057c465d970");
  static final isEnabledCharacteristicUuid = Uuid.parse("40760004-8912-48af-9146-b057c465d970");

  // map of known characteristic names
  static final characteristicNames = {
    angleCharacteristicUuid: "Angle",
    targetSpeedCharacteristicUuid: "Target speed",
    targetRotationCharacteristicUuid: "Target rotation",
    usePositionHoldCharacteristicUuid: "Use position hold",
    isEnabledCharacteristicUuid: "Is enabled",
  };

  final DiscoveredDevice device;
  final BleDeviceConnector bleDeviceConnector;
  final BleDeviceInteractor bleDeviceInteractor;
  final BleScannerState bleScannerState;
  final ConnectionStateUpdate connectionStateUpdate;

  @override
  Widget build(BuildContext context) {
    final discoveredServices = useState<List<DiscoveredService>>([]);
    final characteristicValues = useState<Map<Uuid, List<int>>>({});
    final speedInput = useState(0);
    final rotationInput = useState(0);

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
    }, [connectionStateUpdate.connectionState]);

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
      if (connectionStateUpdate.connectionState != DeviceConnectionState.connected) {
        return () {};
      }

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
              "Subscribing to ${getCharacteristicName(characteristic.characteristicId)} on service ${characteristic.serviceId}",
            );

            // subscribe and update characteristic values
            final characteristicSubscription =
                bleDeviceInteractor.subScribeToCharacteristic(characteristic).listen((value) {
              // debugPrint(
              //   "Got ${getCharacteristicName(characteristic.characteristicId)} update: ${value.join(",")}",
              // );

              characteristicValues.value[characteristic.characteristicId] = value;
              characteristicValues.notifyListeners();
            });

            characteristicSubscriptions.add(characteristicSubscription);
          }

          // attempt to read initial characteristic value if readable
          if (discoveredCharacteristic.isReadable) {
            try {
              bleDeviceInteractor.readCharacteristic(characteristic).then((value) {
                debugPrint("Read ${getCharacteristicName(characteristic.characteristicId)} value: ${value.toString()}");

                characteristicValues.value[characteristic.characteristicId] = value;
                characteristicValues.notifyListeners();
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
    }, [discoveredServices.value.length, connectionStateUpdate.connectionState]);

    // sets target speed characteristic
    final setTargetSpeed = useCallback((int speed) {
      debugPrint("Setting target speed: $speed");

      final characteristic = QualifiedCharacteristic(
        deviceId: device.id,
        serviceId: controlServiceUuid,
        characteristicId: targetSpeedCharacteristicUuid,
      );

      bleDeviceInteractor.writeCharacterisiticWithoutResponse(characteristic, buildCharacteristicInt32(speed));
    }, []);

    // sets target rotation characteristic
    final setTargetRotation = useCallback((int rotation) {
      debugPrint("Setting target rotation: $rotation");

      final characteristic = QualifiedCharacteristic(
        deviceId: device.id,
        serviceId: controlServiceUuid,
        characteristicId: targetRotationCharacteristicUuid,
      );

      bleDeviceInteractor.writeCharacterisiticWithoutResponse(characteristic, buildCharacteristicInt32(rotation));
    }, []);

    // sets use position hold characteristic
    final setUsePositionHold = useCallback((bool enabled) {
      debugPrint("Setting position hold: ${enabled ? "enabled" : "disabled"}");

      final characteristic = QualifiedCharacteristic(
        deviceId: device.id,
        serviceId: controlServiceUuid,
        characteristicId: usePositionHoldCharacteristicUuid,
      );

      bleDeviceInteractor.writeCharacterisiticWithoutResponse(characteristic, buildCharacteristicBool(enabled));
    }, []);

    // sets robot enabled characteristic
    final setIsEnabled = useCallback((bool enabled) {
      debugPrint("Setting robot state: ${enabled ? "enabled" : "disabled"}");

      final characteristic = QualifiedCharacteristic(
        deviceId: device.id,
        serviceId: controlServiceUuid,
        characteristicId: isEnabledCharacteristicUuid,
      );

      bleDeviceInteractor.writeCharacterisiticWithoutResponse(characteristic, buildCharacteristicBool(enabled));
    }, []);

    // apply target speed
    useEffect(() {
      setTargetSpeed(speedInput.value);

      return () {};
    }, [speedInput.value]);

    // apply target rotation
    useEffect(() {
      setTargetRotation(rotationInput.value);

      return () {};
    }, [rotationInput.value]);

    // detect robot
    final robotStatusService =
        discoveredServices.value.firstWhereOrNull((service) => service.serviceId == statusServiceUuid);
    final isRobot = robotStatusService != null;

    // handle robot not detected
    if (!isRobot) {
      return const ErrorScreen(error: "Given BLE device does not appear to be valid");
    }

    // get characteristic values
    final angle = parseCharacteristicFloat(characteristicValues.value[angleCharacteristicUuid] ?? [0, 0, 0, 0]);
    // final targetSpeed =
    //     parseCharacteristicInt32(characteristicValues.value[targetSpeedCharacteristicUuid] ?? [0, 0, 0, 0]);
    // final targetRotation =
    //     parseCharacteristicInt32(characteristicValues.value[targetRotationCharacteristicUuid] ?? [0, 0, 0, 0]);
    final usePositionHold =
        parseCharacteristicBool(characteristicValues.value[usePositionHoldCharacteristicUuid] ?? [0]);
    final isEnabled = parseCharacteristicBool(characteristicValues.value[isEnabledCharacteristicUuid] ?? [0]);

    // render device details
    return Column(
      children: [
        Row(
          children: [
            Expanded(
              child: ListTile(
                title: const Text("Angle"),
                subtitle: Text("${angle.toStringAsFixed(2)} degrees"),
              ),
            ),
            // Expanded(
            //   child: ListTile(
            //     title: const Text("Target speed"),
            //     subtitle: Text(targetSpeed.toString()),
            //   ),
            // ),
            // Expanded(
            //   child: ListTile(
            //     title: const Text("Target rotation"),
            //     subtitle: Text(targetRotation.toString()),
            //   ),
            // ),
            Expanded(
              child: ListTile(
                title: const Text("Enabled"),
                subtitle: Align(
                  alignment: Alignment.topLeft,
                  child: Switch(
                    value: isEnabled,
                    onChanged: (enabled) => setIsEnabled(enabled),
                  ),
                ),
              ),
            ),
            Expanded(
              child: ListTile(
                title: const Text("Use position hold"),
                subtitle: Align(
                  alignment: Alignment.topLeft,
                  child: Switch(
                    value: usePositionHold,
                    onChanged: (enabled) => setUsePositionHold(enabled),
                  ),
                ),
              ),
            ),
          ],
        ),
        Expanded(
          child: Row(
            mainAxisAlignment: MainAxisAlignment.spaceEvenly,
            children: [
              Joystick(
                mode: JoystickMode.vertical,
                listener: (event) {
                  speedInput.value = (event.y * 5.0).round();
                },
                onStickDragEnd: () {
                  speedInput.value = 0;
                },
              ),
              Joystick(
                mode: JoystickMode.horizontal,
                listener: (event) {
                  rotationInput.value = (event.x * 20.0).round();
                },
                onStickDragEnd: () {
                  rotationInput.value = 0;
                },
              ),
            ],
          ),
        ),
      ],
    );
  }

  String getCharacteristicName(Uuid id) {
    final name = characteristicNames[id];

    return name != null ? "$name ($id)" : "characteristic ${id.toString()}";
  }
}
