import 'package:flutter/material.dart';
import 'package:flutter_hooks/flutter_hooks.dart';
import 'package:provider/provider.dart';
import 'package:flutter_reactive_ble/flutter_reactive_ble.dart';

import '../src/ble/ble_device_connector.dart';
import '../src/ble/ble_device_interactor.dart';
import '../src/ble/ble_scanner.dart';

class DeviceListScreen extends HookWidget {
  const DeviceListScreen({Key? key}) : super(key: key);

  @override
  Widget build(BuildContext context) {
    return Consumer5<BleScanner, BleDeviceConnector, BleDeviceInteractor, BleScannerState?, ConnectionStateUpdate>(
      builder: (
        _,
        bleScanner,
        bleDeviceConnector,
        bleDeviceInteractor,
        bleScannerState,
        connectionStateUpdate,
        __,
      ) =>
          Scaffold(
        appBar: AppBar(
          title: const Text('Scan for devices'),
          actions: const [CircularProgressIndicator()],
        ),
        body: DeviceList(
          bleScanner: bleScanner,
          bleDeviceConnector: bleDeviceConnector,
          bleDeviceInteractor: bleDeviceInteractor,
          bleScannerState: bleScannerState ??
              const BleScannerState(
                discoveredDevices: [],
                scanIsInProgress: false,
              ),
          connectionStateUpdate: connectionStateUpdate,
        ),
      ),
    );
  }
}

class DeviceList extends HookWidget {
  const DeviceList({
    Key? key,
    required this.bleScanner,
    required this.bleDeviceConnector,
    required this.bleDeviceInteractor,
    required this.bleScannerState,
    required this.connectionStateUpdate,
  }) : super(key: key);

  final BleScanner bleScanner;
  final BleDeviceConnector bleDeviceConnector;
  final BleDeviceInteractor bleDeviceInteractor;
  final BleScannerState bleScannerState;
  final ConnectionStateUpdate connectionStateUpdate;

  @override
  Widget build(BuildContext context) {
    final discoveredServices = useState<List<DiscoveredService>>([]);

    // automatically start scanning
    useEffect(() {
      bleScanner.startScan([]);

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

    // render list of devices
    return Column(children: [
      ListTile(
        tileColor: Colors.black12,
        title: const Text("Scanning"),
        subtitle: Text("Found ${bleScannerState.discoveredDevices.length.toString()} devices"),
      ),
      ListTile(
        tileColor: Colors.black12,
        title: const Text("Services"),
        subtitle: Text("Found ${discoveredServices.value.length.toString()} services"),
      ),
      Expanded(
        child: ListView.builder(
          shrinkWrap: true,
          itemCount: bleScannerState.discoveredDevices.length,
          itemBuilder: (context, index) => ListTile(
            title: Text(bleScannerState.discoveredDevices[index].name.isNotEmpty
                ? "${bleScannerState.discoveredDevices[index].name} (${bleScannerState.discoveredDevices[index].id})"
                : bleScannerState.discoveredDevices[index].id),
            subtitle: Text(connectionStateUpdate.deviceId == bleScannerState.discoveredDevices[index].id
                ? connectionStateUpdate.connectionState.toString()
                : "RSSI: ${bleScannerState.discoveredDevices[index].rssi}"),
            onTap: () {
              bleDeviceConnector.connect(bleScannerState.discoveredDevices[index].id);
            },
          ),
        ),
      ),
    ]);
  }
}
