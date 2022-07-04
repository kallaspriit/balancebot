import 'package:flutter/material.dart';
import 'package:flutter_hooks/flutter_hooks.dart';
import 'package:provider/provider.dart';
import 'package:flutter_reactive_ble/flutter_reactive_ble.dart';

import '../src/ble/ble_scanner.dart';

class DeviceListScreen extends HookWidget {
  const DeviceListScreen({Key? key}) : super(key: key);

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: const Text('Scan for devices'),
        actions: const [CircularProgressIndicator()],
      ),
      body: Consumer2<BleScanner, BleScannerState?>(
        builder: (_, bleScanner, bleScannerState, __) => DeviceList(
          bleScanner: bleScanner,
          bleScannerState: bleScannerState ??
              const BleScannerState(
                discoveredDevices: [],
                scanIsInProgress: false,
              ),
        ),
      ),
    );
  }
}

class DeviceList extends HookWidget {
  const DeviceList(
      {Key? key, required this.bleScanner, required this.bleScannerState})
      : super(key: key);

  final BleScanner bleScanner;
  final BleScannerState bleScannerState;

  @override
  Widget build(BuildContext context) {
    useEffect(() {
      bleScanner.startScan([]);

      return () {};
    }, [bleScanner]);

    return Column(children: [
      ListTile(
        tileColor: Colors.black12,
        title: const Text("Scanning"),
        subtitle: Text(
            "Found ${bleScannerState.discoveredDevices.length.toString()} devices"),
      ),
      Expanded(
        child: ListView.builder(
          shrinkWrap: true,
          itemCount: bleScannerState.discoveredDevices.length,
          itemBuilder: (context, index) => ListTile(
            title: Text(bleScannerState.discoveredDevices[index].name.isNotEmpty
                ? "${bleScannerState.discoveredDevices[index].name} (${bleScannerState.discoveredDevices[index].id})"
                : bleScannerState.discoveredDevices[index].id),
            subtitle:
                Text("RSSI: ${bleScannerState.discoveredDevices[index].rssi}"),
          ),
        ),
      ),
    ]);
  }
}
