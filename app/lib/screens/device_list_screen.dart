import 'package:flutter/material.dart';
import 'package:flutter_hooks/flutter_hooks.dart';
import 'package:provider/provider.dart';
import '../src/ble/ble_scanner.dart';
import 'device_details_screen.dart';

class DeviceListScreen extends HookWidget {
  const DeviceListScreen({Key? key}) : super(key: key);

  @override
  Widget build(BuildContext context) {
    return Consumer2<BleScanner, BleScannerState?>(builder: (
      _,
      bleScanner,
      maybeBleScannerState,
      __,
    ) {
      // provide default scanner state
      final bleScannerState = maybeBleScannerState ??
          const BleScannerState(
            discoveredDevices: [],
            scanIsInProgress: false,
          );

      return Scaffold(
        appBar: AppBar(
          title: const Text("Scanning for devices"),
          actions: [
            Center(
              child: Padding(
                padding: const EdgeInsets.symmetric(horizontal: 16),
                child: Text("Found ${bleScannerState.discoveredDevices.length.toString()} devices"),
              ),
            ),
          ],
        ),
        body: DeviceList(
          bleScanner: bleScanner,
          bleScannerState: bleScannerState,
        ),
      );
    });
  }
}

class DeviceList extends HookWidget {
  const DeviceList({
    Key? key,
    required this.bleScanner,
    required this.bleScannerState,
  }) : super(key: key);

  final BleScanner bleScanner;
  final BleScannerState bleScannerState;

  @override
  Widget build(BuildContext context) {
    // automatically start scanning
    useEffect(() {
      bleScanner.startScan([]);

      return () {};
    }, [bleScanner]);

    // render list of devices
    return ListView.builder(
      itemCount: bleScannerState.discoveredDevices.length,
      itemBuilder: (context, index) => ListTile(
        title: Text(bleScannerState.discoveredDevices[index].name.isNotEmpty
            ? "${bleScannerState.discoveredDevices[index].name} (${bleScannerState.discoveredDevices[index].id})"
            : bleScannerState.discoveredDevices[index].id),
        subtitle: Text("RSSI: ${bleScannerState.discoveredDevices[index].rssi}"),
        onTap: () => Navigator.push<void>(
          context,
          MaterialPageRoute(
            builder: (_) => DeviceDetailsScreen(
              deviceId: bleScannerState.discoveredDevices[index].id,
            ),
          ),
        ),
      ),
    );
  }
}
