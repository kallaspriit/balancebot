import 'package:flutter/material.dart';
import 'package:flutter_reactive_ble/flutter_reactive_ble.dart';
import 'package:provider/provider.dart';

import 'ble_status_screen.dart';
import 'device_list_screen.dart';

class HomeScreen extends StatelessWidget {
  const HomeScreen({
    Key? key,
  }) : super(key: key);

  @override
  Widget build(BuildContext context) => Consumer<BleStatus?>(
        builder: (_, status, __) {
          // handle ble errors
          if (status != BleStatus.ready) {
            return BleStatusScreen(status: status ?? BleStatus.unknown);
          }

          // render list of devices
          return const DeviceListScreen();
        },
      );
}
