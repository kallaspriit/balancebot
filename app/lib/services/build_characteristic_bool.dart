import 'dart:typed_data';

List<int> buildCharacteristicBool(bool enabled) {
  final byteData = Uint8List(1).buffer.asByteData();

  byteData.setInt8(0, enabled ? 1 : 0);

  return byteData.buffer.asInt8List();
}
