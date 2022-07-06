import 'dart:typed_data';

List<int> buildCharacteristicInt32(int value) {
  final byteData = Uint8List(4).buffer.asByteData();

  byteData.setInt32(0, value, Endian.little);

  return byteData.buffer.asInt8List();
}
