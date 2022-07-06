import 'dart:typed_data';

ByteData getUint8ListByteData(List<int> value) {
  return Uint8List.fromList(value).buffer.asByteData();
}
