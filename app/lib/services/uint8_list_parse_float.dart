import 'dart:typed_data';

double unint8ListParseFloat(List<int> value) {
  final buffer = Uint8List.fromList(value).buffer;
  final byteData = buffer.asByteData();
  // final byteDataCreator = ByteData.view(buffer);

  return byteData.getFloat32(0, Endian.little);
}
