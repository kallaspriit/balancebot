import 'dart:typed_data';

bool unint8ListParseBool(List<int> value) {
  final buffer = Uint8List.fromList(value).buffer;
  final byteDataCreator = ByteData.view(buffer);

  return byteDataCreator.getUint8(0) == 1;
}
