import 'dart:typed_data';

import 'get_uint8_list_byte_data.dart';

double parseCharacteristicFloat(List<int> value) {
  return getUint8ListByteData(value).getFloat32(0, Endian.little);
}
