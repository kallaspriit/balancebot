import 'dart:typed_data';

import 'get_uint8_list_byte_data.dart';

int parseCharacteristicInt32(List<int> value) {
  return getUint8ListByteData(value).getInt32(0, Endian.little);
}
