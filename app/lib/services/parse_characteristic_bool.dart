import 'get_uint8_list_byte_data.dart';

bool parseCharacteristicBool(List<int> value) {
  return getUint8ListByteData(value).getInt8(0) == 1;
}
