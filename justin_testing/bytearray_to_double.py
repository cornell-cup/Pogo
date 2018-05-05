import struct

# must be 4 bytes
bytearray_as_string = '\xdb\x0fI@'

output = struct.unpack('f', bytearray_as_string)

print(output)