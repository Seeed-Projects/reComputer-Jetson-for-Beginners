"""
License
The MIT License (MIT)

Copyright (C) 2020 Ville Laine, Aikuiskoulutus Taitaja

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
"""

# Necessary functions
def calc_crc8(bytes_list):
    # CRC init value
    crc = 0xff

    # loop through bytes
    for i in bytes_list:
        for bit in range(0, 8):
            if (i ^ crc) & 0x80:
                crc = (crc << 1) ^ 0x31
            else:
                crc = (crc << 1)
            i = i << 1
        crc = crc & 0xFF

    return crc


def validate_crc8(int_list):
    ok = 1
    pos = 0
    values = []
    size = 3

    # Split list
    while len(int_list) > size:
        pice = int_list[:size]
        values.append(pice)
        int_list = int_list[size:]
        values.append(int_list)

    # loop through all words and respective crc [HByte, LByte, CRC]
    for value in values:
        pos = pos + 1

        # CRC init value
        crc = 0xff

        # loop through only HByte and LByte
        for i in value[0:2]:
            for bit in range(0, 8):
                if (i ^ crc) & 0x80:
                    crc = (crc << 1) ^ 0x31
                else:
                    crc = (crc << 1)
                i = i << 1
            crc = crc & 0xFF

        # Check CRC status when word processed and exit if crc checksum fails
        if crc != value[2]:
            ok = 0
            # crc_status = [ok, pos]
            print("crc check failed!")
            return [ok, pos]

    if ok == 1:
        pos = 0
    # crc_status = [ok, pos]
    return [ok, pos]


# Convert two bytes to one int(word) = int(HByte+LByte)
# example (HByte = 0x0a, LByte = 0xf4) = 0x0af4 = 2804
# Returns hexadecimal representation
def bytes_to_int(bytes_list):
    return_str = ""

    for i in bytes_list:
        temp_str = str(hex(i))[2:]

        # Add trailing zero if value is 0x0
        if int(temp_str, 16) == 0:
            temp_str = temp_str + "0"
        return_str = return_str + temp_str

    return int(return_str, 16)


# Convert int(word) = int(HByte+LByte) into two bytes
# example 2804 = 0x0af4 = (HByte = 0x0a, LByte = 0xf4)
def int_to_bytes(number):
    tmp_str = str(hex(number))
    hbyte = "0x"+tmp_str[2:4]
    lbyte = "0x"+tmp_str[-2:]
    return [int(hbyte, 16), int(lbyte, 16)]
