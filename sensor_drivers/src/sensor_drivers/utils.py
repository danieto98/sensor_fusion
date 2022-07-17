#!/usr/bin/env python3

def hex2Dec(hex):
    return int(hex, 16) if hex[0] in '01234567' else int(hex, 16) - 2**16