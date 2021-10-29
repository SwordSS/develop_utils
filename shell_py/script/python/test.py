#!/usr/bin/env python3
import os

import sys

import fcntl

fcntl.fcntl(sys.stdin, fcntl.F_SETFL, os.O_NONBLOCK)

try:

	cc = sys.stdin.read()
        print(cc)

except TypeError as e:

	print('no std input readed')


