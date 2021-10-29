#!/usr/bin/env python3

# import sys
# import time

# while(1):
#     for line in sys.stdin.readlines():
#         if not line:
#             time.sleep(0.1)
#             break
#         else:
#             print(line)

import sys

# i = -1
# for line in sys.stdin:
#     i = i + 1
#     print(type(line))
#     print(str(i))
#     print (line)

# i = 0
# while(1):
    
#     string = ' '
#     while(str(string[-1])!="d"):
#         info = sys.stdin.read(1) 
#         string = string + info
#     i=i+1
#     print(str(i)+' '+str(string))

i=0
while(1):
    i= i+1
    info = sys.stdin.read(1) 
    print(info+str(i))

        