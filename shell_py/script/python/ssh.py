#!/usr/bin/env python3
# import os,sys

# if __name__ == '__main__':
#     #os.system('../shell/ssh.sh')
#     os.system('sshfs yyj@192.168.151.110:/home/yyj/bag /home/yyj/bag')

f = open("/tmp/myfifo")
while 1:
    print(f.readline(), end = "")

f.close()
