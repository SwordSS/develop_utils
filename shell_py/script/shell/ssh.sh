#!/usr/bin/expect
#所有的expect命令的等待响应的超时时间
set timeout 30

spawn sshfs yyj@192.168.151.110:/home/yyj/bag /home/yyj/bag
#expect "password:" # 捕获到密码
#send "test\r" # 输入密码并回车
#expect {
#"*yes/no*" {send "yes\r"; exp_continue}
#"password:" {send "1\r"}
#}
 
# 允许用户进行交互
interact
