#!/bin/sh

network="wlo1" #更换为自己的网卡驱动名


sudo iptables -F
# 配置基本规则，允许进及转发
sudo iptables -P INPUT ACCEPT
sudo iptables -P FORWARD ACCEPT
# 允许所有主机均可用该转发
sudo iptables -t nat -A POSTROUTING -o $network -j MASQUERADE
