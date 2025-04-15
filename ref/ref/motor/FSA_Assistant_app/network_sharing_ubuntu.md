
# 网络共享脚本使用说明
## 使用前
* 配置本机网络允许转发，修改/etc/sysctl.conf  
  `sudo gedit /etc/sysctl.conf`  
  `# 添加下边这行，或者将其注释解除`  
  `net.ipv4.ip_forward=1`
* 在终端当中使用`ifconfig`查看本机可以上网网卡的name,并将脚本中`network`变量的值替换为本机可以上网网卡的name 

* 以上步骤(配置本机网络允许转发)只需要配置一次
## 使用
在终端当中进入FSA_Assistant文件夹，输入`./network_sharing_linux.sh`并执行
## 注意事项
* 如果出现执行脚本后仍然不能网络共享的情况，优先检查电脑是否可以访问互联网
* 如果不想每次开机都执行一次可以写入开机自启动文件中
