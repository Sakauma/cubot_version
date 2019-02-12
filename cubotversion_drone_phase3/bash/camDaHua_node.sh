#!/bin/sh
echo "\n======Reload Camera Driver===========\n"
cd /opt/DahuaTech/MVViewer/module/GigEDriver$ 
sudo ./loadDrv.sh

echo "\n======Run Camera Node===========\n"
cd /home/cubot/robomaster2018/uav/bin
./camDaHua_node
