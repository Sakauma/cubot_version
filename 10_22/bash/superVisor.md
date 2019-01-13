[program:camDaHua_node]
user=root
autorestart=true
startretries=10000
priority=997
directory=/home/cubot/roboMaster2018/infantry_sample/bash
command=cd /home/cubot/roboMaster2018/infantry_sample/bash
command=sudo sh camDaHua_node.sh
stdout_logfile=/home/cubot/roboMaster2018/infantry_sample/log/camDaHua_cout.log
stderr_logfile=/home/cubot/roboMaster2018/infantry_sample/log/camDaHua_cerr.log
password=123

[program:imgProc]
user=root
autorestart=true
startretries=10000
priority=998
directory=/home/cubot/roboMaster2018/infantry_sample/bash
command=cd /home/cubot/roboMaster2018/infantry_sample/bash
command=sudo sh imgProc.sh
stdout_logfile=/home/cubot/roboMaster2018/infantry_sample/log/imgProc_cout.log
stderr_logfile=/home/cubot/roboMaster2018/infantry_sample/log/imgProc_cerr.log
password=123

