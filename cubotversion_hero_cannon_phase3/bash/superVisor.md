[program:cannon_camDaHua_node]
user=root
autorestart=true
startretries=10000
priority=997
directory=/home/cubot/hero/hero_cannon/bash
command=cd /home/cubot/hero/hero_cannon/bash
command=sudo sh cannon_camDaHua_node.sh
stdout_logfile=/home/cubot/hero/logs/cannon_camDaHua_cout.log
stderr_logfile=/home/cubot/hero/logs/cannon_camDaHua_cerr.log
password=123

[program:cannon_imgProc]
user=root
autorestart=true
startretries=10000
priority=998
directory=/home/cubot/hero/hero_cannon/bash
command=cd /home/cubot/hero/hero_cannon/bash
command=sudo sh cannon_imgProc.sh
stdout_logfile=/home/cubot/hero/logs/cannon_imgProc_cout.log
stderr_logfile=/home/cubot/hero/logs/cannon_imgProc_cerr.log
password=123



