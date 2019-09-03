@echo off
REM %DATE:~0,10%  2017/07/06
set dd=%DATE:~0,10%
set tt=%time:~0,8%
set hour=%tt:~0,2%
echo =======================================================
echo          Starting automatic git commit push
echo =======================================================
REM change file directory
cd C:\Users\lf\Desktop\cc_test
REM start git script 
echo %~dp0
git status
git add .
git commit -m "auto_backup %dd:/=-% %tt%"
git push origin master

pause
