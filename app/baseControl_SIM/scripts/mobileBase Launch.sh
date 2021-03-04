gnome-terminal
ssh cer-user@192.168.100.10 -X
expect "assword:"
send "cerAdmin\r"
interact
yarpserver
####################################### OR TRY THE FOLLOWING
#!/usr/bin/expect
eval spawn ssh -oStrictHostKeyChecking=no -oCheckHostIP=no cer-user@192.168.100.10 -X
#use correct prompt
set prompt ":|#|\\\$"
interact -o -nobuffer -re $prompt return
send "cerAdmin\r"
interact -o -nobuffer -re $prompt return
send "yarpserver\r"
interact

gnome-terminal
ssh cer-user@192.168.100.10 -X
cd /usr/local/src/robot/cer/app/robots/CER01/
robotInterface

gnome-terminal
ssh cer-user@192.168.100.10 -X
cd /usr/local/src/robot/cer/app/robots/CER01/
yarpmotorgui --openloop

gnome-terminal
ssh cer-user@192.168.100.10 -X
cd /usr/local/src/robot/cer/app/baseControl/conf
baseControl

gnome-terminal
ssh cer-user@192.168.100.10 -X
yarpdev --device inertial --subdevice xsensmtx -d /dev/ttyusb0


gnome-terminal
yarp conf 192.168.100.10 10000

gnome-terminal
yarpdatadumper --connect /inertial

gnome-terminal
sudo rmmod xpad
sudo xboxdrv

gnome-terminal
cd $ICUB_DIR
joystickCtrl --from /home/luca/workspace/icub-main/app/joystickCtrl/conf/cer_linux.ini

gnome-terminal
yarp connect /joystickCtrl:o /cer/joystick:i
