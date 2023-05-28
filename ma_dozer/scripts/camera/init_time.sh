#!/bin/bash

DATE=`date -d "1 seconds" +"%Y-%m-%d %T"`

sshpass -p Lifeisgood! ssh pi@192.168.0.101 "echo Lifeisgood! | sudo -S date --set '$DATE'"
sshpass -p Lifeisgood! ssh pi@192.168.0.102 "echo Lifeisgood! | sudo -S date --set '$DATE'"

echo "Done!"