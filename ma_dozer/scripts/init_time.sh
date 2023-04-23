#!/bin/bash

DATE=`date +"%Y-%m-%d %T"`

sshpass -p Lifeisgood! ssh dozer@192.168.0.100 "echo Lifeisgood! | sudo -S date --set '$DATE'"

echo "Done!"