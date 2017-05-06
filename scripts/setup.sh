######################################################
# Installation Script for all of robosoccer's robots
######################################################

sudo apt-get update

#### Basic requirements ####
sudo apt install openssh-server
sudo systemctl restart sshd.service
sudo service restart ssh

sudo apt-get install xfce4 xfce4-goodies tightvncserver
mv ~/.vnc/xstartup ~/.vnc/xstartup.bak
echo "#!/bin/bash" >> ~/.vnc/xstartup
echo "xrdb $HOME/.Xresources" >> ~/.vnc/xstartup
echo "startxfce4 &" >> ~/.vnc/xstartup

#### ROS ####

