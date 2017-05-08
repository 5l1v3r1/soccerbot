######################################################
# Installation Script for all of robosoccer's robots
######################################################

sudo apt-get update

#### Basic requirements ####
sudo apt install openssh-server -y
sudo systemctl restart sshd.service
sudo service restart ssh

sudo apt-get install xfce4 xfce4-goodies tightvncserver -y
mv ~/.vnc/xstartup ~/.vnc/xstartup.bak
mv xstartup ~/.vnc/xstartup

#### ROS ####
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
sudo apt-get update
sudo apt-get install ros-kinetic-desktop-full -y
sudo rosdep init
rosdep update
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
source /opt/ros/kinetic/setup.bash
echo "source /opt/ros/kinetic/setup.zsh" >> ~/.zshrc
source ~/.zshrc
sudo apt-get install python-rosinstall -y


