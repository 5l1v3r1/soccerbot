######################################################
# Installation Script for all of robosoccer's robots
######################################################

sudo apt-get update
cd ../
git submodule init
git submodule update
cd scripts/

#### Basic requirements ####
sudo apt install openssh-server -y
sudo systemctl restart sshd.service
sudo service ssh start

sudo apt-get install xfce4 xfce4-goodies tightvncserver -y
mkdir ~/.vnc
cp -f xstartup  ~/.vnc/xstartup
chmod +x ~/.vnc/xstartup

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

sudo apt-get install python-rosinstall -y

### NETBEANS ###
wget download.netbeans.org/netbeans/8.2/final/bundles/netbeans-8.2-cpp-linux-x64.sh

chmod +x netbeans-8.2-cpp-linux-x64.sh

sudo ./netbeans-8.2-cpp-linux-x64.sh --silent #shhhh
rm -rf netbeans-8.2-cpp-linux-x64.sh #dont need installer after installling

roscd
cd ..
mkdir ~/.netbeans/8.1/etc/
echo ". $(pwd)/setup.sh" > ~/.netbeans/8.1/etc/netbeans.conf

### VIM ###
sudo apt-get install vim -y

sh configure_simspark.sh

### GAZEBO ###
wget osrf-distributions.s3.amazonaws.com/gazebo/gazebo8_install.sh
chmod +x gazebo8_install.sh
cd ..
sh scripts/gazebo8_install.sh
cd scripts/
rm -rf ../gazebo8_install.sh

sudo apt-get install meshlab -y

### OTHER ###
if ! grep -q ROS_WD ~/.bashrc;
then
    cd ../soccer/
    echo "export ROS_WD=$(pwd)" >> ~/.bashrc
fi
source ~/.bashrc

### ROS EXTERNAL DEPENDENCIES ###
sudo apt-get install ros-kinetic-geographic-msgs -y

rosinstall ../soccer/include/ ../soccer/include/packages.rosinstall
echo "source ~/soccerbot/soccer/devel/setup.bash" >> ~/.bashrc
sudo apt-get install libopencv-dev python-opencv -y


### MATLAB ###
sudo apt-get install default-jre -y
sudo apt-get install default-jdk -y

## Libportaudio
sudo apt-get install libasound-dev
cd ../libraries/portaudio/
./configure && make
sudo make install
cd ../../scripts

## AWS
pip install awscli

## Miscellaneous
sudo apt-get install openni2-utils -y

