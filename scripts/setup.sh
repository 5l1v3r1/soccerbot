######################################################
# Installation Script for all of robosoccer's robots
######################################################

sudo apt-get update

#### Basic requirements ####
sudo apt install openssh-server -y
sudo systemctl restart sshd.service
sudo service ssh start

sudo apt-get install xfce4 xfce4-goodies tightvncserver -y
cp -r .vnc  ~/.vnc
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
wget wget download.netbeans.org/netbeans/8.2/final/bundles/netbeans-8.2-cpp-linux-x64.sh

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
rm -rf gazebo8_install.sh

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
if ! grep -q /include/setup.bash ~/.bashrc;
then
    echo "source $ROS_WD/include/setup.bash" >> ~/.bashrc
fi

### MATLAB ###
wget http://esd.mathworks.com/R2017a/Linux_x86_64/INST_432557/matlab_R2017a_glnxa64.zip
unzip matlab_R2017a_glnxa42.zip
