# UTRA Soccer Team Links
(Please contact Jason at soccer@utra.ca for all access)

Facebook Group
https://www.facebook.com/groups/UTRASoccer/

Google Drive
https://drive.google.com/drive/folders/0B8OHQTLVTR6GTnlfRkFGZk5EQVU?usp=sharing

Slack
http://utrarobosoccer.slack.com

Competition Rules (Child Size)
https://www.robocuphumanoid.org/materials/rules/

Competition Information and Dates (Humanoid)
http://wiki.robocup.org/Humanoid_League

Competition Information and Dates (Simulation)
http://wiki.robocup.org/Soccer_Simulation_League

Project Planner
https://drive.google.com/drive/folders/0B8OHQTLVTR6GQ3dqamQxMkZrUmc

Shopping List
https://drive.google.com/drive/folders/0B8OHQTLVTR6GUnZOVXd6VC00UG8

# Virtual Machine Installation Instructions
1. Obtain the Preconfigured Virtual Machine and copy it into the default location (etc $HOME/Virtual Machines)
https://drive.google.com/drive/folders/0B8OHQTLVTR6Ga2xfU0Z6NVVMSFE

2. Install VMWareFusion or VirtualBox VM
INSERT LINKS HERE

3. Open the Virtual Machine from the VM Emulator (Add)

4. Login to the Virtual Machine
User: Soccer
Password: soccer

5. Open your home directory in a terminal window

6. GIT Update your home directory with your team branch.
git pull origin simulation
git pull origin control

# Simulation Team Setup Instructions

# Control Team Setup Instructions

1. Update your home directory with your team branch in your home directory
git pull origin control

2. Source the directory
cd soccer
source devel/setup.bash

3. Install all dependencies
cd src
rosdep check <package_name>
rosdep resolve *

4. Make your project
cd $HOME
cd soccer
catkin_make

