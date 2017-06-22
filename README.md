<<<<<<< HEAD
# UTRA Soccer Control Team
Bipedal control! Based on [UT Austin Villa](https://github.com/LARG/utaustinvilla3d).

## Building
Make sure that you have rcssserver3d-dev (the -dev part is important) installed. [Instructions here](http://simspark.sourceforge.net/wiki/index.php/Installation_on_Linux).

```bash
cd build
cmake ..
make
```

## Running
Once you've built successfully, start the simulation server in one terminal window using:

```bash
rcsoccersim3d
```

The soccer field should come up. Then start an agent in another window with (from inside of the build directory)

```bash
./utracontrol
```

May the torque be with you.
=======
# UTRA Soccer Team  
We are an amateur soccer team at the University of Toronto making an attempt to compete in the 2018 RoboCup

## Team Links
(Please contact Jason at soccer@utra.ca for all access)

- Github Repository
https://github.com/utra-robosoccer/soccerbot/

- Facebook Group
https://www.facebook.com/groups/universityoftorontorobosoccer/

- Google Drive
https://drive.google.com/drive/folders/0B8OHQTLVTR6GTnlfRkFGZk5EQVU?usp=sharing

- Server
http://www.soccer.mcveillance.net/index.html

## Readings

- Competition Information and Dates (Humanoid)
http://wiki.robocup.org/Humanoid_League

- Competition Information and Dates (Simulation)
http://wiki.robocup.org/Soccer_Simulation_League

- Project Planner
https://drive.google.com/drive/folders/0B8OHQTLVTR6GQ3dqamQxMkZrUmc

- ROS Tutorials
http://wiki.ros.org/ROS/Tutorials

## Installation
1. Obtain a copy of Ubuntu, either through a virtual machine or a dual boot (Dual boot preferred)

2. Clone this repository
git clone https://github.com/utra-robosoccer/soccerbot/

3. Execute the installation scripts
```
cd scripts/
sudo sh setup.sh
sudo sh configure_simspark.sh
```

4. Configure your name and email properly
```
git config user.email "your_email@example.com"
git config user.name "Your Name"
```
>>>>>>> 3009868590ad80b5f6f5b3fb5afacf51a9f3aa85
