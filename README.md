# UTRA Soccer Team
We are an amateur soccer team at the University of Toronto making an attempt to compete in the 2018 RoboCup

## Team Links
(Please contact Jason at soccer@utra.ca for all access)

- Github Repository
https://github.com/utra-robosoccer/soccerbot/

- Facebook Group
https://www.facebook.com/groups/UTRASoccer/

- Google Drive
https://drive.google.com/drive/folders/0B8OHQTLVTR6GTnlfRkFGZk5EQVU?usp=sharing

- Slack
http://utrarobosoccer.slack.com

- Meeting Calender Subscription
https://calendar.google.com/calendar/ical/soccer%40utra.ca/public/basic.ics

- Competition Rules (Child Size)
https://www.robocuphumanoid.org/materials/rules/

- Competition Information and Dates (Humanoid)
http://wiki.robocup.org/Humanoid_League


- Competition Information and Dates (Simulation)
http://wiki.robocup.org/Soccer_Simulation_League

- Project Planner
https://drive.google.com/drive/folders/0B8OHQTLVTR6GQ3dqamQxMkZrUmc

- Shopping List
https://drive.google.com/drive/folders/0B8OHQTLVTR6GUnZOVXd6VC00UG8

## Virtual Machine Installation Instructions
1. Obtain the Preconfigured Virtual Machine and copy it into the default location (etc $HOME/Virtual Machines)
https://drive.google.com/open?id=0BytZsWzpd8xeakROOEptaWkxdDA

2. Install VMWareFusion or VirtualBox VM
INSERT LINKS HERE

3. Open the Virtual Machine from the VM Emulator (Add)

4. Login to the Virtual Machine
User: Soccer
Password: soccer

5. Open your home directory in a terminal window

6. Configure your name and email properly
```
git config user.email "your_email@example.com"
git config user.name "Your Name"
```

7. GIT Update your home directory with your team branch.
```
git pull origin simulation
git pull origin control
```

## 3D Simulation Team Setup Instructions
http://simspark.sourceforge.net/wiki/index.php/Installation_on_Linux
Do Ubuntu Part 2 (but prerequisites first)

## 2D Simulation Team Steup Instructions
Follow instructions in the Install file

## Control Team Setup Instructions

Update your home directory with your team branch in your home directory
```
git clone https://github.com/utra-robosoccer/soccerbot
cd soccerbot
```

Source the directory
```
cd soccer
source devel/setup.bash
```

Install all dependencies
```
cd src
rosdep check <package_name>
rosdep resolve *
```

Make your project
```
cd $HOME
cd soccer
catkin_make
```

Please see the github Wiki for more information
