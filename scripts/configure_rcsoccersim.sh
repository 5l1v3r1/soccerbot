sudo apt-get install autoconf automake -y
git pull --recurse-submodules
git submodule update --recursive --remote

# Configure simspark monitor

cd ../rcsoccersim/rcssserver
git pull origin master
autoconf
aclocal
autoreconf --install
automake --add-missing
sudo apt-get install libaudio-dev
sudo apt-get install flex bison -y
sudo ./configure
sudo make
sudo make install
sudo ldconfig

cd ../../scripts

# Configure simspark monitor

cd ../rcsoccersim/rcssmonitor
git pull origin master
autoconf
aclocal
autoreconf --install
automake --add-missing
sudo ./configure
sudo apt-get install libaudio-dev
sudo make
sudo make install

cd ../../scripts

# Configure log monitor

cd ../rcsoccersim/rcsslogplayer
git pull origin master
autoconf
aclocal
autoreconf --install
automake --add-missing
sudo ./configure
sudo make
sudo make install
sudo ldconfig

cd ../../scripts

# More setup
sudo chmod +x ~/.rcssserver/

