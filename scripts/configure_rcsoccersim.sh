# Configure simspark monitor

cd ../rcsoccersim/rcssserver
git pull origin master
autoconf
aclocal
autoreconf --install
automake --add-missing
sudo ./configure
sudo apt-get install libaudio-dev
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

