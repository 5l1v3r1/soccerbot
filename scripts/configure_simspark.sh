sudo apt-add-repository ppa:gnurubuntu/rubuntu -y
sudo apt-get update

sudo apt-get install g++ subversion cmake libfreetype6-dev libode-dev libsdl-dev ruby ruby-dev libdevil-dev libboost-dev libboost-thread-dev libboost-regex-dev libboost-system-dev qt4-default -y
sudo apt-get install tetex-extra latex2html imagemagick -y
sudo apt-get install doxygen -y
sudo apt-get install libqt4-opengl-dev -y

cd ..
sudo rm -rf simspark
svn co https://svn.code.sf.net/p/simspark/svn/trunk simspark

# Simspark
cd simspark/
svn update
cd spark
mkdir build
cd build
cmake ..
make
sudo make install
sudo ldconfig

# RCSserver3D
cd ../../rcssserver3d
mkdir build
cd build
cmake ..
make
sudo make install
sudo ldconfig
