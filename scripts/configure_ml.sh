
# Tensorflow installation
cd ../libraries/
sudo apt-get install python3-pip python3-dev python-virtualenv -y # for Python 3.n
virtualenv --system-site-packages -p python3 tensorflow # for Python 3.n
source tensorflow/bin/activate # bash, sh, ksh, or zsh
easy_install -U pip
source tesnsorflow/bin/activate
deactivate
