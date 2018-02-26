cd ~/Documents/Software
git clone https://github.com/laurentkneip/opengv
sudo apt-get install -y build-essential cmake libeigen3-dev
cd opengv
mkdir build
cd build
cmake -DBUILD_PYTHON=ON ..
read  -p "Input Selection:" mainmenuinput
# In python/CMakeLists.txt, replace
# set_target_properties(pyopengv PROPERTIES
#     PREFIX ""
#     SUFFIX ".so"
#     CXX_STANDARD 11
#     CXX_STANDARD_REQUIRED ON
# )

make -j 6
sudo make install

cd ~/Documents/Software
sudo apt-get install -y cmake libgoogle-glog-dev libatlas-base-dev libeigen3-dev libsuitesparse-dev
#git clone https://ceres-solver.googlesource.com/ceres-solver
wget http://ceres-solver.org/ceres-solver-1.13.0.tar.gz
tar zxf ceres-solver-1.13.0.tar.gz
cd ceres-solver-1.13.0/
mkdir build && cd build
cmake ..
make -j 6
sudo make install


cd ~/Documents/Software
git clone git@github.com:mapillary/OpenSfM.git
sudo apt-get install virtualenv
virtualenv SfMenv
source  SfMenv/bin/activate
pip install -r requirements.txt
python setup.py build
