set -eu
SCRIPT_DIR=$(cd $(dirname $0); pwd)

cd ${SCRIPT_DIR}
mkdir install/gtsam/build -p 
cd install/gtsam/build 
cmake .. -DGTSAM_BUILD_PYTHON=1 -DGTSAM_PYTHON_VERSION=3.10.12 
make check 
make install 
make python-install

cd ${SCRIPT_DIR}
python3 -m pip install -r requirements.txt 