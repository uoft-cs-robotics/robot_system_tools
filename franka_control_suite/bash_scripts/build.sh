SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
source ${SCRIPT_DIR}/setup_env.sh

mkdir -p build
cd build

cmake -DFranka_DIR=/home/snl/libfranka/build ..

make

cd -

