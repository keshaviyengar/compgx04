#!/bin/bash

# This script automates installing GTSAM on Linux and macOS targets
# The toolbox is installed, by default, in the directory "/opt/gtsam/compgx04".

INSTALL_DIR=/opt/gtsam/compgx04

if [ -d  "${INSTALL_DIR}" ]; then
    echo Install directory ${INSTALL_DIR} exists already
else
    echo Creating install directory ${INSTALL_DIR}
    sudo mkdir -p ${INSTALL_DIR}
    sudo chown ${USER} ${INSTALL_DIR}
fi

# Create the build directory if necessary

if [ ! -d ../build ]; then
    echo Creating the build subdirectory
    mkdir ../build
fi

pushd ../build > /dev/null
cmake ../gtsam -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/opt/gtsam/develop \
      -DGTSAM_INSTALL_MATLAB_TOOLBOX=TRUE -DGTSAM_BUILD_EXAMPLES_ALWAYS=FALSE \
      -DCMAKE_INSTALL_PREFIX=${INSTALL_DIR} -DGTSAM_BUILD_TESTS=OFF -DGTSAM_BUILD_UNSTABLE=FALSE
make -j2 install

popd
