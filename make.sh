#!/usr/bin/env bash

# If run with ./make.sh -c, clear the build directory then exit
if [ "$1" == "-c" ]; then
    rm -rf ./go2/build
    echo -e "\n===== ./go2/build directory cleared =====\n"
    exit 0
fi

# If run with ./make.sh, check if build directory exists, if not, create it
if [ ! -d "./go2/build" ]; then
    mkdir -p ./go2/build
fi

# Build go2 controller
cd ./go2/build/
cmake .. && make -j8
