echo "Configuring and building movesense_sensor..."

export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:${PWD}
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j
cd ..

echo "Finished build movesense_sensor"
