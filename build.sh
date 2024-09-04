#!/bin/bash 

case $1 in
d|D) 
cmake -DCMAKE_BUILD_TYPE=Debug CMakeLists.txt
echo "Making Debug"
make -j12
;;&
r|R) 
cmake -DCMAKE_BUILD_TYPE=Release CMakeLists.txt
echo "Making Release"
make -j12
;;&
b|B) 
cmake -DCMAKE_BUILD_TYPE=RelWithDebInfo CMakeLists.txt
echo "Making RelWithDebInfo"
make -j12
;;&
--default)
echo "Options are d = debug, r = release, b = both"
;;&
esac


