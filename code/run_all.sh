#!/usr/bin/env bash

# If project not ready, generate cmake file.
if [[ ! -d build ]]; then
    echo "good"
else
    rm -rf build
fi
mkdir -p build
cd build
cmake ..
make -j
cd ..

# Run all testcases. 
# You can comment some lines to disable the run of specific examples.
mkdir -p output
# bin/PJ testcases/scene01_basic.txt output/scene01.bmp
# bin/PJ testcases/scene02_cube.txt output/scene02.bmp
# bin/PJ testcases/scene03_sphere.txt output/scene03.bmp
# bin/PJ testcases/scene04_axes.txt output/scene04.bmp
# bin/PJ testcases/scene05_bunny_200.txt output/scene05.bmp
# bin/PJ testcases/scene06_bunny_1k.txt output/scene06.bmp
# bin/PJ testcases/scene07_shine.txt output/scene07.bmp
# bin/PJ testcases/scene08_core.txt output/scene08.bmp
bin/PJ testcases/scene09_norm.txt output/scene09.bmp
bin/PJ testcases/scene10_wineglass.txt output/scene10.bmp

