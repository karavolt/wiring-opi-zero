#!/bin/bash

git clone https://github.com/zhaolei/WiringOP.git -b h3 wiringpi
cd ./wiringpi/wiringPi/
make static
cd ../../
node-gyp rebuild
