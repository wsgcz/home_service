#!/bin/bash
#This is the bash script for voice dynamic link lib
sudo cp ../libs/x64/* /usr/lib
sudo ldconfig
sudo cp ../libs/x64/* /usr/bin/lib
sudo ldconfig
sed '/appid = /s//{$1}' ../src/
sed '{$2}' ../src/
