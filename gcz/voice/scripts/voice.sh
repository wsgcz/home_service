#!/bin/bash
sudo rm -f /usr/lib/libmsc.so
sudo rm -f /usr/lib/libw_ivw.so
sudo rm -f /usr/local/lib/libmsc.so
sudo rm -f /usr/local/lib/libw_ivw.so
sudo cp ../libs/x64/* /usr/lib
sudo ldconfig
sudo cp ../libs/x64/* /usr/local/lib
sudo ldconfig