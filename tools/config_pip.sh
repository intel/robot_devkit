#!/bin/sh

mkdir -p ~/.pip
cd ~/.pip
touch pip.conf

# Set pack-age index for users in China
echo "[global]" > pip.conf
echo "index-url = https://pypi.tuna.tsinghua.edu.cn/simple" >> pip.conf

# copy to ROOT for sudoers
sudo mkdir -p /root/.pip
sudo cp ~/.pip/pip.conf /root/.pip/
