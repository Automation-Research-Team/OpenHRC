#!/bin/bash

sudo apt update
sudo apt upgrade -y
sudo apt -y install ca-certificates curl gnupg python3-pip
sudo mkdir -p /etc/apt/keyrings
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu jammy stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
sudo apt -y install docker-ce docker-ce-cli containerd.io docker-compose-plugin
sudo gpasswd -a $USER docker
pip install rocker