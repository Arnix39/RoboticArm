#!/bin/bash

sudo service udev restart
sudo udevadm control --reload-rules && sudo udevadm trigger 
