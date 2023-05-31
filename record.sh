#!/usr/bin/bash
rosbag record --duration=60 -e "(/bep/serial_data|/dvs/(.*)|/camera(.)/(.*))" -o bags/BEP-TEST -b 0
