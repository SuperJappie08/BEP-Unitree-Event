#!/usr/bin/bash
rosbag record --duration=30 -e "(/bep/serial_data|/dvs/(.*)|/camera(.)/(.*))" -O BEP-TEST -b 0
