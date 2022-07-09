#!/bin/bash

./pid_controller/pid_controller "0.3" "0.001" "0.3" "0.2" "0.0095" "0.125" &
sleep 1.0
python3 simulatorAPI.py
