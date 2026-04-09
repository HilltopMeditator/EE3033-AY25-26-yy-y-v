#!/bin/bash

# Synchronize time between robot and laptop
if [[ $ROS_IP == 192.168.0.* ]]; then
        ssh bot "sudo date -s @$(echo "$(date +%s.%N)" | bc)"
        echo "Time on Laptop and Machine"
        date +%T:%N; ssh bot date +%T:%N
fi
