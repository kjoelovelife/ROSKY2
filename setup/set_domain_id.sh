#!/usr/bin/bash
echo "========================================================"
echo "Setting ROS_DOMAIN_ID..."
if [ $# -gt 0 ]; then
    # provided a number in range 0-232, use it as ROS_DOMAIN_ID
    export ROS_DOMIN_ID=$1
else
    echo "No DOMAIN_ID provided. Using 30."
    export ROS_DOMAIN_ID=30
fi

echo "ROS_DO_MAIN_ID set to $ROS_DOMAIN_ID"
echo ""
