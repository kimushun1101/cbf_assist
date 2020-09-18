#!/bin/bash
cd `dirname $0`
cd ../../

colcon build --packages-select cbf_assist
# source install/setup.bash

cd src/cbf_assist
