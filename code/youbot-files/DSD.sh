#!/bin/bash

# Create a new tmux session named 'quadrants', in detached mode
tmux new-session -d -s quadrants

# Split the window into two vertical panes (left and right)
tmux split-window -h

# Split the left pane into two horizontal panes (top-left and bottom-left)
tmux split-window -v -t quadrants:0.0

# Split the right pane into two horizontal panes (top-right and bottom-right)
tmux split-window -v -t quadrants:0.1

# Start youbot-ROS interface in the top-left pane (quadrants:0.0)
tmux send-keys -t quadrants:0.0 "sudo bash -c 'source /opt/ros/hydro/setup.bash  && roslaunch youbot_driver_ros_interface youbot_driver.launch'
" C-m

# Start the keyboard control interface in the bottom-left pane (quadrants:0.2)
tmux send-keys -t quadrants:0.2 "rosrun youbot_control keyboard.py" C-m

# Send 'echo 3' to the top-right pane (quadrants:0.1)
tmux send-keys -t quadrants:0.1 "echo 3" C-m

# Send 'echo 4' to the bottom-right pane (quadrants:0.3)
tmux send-keys -t quadrants:0.3 "echo 4" C-m

# Attach to the tmux session so you can see the result
tmux attach-session -t 

