#!/usr/bin/env bash

SCRIPT_DIR="${HOME}/ros2_ws/src/gp8_ros2"

echo "Starting GP8 Simulation..."

# Simulation in the current terminal window
gnome-terminal \
    --tab \
    --title="GP8 Simulation" \
    -- bash "$SCRIPT_DIR/start_simulation.sh"

sleep 3

echo "Starting Joint Listener..."

gnome-terminal \
    --tab \
    --title="Joint Listener" \
    -- bash "$SCRIPT_DIR/start_listener.sh"

sleep 1

echo "Starting Joint Logger..."

gnome-terminal \
    --tab \
    --title="Joint Logger" \
    -- bash "$SCRIPT_DIR/start_logger.sh"

sleep 1

echo "Starting Joint Plotter..."

gnome-terminal \
    --tab \
    --title="Joint Plotter" \
    -- bash "$SCRIPT_DIR/start_plotter.sh"

echo "All GP8 processes launched."
