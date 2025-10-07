#!/bin/bash

# Paths
PROJECT_DIR="$HOME/University/5/RC/lab1-code-25-26"
BIN_DIR="$PROJECT_DIR/bin"
CABLE_DIR="$PROJECT_DIR/cable"

# Open Terminal 1: cable.c
gnome-terminal -- bash -c "
cd $CABLE_DIR;
gcc -Wall cable.c -o cable;
sudo ./cable;
exec bash"

# Give cable a second to start
sleep 2

# Find the PTY devices created by cable
# Note: You can also hardcode /dev/pts/9 and /dev/pts/10 if they don't change
TX_PORT="/dev/pts/9"
RX_PORT="/dev/pts/10"

# Open Terminal 2: read_noncanonical (transmitter)
gnome-terminal -- bash -c "
cd $PROJECT_DIR;
$BIN_DIR/read_noncanonical $TX_PORT;
exec bash"

# Open Terminal 3: sender simulation
gnome-terminal -- bash -c "
echo -ne '\x01\x02\x03\x04\x05' > $RX_PORT;
echo 'Sent test bytes to $RX_PORT';
exec bash"
