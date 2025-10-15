#!/bin/bash

# Configuration and Paths
PROJECT_DIR="$HOME/University/5/RC/lab1-code-25-26"
BIN_DIR="$PROJECT_DIR/bin"
CABLE_DIR="$PROJECT_DIR/cable"

# --- Step 1: Compile the Project ---
# Note: You should replace 'make' with your actual compilation command if you don't use a Makefile.
echo "--- Compiling Project Executables ---"
cd "$PROJECT_DIR"
# Assuming 'make' builds all necessary files (transmitter, receiver)
make 
cd - > /dev/null

# --- Step 2: Start the Virtual Serial Cable (Terminal 1) ---
echo "--- Starting Virtual Cable (Terminal 1) ---"
gnome-terminal -- bash -c "
cd $CABLE_DIR;
# Compile cable.c if not already done
echo 'Compiling cable program...';
gcc -Wall cable.c -o cable;
echo 'Starting virtual cable... Keep this terminal open!';
# Run the cable program (may require sudo)
sudo ./cable;
# The cable program should output the PTY names (e.g., /dev/pts/9 and /dev/pts/10)
echo 'Cable program finished or interrupted.';
exec bash"

# Give cable time to start and establish PTYs
sleep 3

# --- Step 3: Define PTY Ports ---
# IMPORTANT: You must manually check the PTYs created by the 'cable' program 
# in Terminal 1 and replace these hardcoded values if they are different!
TX_PORT="/dev/pts/9"
RX_PORT="/dev/pts/10"
echo "--- Assuming Ports: TX=$TX_PORT, RX=$RX_PORT ---"

# --- Step 4: Run the Receiver Application (Terminal 2) ---
echo "--- Starting Receiver (Terminal 2) on $RX_PORT ---"
gnome-terminal -- bash -c "
cd $PROJECT_DIR;
echo 'Running Receiver on $RX_PORT...';
$BIN_DIR/receiver $RX_PORT;
echo 'Receiver application finished.';
exec bash"

# Give the receiver a moment to open the port
sleep 1

# --- Step 5: Run the Transmitter Application (Terminal 3) ---
echo "--- Starting Transmitter (Terminal 3) on $TX_PORT ---"
gnome-terminal -- bash -c "
cd $PROJECT_DIR;
echo 'Running Transmitter on $TX_PORT...';
$BIN_DIR/transmitter $TX_PORT;
echo 'Transmitter application finished.';
exec bash"

echo "All terminals launched. Monitor them for the M1 exchange."
