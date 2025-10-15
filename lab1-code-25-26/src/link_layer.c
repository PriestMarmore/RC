// Link layer protocol implementation

#include "link_layer.h"
#include "serial_port.h"

// Others
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <fcntl.h>
#include <termios.h>
#include <signal.h>
#include <errno.h>

// MISC
#define _POSIX_SOURCE 1 // POSIX compliant source

// Frame constants
#define FLAG 0x7E
#define A_TX 0x03   // Address field for Transmitter (Commands) / Receiver (Replies)
#define A_RX 0x01   // Address field for Receiver (Commands) / Transmitter (Replies)
#define C_SET 0x03
#define C_UA 0x07
#define C_DISC 0x0B
#define SU_FRAME_SIZE 5

// Control field values for I-frames (Ns=0 or Ns=1)
#define C_I_0 0x00
#define C_I_1 0x80

// Control field values for RR/REJ frames (Nr=0 or Nr=1)
// RR0: ready to receive I(0), Acknowledges I(1)
#define C_RR_0 0xAA
// RR1: ready to receive I(1), Acknowledges I(0)
#define C_RR_1 0xAB
// REJ0: rejects I(0)
#define C_REJ_0 0x54
// REJ1: rejects I(1)
#define C_REJ_1 0x55

// Stuffing constants
#define ESCAPE_BYTE 0x7D
#define XOR_BYTE 0x20

// Global variables for connection state
int fd1 = -1; // File descriptor for the serial port
LinkLayer g_linkLayer; // Stores the connection parameters

// Sequence number for I-frames (Transmitter Ns, Receiver Nr)
unsigned char g_Ns = 0; // Next sequence number to SEND (0 or 1)
unsigned char g_Nr = 0; // Next sequence number EXPECTED (0 or 1)

// Helper
void create_su_frame(unsigned char *frame, unsigned char A, unsigned char C) {
    frame[0] = FLAG;
    frame[1] = A;
    frame[2] = C;
    frame[3] = A ^ C; // BCC1 (A XOR C)
    frame[4] = FLAG;
}

int read_su_frame(int fd, unsigned char *buffer) {
    enum State {
        START,
        FLAG_RCV,
        A_RCV,
        C_RCV,
        BCC_RCV,
        STOP
    } state = START;

    unsigned char byte;
    int bytes_received = 0;

    // Loop until a full frame is received or read() times out (if VTIME > 0)
    while (state != STOP) {
        // Read one byte. Read will be interrupted by SIGALRM or timeout.
        int res = read(fd, &byte, 1);
        
        if (res <= 0) {
            // Check for interruption by signal (SIGALRM)
            if (errno == EINTR) return 0;
            // Timeout or error (res == 0 or res == -1)
            return 0;
        }

        switch (state) {
            case START:
                if (byte == FLAG) {
                    state = FLAG_RCV;
                    buffer[bytes_received++] = byte;
                }
                break;
            case FLAG_RCV:
                if (byte == FLAG) {
                    bytes_received = 1; // Start over, ensure array only has one FLAG
                } else if (byte == A_TX || byte == A_RX) {
                    state = A_RCV;
                    buffer[bytes_received++] = byte;
                } else {
                    state = START;
                    bytes_received = 0;
                }
                break;
            case A_RCV:
                if (byte == FLAG) {
                    state = FLAG_RCV;
                    bytes_received = 1;
                } 
                // Check for valid C field for SU frames (SET, UA, DISC)
                else if (byte == C_SET || byte == C_UA || byte == C_DISC) { 
                    state = C_RCV;
                    buffer[bytes_received++] = byte;
                } else {
                    state = START;
                    bytes_received = 0;
                }
                break;
            case C_RCV:
                if (byte == FLAG) {
                    state = FLAG_RCV;
                    bytes_received = 1;
                } 
                // BCC1 check: A XOR C
                else if (byte == (buffer[1] ^ buffer[2])) { 
                    state = BCC_RCV;
                    buffer[bytes_received++] = byte;
                } else {
                    state = START;
                    bytes_received = 0;
                }
                break;
            case BCC_RCV:
                if (byte == FLAG) {
                    buffer[bytes_received++] = byte;
                    state = STOP; // Valid frame found
                } else {
                    // Mismatched BCC1 and missing closing FLAG, reset
                    state = START;
                    bytes_received = 0;
                }
                break;
            case STOP:
                break;
        }

        if (bytes_received > SU_FRAME_SIZE) {
            // Should not happen if logic is correct, but safe to reset
            state = START;
            bytes_received = 0;
        }
    }
    
    // Check final frame size
    if (bytes_received == SU_FRAME_SIZE) {
        return SU_FRAME_SIZE;
    }

    return 0;
}

////////////////////////////////////////////////
// LLOPEN
////////////////////////////////////////////////
int llopen(LinkLayer connectionParameters)
{
    // TODO: Implement this function

    // Store parameters globally for potential later use if needed (M4)
    g_linkLayer = connectionParameters;

    // 1. OPEN AND CONFIGURE SERIAL PORT (CRITICAL STEP)
    // We assume openSerialPort is defined in serial_port.h and returns the FD or -1 on failure.
    fd1 = openSerialPort(g_linkLayer.serialPort, g_linkLayer.baudRate);
    
    if (fd1 < 0) {
        // openSerialPort should print its own specific error via perror, 
        // but we add a general fail message.
        fprintf(stderr, "ERROR: llopen failed to open serial port %s.\n", g_linkLayer.serialPort);
        return -1;
    }
    printf("Serial port %s opened successfully (fd=%d).\n", g_linkLayer.serialPort, fd1);


    if (connectionParameters.role == LlTx) {
    
        // Test string to exchange (M1: "Exchange strings")
        unsigned char test_string[] = "Hello Receiver!";
        
        printf("Tx: Sending test string to Rx...\n");
        // Use the low-level serial port write function
        int written = writeBytesSerialPort(test_string, strlen((char*)test_string));

        if (written < 0) {
            perror("writeBytesSerialPort failed");
            return -1;
        }
        printf("Tx: Sent %d bytes.\n", written);
        
        // For M1, we don't need a timeout/retransmission loop yet, 
        // just a simple attempt to read back confirmation.
        
        unsigned char response[50];
        int bytes_read = 0;
        
        // Since VMIN=0/VTIME=1 is set, read() won't block forever.
        printf("Tx: Waiting for response...\n");
        bytes_read = read(fd1, response, 50); // Read directly from file descriptor or use readBytesSerialPort

        if (bytes_read > 0) {
            response[bytes_read] = '\0';
            printf("Tx: Received M1 confirmation: %s\n", response);
            return fd1;
        } else {
            printf("Tx: Did not receive M1 confirmation.\n");
            return -1;
        }
    }

    else if (connectionParameters.role == LlRx) {

        unsigned char buffer[50];
        int bytes_read = 0;

        printf("Rx: Waiting for test string from Tx...\n");
        
        // Read directly from file descriptor or use readBytesSerialPort.
        // The VMIN/VTIME setting means this read will eventually time out 
        // if Tx hasn't sent anything. You may need to loop here for a robust test.
        
        // For a simple M1 test, loop until data is received or you give up.
        while (bytes_read <= 0) {
            bytes_read = read(fd1, buffer, 50);
        }
        
        if (bytes_read > 0) {
            buffer[bytes_read] = '\0';
            printf("Rx: Received test string: %s\n", buffer);
            
            // Send a confirmation back (Exchange strings)
            unsigned char confirmation[] = "Rx ACK M1";
            printf("Rx: Sending M1 confirmation...\n");
            writeBytesSerialPort(confirmation, strlen((char*)confirmation));
            
            return fd1;
        } else {
            printf("Rx: Failed to read any bytes.\n");
            return -1;
        }
    }
    
    return 0;
}

////////////////////////////////////////////////
// LLWRITE
////////////////////////////////////////////////
int llwrite(const unsigned char *buf, int bufSize)
{
    // TODO: Implement this function

        return 0;
    }

////////////////////////////////////////////////
// LLREAD
////////////////////////////////////////////////
int llread(unsigned char *packet)
{
    // TODO: Implement this function

    return 0;
}

////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////
int llclose()
{
    // TODO: Implement this function

    return 0;
}
