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

// Frame constants (BCC1 = A ^ C)
#define FLAG 0x7E
#define A_TX 0x03   // Address field for Transmitter (Commands) / Receiver (Replies)
#define A_RX 0x01   // Address field for Receiver (Commands) / Transmitter (Replies)
#define C_SET 0x03
#define C_UA 0x07
#define C_DISC 0x0B
#define SU_FRAME_SIZE 5 // Size of SET, UA, DISC frames

// Control field values for I-frames (Ns=0 or Ns=1)
#define C_I_0 0x00
#define C_I_1 0x80 // RESTORED TEMPLATE VALUE

// Control field values for RR/REJ frames (Nr=0 or Nr=1)
// RR0: ready to receive I(0), Acknowledges I(1)
#define C_RR_0 0xAA // RESTORED TEMPLATE VALUE
// RR1: ready to receive I(1), Acknowledges I(0)
#define C_RR_1 0xAB // RESTORED TEMPLATE VALUE
// REJ0: rejects I(0)
#define C_REJ_0 0x54 // RESTORED TEMPLATE VALUE
// REJ1: rejects I(1)
#define C_REJ_1 0x55 // RESTORED TEMPLATE VALUE

// Stuffing constants
#define ESCAPE_BYTE 0x7D
#define XOR_BYTE 0x20

// Global variables for connection state
int fd1 = -1; // File descriptor for the serial port
LinkLayer g_linkLayer; // Stores the connection parameters

// Sequence number for I-frames (Transmitter Ns, Receiver Nr)
unsigned char g_Ns = 0; // Next sequence number to SEND (0 or 1)
unsigned char g_Nr = 0; // Next sequence number EXPECTED (0 or 1)

// External functions defined in alarm_sigaction.c
extern void setupAlarmHandler();
// Declaration now matches alarm_sigaction.c and link_layer.h (uses int)
extern void enableAlarm(int timeout, int maxRetransmissions); 
extern void disableAlarm();
extern int isAlarmSet();
extern void clearAlarm();
extern int getRetransmissionCount();

// Helper functions
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
        // Read one byte. Read will be interrupted by SIGALARM or timeout.
        int res = read(fd, &byte, 1);
        
        if (res <= 0) {
            // Check for interruption by signal (SIGALARM)
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
                    bytes_received = 1; 
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
                    state = START;
                    bytes_received = 0;
                }
                break;
            case STOP:
                break;
        }

        if (bytes_received > SU_FRAME_SIZE) {
            state = START;
            bytes_received = 0;
        }
    }
    
    if (bytes_received == SU_FRAME_SIZE) {
        return bytes_received;
    }

    return 0;
}


////////////////////////////////////////////////
// LLOPEN (M2 & M4 Implementation)
////////////////////////////////////////////////
int llopen(LinkLayer connectionParameters)
{
    g_linkLayer = connectionParameters;

    // 1. OPEN AND CONFIGURE SERIAL PORT (M1)
    fd1 = openSerialPort(g_linkLayer.serialPort, g_linkLayer.baudRate);
    
    if (fd1 < 0) {
        fprintf(stderr, "ERROR: llopen failed to open serial port %s.\n", g_linkLayer.serialPort);
        return -1;
    }
    printf("Serial port %s opened successfully (fd=%d).\n", g_linkLayer.serialPort, fd1);


    if (connectionParameters.role == LlTx) {
        // --- TRANSMITTER LOGIC (M2 & M4) ---

        // Setup the alarm signal handler
        setupAlarmHandler();
        
        unsigned char set_frame[SU_FRAME_SIZE];
        unsigned char response_frame[SU_FRAME_SIZE];
        int bytes_read = 0;
        
        // Create the SET command frame: F, A_TX, C_SET, BCC1, F
        create_su_frame(set_frame, A_TX, C_SET);

        // Start the retransmission loop
        while (getRetransmissionCount() < g_linkLayer.nRetransmissions) {
            
            // 1. Send SET frame
            printf("Tx: Sending SET frame...\n");
            int written = writeBytesSerialPort(set_frame, SU_FRAME_SIZE);

            if (written < 0) {
                perror("Tx: writeBytesSerialPort failed");
                llclose();
                return -1;
            }
            
            // 2. Enable Timer
            // Note: The signature for enableAlarm is (int timeoutSec, int maxTries)
            enableAlarm(g_linkLayer.timeout, g_linkLayer.nRetransmissions);

            // 3. Wait for UA response
            printf("Tx: Waiting for UA response...\n");
            
            while (!isAlarmSet()) {
                bytes_read = read_su_frame(fd1, response_frame);

                if (bytes_read == SU_FRAME_SIZE) {
                    // Check if the received frame is a valid UA (A=0x03, C=0x07)
                    if (response_frame[1] == A_TX && response_frame[2] == C_UA) {
                        printf("Tx: Received valid UA frame. Connection established.\n");
                        disableAlarm();
                        return fd1; // Success!
                    } else {
                        // Received a frame, but it's not the expected UA. Ignore and keep waiting.
                        printf("Tx: Received frame, but not UA (A=0x%02x, C=0x%02x). Ignoring.\n", 
                                response_frame[1], response_frame[2]);
                    }
                } 
                // If bytes_read is 0 (read() timed out but alarm hasn't fired yet), continue waiting.
            }
            
            // If we exit the inner loop, it means isAlarmSet() is true (timeout).
            disableAlarm();
            clearAlarm(); // Reset flag for the next retransmission attempt
            
            // The outer loop will increment the retransmission count implicitly in alarmHandler
            // and check it against nRetransmissions.
        }

        // If loop completes without returning, max retransmissions reached
        printf("Tx: Failed to establish connection after %d attempts\n", g_linkLayer.nRetransmissions);
        llclose();
        return -1;
    }

    else if (connectionParameters.role == LlRx) {
        // --- RECEIVER LOGIC (M2) ---

        unsigned char received_frame[SU_FRAME_SIZE];
        int bytes_read = 0;
        
        printf("Rx: Waiting for SET frame...\n");

        // Loop indefinitely until a valid SET frame is received
        while (1) {
            bytes_read = read_su_frame(fd1, received_frame);

            if (bytes_read == SU_FRAME_SIZE) {
                // Check if the received frame is a valid SET (A=0x03, C=0x03)
                if (received_frame[1] == A_TX && received_frame[2] == C_SET) {
                    printf("Rx: Received valid SET frame. Sending UA...\n");
                    
                    // 1. Create UA response frame: F, A_TX, C_UA, BCC1, F
                    unsigned char ua_frame[SU_FRAME_SIZE];
                    create_su_frame(ua_frame, A_TX, C_UA); 
                    
                    // 2. Send UA response
                    if (writeBytesSerialPort(ua_frame, SU_FRAME_SIZE) < 0) {
                        perror("Rx: writeBytesSerialPort (UA) failed");
                        llclose();
                        return -1;
                    }
                    printf("Rx: Connection established.\n");
                    return fd1; // Success
                } else {
                    // Received a valid control frame, but not SET. Ignore and keep waiting.
                    printf("Rx: Received a frame, but not SET (A=0x%02x, C=0x%02x). Ignoring.\n", received_frame[1], received_frame[2]);
                }
            } else if (bytes_read < 0) {
                 perror("Rx: Error reading response frame");
                 llclose();
                 return -1;
            }
        }
    }
    
    return -1; // Should not be reached if one of the roles is executed
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
    // Restores original termios settings and closes the port
    if (fd1 != -1) {
        // disable any active alarm just in case
        disableAlarm(); 
        int res = close(fd1); 
        fd1 = -1;
        printf("Serial port closed.\n");
        return res;
    }
    return -1;
}
