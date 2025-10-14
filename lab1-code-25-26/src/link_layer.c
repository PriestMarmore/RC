// Link layer protocol implementation

#include "link_layer.h"
#include "serial_port.h"

// MISC
#define _POSIX_SOURCE 1 // POSIX compliant source

// Frame constants
#define FLAG 0x7E
#define A_TX 0x03   // Address field for Transmitter (Commands) / Receiver (Replies)
#define A_RX 0x01   // Address field for Receiver (Commands) / Transmitter (Replies)
#define C_SET 0x03
#define C_UA 0x07
#define C_DISC 0x0B

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
int fd = -1; // File descriptor for the serial port
LinkLayer g_linkLayer; // Stores the connection parameters

// Sequence number for I-frames (Transmitter Ns, Receiver Nr)
unsigned char g_Ns = 0; // Next sequence number to SEND (0 or 1)
unsigned char g_Nr = 0; // Next sequence number EXPECTED (0 or 1)

////////////////////////////////////////////////
// LLOPEN
////////////////////////////////////////////////
int llopen(LinkLayer connectionParameters)
{
    // TODO: Implement this function

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
