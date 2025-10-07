// Example of how to read from the serial port in non-canonical mode
//
// Modified by: Eduardo Nuno Almeida [enalmeida@fe.up.pt]
// Added: State machine for SET frame reception and UA response
// Cleaned up debug output to avoid repeated FLAGs

#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <termios.h>
#include <unistd.h>

#define _POSIX_SOURCE 1 // POSIX compliant source
#define FALSE 0
#define TRUE 1

#define BAUDRATE 38400

int fd = -1;           // File descriptor for open serial port
struct termios oldtio; // Serial port settings to restore on closing
volatile int STOP = FALSE;

int openSerialPort(const char *serialPort, int baudRate);
int closeSerialPort();
int readByteSerialPort(unsigned char *byte);
int writeBytesSerialPort(const unsigned char *bytes, int nBytes);

// ---------------------------------------------------
// STATE MACHINE FOR RECEIVING SET
// ---------------------------------------------------
typedef enum {
    START,
    FLAG_RCV,
    A_RCV,
    C_RCV,
    BCC_OK,
    STOP_STATE
} State;

// ---------------------------------------------------
// MAIN
// ---------------------------------------------------
int main(int argc, char *argv[])
{
    if (argc < 2)
    {
        printf("Incorrect program usage\n"
               "Usage: %s <SerialPort>\n"
               "Example: %s /dev/ttyS0\n",
               argv[0], argv[0]);
        exit(1);
    }

    // Open serial port device for reading and writing
    const char *serialPort = argv[1];
    if (openSerialPort(serialPort, BAUDRATE) < 0)
    {
        perror("openSerialPort");
        exit(-1);
    }
    printf("Serial port %s opened\n", serialPort);

    // Create UA frame to send after SET is received
    unsigned char UA[5];
    UA[0] = 0x7E;           // FLAG
    UA[1] = 0x01;           // Address field for Receiver
    UA[2] = 0x07;           // Control field UA
    UA[3] = UA[1] ^ UA[2];  // BCC1 = A ^ C
    UA[4] = 0x7E;           // FLAG

    State state = START;
    unsigned char A, C, BCC;
    unsigned char byte;

    // ---------------------------------------------------
    // Read bytes from serial port and process with state machine
    // ---------------------------------------------------
    while (!STOP)
    {
        int bytes = readByteSerialPort(&byte);
        if (bytes > 0)
        {
            // Debug: print each byte received until frame is fully received
            if (state != STOP_STATE)
                printf("Byte received: 0x%02X\n", byte);

            switch(state)
            {
                case START:
                    if (byte == 0x7E) // FLAG received
                        state = FLAG_RCV;
                    break;

                case FLAG_RCV:
                    if (byte == 0x7E) // repeated FLAG
                        state = FLAG_RCV;
                    else if (byte == 0x03) // A for SET
                    {
                        A = byte;
                        state = A_RCV;
                    }
                    else
                        state = START; // any other byte, restart
                    break;

                case A_RCV:
                    if (byte == 0x7E)
                        state = FLAG_RCV;
                    else if (byte == 0x03) // C = SET
                    {
                        C = byte;
                        state = C_RCV;
                    }
                    else
                        state = START;
                    break;

                case C_RCV:
                    if (byte == 0x7E)
                        state = FLAG_RCV;
                    else if (byte == (A ^ C)) // BCC check
                    {
                        BCC = byte;
                        state = BCC_OK;
                    }
                    else
                        state = START;
                    break;

                case BCC_OK:
                    if (byte == 0x7E) // final FLAG
                        state = STOP_STATE;
                    else
                        state = START;
                    break;

                case STOP_STATE:
                    STOP = TRUE; // frame successfully received
                    break;
            }
        }
    }

    printf("SET frame received!\n");

    // Send UA frame back to transmitter
    writeBytesSerialPort(UA, 5);
    printf("UA frame sent.\n");

    // Close serial port
    if (closeSerialPort() < 0)
    {
        perror("closeSerialPort");
        exit(-1);
    }
    printf("Serial port %s closed\n", serialPort);

    return 0;
}

// ---------------------------------------------------
// SERIAL PORT LIBRARY IMPLEMENTATION
// ---------------------------------------------------
int openSerialPort(const char *serialPort, int baudRate)
{
    int oflags = O_RDWR | O_NOCTTY | O_NONBLOCK;
    fd = open(serialPort, oflags);
    if (fd < 0) { perror(serialPort); return -1; }

    if (tcgetattr(fd, &oldtio) == -1) { perror("tcgetattr"); return -1; }

    tcflag_t br;
    switch (baudRate)
    {
        case 1200: br = B1200; break;
        case 1800: br = B1800; break;
        case 2400: br = B2400; break;
        case 4800: br = B4800; break;
        case 9600: br = B9600; break;
        case 19200: br = B19200; break;
        case 38400: br = B38400; break;
        case 57600: br = B57600; break;
        case 115200: br = B115200; break;
        default: fprintf(stderr, "Unsupported baud rate\n"); return -1;
    }

    struct termios newtio;
    memset(&newtio, 0, sizeof(newtio));

    newtio.c_cflag = br | CS8 | CLOCAL | CREAD;
    newtio.c_iflag = IGNPAR;
    newtio.c_oflag = 0;
    newtio.c_lflag = 0;

    // VTIME and VMIN as per lab guide
    newtio.c_cc[VTIME] = 0; // no timeout
    newtio.c_cc[VMIN]  = 1; // blocking read, byte by byte

    tcflush(fd, TCIOFLUSH);
    if (tcsetattr(fd, TCSANOW, &newtio) == -1) { perror("tcsetattr"); close(fd); return -1; }

    // Clear O_NONBLOCK to ensure blocking reads
    oflags ^= O_NONBLOCK;
    if (fcntl(fd, F_SETFL, oflags) == -1) { perror("fcntl"); close(fd); return -1; }

    return fd;
}

int closeSerialPort()
{
    if (tcsetattr(fd, TCSANOW, &oldtio) == -1) { perror("tcsetattr"); return -1; }
    return close(fd);
}

int readByteSerialPort(unsigned char *byte)
{
    return read(fd, byte, 1);
}

int writeBytesSerialPort(const unsigned char *bytes, int nBytes)
{
    return write(fd, bytes, nBytes);
}
