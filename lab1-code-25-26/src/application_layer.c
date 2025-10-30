#include "application_layer.h"
#include "link_layer.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

// Assuming MAX_PAYLOAD_SIZE is defined in link_layer.h
#ifndef MAX_PACKET_SIZE
// L7 Packet size: C(1) + L2(1) + L1(1) + P(MAX_PAYLOAD_SIZE)
#define MAX_PACKET_SIZE (MAX_PAYLOAD_SIZE + 3) 
#endif


// Max size of the data field (payload) inside a data packet (Control, L2, L1 + Data)
#define MAX_DATA_PAYLOAD_SIZE 1024

// Packet Control Field Definitions (Matching Slide 28 of the specification)
#define C_START 0x01
#define C_DATA 0x02
#define C_END 0x03

// TLV Type Definitions (Matching Slide 28 of the specification)
#define T_FILESIZE 0x00
#define T_FILENAME 0x01

// Helper function: Builds a START or END control packet
int build_control_packet(unsigned char *packet, unsigned char type, const char *filename, long fileSize) {
    if (!packet || !filename) return -1;

    int current_index = 0;

    // 1. Control Field (C) - 0x01 (START) or 0x03 (END)
    packet[current_index++] = type;

    // --- File Size TLV (Type-Length-Value) ---

    // T1: File Size Type (0x00)
    packet[current_index++] = T_FILESIZE;

    // L1: File Size Length (always 8 bytes for long)
    packet[current_index++] = 8;
    
    // V1: File Size (8 bytes, standard long size)
    unsigned char fileSize_bytes[8];
    // Copy the long long size into a byte array (assuming little-endian architecture for simplicity)
    for (int i = 0; i < 8; i++) {
        fileSize_bytes[i] = (unsigned char)((fileSize >> (i * 8)) & 0xFF);
    }
    memcpy(&packet[current_index], fileSize_bytes, 8);
    current_index += 8;

    // --- Filename TLV ---
    
    // T2: Filename Type (0x01)
    packet[current_index++] = T_FILENAME;

    // L2: Filename Length
    size_t filename_len = strlen(filename);
    packet[current_index++] = (unsigned char)filename_len; // Filename length L2 (max 255)

    // V2: Filename Value
    memcpy(&packet[current_index], filename, filename_len);
    current_index += filename_len;

    return current_index;
}

// Helper function: Builds a DATA packet (C, L2, L1, P)
// Note: Sequence number is handled by the Link Layer I-Frame, not the L7 packet.
int build_data_packet(unsigned char *packet, const unsigned char *data, int dataSize) {
    if (!packet || !data) return -1;

    int current_index = 0;

    // 1. Control Field (C) - 0x02 (DATA)
    packet[current_index++] = C_DATA;

    // 2. Length Fields (L2, L1) - Data size is L2*256 + L1
    packet[current_index++] = (unsigned char)(dataSize / 256); // L2 (MSB)
    packet[current_index++] = (unsigned char)(dataSize % 256); // L1 (LSB)

    // 3. Data (P)
    memcpy(&packet[current_index], data, dataSize);
    current_index += dataSize;

    return current_index;
}

// Forward declarations for internal use (or defined in .h)
int llwrite_application(const char *filename);
int llread_application(const char *filename_rx);


// -----------------------------------------------------------------------------
// APPLICATION LAYER ENTRY POINT 
// -----------------------------------------------------------------------------

void applicationLayer(const char *serialPort, const char *role, int baudRate,
                      int nTries, int timeout, const char *filename) {
    // Preparar parâmetros para a camada de enlace
    LinkLayer connectionParameters;
    memset(&connectionParameters, 0, sizeof(connectionParameters));
    strncpy(connectionParameters.serialPort, serialPort, sizeof(connectionParameters.serialPort) - 1);
    connectionParameters.baudRate = baudRate;
    connectionParameters.nRetransmissions = nTries;
    connectionParameters.timeout = timeout;
    connectionParameters.role = (strcmp(role, "tx") == 0) ? LlTx : LlRx;

    // Estabelecer ligação de enlace
    int fd = llopen(connectionParameters);
    if (fd < 0) {
        printf("Application: Error establishing link layer connection.\n");
        return;
    }

    int result = -1;

    if (strcmp(role, "tx") == 0) {
        // TRANSMITTER
        result = llwrite_application(filename);
    } else {
        // RECEIVER
        result = llread_application(filename);
    }

    // Fechar ligação (o teu llclose() não recebe fd)
    if (llclose() < 0) {
        printf("Application: Error closing link layer connection.\n");
        return;
    }

    if (result < 0) {
        printf("Application: Transfer failed.\n");
    } else {
        printf("Application: Transfer completed successfully.\n");
    }
}


// -----------------------------------------------------------------------------
// TRANSMITTER LOGIC (Reads entire file into memory before transmission)
// -----------------------------------------------------------------------------

int llwrite_application(const char *filename) {
    // 1. Get File Information
    struct stat file_stat;
    if (stat(filename, &file_stat) == -1) {
        perror("Error getting file stats");
        return -1;
    }
    long file_size = file_stat.st_size;

    // Use standard C library streams (FILE*) 
    FILE *file_stream = fopen(filename, "rb"); // Open in read-binary mode
    if (file_stream == NULL) {
        perror("Error opening file stream for reading (rb)");
        return -1;
    }
    
    // DEBUG: Log file information
    printf("AppTx: Opened file %s successfully. File size reported by stat: %lu bytes.\n", 
           filename, (long unsigned)file_size);
    
    // --- 0. CRITICAL: READ ENTIRE FILE INTO MEMORY ---
    unsigned char *file_buffer = NULL;
    if (file_size > 0) {
        file_buffer = (unsigned char *)malloc(file_size);
        if (file_buffer == NULL) {
            perror("AppTx: Error allocating memory for file buffer");
            fclose(file_stream);
            return -1;
        }

        // Read the entire file content into the buffer in one go
        size_t bytes_read_all = fread(file_buffer, 1, file_size, file_stream);

        if (bytes_read_all != file_size) {
            fprintf(stderr, "AppTx: Critical: Could only read %zu of %lu bytes into memory. Check ferror/feof.\n", 
                    bytes_read_all, (long unsigned)file_size);
            free(file_buffer);
            fclose(file_stream);
            return -1;
        }
        
        printf("AppTx: Successfully loaded %zu bytes into memory buffer.\n", bytes_read_all);
    }
    
    // Close the file stream immediately after reading into memory
    fclose(file_stream); 

    // --- 1. SEND START CONTROL PACKET ---
    unsigned char start_packet[2048]; // Buffer large enough for control packet
    int start_packet_size = build_control_packet(start_packet, C_START, filename, file_size);
    
    printf("AppTx: Sending START packet (size: %d bytes, file: %s, size: %lu).\n", 
           start_packet_size, filename, (long unsigned)file_size);

    if (llwrite(start_packet, start_packet_size) < 0) {
        printf("AppTx: Error writing START packet. Aborting.\n");
        if (file_buffer != NULL) free(file_buffer);
        return -1;
    }

    // --- 2. SEND DATA PACKETS ---
    int total_data_sent = 0;
    
    // Pointers for iterating through the buffer
    unsigned char *current_position = file_buffer;
    long remaining_bytes = file_size;

    // Loop to read buffer in chunks (MAX_DATA_PAYLOAD_SIZE), create data packets, and send them
    while (remaining_bytes > 0) {
        
        // Determine chunk size
        int chunk_size = (remaining_bytes > MAX_DATA_PAYLOAD_SIZE) ? 
                         MAX_DATA_PAYLOAD_SIZE : (int)remaining_bytes;

        // 2a. Build Data Packet (C, L2, L1, P)
        unsigned char data_packet[MAX_DATA_PAYLOAD_SIZE + 3]; // 3 bytes for C, L2, L1
        int data_packet_size = build_data_packet(data_packet, current_position, chunk_size);

        // 2b. Send Data Packet using llwrite
        int llwrite_result = llwrite(data_packet, data_packet_size);

        if (llwrite_result < 0) {
            printf("AppTx: Error writing data packet. Aborting.\n");
            if (file_buffer != NULL) free(file_buffer); // Free buffer on error
            return -1;
        }

        // Update pointers and counters
        total_data_sent += chunk_size;
        current_position += chunk_size;
        remaining_bytes -= chunk_size;
        
        printf("AppTx: Sent data packet (L7 size=%d). Total sent: %d/%lu\n", 
               data_packet_size, total_data_sent, (long unsigned)file_size);
    }
    
    // Clean up the memory buffer after transmission
    if (file_buffer != NULL) {
        free(file_buffer);
    }
    
    // Fallback: If no data was sent but the file was expected to have data.
    if (total_data_sent == 0 && file_size > 0) {
        printf("AppTx DEBUG: Data transfer loop skipped unexpectedly, despite memory read success.\n");
    }

    // --- 3. SEND END CONTROL PACKET ---
    unsigned char end_packet[2048];
    int end_packet_size = build_control_packet(end_packet, C_END, filename, file_size);
    
    printf("AppTx: Sending END packet (size: %d bytes).\n", end_packet_size);

    if (llwrite(end_packet, end_packet_size) < 0) {
        printf("AppTx: Error writing END packet. Aborting.\n");
        return -1;
    }

    printf("AppTx: File transfer complete. Total bytes sent: %d.\n", total_data_sent);
    return total_data_sent;
}


// -----------------------------------------------------------------------------
// RECEIVER LOGIC
// -----------------------------------------------------------------------------

// Helper function: Parses a control packet (START or END)
long parse_control_packet(const unsigned char *packet, int size, char *filename_out, int max_filename_len) {
    if (size < 1) return -1;
    
    // Check Control Field (C) - 0x01 (START) or 0x03 (END)
    unsigned char type = packet[0];
    if (type != C_START && type != C_END) return -1; 
    
    int current_index = 1;
    long file_size = -1;

    while (current_index < size) {
        unsigned char t_field = packet[current_index++];
        unsigned char l_field = packet[current_index++];
        
        if (current_index + l_field > size) {
            fprintf(stderr, "AppRx: Malformed TLV structure in control packet.\n");
            return -1;
        }

        if (t_field == T_FILESIZE && l_field == 8) {
            // Read 8 bytes for file size (V)
            file_size = 0;
            for (int i = 0; i < 8; i++) {
                // Ensure correct byte order is used based on how Tx wrote it (little-endian assumed)
                file_size |= ((long)packet[current_index + i] << (i * 8));
            }
        } else if (t_field == T_FILENAME && l_field > 0 && filename_out) {
            // Read filename (V)
            int len = (l_field < max_filename_len - 1) ? l_field : max_filename_len - 1;
            memcpy(filename_out, &packet[current_index], len);
            filename_out[len] = '\0';
        }

        current_index += l_field;
    }
    
    return file_size;
}

// Helper function: Parses a data packet (C, L2, L1, P)
// Returns the size of the data payload (P), or -1 on error.
int parse_data_packet(const unsigned char *packet, int size, unsigned char *data_out) {
    if (size < 3) return -1; // Minimum L7 packet size is 3 bytes (C, L2, L1)

    // 1. Check Control Field (C) - should be 0x02 (DATA)
    if (packet[0] != C_DATA) return -1;

    // 2. Length Fields (L2, L1)
    unsigned int data_size = (packet[1] * 256) + packet[2]; // L2 * 256 + L1

    // 3. Check actual data length vs expected length
    if (size != 3 + data_size) {
        fprintf(stderr, "AppRx: Data packet length mismatch! Header says %u, actual is %d (Raw packet size: %d).\n", 
                data_size, size - 3, size);
        return -1; 
    }
    
    // 4. Copy Data (P)
    if (data_out) {
        memcpy(data_out, &packet[3], data_size);
    }

    return data_size;
}


int llread_application(const char *filename_rx) {
    unsigned char packet_buffer[MAX_PACKET_SIZE]; 
    int packet_size;
    long expected_file_size = 0;
    long total_bytes_received = 0;
    char rx_filename[256];
    strncpy(rx_filename, filename_rx, sizeof(rx_filename) - 1);
    rx_filename[sizeof(rx_filename) - 1] = '\0';

    FILE *rx_file_fd = NULL;

    // --- 1. RECEIVE START CONTROL PACKET ---
    packet_size = llread(packet_buffer);
    
    if (packet_size < 0 || packet_buffer[0] != C_START) {
        printf("AppRx: Failed to receive valid START packet. Aborting.\n");
        return -1;
    }

    expected_file_size = parse_control_packet(packet_buffer, packet_size, rx_filename, sizeof(rx_filename));
    
    if (expected_file_size < 0) {
        printf("AppRx: Failed to parse START packet.\n");
        return -1;
    }

    printf("AppRx: Received START packet. File: %s (Size: %lu bytes).\n", rx_filename, expected_file_size);

    // Open output file
    rx_file_fd = fopen(rx_filename, "wb");
    if (rx_file_fd == NULL) {
        perror("Error opening output file for writing");
        return -1;
    }

    // --- 2. RECEIVE DATA PACKETS ---
    while (total_bytes_received < expected_file_size) {
        packet_size = llread(packet_buffer);
        
        if (packet_size < 0) {
            printf("AppRx: Error reading data packet from Link Layer. Aborting.\n");
            fclose(rx_file_fd);
            return -1;
        }

        if (packet_size == 0) {
            // llread returned 0 (e.g., read timeout or duplicate frame discarded). Continue waiting.
            continue;
        }

        if (packet_buffer[0] == C_END) {
            // End packet received: This should technically only happen if file_size was 0 or 
            // if the protocol implementation is slightly off, but it's safe to break.
            break;
        }

        if (packet_buffer[0] == C_DATA) {
            unsigned char data_buffer[MAX_DATA_PAYLOAD_SIZE];
            
            // Parse data packet
            // Note: Sequence check is handled in the Link Layer (llread)
            int data_size = parse_data_packet(packet_buffer, packet_size, data_buffer);
            
            // Robustly check for data parsing errors
            if (data_size < 0) {
                printf("AppRx: Data packet content corruption (L7 length mismatch). Aborting.\n");
                fclose(rx_file_fd);
                return -1;
            }

            if (data_size > 0) {
                // Correctly received and parsed packet: write data
                size_t written = fwrite(data_buffer, 1, data_size, rx_file_fd);
                if (written != data_size) {
                     perror("AppRx: Error writing data to file");
                     fclose(rx_file_fd);
                     return -1;
                }
                total_bytes_received += data_size;
                
                printf("AppRx: Wrote %zu bytes. Total received: %lu/%lu\n", 
                       written, total_bytes_received, expected_file_size);
            }
        } else {
             // Unexpected control byte received in data loop
             printf("AppRx: Unexpected control byte (0x%02X) received in data loop. Aborting.\n", packet_buffer[0]);
             fclose(rx_file_fd);
             return -1;
        }
    }
    
    // --- 3. RECEIVE END CONTROL PACKET ---
    
    // If the loop finished by meeting the expected file size, the buffer currently 
    // holds the *last C_DATA packet*. We must read the next frame (the C_END packet).
    if (packet_buffer[0] != C_END) {
        printf("AppRx: Waiting for final END packet...\n");
        packet_size = llread(packet_buffer);
        
        if (packet_size < 0 || packet_buffer[0] != C_END) {
             printf("AppRx: Failed to receive valid END packet after data transfer. Aborting.\n");
             fclose(rx_file_fd);
             return -1;
        }
    }

    // At this point, packet_buffer[0] should be C_END
    parse_control_packet(packet_buffer, packet_size, NULL, 0); // Parse END packet (mostly for logging)
    printf("AppRx: Received END packet.\n");
    
    fclose(rx_file_fd);

    // --- 4. VERIFY AND CLOSE ---
    if (total_bytes_received != expected_file_size) {
        printf("AppRx: Transfer mismatch! Expected %lu bytes, received %lu bytes.\n", 
               expected_file_size, total_bytes_received);
        return -1;
    }

    printf("AppRx: File received and saved successfully: %s (%lu bytes).\n", rx_filename, total_bytes_received);
    return 0;
}
