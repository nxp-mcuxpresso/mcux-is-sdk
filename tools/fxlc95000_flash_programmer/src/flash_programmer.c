
#include <string.h>
#include <termios.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <unistd.h>

typedef unsigned char octet;

typedef struct
{
    char *str;
    long val;
} baud_t;

enum
{
    READY = 0,
    RUNNING,
    STOPPED
};

#define FULL 2
#define WARN 1
#define NONE 0

#define DEFAULT_BAUDRATE B115200
#define CI_MAX_MSG_LEN 64
#define HDLC_PKT_MARKER 0x7E
#define FLASH_OFFSET 10

#define REPORT_LEVEL(l, s, args...) \
    if (ll >= l)                    \
        fprintf(stderr, s, ##args);

static FILE *flashfile;
static char ciFixedBytes[] = "21000000044C";
static int fd, ec = 0, s = 0, fc = 0, ll = FULL;
static unsigned char Flash_Programmer_State = READY;
octet ciFlashStartBytes[] = {0x7E, 0x21, 0x00, 0x00, 0x02, 0x03, 0x4C, 0x7E};
octet ciFlashEndBytes[] = {0x7E, 0x21, 0x00, 0x00, 0x02, 0x05, 0x4C, 0x7E};

void stopStreaming();
void usage(char *name);
void int_handler(int sig);
int get_baud_rate(char *bstr);
int getFormattedFlashCmd(FILE *flashfile, octet *buf, int size);

int main(int ac, char **av)
{
    octet buf[64];
    char portName[64];
    int sc, mc, nr, i, cp, br;
    struct termios oldtio, newtio;
    struct sigaction sigIntHandler;

    /* Initialize to invalid value */
    br = DEFAULT_BAUDRATE;
    flashfile = NULL;
    cp = -1;
    nr = -1;

    /*
     **  Parse commandline arguments
     */
    if (ac < 3)
    {
        REPORT_LEVEL(WARN, "Not enough arguments\n");
        usage(av[0]);
    }

    for (i = 1; i < ac; i++)
    {
        if (av[i][0] == '-')
        {
            switch (av[i][1])
            {
                case 'B':
                    br = get_baud_rate(av[i] + 1);
                    if (br == -1)
                    {
                        REPORT_LEVEL(WARN, "Invalid baud rate %s\n", av[i] + 2);
                        usage(av[0]);
                    }
                    REPORT_LEVEL(WARN, "Using baud rate: %s(0x%0x)\n", av[i] + 1, br);
                    break;
                case 'p':
                    cp = atoi(av[i] + 2) - 1; // (Window's COM5 is cygwin /dev/ttyS4 )
                    break;
                case 'q':
                    if (av[i][2] == 'q')
                        ll = NONE;
                    else
                        ll = WARN;
                    break;
                default:
                    REPORT_LEVEL(WARN, "Unrecognized option %s\n", av[i]);
                    usage(av[0]);
            }
        }
        else
        {
            flashfile = fopen(av[i], "r");
        }
    }

    if (cp == -1)
    {
        REPORT_LEVEL(WARN, "No COM port specified. Use -p option.\n");
        usage(av[0]);
    }
    if (flashfile == NULL)
    {
        REPORT_LEVEL(WARN, "No SREC/S19 file specified.\n");
        usage(av[0]);
    }

    sprintf(portName, "/dev/ttyS%d", cp);
    REPORT_LEVEL(FULL, "Using com port %s\n", portName);

    /* Open the serial port for read/write */
    fd = open(portName, O_RDWR);

    if (fd < 0)
    {
        perror(portName);
        REPORT_LEVEL(WARN, "Unable to open port %s\n", portName);
        exit(-1);
    }

    /* register a callbak to turn of streaming whenever we exit */
    atexit(stopStreaming);

    /* Install signal handler to catch Ctrl-C, etc, to make sure properly stop streaming, and close files */
    sigIntHandler.sa_handler = int_handler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;

    sigaction(SIGINT, &sigIntHandler, NULL);
    sigaction(SIGHUP, &sigIntHandler, NULL);
    sigaction(SIGTERM, &sigIntHandler, NULL);

    fcntl(fd, F_SETOWN, getpid());
    tcgetattr(fd, &oldtio); /* Save current port settings */
    tcgetattr(fd, &newtio); /* Load current port settings */

    cfsetispeed(&newtio, br);
    cfsetospeed(&newtio, br);

    tcflush(fd, TCIFLUSH);
    tcsetattr(fd, TCSANOW, &newtio);

    REPORT_LEVEL(NONE, "\nFlash Programmer running...\n");
    while ((i = getFormattedFlashCmd(flashfile, buf, sizeof(buf))))
    {
        if (write(fd, buf, i))
        {
            s++;
            mc = 0;
            REPORT_LEVEL(FULL, "\nLine [%d] sending [%d] bytes of SREC FLASH Data to embedded app.\n", s, i);
            while (mc < i)
            {
                REPORT_LEVEL(FULL, "%02X", (int)buf[mc++]);
            }
            REPORT_LEVEL(FULL, "\n");
            if (s == 1)
            { /* The maximum erase time will be 127 X Terase = approximately 3.5 seconds.*/
                REPORT_LEVEL(FULL, "\nWait 3 seconds for Mass Erase to complete.\n");
                sleep(3);
            }
            mc = 0;
            nr = 0;
            sc = 0;
            while (mc < 2 && read(fd, buf + nr, 1))
            {
                REPORT_LEVEL(FULL, "%x ", buf[nr]);
                if (buf[nr] == 0x7E)
                    mc++;
                if (mc == 0) /* Skip characters outside of HDLC_PKT_MARKER */
                    sc++;
                nr++;
            }
            REPORT_LEVEL(FULL, "\nReceived %d bytes response from embedded app.\n", nr);
            if (nr - sc == 6 || nr - sc == 7) // SQN can be 2 byte encoded if escape character is present)
            {
                if (buf[1] != 0xA1)
                {
                    fc++;
                    REPORT_LEVEL(WARN, "\nCommand processing failed for Line [%d].\n", s);
                }
            }
            else
            {
                fc++;
                REPORT_LEVEL(WARN, "\nInvalid response message for Line [%d].\n", s);
            }
        }
        else
        {
            REPORT_LEVEL(WARN, "Failed to write on COM port.\n");
        }
    }

    tcsetattr(fd, TCSANOW, &oldtio);
    exit(0);
}

int getFormattedFlashCmd(FILE *flashfile, octet *buf, int size)
{
    // Each line of SREC file is TLV encoded (2 char TAG, 2 char length, <...>, 2 char CRC).
    char byte[2], sqnField[2], lengthFied[4];
    int character, characters, offset;

    /* If it the first call for reading a flash transmission, send the flash start transmission marker. */
    if (Flash_Programmer_State == READY)
    {
        memcpy(buf, ciFlashStartBytes, sizeof(ciFlashStartBytes));
        Flash_Programmer_State = RUNNING;

        return sizeof(ciFlashStartBytes);
    }

    // Leave 10 extra character head room to accommodate HDLC Marker, ISSDK Header and Configuration Bytes.
    while (NULL != fgets((char *)buf + FLASH_OFFSET, size - FLASH_OFFSET, flashfile))
    {
        if (buf[FLASH_OFFSET] == 'S' && buf[FLASH_OFFSET + 1] == '3')
        {
            byte[0] = buf[FLASH_OFFSET + 2];
            byte[1] = buf[FLASH_OFFSET + 3];
            characters = FLASH_OFFSET + strtol(byte, NULL, 16) * 2;

            /* Get Sequence and Length Field strings */
            sprintf(sqnField, "%.2X", s % 256); // Use line no as sequence number.
            sprintf(lengthFied, "%.4X", (characters - FLASH_OFFSET + 2) /
                                            2); // Length = Actual bytes of Flash + 2 Configuration Bytes - CRC Byte.

            memcpy(buf + 2, ciFixedBytes, strlen(ciFixedBytes)); // Overwrite with the ISSK header characters.
            memcpy(buf + 4, sqnField, strlen(sqnField));         // Update ISSK header SQN field.
            memcpy(buf + 6, lengthFied, strlen(lengthFied));     // Update ISSK header LENGTH field.

            buf[0] = HDLC_PKT_MARKER; // Overwrite HDLC Start marker.
            // Convert string to bytes.
            for (character = 2, offset = 1; character <= characters; character += 2, offset++)
            {
                byte[0] = buf[character];
                byte[1] = buf[character + 1];
                buf[offset] = strtol(byte, NULL, 16);

                // Handle CI escape sequences.
                if (buf[offset] == 0x7D)
                {
                    buf[++offset] = 0x5D;
                }
                else if (buf[offset] == 0x7E)
                {
                    buf[offset] = 0x7D;
                    buf[++offset] = 0x5E;
                }
            }
            buf[offset] = HDLC_PKT_MARKER; // Append HDLC end marker.
            if (offset++ > CI_MAX_MSG_LEN)
            {
                ec++;
                offset = 0;
            }
            return offset;
        }
    }

    /* If you reach here first time, then it means all lines have been read from the file.
     * Send the end flash transmission marker. */
    if (Flash_Programmer_State == RUNNING)
    {
        memcpy(buf, ciFlashEndBytes, sizeof(ciFlashEndBytes));
        Flash_Programmer_State = STOPPED;

        return sizeof(ciFlashEndBytes);
    }

    return 0;
}

void stopStreaming()
{
    REPORT_LEVEL(NONE, "\nFlash Programmer successfully sent [%d] Commands to Embedded app.\n", s);
    if (ec > 0)
    {
        REPORT_LEVEL(WARN, "Found [%d] FLASH Lines with greater than CI MAX MSG LEN of (%d).\n", ec, CI_MAX_MSG_LEN);
    }
    if (fc > 0)
    {
        REPORT_LEVEL(WARN, "Found [%d] Lines with incorrect response.\n", fc);
    }
    close(fd);
    fclose(flashfile);
}

void int_handler(int sig)
{
    exit(1);
}

#define BAUDENTRY(b)        \
    {                       \
        .str = #b, .val = b \
    }

baud_t baudrates[] = {
    BAUDENTRY(B0),       BAUDENTRY(B50),      BAUDENTRY(B75),      BAUDENTRY(B110),     BAUDENTRY(B134),
    BAUDENTRY(B150),     BAUDENTRY(B200),     BAUDENTRY(B300),     BAUDENTRY(B600),     BAUDENTRY(B1200),
    BAUDENTRY(B1800),    BAUDENTRY(B2400),    BAUDENTRY(B4800),    BAUDENTRY(B9600),    BAUDENTRY(B19200),
    BAUDENTRY(B38400),   BAUDENTRY(B57600),   BAUDENTRY(B115200),  BAUDENTRY(B128000),  BAUDENTRY(B230400),
    BAUDENTRY(B256000),  BAUDENTRY(B460800),  BAUDENTRY(B500000),  BAUDENTRY(B576000),  BAUDENTRY(B921600),
    BAUDENTRY(B1000000), BAUDENTRY(B1152000), BAUDENTRY(B1500000), BAUDENTRY(B2000000), BAUDENTRY(B2500000),
    BAUDENTRY(B3000000),
};

int numBaudRates = sizeof(baudrates) / sizeof(baud_t);

int get_baud_rate(char *bstr)
{
    int i;
    for (i = 0; i < numBaudRates; i++)
    {
        if (!strcmp(bstr, baudrates[i].str))
        {
            return baudrates[i].val;
        }
    }
    return -1;
}

void usage(char *name)
{
    int i;

    fprintf(stderr, "Usage: %s [-q(q)] -p<comm port #> [-B<Baud Rate>] <flash file>\n", name);
    fprintf(stderr, "       -p#  The Windows Comm port number.  E.g., -p5 for COM5\n");
    fprintf(stderr, "            E.g., -s1 for Sensor #1 in Sensor Subscription List.\n");
    fprintf(stderr, "       -q   Don't echo data bytes.\n");
    fprintf(stderr, "       -qq  Don't echo anything including error messages.\n");
    fprintf(stderr, "       -B#  The baud rate to connect.\n");
    fprintf(stderr, "            Valid baud rate specifiers are:\n");
    fprintf(stderr, "                 ");
    for (i = 0; i < numBaudRates; i++)
    {
        if (i % 4 == 0)
            fprintf(stderr, "\n                 ");
        fprintf(stderr, "%9s,", baudrates[i].str);
    }
    fprintf(stderr, "\n\n   flashfile  The name of the SREC/S19 file for reading the FLASH bytes.\n");

    exit(-1);
}
