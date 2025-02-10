/* ECG Simulator from SD Card - Sender
 * 
 * Reads data from an SD card and sends the corresponding ADC value to a
 * receiver Nucleo over the serial communcation lines. The data on the SD card
 * is assumed to have 10 bytes per data point, the relevant 11-bit ADC value
 * stored as ASCII characters occupying positions 5-8. Each data point is
 * terminated with a new line character. The data point is converted to a short
 * and sent over the communication lines at a fixed rate.
 *
 * This program assumes that the ECG files are stored as txt files and are in a
 * folder titled "MITBIH" (case sensitive).
 * 
 *
 * Originally written by Lucas Ratajczyk, date N/A
 * Modified by Royal Oakes, 2020-02-02
 */


#include "mbed.h"
#include "SDFileSystem.h"

const int BLK_SIZE = 512; // Block size of SD card in bytes
const int BLK_NUM = 4;    // Number of blocks to be loaded during runtime
int DP_SIZE = 10;   // Number of bytes in a single data point

Serial pc(USBTX,USBRX);

// Define transmitter and receiver for Serial communication with Receiver 
// (which will receive extracted simulator data)
Serial receiver(D1,D0); 

// Initialize the SD-card file system object using the necessary SPI pins for 
// our Nucleo board A3 = CS, A4 = SCK, A5 = MISO, A6 = MOSI, VCC = 5V, GND
SDFileSystem sd(A6, A5, A4, A3, "sd");  

DigitalOut out(D10);        // Digital output for use as a debugging flag
Ticker SampleTicker;        // Ticker for ISR

float SampleRate = 360.0;   // Sampling frequency of the ISR
float SamplePeriod;         // Sampling period for the ISR

FILE *fp = NULL;                   // Initialize pointer to our file
DIR *dp = NULL;                    // Directory pointer

// Variables for circular buffer
char dbuff[BLK_NUM*BLK_SIZE];   // Buffer for data
int lidx = 0;   // Index of the next byte to be loaded into dbuff
int ridx = 0;   // Index of the next byte to be read from dbuff
int rblk = 0;   // The current block being read from dbuff

char buff[100] = "/sd/MITBIH/"; // A buffer for the file to be run
int fstart = 0, fend = 0;       // Beginning and end of the file to be run

Timer t; // Timer for debugging

typedef union _data { // Use this to send a 2 byte value to pc and device
    short s;
    char h[sizeof(short)];
} myShort;

// CLI user interface
int s = 0;
char const max_cmd = 30;
char cmd_buf[max_cmd] = {0};

// Function prototypes
void loadc(void);
char readc(void);
void send_sp(void);

uint32_t _f_read_dpsize(FILE* fp) {
    uint32_t i = 0;
    uint32_t limit = 100;
    uint32_t dp_size;
    char temp[64] = {0};
    char hold = 0;

    fseek(fp, 0, SEEK_SET);

    while(i < limit) {
        hold = (char) fgetc(fp);
        if (hold == (char) EOF || hold == '\r' || hold == '\n') {
            break;
        }
        i++;
    } 

    if (hold == (char) EOF || i >= limit) { // EOF reached before first delimiter
        return 0;
    }

    if (hold == '\r') { // Windows CRLF
        dp_size = i + 2;
    }

    if (hold == '\n') { // UNIX LF
        dp_size = i + 1;
    }
    DP_SIZE = dp_size;
    return dp_size;
}

void get_cmd(void) {
    s = 0;
    char temp = 0;
    while (1) {
        if (pc.readable()) {
            temp = pc.getc();
        }

        switch (temp) {
            case 0x00: // Nothing has been pressed.
                break;
            case 0x0d: // Enter is pressed.
            case 0x0a:
                cmd_buf[s] = '\0';
                return;
            case 0x7f: // Backspace or delete has been pressed.
            case 0x7e:
                pc.printf("\33[2K\r");
                s = -1;
                return;
            default:   // A character has been pressed.
                cmd_buf[s++] = temp;
                temp = 0;
                pc.printf("%c", cmd_buf[s-1]);
        }

        if (s >= max_cmd) {
            pc.printf("\33[2K\r");
            pc.printf("Input Buffer Overflow. Buffer Flushed.\n\r");
            s = -1;
            return;
        }
    }
}

int main() {
    pc.baud(115200);
    pc.printf("\33[2J\r");
    STARTOVER:
    if(fp != NULL) {
        fclose(fp);
    }
    sd.unmount();
    pc.printf("\r\n--------------- Welcome! ---------------");
    pc.printf("\r\n     BME 463 File Reader, Version 2     ");
    pc.printf("\r\n---------------- START! ----------------\r\n");

    receiver.baud(115200);

    sd.mount(); // Mount the SD card (whatever that means)
    
    // Open the directory with ECG files
    struct dirent *dirp;
    if ((dp = opendir("/sd/MITBIH")) == NULL) { 
        pc.printf("\r\nMITBIH directory not found.");
        pc.printf("\r\nPress any key to restart.");
        while(!pc.readable());
        pc.getc();
        goto STARTOVER;
    }
    
    pc.printf("\r\nAvailable files in MITBIH:");
    
    //unsigned int nfiles = 0;
    int fsize = 0;
    int nfiles = 0;
    while ((dirp = readdir(dp)) != NULL) { // List all the files in the MITBIH dir
        nfiles++;
        sprintf(&buff[11], "%s", dirp->d_name);
        
       // Get file size
        fp = fopen(buff, "r");
        _f_read_dpsize(fp);
        fseek(fp, 0, SEEK_END);
        fsize = ftell(fp) / DP_SIZE;
        fclose(fp);
        
        pc.printf("\r\n%s [0,%d)",dirp->d_name, fsize);
    }
    
    if (nfiles == 0) {
        pc.printf("\r\nThere are no available files in MITBIH.");
        pc.printf("\r\nPress any key to restart.");
        while(!pc.readable());
        pc.getc();
        goto STARTOVER;
    }
    
    // Get file to run
    pc.printf("\r\n\r\n");
    s = -1;
    while (s == -1) {
        pc.printf("Please choose a file to run: ");
        get_cmd();
    }
    sscanf(cmd_buf, "%s", &buff[11]);
    //scanf("%s", &buff[11]);
    //pc.printf("\r\n%s", &buff[11]);

    // Open the file 
    if ((fp = fopen(buff, "r")) == NULL) { // Open the file
        pc.printf("\r\nCould not open \"%s\".", buff);
        pc.printf("\r\nPress any key to restart.");
        while(!pc.readable());
        pc.getc();
        goto STARTOVER;
    }
    _f_read_dpsize(fp);
    
    // Get start time
    pc.printf("\r\n");
    s = -1;
    while (s == -1) {
        pc.printf("Choose start position: ");
        get_cmd();
    }
    sscanf(cmd_buf, "%d", &fstart);
    fstart *= DP_SIZE;
    // pc.printf("\r\nStart: ");
    // scanf("%d", &fstart);
    // pc.printf("%d", fstart);
    
    // Get end time
    pc.printf("\r\n");
    s = -1;
    while (s == -1) {
        pc.printf("Choose end position (zero runs to end of file): ");
        get_cmd();
    }
    sscanf(cmd_buf, "%d", &fend);
    fend *= DP_SIZE;
    // pc.printf("\r\nEnd (zero runs to end of file): ");
    // scanf("%d", &fend);
    // pc.printf("%d", fend);

    // Check that start and end are appropriate
    fseek(fp, 0, SEEK_END);
    fsize = ftell(fp);
    if (fstart > fend && fend != 0) {
        pc.printf("\r\nStart position is larger than end position.");
        pc.printf("\r\nPress any key to restart.");
        while(!pc.readable());
        pc.getc();
        goto STARTOVER;
    }
    
    if (fstart >= fsize) { 
        pc.printf("\r\nStart position is larger than file size");
        pc.printf("\r\nPress any key to restart.");
        while(!pc.readable());
        pc.getc();
        goto STARTOVER;
    }
    
    if (fend >= fsize || fend == 0) { // if fend is too large or zero, run to the end of the file
        fend = fsize;
    }

    pc.printf("\r\n");
    s = -1;
    while (s == -1) {
        pc.printf("Enter sample frequency (Hz): ");
        get_cmd();
    }
    
    sscanf(cmd_buf, "%f", &SampleRate);

    if (SampleRate <= 0) {
        pc.printf("\r\nSample Frequency too low!");
        pc.printf("\r\nPress any key to restart.");
        while(!pc.readable());
        pc.getc();
        goto STARTOVER;
    }

    if (SampleRate > 1000) {
        pc.printf("\r\nSample Frequency too high!");
        pc.printf("\r\nPress any key to restart.");
        while(!pc.readable());
        pc.getc();
        goto STARTOVER;
    }
    
    fseek(fp, fstart, SEEK_SET);             // Move the pointer to fstart
    SamplePeriod = (float) 1.0f/SampleRate;  // Calculate the sample period 
    
    //t.start();  // Timer for debugging                          
    
    // Load data from SD card and fill dbuff
    for (int i = 0; i < BLK_NUM*BLK_SIZE; i++)
        loadc();

    pc.printf("\r\n\r\n~oOo~ File Running ~oOo~\r\n");
    
    // Start sending data
    SampleTicker.attach(&send_sp,SamplePeriod);
    
    // Load new bytes into dbuff if ridx is close to lidx
    int d = 0;
    int dots = 0;
    int d_thresh = 500000;
    int dots_clear = 4;
    while(1) {
        // if (feof(fp)) {     // If we have reached the end of the file...
        //     fclose(fp);         // Close the file
        //     sd.unmount();       // Unmount the SD card (whatever that means)
        // } 
        
        // Check if ridx has moved beyond the current blk
        if (rblk < BLK_NUM - 1 && ridx >= BLK_SIZE * (rblk+1)) {
            for (int i = 0; i < BLK_SIZE; i++)
                loadc();
            
            rblk++;
        } else if (rblk == BLK_NUM - 1 && ridx < (BLK_SIZE*rblk) - 1) {
            for (int i = 0; i < BLK_SIZE; i++)
                loadc();
            
            rblk = 0;
        } else {
        }

        if (pc.readable()) {
            pc.getc();
            SampleTicker.detach();
            pc.printf("\r\nSTART OVER!\r\n");
            goto STARTOVER;
        }

        if (d++ >= d_thresh) {
            if (dots < dots_clear - 1) {
                pc.printf(". ");
                dots++;
                d = 0;
            } else {
                dots++;
                d = 0;
            }
            
        }

        if (dots >= dots_clear) {
            pc.printf("\33[2K\r");
            dots = 0;
        }
    } // end while
} // end main

/* Load a byte from the SD card. If the current byte is greater than fend, wrap
 * to fstart.
 */
void loadc(void) {
    dbuff[lidx++] = getc(fp);
    
    if (lidx >= BLK_NUM*BLK_SIZE)
        lidx = 0;
    
    if (ftell(fp) >= fend)
        fseek(fp, fstart, SEEK_SET);
        
    return;
}

/* Read a byte from dbuff and increment ridx.
 */
char readc(void) {
    char temp = dbuff[ridx++];
    
    if (ridx >= BLK_NUM*BLK_SIZE)
        ridx = 0;
        
    return temp;
}

/* Send a data point to the reciever. This function assumes that relevant data
 * is after the 5th index within the temporary buffer.
 */
void send_sp(void) {
    char tbuff[DP_SIZE];  // Holds the data point
    myShort temp;    // Holds the short to be sent
    
    // Load data point
    for (int i = 0; i < DP_SIZE; i++)
        tbuff[i] = readc();
    
    // Replace '\n' with '\0';
    tbuff[DP_SIZE - 1] = '\0';
    
    // Convert string to short
    sscanf(tbuff, "%hu", &temp.s);
    
    // Send data
    for (int i = 0; i < sizeof(short); i++)
        receiver.putc(temp.h[i]);
    receiver.putc('\0');
    
    // Debugging print.
    //pc.printf("%hi,%f\n", temp.s, t.read());
    return;
}