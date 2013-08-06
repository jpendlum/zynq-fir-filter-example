/******************************************************************************
**  This is free software: you can redistribute it and/or modify
**  it under the terms of the GNU General Public License as published by
**  the Free Software Foundation, either version 3 of the License, or
**  (at your option) any later version.
**
**  This is distributed in the hope that it will be useful,
**  but WITHOUT ANY WARRANTY; without even the implied warranty of
**  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
**  GNU General Public License for more details.
**
**  You should have received a copy of the GNU General Public License
**  along with this code.  If not, see <http://www.gnu.org/licenses/>.
**
**
**
**  File:         zynq_fir_filter_example.vhd
**  Author(s):    Jonathon Pendlum (jon.pendlum@gmail.com)
**  Dependencies: Zynq Example FPGA Project - https://www.github.com/jpendlum/zynq-acp-example
**                Zynq ACP Kernel Driver - https://bitbucket.org/mfischer/user-peripheral-kmod-public
**  Description:  Example code that configures and utilizes a fixed point FIR
**                filter in the programmable logic of a Xilinx Zynq FPGA. This
**                code is used in conjunction with Moritz Fischer's Zynq ACP
**                Linux kernel driver and the zynq-acp-example Xilinx FPGA
**                code.
**
**                Note: The FPGA example coprocessor is a 21-tap, symmetric,
**                      fixed point FIR filter with reloadable coefficients
**                      generated with Xilinx's coregen. The coefficients are
**                      32-bit integers, the filter expects input samples to
**                      be 32-bit integers, and outputs 68-bit samples
**                      truncated (with convergent even rounding) to 64-bits.
**                      http://www.xilinx.com/support/documentation/ip_documentation/fir_compiler/v6_3/ds795_fir_compiler.pdf
**
******************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <libudev.h>
#include <unistd.h>
#include <signal.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <string.h>

// The kernel driver allocates 2^17 bytes (2^15 addressable) for control registers
// and 2^10*4096 bytes for data.
//
// The control registers are broken into three banks based on Bits 13:12 of the memory
// address:
// 00: host to slave commands (reads)
// 01: slave to host commands (writes)
// 10: global settings
// 11: Unused
#define OFFSET_H2S          (0)
#define OFFSET_S2H          (1 << 10)
#define OFFSET_GLOBAL       (1 << 11)
#define OFFSET_FIR_CONFIG   (1*8) + OFFSET_H2S
#define OFFSET_FIR_RELOAD   (2*8) + OFFSET_H2S

#define FIFO_WR_CLEAR       0
#define FIFO_WR_ADDR        1
#define FIFO_WR_SIZE        2
#define FIFO_WR_STS_RDY     3
#define FIFO_WR_STS         4
#define FIFO_RD_SIG         0
#define FIFO_RD_STATUS      1
#define FIFO_RD_STATUS_CNT  2
#define FIFO_RD_ADDR_CNT    3
#define FIFO_RD_SIZE_CNT    4

long read_fpga_status();
static int get_params_from_sysfs(unsigned long *buffer_length, unsigned long *control_length, unsigned long *phys_addr);
int open_driver(int *fd, unsigned long *buffer_length, unsigned long *control_length, unsigned long *phys_addr,
    unsigned int **control_regs, unsigned int **buff);
int close_driver(int fd, unsigned long buffer_length, unsigned long control_length, unsigned int *control_regs,
    unsigned int *buff);
void ctrl_c(int dummy);

// Global variable used to kill final loop
int kill_prog = 0;



int main()
{
    int fd;
    unsigned long int buffer_length;
    unsigned long int control_length;
    unsigned long int phys_addr;
    unsigned int *control_regs;
    unsigned int *buff;
    long long int *samples;
    unsigned long int samples_length;
    unsigned long int number_of_samples;
    unsigned long long coefficients[11];
    int num_errors = 0;
    int num_iterations = 0;
    int i = 0;

    printf("\n");
    printf("Read the FPGA status... \n");
    if (read_fpga_status() != 1)
    {
        printf("FPGA not programmed!\n");
        return(0);
    }
    printf("Setup the driver... \n");
    open_driver(&fd,&buffer_length,&control_length,&phys_addr,&control_regs,&buff);
    // ACP bus is 64 bits wide, so use appropriate sized variable
    samples = (long long int *)buff;
    // Buffer length is in bytes
    samples_length = buffer_length/sizeof(long long int);
    printf("\n");
    printf("Control Registers Address: \t%p\n",control_regs);
    printf("Control Registers Length: \t%lx\n",control_length);
    printf("Sample Buffer Address: \t\t%p\n",samples);
    printf("Sample Buffer Length: \t\t%lx\n",samples_length);

    /************************************************************************************
     * Set coefficients then test impulse response of the FPGA FIR Filter
     ***********************************************************************************/
    // Note, the control registers have the following format:
    // Bits  0-3: Control registers
    //            Write (i.e. reg[1] = x):
    //            0 = FIFO clear
    //            1 = Write data to address FIFO
    //            2 = Write data to size FIFO
    //            3 = Strobe status FIFO read enable (i.e. pop one value)
    //            4 = Write data to status FIFO (loopback test)
    //            Read (i.e. x = reg[1]):
    //            0 = 16-bit hardwired word (0xACE0) appended with 16-bit current stream value (only 2 LSBs relevant)
    //            1 = Output of status FIFO
    //            2 = Status FIFO word count
    //            3 = Command address FIFO word count
    //            4 = Command size FIFO word count
    //       4-5: Stream select
    //       6-8: Unused
    //      9-10: 0 = Host to Slave commands (i.e. read from RAM commands)
    //            1 = Slave to Host commands (i.e. write to RAM commands)
    //            2 = Global settings (Set CACHE and USER options for AXI bus)
    //            3 = Unused
    control_regs[OFFSET_GLOBAL] = 0;                // Soft reset command (resets everything)
    control_regs[OFFSET_S2H+FIFO_WR_CLEAR] = 0;     // Reset S2H FIFO command, the data is ignored
    control_regs[OFFSET_H2S+FIFO_WR_CLEAR] = 0;     // Reset S2H FIFO command, the data is ignored
    printf("Read signature: \t\t%x\n",control_regs[FIFO_RD_SIG]);
    printf("Status FIFO output: \t\t%x\n",control_regs[FIFO_RD_STATUS]);
    printf("Status FIFO Word Count: \t%d\n",control_regs[FIFO_RD_STATUS_CNT]);
    printf("Address FIFO Word Count: \t%d\n",control_regs[FIFO_RD_ADDR_CNT]);
    printf("Size FIFO Word Count: \t\t%d\n",control_regs[FIFO_RD_SIZE_CNT]);

    // Set new coefficients for 21-tap _symmetric_ FIR filter.
    coefficients[0] = 6;     // b_0 & b_20
    coefficients[1] = 0;     // b_1 & b_19
    coefficients[2] = -4;    // b_2 & b_18
    coefficients[3] = -3;    // b_3 & b_17
    coefficients[4] = 5;     // b_4 & b_16
    coefficients[5] = 6;     // b_5 & b_15
    coefficients[6] = -6;    // b_6 & b_14
    coefficients[7] = -13;   // b_7 & b_13
    coefficients[8] = 7;     // b_8 & b_12
    coefficients[9] = 44;    // b_9 & b_11
    coefficients[10] = 64;   // b_10
    for (i = 0; i < 11; i++)
    {
        samples[i] = coefficients[i];
    }
    // Note: In general it is better to setup the write address before the read address. If the
    //       accelerator has to wait to write data and does not have sufficient internal buffering,
    //       then an overflow could occur.
    // Note: When sending a write or read address, the actual write / read action is triggered
    //       after both the address and the number of bytes to read is sent.
    // Load new coefficients by setting the stream to 2 (see OFFSET_FIR_RELOAD) and writing
    // the physical address + size to the control register FIFOs.
    control_regs[OFFSET_FIR_RELOAD+FIFO_WR_ADDR] = phys_addr;
    control_regs[OFFSET_FIR_RELOAD+FIFO_WR_SIZE] = 11 * sizeof(long long int);
    // The read command blocks until an interrupt from the FPGA signals that the transfer is finished
    read(fd,0,0);
    // Set config word to load new coefficient. In this case, the generated FIR
    samples[11] = 0;
    // Commit coefficients
    control_regs[OFFSET_FIR_CONFIG+FIFO_WR_ADDR] = phys_addr + 11 * sizeof(long long int);
    control_regs[OFFSET_FIR_CONFIG+FIFO_WR_SIZE] = 1 * sizeof(long long int);
    // BUG: The read below causes the program to freeze. The freeze occurs due to the kernel driver 
    //      waiting for an interrupt that never comes (or possibly comes early) after calling 
    //      wait_event_interruptible() in the read() handler. This problem does not occur with the
    //      other read calls in this program, so perhaps it has to do with fact that this AXI
    //      transfer is only one 64-bit message long?
    // read(fd,0,0);

    // Write impulse. Use a larger impulse to counteract the LSB truncation in the FIR filter.
    samples[0] = 1 << 5;
    for (i = 1; i < 32; i++)
    {
        samples[i] = 0;
    }
    number_of_samples = 32 * sizeof(long long int);
    // Set the write address (offset in memory by number_of_samples)
    control_regs[OFFSET_S2H+FIFO_WR_ADDR] = phys_addr + number_of_samples;
    // Set the number of bytes to write
    control_regs[OFFSET_S2H+FIFO_WR_SIZE] = number_of_samples;
    read(fd,0,0);
    // Set the read address
    control_regs[OFFSET_H2S+FIFO_WR_ADDR] = phys_addr;
    // Set the number of bytes to read
    control_regs[OFFSET_H2S+FIFO_WR_SIZE] = number_of_samples;
    read(fd,0,0);

    // Read all status FIFOs. If not, the status FIFOs will fill and eventually cause
    // the AXI datamover in the FPGA to ignore further requests.
    control_regs[OFFSET_H2S+FIFO_WR_STS_RDY] = 0;
    control_regs[OFFSET_S2H+FIFO_WR_STS_RDY] = 0;
    control_regs[OFFSET_FIR_CONFIG+FIFO_WR_STS_RDY] = 0;
    control_regs[OFFSET_FIR_RELOAD+FIFO_WR_STS_RDY] = 0;

    // Read result. Should be the FIR impulse response, which is simply coefficents set earlier.
    printf("\n");
    printf("---- Impulse response ----\n");
    for (i = 32; i < 42; i++)
    {
        printf("samples[%2d]: %3lld\t\tExpected: %3lld\n",i,samples[i],coefficients[i-32]);
    }
    for (i = 42; i < 53; i++)
    {
        printf("samples[%2d]: %3lld\t\tExpected: %3lld\n",i,samples[i],coefficients[42-i+10]);
    }

    printf("\n");
    printf("---- Self-verification ----\n");
    printf("Randomly sets coefficients and verifies impulse reponse.\n");
    printf("Any errors detected will be reported.\n");
    printf("Use CTRL-C to exit...\n");

    // Capture CTRL-C
    signal(SIGINT, ctrl_c);

    srand (time(NULL));
    while (kill_prog == 0)
    {
        for (i = 0; i < 11; i++)
        {
            // Clear lower 5 bits due to LSB truncation in FIR filter
            coefficients[i] = (long long int)((rand() & 0x000000FF) << 5);
        }
        for (i = 0; i < 11; i++)
        {
            samples[i] = coefficients[i];
        }
        // Set coefficients
        control_regs[OFFSET_FIR_RELOAD+FIFO_WR_ADDR] = phys_addr;
        control_regs[OFFSET_FIR_RELOAD+FIFO_WR_SIZE] = 11 * sizeof(long long int);
        read(fd,0,0);
        samples[11] = 0;
        control_regs[OFFSET_FIR_CONFIG+FIFO_WR_ADDR] = phys_addr + 11 * sizeof(long long int);
        control_regs[OFFSET_FIR_CONFIG+FIFO_WR_SIZE] = 1 * sizeof(long long int);
	// BUG: The read below causes the program to freeze. The freeze occurs due to the kernel driver 
	//      waiting for an interrupt that never comes (or possibly comes early) after calling 
	//      wait_event_interruptible() in the read() handler. This problem does not occur with the
	//      other read calls in this program, so perhaps it has to do with fact that this AXI
	//      transfer is only one 64-bit message long?
	// read(fd,0,0);

        // Send impulse
        samples[0] = 1 << 5;
        for (i = 1; i < 32; i++)
        {
            samples[i] = 0;
        }
        number_of_samples = 32 * sizeof(long long int);
        control_regs[OFFSET_S2H+FIFO_WR_ADDR] = phys_addr + number_of_samples;
        control_regs[OFFSET_S2H+FIFO_WR_SIZE] = number_of_samples;
        read(fd,0,0);
        control_regs[OFFSET_H2S+FIFO_WR_ADDR] = phys_addr;
        control_regs[OFFSET_H2S+FIFO_WR_SIZE] = number_of_samples;
        read(fd,0,0);

        // Read all status FIFOs
        control_regs[OFFSET_H2S+FIFO_WR_STS_RDY] = 0;
        control_regs[OFFSET_S2H+FIFO_WR_STS_RDY] = 0;
        control_regs[OFFSET_FIR_CONFIG+FIFO_WR_STS_RDY] = 0;
        control_regs[OFFSET_FIR_RELOAD+FIFO_WR_STS_RDY] = 0;

        // Verify impulse response
        for (i = 32; i < 42; i++)
        {
            if (samples[i] != coefficients[i-32])
            {
                num_errors++;
            }
        }
        for (i = 42; i < 53; i++)
        {
            if (samples[i] != coefficients[42-i+10])
            {
                num_errors++;
            }
        }
        if (num_errors > 0)
        {
            for (i = 32; i < 42; i++)
            {
                printf("samples[%2d]: %3lld\t\tExpected: %lld\n",i,samples[i],coefficients[i-32]);
            }
            for (i = 42; i < 53; i++)
            {
                printf("samples[%2d]: %3lld\t\tExpected: %lld\n",i,samples[i],coefficients[42-i+10]);
            }
            printf("Status FIFO Word Count: \t%d\n",control_regs[FIFO_RD_STATUS_CNT]);
            printf("Address FIFO Word Count: \t%d\n",control_regs[FIFO_RD_ADDR_CNT]);
            printf("Size FIFO Word Count: \t\t%d\n",control_regs[FIFO_RD_SIZE_CNT]);
            break;
        }
        num_iterations++;
    }
    printf("Number of successful iterations: %d\n",num_iterations);

    close_driver(fd,buffer_length,control_length,control_regs,buff);

    return(0);
}

int open_driver(
    int *fd,
    unsigned long *buffer_length,
    unsigned long *control_length,
    unsigned long *phys_addr,
    unsigned int **control_regs,
    unsigned int **buff)
{
    // open the file descriptor to our kernel module
    const char* dev = "/dev/user_peripheral";

    // Open with read / write access and block until write has completed
    *fd = open(dev, O_RDWR|O_SYNC);
    if (*fd == 0)
    {
        printf("Failed to open %s\n",dev);
        perror("");
        return(-1);
    }

    // Get user peripheral parameters
    if (get_params_from_sysfs(buffer_length, control_length, phys_addr) != 0)
    {
        close(*fd);
        return(-1);
    }

    // mmap the control and data regions into virtual space
    *control_regs = (unsigned int*)mmap(NULL, *control_length, PROT_READ|PROT_WRITE, MAP_SHARED, *fd, 0x1000);
    if (control_regs == MAP_FAILED)
    {
        perror("Error mapping control_regs");
        close(*fd);
        return(-1);
    }

    *buff = (unsigned int*)mmap(NULL, *buffer_length, PROT_READ|PROT_WRITE, MAP_SHARED, *fd, 0x2000);
    if (buff == MAP_FAILED)
    {
        perror("Error mapping buff");
        close(*fd);
        return(-1);
    }

    // zero out the data region
    memset((void *)*buff, 0, (unsigned int)(*buffer_length));
    return(0);
}

int close_driver(
    int fd,
    unsigned long buffer_length,
    unsigned long control_length,
    unsigned int *control_regs,
    unsigned int *buff)
{
    munmap(control_regs,control_length);
    munmap(buff,buffer_length);
    close(fd);
    return(0);
}

long read_fpga_status()
{
    struct udev *udev;
    struct udev_enumerate *enumerate;
    struct udev_list_entry *device;
    struct udev_device *dev;
    const char *path;
    long prog_done = 0;

    udev = udev_new();
    if (!udev) {
        printf("ERROR: Failed udev_new()\n");
        return -1;
    }

    // Enumerate devcfg
    enumerate = udev_enumerate_new(udev);
    udev_enumerate_add_match_sysname(enumerate, "f8007000.devcfg");
    udev_enumerate_scan_devices(enumerate);
    device = udev_enumerate_get_list_entry(enumerate);

    // List should have only one entry
    if (udev_list_entry_get_next(device) != NULL)
    {
        printf("ERROR: Found more than one devcfg device!\n");
        return(-1);
    }

    // Create udev device
    path = udev_list_entry_get_name(device);
    dev = udev_device_new_from_syspath(udev, path);

    prog_done = atol(udev_device_get_sysattr_value(dev, "prog_done"));

    printf("%s/prog_done = %ld\n", udev_device_get_syspath(dev),prog_done);

    udev_enumerate_unref(enumerate);
    udev_unref(udev);

    return(prog_done);
}

static int get_params_from_sysfs(
    unsigned long *buffer_length,
    unsigned long *control_length,
    unsigned long *phys_addr)
{
    struct udev *udev;
    struct udev_enumerate *enumerate;
    struct udev_list_entry *devices, *dev_list_entry;
    struct udev_device *dev;

    udev = udev_new();
    if (!udev) {
        printf("Fail\n");
        return(-1);
    }


    enumerate = udev_enumerate_new(udev);
    udev_enumerate_add_match_sysname(enumerate, "40000000.user_peripheral");
    udev_enumerate_scan_devices(enumerate);
    devices = udev_enumerate_get_list_entry(enumerate);

    udev_list_entry_foreach(dev_list_entry, devices) {
        const char *path;

        path = udev_list_entry_get_name(dev_list_entry);
        dev = udev_device_new_from_syspath(udev, path);

        printf("Sys Path: %s\n", udev_device_get_syspath(dev));

        *buffer_length = atol(udev_device_get_sysattr_value(dev, "xx_buf_len"));
        *control_length = atol(udev_device_get_sysattr_value(dev, "regs_len"));
        *phys_addr = atol(udev_device_get_sysattr_value(dev, "xx_phys_addr"));

        printf("buffer_length = %lX\n", *buffer_length);
        printf("control_length = %lX\n", *control_length);
        printf("phy_addr = %lX\n", *phys_addr);
    }

    udev_enumerate_unref(enumerate);
    udev_unref(udev);

    return(0);
}

void ctrl_c(int dummy)
{
    kill_prog = 1;
    return;
}
