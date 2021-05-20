/**************************************************************
*
* Lattice Semiconductor Corp. Copyright 2011
* 
*
***************************************************************/


/**************************************************************
* 
* Revision History of hardware.c
* 
* 
* Support v1.0
***************************************************************/
#ifdef __UBOOT__
#include <common.h>
#include <i2c.h>
#include <dm/uclass.h>
#else
#include <fcntl.h>				// open()
#include <sys/ioctl.h>			// ioctl()
#include <unistd.h>				// close()
#include <stdio.h>
#include <string.h>
#include <linux/i2c-dev.h>		// I2C_SLAVE, ...
#include <linux/i2c.h>
#endif
#include <stdlib.h>
#include "opcode.h"

/*************************************************************
*                                                            *
* Opcode for discrete pins toggling							 *
*                                                            *
*************************************************************/
//#define signalSCL		  0x01    //I2C Bus
//#define signalSDA		  0x08    
//#define signalSCL_IN    0x0003    
//#define signalSDA_IN	  0x0004    
//#define signalTRST      0x80    
/********************************************************************************
* Declaration of global variables 
*
*********************************************************************************/
//unsigned short g_usCpu_Frequency  = 1000;   /*Enter your CPU frequency here, unit in MHz.*/

#define TX_BUFFER_SIZE (200 << 10)

/***************************************************************
*
* Functions declared in hardware.c module.
*
***************************************************************/
int ReadBytesAndSendNACK( int length, unsigned char *a_ByteRead, int NAck );
int SendBytesAndCheckACK(int length, unsigned char *a_bByteSend);
int ToggleTRST(int toggle);
int EnableHardware(void);
void DisableHardware(void);

#ifdef __UBOOT__
static struct udevice *i2c_dev = NULL;
#else

/*struct i2c_msg {
	uint addr;
//	uint flags;
	uint len;
	unsigned char *buf;
};*/

int i2c_fd = -1;

#define udelay usleep

#endif
static struct i2c_msg i2c_msg[2];
static unsigned char i2c_check_chip_address = 1;	// we verify the chip address after I2c (re-)start
//static unsigned int i2c_transmit_bytes = 0;
static unsigned char *i2c_transmit_buf = NULL;


/*************************************************************
*                                                            *
* ToggleTRST                                                 *
*                                                            *
* INPUT:                                                     *
*     CRESET signal value.                                   *
*                                                            *
* RETURN:                                                    *
*     int                                                   *
*                                                            *
* DESCRIPTION:                                               *
*     This function is used to set the value of              *
*     the CRESET signal on the I2C Bus					     *
*                                                            *
*     NOTE: This function should be modified in an embedded  *
*     system!                                                *
*                                                            *
*************************************************************/
int ToggleTRST(int toggle)
{
	const unsigned char chip_addr = 0x21;

//	printf("%s() => %u\n",  __func__, toggle);

	// TCA6408
#ifdef __UBOOT__
	struct udevice *dev;
	unsigned char buf = 0;
	if (i2c_get_chip_for_busnum(0, chip_addr, 1, &dev)) {
		printf("%s: Cannot find chip 0x%x for a bus 0\n", __func__, chip_addr);
		return -1;
	}

	// Output port register: GPIO4 = TRST
	buf = toggle ? 0x10 : 0x00;
	if (dm_i2c_write(dev, 0x01, &buf, 1)) {
		printf("%s: error setting output register\n", __func__);
		return -1;
	}

	// Configuration register: GPIO4 is output (TRST)
	buf = ~0x10;
	if (dm_i2c_write(dev, 0x03, &buf, 1)) {
		printf("%s: error setting output register\n", __func__);
		return -1;
	}
#else
	unsigned char buf[2];
	int tca_fd = open("/dev/i2c-0", O_RDWR);
	if (tca_fd < 0) {
		printf("%s: Cannot open device!\n", __func__);
		return -1;
	}

	// set i2c device addr
	if (ioctl(tca_fd, I2C_SLAVE, chip_addr) < 0) {
		printf("%s: Failed to set I2C slave address!\n", __func__);
		return -1;
	}

	// Output port register: GPIO4 = TRST
	buf[0] = 0x01;
	buf[1] = toggle ? 0x10 : 0x00;
	if (write(tca_fd, buf, 2) < 0) {
		printf("%s: Could not execute transfer: %d\n", __func__);
		return -1;
	}

	// Configuration register: GPIO4 is output (TRST)
	buf[0] = 0x03;
	buf[1] = ~0x10;
	if (write(tca_fd, buf, 2) < 0) {
		printf("%s: Could not execute transfer: %d\n", __func__);
		return -1;
	}
	
	close(tca_fd);
#endif

	//signalTRST =  toggle;
	//SetI2CDelay(1);
	return 0;
}
/*************************************************************
*                                                            *
* SetI2CStartCondition                                       *
*                                                            *
* INPUT:                                                     *
*     None.                                                  *
*                                                            *
* RETURN:                                                    *
*     int                                                   *
*                                                            *
* DESCRIPTION:                                               *
*     This function is used to issue a start sequence on the *
*     I2C Bus								                 *
*                                                            *
*     NOTE: This function should be modified in an embedded  *
*     system!                                                *
*                                                            *
*************************************************************/
int SetI2CStartCondition(void)
{
//	printf("%s()\n",  __func__);
	
//	i2c_transmit_bytes = 0;
	i2c_check_chip_address = 1;
	i2c_msg[0].addr = 0x40;
	i2c_msg[0].len = 0;
	i2c_msg[0].buf = i2c_transmit_buf;
	i2c_msg[0].flags = 0;
	i2c_msg[1].addr = 0x40;
	i2c_msg[1].len = 0;
	i2c_msg[1].flags = I2C_M_RD;
	//SCL SDA high
	//signalSDA = 1;
	//SetI2CDelay(1);
	//signalSCL = 1;
	//SetI2CDelay(1);
	//SCL high SDA low
	//signalSDA = 0;
	//SetI2CDelay(1);
	//SCL low SDA low
	//signalSCL = 0;
	//SetI2CDelay(1);
	
	return 0;
}
/*************************************************************
*                                                            *
* SetI2CReStartCondition                                     *
*                                                            *
* INPUT:                                                     *
*     None.                                                  *
*                                                            *
* RETURN:                                                    *
*     None                                                   *
*                                                            *
* DESCRIPTION:                                               *
*     This function is used to issue a start sequence on the *
*     I2C Bus								                 *
*                                                            *
*     NOTE: This function should be modified in an embedded  *
*     system!                                                *
*                                                            *
*************************************************************/
int SetI2CReStartCondition(void)
{
//	printf("%s()\n",  __func__);
	i2c_check_chip_address = 1;
	//SCL SDA high
	//signalSDA = 1;
	//SetI2CDelay(1);
	//signalSCL = 1;
	//SetI2CDelay(1);
	//SCL high SDA low
	//signalSDA = 0;
	//SetI2CDelay(1);
	//SCL low SDA low
	//signalSCL = 0;
	//SetI2CDelay(1);
	return 0;
	
}
/*************************************************************
*                                                            *
* SetI2CStopCondition                                        *
*                                                            *
* INPUT:                                                     *
*     None.                                                  *
*                                                            *
* RETURN:                                                    *
*     None                                                   *
*                                                            *
* DESCRIPTION:                                               *
*     This function is used to issue a stop sequence on the  *
*     I2C Bus								                 *
*                                                            *
*     NOTE: This function should be modified in an embedded  *
*     system!                                                *
*                                                            *
*************************************************************/
int SetI2CStopCondition(void)
{
//	printf("%s()\n",  __func__);

	if (i2c_msg[0].len != 0)
	{
		int result;
//		printf("%s() -> dm_i2c_xfer(): len[0]=%u\n",  __func__, i2c_msg[0].len);
#ifdef __UBOOT__
		result = dm_i2c_xfer(i2c_dev, &i2c_msg[0], 1);
#else
		result = write(i2c_fd, i2c_msg[0].buf, i2c_msg[0].len);
#endif

		i2c_msg[0].len = 0;
		if (result < 0) {
			printf("%s: Could not execute transfer: %d\n", __func__, result);
			return result;
		}
	}
	//SCL high SDA low
	//signalSDA = 0;
	//SetI2CDelay(1);
	//signalSCL = 1;
	//SetI2CDelay(1);
	//SCL high SDA high
	//signalSDA = 1;
	//SetI2CDelay(1);
	return 0;
}

/*************************************************************
*                                                            *
* READBYTESANDSENDNACK                                         *
*                                                            *
* INPUT:                                                     *
*     None.                                                  *
*                                                            *
* RETURN:                                                    *
*     Returns the bit read back from the device.             *
*                                                            *
* DESCRIPTION:                                               *
*     This function is used to read the TDO pin from the     *
*     input port.                                            *
*                                                            *
*     NOTE: This function should be modified in an embedded  *
*     system!                                                *
*                                                            *
*************************************************************/
int ReadBytesAndSendNACK(int length, unsigned char *a_ByteRead, int NAck)
{
	int result;

//	printf("%s(): length=%i, NAck=%i\n", __func__, length, NAck);
	
	if (i2c_check_chip_address) {
		printf("%s(): missing chip address\n",  __func__);
		return -1;
	}

	i2c_msg[1].buf = a_ByteRead;
	i2c_msg[1].len = (length + 7 ) / 8;

//	printf("%s() -> dm_i2c_xfer(): len[0]=%u, len[1]=%u\n",  __func__, i2c_msg[0].len, i2c_msg[1].len);

#ifdef __UBOOT__
	result = dm_i2c_xfer(i2c_dev, &i2c_msg[0], 2);
	i2c_msg[0].len = 0;
#else
	struct i2c_rdwr_ioctl_data msgset[1];
	msgset[0].msgs = i2c_msg;
    msgset[0].nmsgs = 2;
	result = ioctl(i2c_fd, I2C_RDWR, &msgset);
	i2c_msg[0].len = 0;
#endif
	if (result < 0) {
		printf("%s: Could not execute transfer: %d\n", __func__, result);
		return result;
	}
	
	return 0;
}
/*************************************************************
*                                                            *
* SENDBYTESANDCHECKACK                                        *
*                                                            *
* INPUT:                                                     *
*                                                            *
*     a_bByteSend: the value to determine of the pin above   *
*     will be written out or not.                            *
*                                                            *
* RETURN:                                                    *
*     true or false.                                         *
*                                                            *
* DESCRIPTION:                                               *
*     To apply the specified value to the pins indicated.    *
*     This routine will likely be modified for specific      *
*                                                            *
*     NOTE: This function should be modified in an embedded  *
*     system!                                                *
*                                                            *
*************************************************************/
int SendBytesAndCheckACK(int length, unsigned char *a_bByteSend)
{
//	printf("%s(): length=%i\n",  __func__, length);

	if (i2c_check_chip_address) {
		if (*a_bByteSend != 0x80 && *a_bByteSend != 0x81) {
			printf("%s(): wrong chip address\n",  __func__);
			return -1;
		}
		i2c_check_chip_address = 0;
		return 0;
	}

	if ((i2c_msg[0].len + (length + 7) / 8) >= TX_BUFFER_SIZE)
		return ERR_OUT_OF_MEMORY;

	memcpy(&i2c_transmit_buf[i2c_msg[0].len], a_bByteSend, (length + 7) / 8);
	
	i2c_msg[0].len += (length + 7) / 8;
//	i2c_transmit_bytes += length;
	/*
	int i,j;
	BYTE ByteWrite = 0;
	for (i = 0; i< (length+7)/8; i++)
	{
	    ByteWrite = a_bByteSend[i];
		for (j = 0; j < 8; j++)
		{
			if( ByteWrite & 0x80 )
				signalSDA = 1;
			else 
				signalSDA = 0;
			signalSCL = 1;
			signalSCL = 0;
			ByteWrite <<= 1;			
		}
		signalSDA = 1;
		signalSCL = 1;
		SetI2CDelay(1);
		if( signalSDA_IN == 1)
		{
			signalSCL = 0;
			//fail to get ACK
			return -1;		
		}
		signalSCL = 0;
	}
	*/
	return 0;
}

/*************************************************************
*                                                            *
* SetI2CDelay                                                *
*                                                            *
* INPUT:                                                     *
*     a_uiDelay: number of waiting time.                     *
*                                                            *
* RETURN:                                                    *
*     None.                                                  *
*                                                            *
* DESCRIPTION:                                               *
*     Users must devise their own timing procedures to       *
*     ensure the specified minimum delay is observed when    *
*     using different platform.  The timing function used    *
*     here is for PC only by hocking the clock chip.         *
*                                                            *
*     NOTE: This function should be modified in an embedded  *
*     system!                                                *
*                                                            *
*************************************************************/
void SetI2CDelay( unsigned int a_msTimeDelay )
{	
//	printf("%s()\n",  __func__);
	udelay(1000 * a_msTimeDelay);
}
/*************************************************************
*                                                            *
* ENABLEHARDWARE                                             *
*                                                            *
* INPUT:                                                     *
*     None.                                                  *
*                                                            *
* RETURN:                                                    *
*     None.                                                  *
*                                                            *
* DESCRIPTION:                                               *
*     This function is called to enable the hardware.        *
*                                                            *
*     NOTE: This function should be modified in an embedded  *
*     system!                                                *
*                                                            *
*************************************************************/

int EnableHardware(void)
{
	struct udevice *bus;
	int result;

//	printf("%s()\n",  __func__);

	// CRESET = 0
	ToggleTRST(0);

#ifdef __UBOOT__
	result = uclass_get_device_by_seq(UCLASS_I2C, 0, &bus);
	if (result) {
		printf("%s: Can't find I2C bus 3\n", __func__);
		return -1;
	}

	result = dm_i2c_probe(bus, 0x40, 0, &i2c_dev);
	if (result) {
		printf("%s: FPGA not detected on I2C: %d\n",
		       __func__, result);
		return -1;
	}
#else
	i2c_fd = open("/dev/i2c-0", O_RDWR);
	if (i2c_fd < 0) {
		printf("%s: Cannot open device!\n", __func__);
		return -1;
	}

	// set i2c device addr
	if (ioctl(i2c_fd, I2C_SLAVE, 0x40) < 0) {
		printf("%s: Failed to set I2C slave address!\n", __func__);
		return -1;
	}
#endif

	i2c_transmit_buf = calloc(1, TX_BUFFER_SIZE);
	if (i2c_transmit_buf == NULL) {
		printf("%s: calloc() failed\n", __func__);
		return -1;
	}

//	printf("%s() done\n", __func__);

	return 0;
}

/*************************************************************
*                                                            *
* DISABLEHARDWARE                                            *
*                                                            *
* INPUT:                                                     *
*     None.                                                  *
*                                                            *
* RETURN:                                                    *
*     None.                                                  *
*                                                            *
* DESCRIPTION:                                               *
*     This function is called to disable the hardware.       *
*                                                            *
*     NOTE: This function should be modified in an embedded  *
*     system!                                                *
*                                                            *
*************************************************************/

void DisableHardware(void)
{
//	printf("%s()\n",  __func__);
	free(i2c_transmit_buf);
#ifndef __UBOOT__
	close(i2c_fd);
#endif
//	SetI2CStopCondition();
}

