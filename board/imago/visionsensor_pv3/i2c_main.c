/**************************************************************
*
* Lattice Semiconductor Corp. Copyright 2011
* 
*
***************************************************************/


/**************************************************************
* 
* Revision History of i2c_main.c
* 
* 
* Support version 1.0
***************************************************************/
#ifdef __UBOOT__
#include <common.h>
#include <fs.h>
#include "vspv3_board.h"
#endif
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "opcode.h"

/***************************************************************
*
* Supported I2C versions.
*
***************************************************************/

const char * const g_szSupportedVersions[] = { "_I2C1.0", 0 };

/*************************************************************
*                                                            *
* EXTERNAL FUNCTIONS                                         *
*                                                            *
*************************************************************/
extern unsigned char GetByte( int a_iCurrentIndex, char a_cAlgo );
extern short ispProcessI2C(void);
extern int EnableHardware(void);
extern void DisableHardware(void);

/*************************************************************
*                                                            *
* GLOBAL VARIABLES                                           *
*                                                            *
*************************************************************/

unsigned char * g_pucAlgoArray = 0;	/*** array to hold the algorithm ***/
unsigned char * g_pucDataArray = 0;	/*** array to hold the data ***/
int g_iAlgoSize = 0;					/*** variable to hold the size of the algorithm array ***/
int g_iDataSize = 0;					/*** variable to hold the size of the data array ***/

/*************************************************************
*                                                            *
* EXTERNAL VARIABLES                                         *
*                                                            *
*************************************************************/

extern unsigned short g_usDataType;
extern int g_iMovingAlgoIndex;
extern int g_iMovingDataIndex;

/*************************************************************
*                                                            *
* ISPI2CNTRYPOINT                                            *
*                                                            *
* INPUT:                                                     *
*     a_pszAlgoFile: this is the name of the algorithm file. *
*                                                            *
*     a_pszDataFile: this is the name of the data file.      *
*     Note that this argument may be empty if the algorithm  *
*     does not require a data file.                          *
*                                                            *
* RETURN:                                                    *
*     The return value will be a negative number if an error *
*     occurred, or 0 if everything was successful            *
*                                                            *
* DESCRIPTION:                                               *
*     This function opens the file pointers to the algo and  *
*     data file.  It intializes global variables to their    *
*     default values and enters the processor.               *
*                                                            *
*************************************************************/

static int fpga_config(void *algo_addr, unsigned int algo_size, void *data_addr, unsigned int data_size)
{
	char szFileVersion[ 9 ] = { 0 };
	short int siRetCode     = 0;
	int iIndex              = 0;
	int cVersionIndex       = 0;
//	FILE * pFile            = NULL;
	
	/*************************************************************
	*                                                            *
	* VARIABLES INITIALIZATION                                   *
	*                                                            *
	*************************************************************/

	g_pucAlgoArray     = (unsigned char *)algo_addr;
	g_pucDataArray     = (unsigned char *)data_addr;
	g_iAlgoSize        = algo_size;
	g_iDataSize        = data_size;
	g_usDataType       = 0;
	g_iMovingAlgoIndex = 0;
	g_iMovingDataIndex = 0;

	if ( GetByte( g_iMovingDataIndex++, 0 ) ) {
		g_usDataType |= COMPRESS;
	}

	/***************************************************************
	*
	* Read and store the version of the VME file.
	*
	***************************************************************/

	for ( iIndex = 0; iIndex < strlen(g_szSupportedVersions[0]); iIndex++ ) {
		szFileVersion[ iIndex ] = GetByte( g_iMovingAlgoIndex++, 1 );
	}

	/***************************************************************
	*
	* Compare the VME file version against the supported version.
	*
	***************************************************************/

	for ( cVersionIndex = 0; g_szSupportedVersions[ cVersionIndex ] != 0; cVersionIndex++ ) {
		for ( iIndex = 0; iIndex < strlen(g_szSupportedVersions[cVersionIndex]); iIndex++ ) {
			if ( szFileVersion[ iIndex ] != g_szSupportedVersions[ cVersionIndex ][ iIndex ] ) {
				siRetCode = ERR_WRONG_VERSION;
				break;
			}	
			siRetCode = 0;
		}

		if ( siRetCode == 0 ) {

			/***************************************************************
			*
			* Found matching version, break.
			*
			***************************************************************/

			break;
		}
	}

	if ( siRetCode < 0 ) {

		/***************************************************************
		*
		* VME file version failed to match the supported versions.
		*
		***************************************************************/

		return ERR_WRONG_VERSION;
	}
                   
	/*************************************************************
	*                                                            *
	* Start the hardware.                                        *
	*                                                            *
	*************************************************************/

	siRetCode = EnableHardware();
	if (siRetCode < 0)
		return siRetCode;

	/*************************************************************
	*                                                            *
	* Begin processing algorithm and data file.                  *
	*                                                            *
	*************************************************************/

	siRetCode = ispProcessI2C();

	/*************************************************************
	*                                                            *
	* Stop the hardware.                                         *
	*                                                            *
	*************************************************************/

    DisableHardware();

    return ( siRetCode );
}

/*************************************************************
*                                                            *
* ERROR_HANDLER                                              *
*                                                            *
* INPUT:                                                     *
*     a_siRetCode: this is the error code reported by the    *
*     processor.                                             *
*                                                            *
*     a_pszMessage: this will store the return message.      *
*                                                            *
* RETURN:                                                    *
*     None.                                                  *
*                                                            *
* DESCRIPTION:                                               *
*     This function assigns an error message based on the    *
*     error reported by the processor.  The program should   *
*     display the error message prior to exiting.            *
*                                                            *
*************************************************************/

void error_handler( short int a_siRetCode, char * a_pszMessage )
{
	switch( a_siRetCode ) {
	case ERR_VERIFY_FAIL:
		strcpy( a_pszMessage, "VERIFY FAIL" );
		break;
	case ERR_FIND_ALGO_FILE:
		strcpy( a_pszMessage, "CANNOT FIND ALGO FILE" );
		break;
	case ERR_FIND_DATA_FILE:
		strcpy( a_pszMessage, "CANNOT FIND DATA FILE" );
		break;
	case ERR_WRONG_VERSION:
		strcpy( a_pszMessage, "WRONG FILE TYPE/VERSION" );
		break;
	case ERR_ALGO_FILE_ERROR:
		strcpy( a_pszMessage, "ALGO FILE ERROR" );
		break;
	case ERR_DATA_FILE_ERROR:
		strcpy( a_pszMessage, "DATA FILE ERROR" );
		break;
	case ERR_OUT_OF_MEMORY:
		strcpy( a_pszMessage, "OUT OF MEMORY" );
		break;
	default:
		strcpy( a_pszMessage, "UNKNOWN ERROR" );
		break;
	}
} 

#ifdef __UBOOT__
static int do_fpga_config(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	int result;
	void *algo_addr, *data_addr;
	unsigned int algo_size, data_size;
	
	if (argc < 4) {
		puts("missing arguments!\n");
		return CMD_RET_FAILURE;
	}

	algo_addr = (void *)simple_strtoul(argv[1], NULL, 16);
	algo_size = simple_strtoul(argv[2], NULL, 16);
	data_addr = (void *)simple_strtoul(argv[3], NULL, 16);
	data_size = simple_strtoul(argv[4], NULL, 16);

	printf("algo_size: %u, data_size:%u\n", algo_size, data_size);

	result = fpga_config(algo_addr, algo_size, data_addr, data_size);
	if (result != CMD_RET_SUCCESS)
		return result;

	return CMD_RET_FAILURE;
}

U_BOOT_CMD(
	fpga_init,	5,	0,	do_fpga_config,
	"Configure FPGA device",
	"<algo_address> <algo_length> <data_address> <data_length>\n"
);
#endif

int fpga_init(const char *algo_file, const char *data_file)
{
	int result;
#ifndef __UBOOT__
	FILE *pFile;
#endif
	void *buf_algo;
	void *buf_data;
	long long algo_size = 4096;
	long long data_size = 256 << 10;

	buf_algo = calloc(algo_size, 1);
	if (buf_algo == NULL) {
		printf("%s(): out of memory\n",  __func__);
		return -1;
	}
	buf_data = calloc(data_size, 1);
	if (buf_data == NULL) {
		printf("%s(): out of memory\n",  __func__);
		return -1;
	}

#ifdef __UBOOT__
	if (fs_set_blk_dev("mmc", "0:1", FS_TYPE_FAT)) {
		printf("%s(): fs_set_blk_dev() failed\n",  __func__);
		return -1;
	}
	if (fs_read(algo_file, (ulong)buf_algo, 0, algo_size, &algo_size) < 0) {
		printf("%s(): error reading file '%s'\n",  __func__, algo_file);
		return -1;
	}
	if (fs_set_blk_dev("mmc", "0:1", FS_TYPE_FAT)) {
		printf("%s(): fs_set_blk_dev() failed\n",  __func__);
		return -1;
	}
	if (fs_read(data_file, (ulong)buf_data, 0, data_size, &data_size) < 0) {
		printf("%s(): error reading file '%s'\n",  __func__, data_file);
		return -1;
	}
#else
	pFile = fopen(algo_file, "rb");
	if (pFile == NULL) {
		printf("%s(): unable to open file '%s'\n",  __func__, algo_file);
		return -1;
	}
	algo_size = fread(buf_algo, 1, algo_size, pFile);
	if (algo_size <= 0) {
		printf("%s(): error reading file '%s'\n",  __func__, algo_file);
		return -1;
	}
	fclose(pFile);

	pFile = fopen(data_file, "rb");
	if (pFile == NULL) {
		printf("%s(): unable to open file '%s'\n",  __func__, data_file);
		return -1;
	}
	data_size = fread(buf_data, 1, data_size, pFile);
	if (data_size <= 0) {
		printf("%s(): error reading file '%s'\n",  __func__, data_file);
		return -1;
	}
	fclose(pFile);
#endif

	result = fpga_config(buf_algo, algo_size, buf_data, data_size);

	free(buf_algo);
	free(buf_data);

	return result;
}

