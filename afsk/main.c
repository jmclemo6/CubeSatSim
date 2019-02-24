/*
 *  Transmits CubeSat Telemetry at 440MHz in AO-7 format
 *
 *  Copyright Alan B. Johnston
 *
 *  Portions Copyright (C) 2018 Jonathan Brandenburg
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

// INA219 code from https://github.com/foglamp/foglamp-south-ina219/blob/develop/include/ina219.h
/*
 * FogLAMP south service plugin
 *
 * Copyright (c) 2018 Dianomic Systems
 *
 * Released under the Apache 2.0 Licence
 *
 * Author: Mark Riddoch
 */

/**
 * Values for the gain setting
 */
enum {
	INA219_CONFIG_GAIN_40MV  =       0x0000,  // 40mV Range
	INA219_CONFIG_GAIN_80MV  =       0x0800,  // 80mV Range
	INA219_CONFIG_GAIN_160MV =       0x1000,  // 160mV Range
	INA219_CONFIG_GAIN_320MV =       0x1800,  // 320mV Range
};

enum
{
	INA219_CONFIG_BVOLTAGERANGE_16V =        0x0000,  // 0-16V Range
	INA219_CONFIG_BVOLTAGERANGE_32V =        0x2000,  // 0-32V Range
};

#define INA219_CONFIG_BADCRES_MASK	0x0780 // ADC Resulotion Mask
enum
{
	INA219_CONFIG_BADCRES_9BIT  =    0x0000,  // 9-bit bus res = 0..511
	INA219_CONFIG_BADCRES_10BIT =    0x0080,  // 10-bit bus res = 0..1023
	INA219_CONFIG_BADCRES_11BIT =    0x0100,  // 11-bit bus res = 0..2047
	INA219_CONFIG_BADCRES_12BIT =    0x0180,  // 12-bit bus res = 0..4097
};

#define INA219_CONFIG_SADCRES_MASK	0x0078 // Mask for shunt ADC resolution bits
enum
{
	INA219_CONFIG_SADCRES_9BIT_1S_84US     = 0x00,  // 1 x 9-bit shunt sample
	INA219_CONFIG_SADCRES_10BIT_1S_148US   = 0x08,  // 1 x 10-bit shunt sample
	INA219_CONFIG_SADCRES_11BIT_1S_276US   = 0x10,  // 1 x 11-bit shunt sample
	INA219_CONFIG_SADCRES_12BIT_1S_532US   = 0x18,  // 1 x 12-bit shunt sample
	INA219_CONFIG_SADCRES_12BIT_2S_1060US  = 0x48,	// 2 x 12-bit shunt samples averaged together
	INA219_CONFIG_SADCRES_12BIT_4S_2130US  = 0x50,  // 4 x 12-bit shunt samples averaged together
	INA219_CONFIG_SADCRES_12BIT_8S_4260US  = 0x58,  // 8 x 12-bit shunt samples averaged together
	INA219_CONFIG_SADCRES_12BIT_16S_8510US = 0x60,  // 16 x 12-bit shunt samples averaged together
	INA219_CONFIG_SADCRES_12BIT_32S_17MS   = 0x68,  // 32 x 12-bit shunt samples averaged together
	INA219_CONFIG_SADCRES_12BIT_64S_34MS   = 0x70,  // 64 x 12-bit shunt samples averaged together
	INA219_CONFIG_SADCRES_12BIT_128S_69MS  = 0x78,  // 128 x 12-bit shunt samples averaged together
};

#define INA219_CONFIG_MODE_MASK		0x0007  // Operating Mode Mask
enum {   
	INA219_CONFIG_MODE_POWERDOWN            = 0x0000,
	INA219_CONFIG_MODE_SVOLT_TRIGGERED      = 0x0001,
	INA219_CONFIG_MODE_BVOLT_TRIGGERED      = 0x0002,
	INA219_CONFIG_MODE_SANDBVOLT_TRIGGERED  = 0x0003,
	INA219_CONFIG_MODE_ADCOFF               = 0x0004,
	INA219_CONFIG_MODE_SVOLT_CONTINUOUS     = 0x0005,
	INA219_CONFIG_MODE_BVOLT_CONTINUOUS     = 0x0006,
	INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS = 0x0007,	
};

typedef enum {
	CONF_16V_400MA,
	CONF_32V_1A,
	CONF_32V_2A
} INA219_CONFIGURATION;

/*
 * INA219 Registers
 */
#define INA219_REG_CONFIG 	               0x00
#define INA219_REG_SHUNTVOLTAGE                0x01
#define INA219_REG_BUSVOLTAGE                  0x02
#define INA219_REG_POWER                       0x03
#define INA219_REG_CURRENT                     0x04
#define INA219_REG_CALIBRATION                 0x05

	int	x_fd;		// I2C bus 0
	int	x_address;	// I2C address of INA219
	int	x_powerMultiplier;
	int	x_currentDivider;
	int	x_calValue;

	int	y_fd;		// I2C bus 0
	int	y_address;	// I2C address of INA219
//	int	y_powerMultiplier;
//	int	y_currentDivider;
//	int	y_calValue;

//#include <unistd.h>                             //Needed for I2C port
#include <fcntl.h>                              //Needed for I2C port
//#include <sys/ioctl.h>                  //Needed for I2C port

#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include "status.h"
#include "ax5043.h"
#include "ax25.h"
#include "spi/ax5043spi.h"
#include <wiringPiI2C.h>
#include <time.h>

// Put your callsign here
#define CALLSIGN "KU2Y"
#define VBATT 15
#define ADC5 17
#define ADC6 18
#define ADC7 19
#define ADC8 20
#define TIME 8
#define UCTEMP 30
#define UPTIME_SEC 8 
#define A 1
#define B 2
#define C 3
#define D 4

#define SENSOR_40 0
#define SENSOR_41 3
#define SENSOR_44 6
#define SENSOR_45 9
#define SENSOR_4A 12
#define VOLTAGE 0
#define CURRENT 1
#define POWER 2
#define VBATT 15

ax5043_conf_t hax5043;
ax25_conf_t hax25;

static void init_rf();
int get_tlm(int tlm[][5]);
long int timestamp;
void config_x25();
void trans_x25();
//long int timestamp;
int tempSensor; 

int upper_digit(int number);
int lower_digit(int number);
int charging = 0;

uint16_t config = (0x2000 | 0x1800 | 0x0180 | 0x0018 | 0x0007 );

int main(void) {
    
//    sleep(20);

    int tlm[7][5];
    int i, j;
    for (i = 1; i < 7; i++) {
        for (j = 1; j < 5; j++) {
		tlm[i][j] = 0;
	}
    }
    timestamp = time(NULL);
	
    int file_i2c;
    //char *filenam1e = (char*)"/dev/i2c-3";
    if ((file_i2c = open("/dev/i2c-3", O_RDWR)) < 0)
    {
            printf("ERROR: /dev/ic2-3 bus not present\n");
            tempSensor = -1;
    } else
    {
            tempSensor = wiringPiI2CSetupInterface("/dev/i2c-3", 0x48);
    }

    printf("tempSensor: %d \n",tempSensor);	

    int arduinoI2C = wiringPiI2CSetupInterface("/dev/i2c-1", 0x4c);
    printf("Arduio write: %d \n", wiringPiI2CWrite(arduinoI2C,42));
    printf("Arduio: %d \n", wiringPiI2CRead(arduinoI2C));

// INA219 test code
/* 16V */
	x_calValue = 8192;
	x_powerMultiplier = 1;
	x_currentDivider = 20;
	config = INA219_CONFIG_BVOLTAGERANGE_16V |
                 INA219_CONFIG_GAIN_40MV |
                 INA219_CONFIG_BADCRES_12BIT |
                 INA219_CONFIG_SADCRES_12BIT_4S_2130US |
               //INA219_CONFIG_SADCRES_12BIT_1S_532US |
                 INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS;
  
	x_fd  = wiringPiI2CSetupInterface("/dev/i2c-0", 0x40);
	printf("Opening of X- fd %d\n", x_fd);

	y_fd  = wiringPiI2CSetupInterface("/dev/i2c-0", 0x41);
	printf("Opening of Y- fd %d\n", y_fd);

	int x;
        for(x = 0; x < 4; x++) { 
	wiringPiI2CWriteReg16(x_fd, INA219_REG_CALIBRATION, x_calValue);
	wiringPiI2CWriteReg16(x_fd, INA219_REG_CONFIG, config);

//	double shuntVolts  = wiringPiI2CReadReg16(x_fd, INA219_REG_SHUNTVOLTAGE) * 0.01;
//	double busVolts  = wiringPiI2CReadReg16(x_fd, INA219_REG_BUSVOLTAGE) * 0.001;
//	double volts = busVolts + (shuntVolts / 1000);
	
	wiringPiI2CWriteReg16(x_fd, INA219_REG_CALIBRATION, x_calValue);
	double current  = wiringPiI2CReadReg16(x_fd, INA219_REG_CURRENT) / x_currentDivider;

	double power  = wiringPiI2CReadReg16(x_fd, INA219_REG_POWER) * x_powerMultiplier;
	
	wiringPiI2CWriteReg16(y_fd, INA219_REG_CALIBRATION, x_calValue);
	wiringPiI2CWriteReg16(y_fd, INA219_REG_CONFIG, config);
	
	wiringPiI2CWriteReg16(y_fd, INA219_REG_CALIBRATION, x_calValue);
	double y_current  = wiringPiI2CReadReg16(y_fd, INA219_REG_CURRENT) / x_currentDivider;

	double y_power  = wiringPiI2CReadReg16(y_fd, INA219_REG_POWER) * x_powerMultiplier;
	

	printf("0x40 current %4.2f power %4.2f 0x41 current %4.2f power %4.2f \n", current, power, y_current, y_power);
	sleep(0.25);
	}

exit(0);

//   uint16_t value;
   int value = 0;
   int result = 0;

   int I2C0 = wiringPiI2CSetupInterface("/dev/i2c-3", 0x40); 
//   printf("I2C0: %d \n", I2C0);
  
//  result = wiringPiI2CWriteReg16 (I2C0, 0x05, 8192) ;
//  printf("Result of WriteReg16: %d \n", result);
  
/*
  uint16_t config = INA219_CONFIG_BVOLTAGERANGE_16V |
                    INA219_CONFIG_GAIN_1_40MV |
                    INA219_CONFIG_BADCRES_12BIT |
                    INA219_CONFIG_SADCRES_12BIT_1S_532US |
                    INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS;
*/
  uint16_t config = 0x000 | 0x0000 | 0x0018 | 0x0007;

// wireWriteRegister(INA219_REG_CONFIG, config);
// result = wiringPiI2CWriteReg16 (I2C0,  0x00, config) ;
// printf("Result of config WriteReg16: %d \n", result);

sleep(1);

// read voltage

// wireReadRegister(INA219_REG_BUSVOLTAGE, &value)
  value = wiringPiI2CReadReg16 (I2C0, 0x02);
  printf("bus voltage value: %d \n", (int16_t)value);

sleep(1);

  value = wiringPiI2CReadReg16 (I2C0, 0x01);
  printf("shunt voltage value: %d \n", (int16_t)value);


sleep(1);

// read current

//  result = wiringPiI2CWriteReg16 (I2C0, 0x05, 8192) ;
//  printf("Result of WriteReg16: %d \n", result);
 
//sleep(1);
 
// Now we can safely read the CURRENT register!
//  value = wiringPiI2CReadReg16 (I2C0, 0x04);
//  printf("current value: %d \n", (int16_t)value);








    exit(0);

    setSpiChannel(SPI_CHANNEL);
    setSpiSpeed(SPI_SPEED);
    initializeSpi();

    int ret;
    uint8_t data[1024];

    init_rf();

//    ax25_init(&hax25, (uint8_t *) "CubeSatSim", '2', (uint8_t *) CALLSIGN, '2',
    ax25_init(&hax25, (uint8_t *) "CubeSatSim", '1', (uint8_t *) CALLSIGN, '1',
    		AX25_PREAMBLE_LEN,
   		 AX25_POSTAMBLE_LEN);
        
        
    /* Infinite loop */
    for (;;) {
        sleep(2);
    	
	// send X.25 packet

 //   	init_rf();

 //   	ax25_init(&hax25, (uint8_t *) "CQ", '2', (uint8_t *) "DX", '2',
 //   		AX25_PREAMBLE_LEN,
 //  		 AX25_POSTAMBLE_LEN);
        
	printf("INFO: Getting TLM Data\n");
	    
	get_tlm(tlm);

	printf("INFO: Preparing X.25 packet\n");
	
	char str[1000];
	char tlm_str[1000];
	
	char header_str[] = "\x03\x0fhi hi ";
	strcpy(str, header_str);
	
        int channel;
	for (channel = 1; channel < 7; channel++) {
//        printf("%d %d %d %d \n", tlm[channel][1], tlm[channel][2], tlm[channel][3], tlm[channel][4]); 
          sprintf(tlm_str, "%d%d%d %d%d%d %d%d%d %d%d%d ", 
	  channel, upper_digit(tlm[channel][1]), lower_digit(tlm[channel][1]),
 	  channel, upper_digit(tlm[channel][2]), lower_digit(tlm[channel][2]), 
	  channel, upper_digit(tlm[channel][3]), lower_digit(tlm[channel][3]), 
	  channel, upper_digit(tlm[channel][4]), lower_digit(tlm[channel][4]));
//	  printf("%s \n",tlm_str);
	  strcat(str, tlm_str);
	}	
        
	char cmdbuffer[1000];

        if (charging) {
      	   FILE* file1 = popen("/home/pi/mopower/mpcmd LED_STAT=1", "r"); 
      	   fgets(cmdbuffer, 1000, file1);
      	   pclose(file1);

//      	   printf("LED state: %s\n", cmdbuffer);
        }
	printf("INFO: Transmitting X.25 packet\n");

        memcpy(data, str, strnlen(str, 256));
        ret = ax25_tx_frame(&hax25, &hax5043, data, strnlen(str, 256));
        if (ret) {
            fprintf(stderr,
                    "ERROR: Failed to transmit AX.25 frame with error code %d\n",
                    ret);
            exit(EXIT_FAILURE);
        }
        ax5043_wait_for_transmit();
      	FILE* file2 = popen("/home/pi/mopower/mpcmd LED_STAT=0", "r"); 
      	fgets(cmdbuffer, 1000, file2);
      	pclose(file2);

//      	printf("LED state: %s\n", cmdbuffer);

        if (ret) {
            fprintf(stderr,
                    "ERROR: Failed to transmit entire AX.25 frame with error code %d\n",
                    ret);
            exit(EXIT_FAILURE);
	}
    }

    return 0;
}

static void init_rf() {
    int ret;
    ret = ax5043_init(&hax5043, XTAL_FREQ_HZ, VCO_INTERNAL);
    if (ret != PQWS_SUCCESS) {
        fprintf(stderr,
                "ERROR: Failed to initialize AX5043 with error code %d\n", ret);
        exit(EXIT_FAILURE);
    }
}

//  Returns lower digit of a number which must be less than 99
//
int lower_digit(int number) {

	int digit = 0;
	if (number < 100) 
		digit = number - ((int)(number/10) * 10);
	else
		printf("ERROR: Not a digit in lower_digit!\n");
	return digit;
}

// Returns upper digit of a number which must be less than 99
//
int upper_digit(int number) {

	int digit = 0;
	if (number < 100) 
		digit = (int)(number/10);
	else
		printf("ERROR: Not a digit in upper_digit!\n");
	return digit;
}
int get_tlm(int tlm[][5]) {
	
//  Reading I2C voltage and current sensors	
	
      char cmdbuffer[1000];
      FILE* file = popen("sudo python /home/pi/CubeSatSim/python/readcurrent.py 2>&1", "r"); 
      fgets(cmdbuffer, 1000, file);
      pclose(file);
      printf("I2C Sensor data: %s\n", cmdbuffer);

      char ina219[16][20];  // voltage, currents, and power from the INA219 current sensors x4a, x40, x41, x44, and x45.
      int i = 0;
      char * data2 = strtok (cmdbuffer," ");

      while (data2 != NULL) {
          strcpy(ina219[i], data2);
  //        printf ("ina219[%d]=%s\n",i,ina219[i]);
          data2 = strtok (NULL, " ");
          i++;
      }
	
  // Reading MoPower telemetry info
	
      file = popen("/home/pi/mopower/mpcmd show data", "r"); 
      fgets(cmdbuffer, 1000, file);
      pclose(file);
//      printf("MoPower data: %s\n", cmdbuffer);

    char mopower[64][14];
//  char str[] ="- This, a sample string.";
    char * pch;
//  printf ("Splitting string \"%s\" into tokens:\n",str);
//  pch = strtok (str," ");
    i = 0;
    pch = strtok (cmdbuffer," ,.-");
    while (pch != NULL)
    {
      strcpy(mopower[i], pch);
//      printf ("mopwer[%d]=%s\n",i,mopower[i]); // pch);
      pch = strtok (NULL, " ");
      i++;
    }
    printf("Battery voltage = %s\n", mopower[16]);	
    if (strtof(mopower[17],NULL) > -0.5) {
	charging = 1;
        printf("Charging on\n");
    }
    else {
 	charging = 0;
        printf("Charging off\n");

    }
	
//	printf("1B: ina219[%d]: %s val: %f \n", SENSOR_40 + CURRENT, ina219[SENSOR_40 + CURRENT], strtof(ina219[SENSOR_40 + CURRENT], NULL));

	tlm[1][A] = (int)(strtof(ina219[SENSOR_4A + CURRENT], NULL) / 15 + 0.5) % 100;  // Current of 5V supply to Pi
	tlm[1][B] = (int) (99.5 - strtof(ina219[SENSOR_40 + CURRENT], NULL)/10) % 100;  // +X current [4]
	tlm[1][D] = (int) (99.5 - strtof(ina219[SENSOR_41 + CURRENT], NULL)/10) % 100;  // +Y current [7]
	tlm[1][C] = (int) (99.5 - strtof(ina219[SENSOR_44 + CURRENT], NULL)/10) % 100;  // +Z current [10] (actually -X current, AO-7 didn't have a Z solar panel?)
	
	tlm[2][A] = 99;
	tlm[2][C] = (int)((time(NULL) - timestamp) / 15) % 100; 
	tlm[2][D] = (int)(50.5 + strtof(ina219[SENSOR_45 + CURRENT], NULL)/10.0) % 100;   // NiMH Battery current
	
	tlm[3][A] = abs((int)((strtof(ina219[SENSOR_45 + VOLTAGE], NULL) * 10) - 65.5) % 100);
	tlm[3][B] = (int)(strtof(ina219[SENSOR_4A + VOLTAGE], NULL) * 10.0) % 100;      // 5V supply to Pi
		   	
        if (tempSensor != -1) {
		int tempValue = wiringPiI2CReadReg16(tempSensor, 0); 
//		printf("Read: %x\n", tempValue);
	        uint8_t upper = (uint8_t) (tempValue >> 8);
        	uint8_t lower = (uint8_t) (tempValue & 0xff);
        	float temp = (float)lower + ((float)upper / 0x100);

		tlm[4][A] = (int)((95.8 - temp)/1.48 + 0.5) % 100;
	}	
	tlm[6][B] = 0 ;
	tlm[6][D] = 49 + rand() % 3; 

// Display tlm
    int k, j;
    for (k = 1; k < 7; k++) {
        for (j = 1; j < 5; j++) {
	printf(" %2d ",	tlm[k][j]);
	}
    printf("\n");
    }	
       return 0;
}
