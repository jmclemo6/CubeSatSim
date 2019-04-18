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

#include <unistd.h>                             //Needed for I2C port
#include <fcntl.h>                              //Needed for I2C port
#include <sys/ioctl.h>                  //Needed for I2C port

#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include "status.h"
#include "ax5043.h"
#include "ax25.h"
#include "spi/ax5043spi.h"
#include <wiringPiI2C.h>
#include <wiringPi.h>
#include <time.h>
#include "ina219.h"

// Put your callsign here
#define CALLSIGN "KN4PMR"
#define VBATT 15
#define ADC5 17
#define ADC6 18
#define ADC7 19
#define ADC8 20
#define TIME 8
#define UCTEMP 30
#define UPTIME_SEC 8 
#define A 0
#define B 1
#define C 2
#define D 3
#define E 4

#define SENSOR_40 0
#define SENSOR_41 3
#define SENSOR_44 6
#define SENSOR_45 9
#define SENSOR_4A 12
#define VOLTAGE 0
#define CURRENT 1
#define POWER 2
#define VBATT 15

uint32_t tx_freq_hz = 440310000;
uint32_t tx_channel = 0;

ax5043_conf_t hax5043;
ax25_conf_t hax25;

static void init_rf();
int get_tlm(int tlm[][5]);
long int timestamp;
void config_x25();
void trans_x25();
int tempSensor; 

int upper_digit(int number);
int lower_digit(int number);
int charging = 0;

const uint16_t config = INA219_CONFIG_BVOLTAGERANGE_16V |
                  INA219_CONFIG_GAIN_40MV |
                  INA219_CONFIG_BADCRES_12BIT |
                  INA219_CONFIG_SADCRES_12BIT_4S_2130US |
                  INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS;

int plus_x_fd;  // i2c-1 @ 0x40
int plus_y_fd;  // i2c-1 @ 0x41
int plus_z_fd;  // i2c-1 @ 0x44
int battery_fd; // i2c-1 @ 0x45
int mopower_fd; // i2c-1 @ 0x4a
int minus_x_fd; // i2c-0 @ 0x40
int minus_y_fd; // i2c-0 @ 0x41

const int calibrationValue = 8192;
const int powerMultiplier = 1;
const int currentDivider = 20;

struct SensorData {
  double current;
  double power;
};

int main(void) {
  wiringPiSetup();
  pinMode(0, OUTPUT);

  for (int blink = 1; blink < 4 ;blink++) {
    digitalWrite (0, HIGH) ; delay (500) ;
    digitalWrite (0,  LOW) ; delay (500) ;
  }
  digitalWrite (0, HIGH) ; 
  
  setSpiChannel(SPI_CHANNEL);
  setSpiSpeed(SPI_SPEED);
  initializeSpi();

  int tlm[7][5];
  memset(tlm, 0, sizeof tlm);

  int file_i2c = access("/dev/i2c-3", W_OK | R_OK);
  if (file_i2c < 0)
  {
    fprintf(stderr,"ERROR: /dev/ic2-3 bus not present\n");
    tempSensor = -1;
  } else
  {
    tempSensor = wiringPiI2CSetupInterface("/dev/i2c-3", 0x48);
  }

  #ifdef DEBUG_LOGGING
      fprintf(stderr,"tempSensor: %d \n",tempSensor);	
  #endif

  // new INA219 current reading code
  const char* i2c_1 = "/dev/i2c-1";
  file_i2c = access(i2c_1, W_OK | R_OK);
  if (file_i2c < 0)
  {
    #ifdef DEBUG_LOGGING
      fprintf(stderr,"ERROR: /dev/ic2-1 bus not present\n");
    #endif

    plus_x_fd = -1;  // Disable reading -X, -Y, and -Z telemetry
    plus_y_fd = -1;
    plus_z_fd = -1;
    battery_fd = -1;
    mopower_fd = -1;
  } else
  {  
    plus_x_fd  = wiringPiI2CSetupInterface(i2c_1, 0x40);
    plus_y_fd  = wiringPiI2CSetupInterface(i2c_1, 0x41);
    plus_z_fd  = wiringPiI2CSetupInterface(i2c_1, 0x44);
    battery_fd = wiringPiI2CSetupInterface(i2c_1, 0x45);
    mopower_fd = wiringPiI2CSetupInterface(i2c_1, 0x4a);

    #ifdef DEBUG_LOGGING
      fprintf(stderr, "File Descriptors of i2c-1 Addresses\n");
      fprintf(stderr, "\t+X @ 0x40: %d\n", plus_x_fd);
      fprintf(stderr, "\t+Y @ 0x41: %d\n", plus_y_fd);
      fprintf(stderr, "\t+Z @ 0x44: %d\n", plus_z_fd);
      fprintf(stderr, "\tBATTERY @ 0x45: %d\n", battery_fd);
      fprintf(stderr, "\tMOPOWER @ 0x4a: %d\n\n", mopower_fd);
    #endif
  }

  const char* i2c_0 = "/dev/i2c-0";
  file_i2c = access(i2c_0, W_OK | R_OK);
  if (file_i2c < 0) {
    #ifdef DEBUG_LOGGING
      fprintf(stderr,"ERROR: /dev/ic2-0 bus not present\n");
    #endif

    minus_x_fd = -1;
    minus_y_fd = -1;
  } else {
    minus_x_fd = wiringPiI2CSetupInterface(i2c_0, 0x40);
    minus_y_fd = wiringPiI2CSetupInterface(i2c_0, 0x41);

    #ifdef DEBUG_LOGGING 
      fprintf(stderr, "File Descriptors of i2c-0 Addresses\n");
      fprintf(stderr, "\t-X @ 0x40: %d\n", minus_x_fd);
      fprintf(stderr, "\t-Y @ 0x41: %d\n\n", minus_y_fd);
    #endif
  }

  int ret;
  uint8_t data[1024];

  tx_freq_hz -= tx_channel * 50000;

  init_rf();

  ax25_init(&hax25, (uint8_t *) "CQ", '1', (uint8_t *) CALLSIGN, '1',
            AX25_PREAMBLE_LEN,
            AX25_POSTAMBLE_LEN);
      
      
  /* Infinite loop */
  for (;;) {
    sleep(1);
    
    fprintf(stderr,"INFO: Getting TLM Data\n");
    
    get_tlm(tlm);

    fprintf(stderr,"INFO: Preparing X.25 packet\n");

    char str[1000];
    char tlm_str[1000];

    char header_str[] = "\x03\xf0hi hi ";
    strcpy(str, header_str);

    for (int channel = 0; channel < 7; channel++) {
      #ifdef DEBUG_LOGGING
        printf("%03d %03d %03d %03d %03d\n", tlm[channel][A], tlm[channel][B], tlm[channel][C], tlm[channel][D], tlm[channel][E]); 
      #endif

      sprintf(tlm_str, "%d%d%d %d%d%d %d%d%d %d%d%d %d%d%d", 
        channel, upper_digit(tlm[channel][A]), lower_digit(tlm[channel][A]),
        channel, upper_digit(tlm[channel][B]), lower_digit(tlm[channel][B]), 
        channel, upper_digit(tlm[channel][C]), lower_digit(tlm[channel][C]), 
        channel, upper_digit(tlm[channel][D]), lower_digit(tlm[channel][D]),
        channel, upper_digit(tlm[channel][E]), lower_digit(tlm[channel][E]));

      #ifdef DEBUG_LOGGING
        printf("%s \n",tlm_str);
      #endif
      strcat(str, tlm_str);
    }

    digitalWrite (0, LOW); 
  
    #ifdef DEBUG_LOGGING
      char cmdbuffer[1000];
      if (charging) {
        FILE* file1 = popen("/home/pi/mopower/mpcmd LED_STAT=1", "r"); 
        fgets(cmdbuffer, 999, file1);
        pclose(file1);

        printf("LED state: %s\n", cmdbuffer);
      }
    #endif

    #ifdef DEBUG_LOGGING 
      fprintf(stderr,"INFO: Transmitting X.25 packet\n");
    #endif

    memcpy(data, str, strnlen(str, 256));
    ret = ax25_tx_frame(&hax25, &hax5043, data, strnlen(str, 256));
    if (ret) {
      fprintf(stderr,
              "ERROR: Failed to transmit AX.25 frame with error code %d\n",
              ret);
      exit(EXIT_FAILURE);
    }

    ax5043_wait_for_transmit();
    digitalWrite (0, HIGH);
  
    #ifdef DEBUG_LOGGING
      FILE* file2 = popen("/home/pi/mopower/mpcmd LED_STAT=0", "r"); 
      fgets(cmdbuffer, 999, file2);
      pclose(file2);
     	printf("LED state: %s\n", cmdbuffer);
    #endif
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
  fprintf(stderr,"Initializing AX5043\n");

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
		fprintf(stderr,"ERROR: Not a digit in lower_digit!\n");
	return digit;
}

// Returns upper digit of a number which must be less than 99
//
int upper_digit(int number) {

	int digit = 0;
	if (number < 100) 
		digit = (int)(number/10);
	else
		fprintf(stderr,"ERROR: Not a digit in upper_digit!\n");
	return digit;
}

struct SensorData read_sensor_data(int sensor) {
    struct SensorData data = {
      .current = 0.0,
      .power = 0.0
    };

    if (sensor == -1) {
      return data;
    }

    wiringPiI2CWriteReg16(sensor, INA219_REG_CALIBRATION, calibrationValue);
    wiringPiI2CWriteReg16(sensor, INA219_REG_CONFIG, config);	
    wiringPiI2CWriteReg16(sensor, INA219_REG_CALIBRATION, calibrationValue);

    data.current = wiringPiI2CReadReg16(sensor, INA219_REG_CURRENT) / currentDivider; 
    data.power = wiringPiI2CReadReg16(sensor, INA219_REG_POWER) * powerMultiplier; 

    return data;
}

void print_sensor_data(struct SensorData* data, char* sensor_name, char* bus, int address) {
    fprintf(stderr, "\t%s on %s @ %#x:\n", sensor_name, bus, address);
    fprintf(stderr, "\t\tcurrent: %04.2f\n", data->current);
    fprintf(stderr, "\t\tpower: %04.2f\n", data->power);
}

int get_tlm(int tlm[][5]) {
  //  Reading I2C voltage and current sensors	
  char cmdbuffer[1000];
  FILE* file;
  int i;
  //FILE* file = popen("sudo python /home/pi/CubeSatSim/python/readcurrent.py 2>&1", "r"); 
  //fgets(cmdbuffer, 999, file);
  //pclose(file);

  #ifdef DEBUG_LOGGING
  //  fprintf(stderr,"I2C Sensor data: %s\n", cmdbuffer);
  #endif

  //char ina219[16][20];  // voltage, currents, and power from the INA219 current sensors x4a, x40, x41, x44, and x45.
  //int i = 0;
  //char * data2 = strtok (cmdbuffer," ");

  //while (data2 != NULL) {
  //  strcpy(ina219[i], data2);
  //  data2 = strtok (NULL, " ");
  //  i++;
  //}

  #ifdef DEBUG_LOGGING
    file = popen("/home/pi/mopower/mpcmd show data", "r"); 
    fgets(cmdbuffer, 999, file);
    pclose(file);

    char mopower[64][14];
    char * pch;
    i = 0;
    pch = strtok (cmdbuffer," ,.-");
    while (pch != NULL)
    {
      strcpy(mopower[i], pch);
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
  #endif

  // read i2c current sensors
  struct SensorData plus_x_data = read_sensor_data(plus_x_fd);
  struct SensorData plus_y_data = read_sensor_data(plus_y_fd);
  struct SensorData plus_z_data = read_sensor_data(plus_z_fd);
  struct SensorData battery_data = read_sensor_data(battery_fd);
  struct SensorData mopower_data = read_sensor_data(mopower_fd);
  struct SensorData minus_x_data = read_sensor_data(minus_x_fd);
  struct SensorData minus_y_data = read_sensor_data(minus_y_fd);

  #ifdef DEBUG_LOGGING
    fprintf(stderr, "Power Information:\n");
    print_sensor_data(&plus_x_data, "+X", "i2c-1", 0x40);
    print_sensor_data(&plus_y_data, "+Y", "i2c-1", 0x41);
    print_sensor_data(&plus_z_data, "+Z", "i2c-1", 0x44);
    print_sensor_data(&battery_data, "BATTERY", "i2c-1", 0x45);
    print_sensor_data(&mopower_data, "MOPOWER", "i2c-1", 0x4a);
    print_sensor_data(&minus_x_data, "-X", "i2c-0", 0x40);
    print_sensor_data(&minus_y_data, "-Y", "i2c-0", 0x41);
    printf("\n");
  #endif
	
  tlm[0][A] = (int) (99.5 - plus_x_data.current / 10) % 100;
  tlm[0][B] = (int) (plus_x_data.power * 10) % 100;
  tlm[0][C] = (int) (99.5 - plus_y_data.current / 10) % 100;
  tlm[0][D] = (int) (plus_y_data.power * 10) % 100;

  tlm[1][A] = (int) (99.5 - plus_z_data.current / 10) % 100;
  tlm[1][B] = (int) (plus_z_data.power * 10) % 100;
  tlm[1][C] = (int) (99.5 - minus_x_data.current / 10) % 100;
  tlm[1][D] = (int) (minus_x_data.power * 10) % 100;

  tlm[2][A] = (int) (99.5 - minus_y_data.current / 10) % 100;
  tlm[2][B] = (int) (minus_y_data.power * 10) % 100;
  tlm[2][C] = (int) (99.5 - battery_data.current / 10) % 100;
  tlm[2][D] = (int) (battery_data.power * 10) % 100;

  tlm[3][A] = (int) (mopower_data.current / 15 + 0.5) % 100;
  tlm[3][B] = (int) (mopower_data.power * 10) % 100;
		   	
  if (tempSensor != -1) {
    int tempValue = wiringPiI2CReadReg16(tempSensor, 0); 
    uint8_t upper = (uint8_t) (tempValue >> 8);
    uint8_t lower = (uint8_t) (tempValue & 0xff);
    float temp = (float)lower + ((float)upper / 0x100);

    tlm[3][C] = (int)((95.8 - temp)/1.48 + 0.5) % 100;
  }	

  tlm[6][B] = 0 ;
  tlm[6][D] = 49 + rand() % 3; 

  // Display tlm
  #ifdef DEBUG_LOGGING
    printf("Telemetry:\n");
    printf("\t");
    for (int k = 0; k < 7; k++) {
      for (int j = A; j <= E; j++) {
        printf(" %02d ",	tlm[k][j]);
      }
      printf("\n\t");
    }	
    printf("\n");
  #endif

  return 0;
}
