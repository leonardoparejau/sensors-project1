/**
 * Copyright (c) 2017, Arm Limited and affiliates.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include <stdio.h>
#include "mbed.h"
#include <string>
#include <math.h>
#include "DigitalOut.h"
#include "InterruptIn.h"
#include "PinNames.h"
#include <cstdint>
#include "lorawan/LoRaWANInterface.h"
#include "lorawan/system/lorawan_data_structures.h"
#include "events/EventQueue.h"

// Application helpers
#include "DummySensor.h"
#include "MBed_Adafruit_GPS.h"
#include "TCS3472_I2C.h"
#include "MMA8451Q.h"
#include "Temp_RH_Sensor.h"
#include "trace_helper.h"
#include "lora_radio_helper.h"

/** RGB Defines **/
#define ENABLE           0x00
#define TCS34725_CDATAL (0x14) /**< Clear channel data low byte */
#define TCS34725_CDATAH (0x15) /**< Clear channel data high byte */
#define TCS34725_RDATAL (0x16) /**< Red channel data low byte */
#define TCS34725_RDATAH (0x17) /**< Red channel data high byte */
#define TCS34725_GDATAL (0x18) /**< Green channel data low byte */
#define TCS34725_GDATAH (0x19) /**< Green channel data high byte */
#define TCS34725_BDATAL (0x1A) /**< Blue channel data low byte */
#define TCS34725_BDATAH (0x1B) /**< Blue channel data high byte */

using namespace events;

// Max payload size can be LORAMAC_PHY_MAXPAYLOAD.
// This example only communicates with much shorter messages (<30 bytes).
// If longer messages are used, these buffers must be changed accordingly.
uint8_t tx_buffer[30];
uint8_t rx_buffer[30];

/*
 * Sets up an application dependent transmission timer in ms. Used only when Duty Cycling is off for testing
 */
#define TX_TIMER                        10s

/**
 * Maximum number of events for the event queue.
 * 10 is the safe number for the stack events, however, if application
 * also uses the queue for whatever purposes, this number should be increased.
 */
#define MAX_NUMBER_OF_EVENTS            10

/**
 * Maximum number of retries for CONFIRMED messages before giving up
 */
#define CONFIRMED_MSG_RETRY_COUNTER     3

//GPS
UnbufferedSerial * gps_Serial = new UnbufferedSerial(PA_9, PA_10,9600); //serial object to use with GPS
Adafruit_GPS myGPS(gps_Serial); //object of Adafruit's GPS class
bool flaggps;
float latitude = 40.2416f;
float longitude = -3.987f;
//Light Sensor
AnalogIn light_sensor(PA_4);
float lightValue;
uint8_t lightValue2;
// Soil Moisture
AnalogIn soil_moisture_sensor(PA_0);
float soilMoistureValue;
uint16_t soilMoistureValue2;
// Temperature_Humidity
I2C sensorI2C(PB_9,PB_8);
uint8_t  _address = 0;
uint8_t  _rx_buf[8] = {0};
uint8_t  _tx_buf[2] = {0}; 
uint32_t _rhData = 0;
int32_t  _tData = 0;
float _rhDataf;
uint16_t _rhDataf2;
float _tDataf;
int16_t _tDataf2;
// RGB Sensor
TCS3472_I2C rgb_sensor(PB_9,PB_8);
int rgb_readings[4];
//uint8_t dominant;
uint8_t red;
uint8_t green;
uint8_t blue;
// Accelerometer
MMA8451Q accel_sensor(PB_9,PB_8,0x1c<<1);
float x,y,z;
//RGB Led
DigitalOut redLed(PH_0);
DigitalOut greenLed(PH_1);
DigitalOut blueLed(PB_13);

/**
* This event queue is the global event queue for both the
* application and stack. To conserve memory, the stack is designed to run
* in the same thread as the application and the application is responsible for
* providing an event queue to the stack that will be used for ISR deferment as
* well as application information event queuing.
*/
static EventQueue ev_queue(MAX_NUMBER_OF_EVENTS *EVENTS_EVENT_SIZE);

/**
 * Event handler.
 *
 * This will be passed to the LoRaWAN stack to queue events for the
 * application which in turn drive the application.
 */
static void lora_event_handler(lorawan_event_t event);

/**
 * Constructing Mbed LoRaWANInterface and passing it the radio object from lora_radio_helper.
 */
static LoRaWANInterface lorawan(radio);

/**
 * Application specific callbacks
 */
static lorawan_app_callbacks_t callbacks;
static uint8_t DEV_EUI[] = { 0x85, 0x39, 0x32, 0x35, 0x59, 0x37, 0x91, 0x94};
static uint8_t APP_EUI[] = { 0x70, 0xb3, 0xd5, 0x7e, 0xd0, 0x00, 0xfc, 0xda };
static uint8_t APP_KEY[] = { 0xf3,0x1c,0x2e,0x8b,0xc6,0x71,0x28,0x1d,0x51,0x16,0xf0,0x8f,0xf0,0xb7,0x92,0x8f };

/**
 * Read Sensors Methods
 */
void read_soil_moisture(void) {	
    soilMoistureValue = soil_moisture_sensor.read_u16();  
		soilMoistureValue = (soilMoistureValue*100)/60000;
		soilMoistureValue2=soilMoistureValue*10;
		printf(" Soil Moisture Sensor Value = %.1f%% \n", soilMoistureValue);
}

void read_light(void) {		
    lightValue = light_sensor.read_u16();
		lightValue = (lightValue*100)/60000;
		lightValue2=lightValue;
		printf(" Light Sensor Value = %d%% \n", lightValue2);
}

void read_temp_hum(void) {				
    int temp;
    unsigned int humidity;
   
    //send humidity command
    _tx_buf[0] = SI7013_READ_RH;
    sensorI2C.write(_address, (char*)_tx_buf, 1);
    sensorI2C.read(_address, (char*)_rx_buf, 2);
   
    /* Store raw RH info */
    humidity = ((uint32_t)_rx_buf[0] << 8) + (_rx_buf[1] & 0xFC);
    /* Convert value to milli-percent */
    humidity = (((humidity) * 15625L) >> 13) - 6000;
   
    //send temperature command
    _tx_buf[0] = SI7013_READ_TEMP;
    sensorI2C.write(_address, (char*)_tx_buf, 1);
    sensorI2C.read(_address, (char*)_rx_buf, 2);
   
    /* Store raw temperature info */
    temp = ((uint32_t)_rx_buf[0] << 8) + (_rx_buf[1] & 0xFC);
    /* Convert to milli-degC */
    temp = (((temp) * 21965L) >> 13) - 46850;
   
    _tData = temp;    
		_tDataf = (_tData+0.0)/1000; 
		_tDataf = roundf(_tDataf * 100) / 100;
		_tDataf2=_tDataf*10;
				
		_rhData = humidity;
		_rhDataf = (_rhData+0.0)/1000; 
		_rhDataf = roundf(_rhDataf * 100) / 100;
		_rhDataf2=_rhDataf*10;
		
		printf(" Temperature: %.1f%cC \r\n", _tDataf,248);
		printf(" Relative Humidity: %.1f%% \r\n", _rhDataf);		
}

void read_sensorRGB()
{		
    rgb_sensor.getAllColors(rgb_readings);
		red = rgb_readings[1]/256;
		green = rgb_readings[2]/256;
		blue = rgb_readings[3]/256;
//  Get dominant color
//		if (red>green and red>blue){
//			dominant = 1;
//		}
//		else if (green>red and green>blue) {
//			dominant = 2;
//		}
//		else if (blue>red and blue>green){
//			dominant = 3;
//		}
		printf(" RGB: Clear: %d, Red: %d (%d)  Green: %d (%d)  Blue: %d (%d) \r\n",rgb_readings[0],rgb_readings[1],red,rgb_readings[2],green, rgb_readings[3], blue);
}

void read_accel()
{		
    x=accel_sensor.getAccX();
    y=accel_sensor.getAccY();
    z=accel_sensor.getAccZ();
		printf(" Accelerometer: x = %f \t y = %f\t z = %f \r\n",x,y,z);
}
/**
 * Entry point for application
 */
int main(void)
{
		//Initialize GPS
		myGPS.begin(9600);  //sets baud rate for GPS communication; note this may be changed via Adafruit_GPS::sendCommand(char *)
    myGPS.sendCommand(PMTK_SET_NMEA_OUTPUT_GGA); //these commands are defined in MBed_Adafruit_GPS.h; a link is provided there for command creation
    myGPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
    myGPS.sendCommand(PGCMD_ANTENNA);
    ThisThread::sleep_for(1s);
	
		// Enable RGB Sensor
    rgb_sensor.enablePowerAndRGBC();
	
		//Check if the temp and humidity sensor is present
    _tx_buf[0] = SI7013_READ_ID2_1;
    _tx_buf[1] = SI7013_READ_ID2_2;   
    _address = SI7021_ADDR;    
    sensorI2C.write(_address, (char*)_tx_buf, 2);
    sensorI2C.read(_address, (char*)_rx_buf, 8);
    //Check ID byte
    if(_rx_buf[0] != SI7021_DEVICE_ID) {
        printf("No sensor present!\r\n");
        while(1);
    }
		
		redLed=1;
		greenLed=1;
		blueLed=1;
		
		//setup tracing
    setup_trace();

    // stores the status of a call to LoRaWAN protocol
    lorawan_status_t retcode;

    // Initialize LoRaWAN stack
    if (lorawan.initialize(&ev_queue) != LORAWAN_STATUS_OK) {
        printf("\r\n LoRa initialization failed! \r\n");
        return -1;
    }

    printf("\r\n Mbed LoRaWANStack initialized \r\n");

    // prepare application callbacks
    callbacks.events = mbed::callback(lora_event_handler);
    lorawan.add_app_callbacks(&callbacks);

    // Set number of retries in case of CONFIRMED messages
    if (lorawan.set_confirmed_msg_retries(CONFIRMED_MSG_RETRY_COUNTER)
            != LORAWAN_STATUS_OK) {
        printf("\r\n set_confirmed_msg_retries failed! \r\n\r\n");
        return -1;
    }

    printf("\r\n CONFIRMED message retries : %d \r\n",
           CONFIRMED_MSG_RETRY_COUNTER);

    // Enable adaptive data rate
    if (lorawan.enable_adaptive_datarate() != LORAWAN_STATUS_OK) {
        printf("\r\n enable_adaptive_datarate failed! \r\n");
        return -1;
    }

    printf("\r\n Adaptive data  rate (ADR) - Enabled \r\n");
    lorawan_connect_t connect_params;
		connect_params.connect_type = LORAWAN_CONNECTION_OTAA;
    connect_params.connection_u.otaa.dev_eui = DEV_EUI;
    connect_params.connection_u.otaa.app_eui = APP_EUI;
    connect_params.connection_u.otaa.app_key = APP_KEY;
    connect_params.connection_u.otaa.nb_trials = 3;
		
    retcode = lorawan.connect(connect_params);

    if (retcode == LORAWAN_STATUS_OK ||
            retcode == LORAWAN_STATUS_CONNECT_IN_PROGRESS) {
    } else {
        printf("\r\n Connection error, code = %d \r\n", retcode);
        return -1;
    }

    printf("\r\n Connection - In Progress ...\r\n");

    // make your event queue dispatching events forever
    ev_queue.dispatch_forever();

    return 0;
}

/**
 * Sends a message to the Network Server
 */
static void send_message()
{
    uint16_t packet_len;
    int16_t retcode;
		
		read_soil_moisture();
		read_temp_hum();
		read_light();
		read_sensorRGB();
		read_accel();

		printf(" Latitude = %f Longitude = %f \r\n", latitude,longitude);
		
		//Send 
		//packet_len = sprintf((char *) tx_buffer, "%.1f;%.1f;%.1f;%.1f;%.1f;",
    //                     latitude,longitude,soilMoistureValue,_tDataf,lightValue);
		union {
			float latitud_temp;
			char temp_array_lat[4];
		} u1;
		union {
			float longitude_temp;
			char temp_array_lon[4];
		} u2;
		union {
			float x_temp;
			char temp_array_x[4];
		} u3;
		union {
			float y_temp;
			char temp_array_y[4];
		} u4;
		union {
			float z_temp;
			char temp_array_z[4];
		} u5;
		u1.latitud_temp=latitude;
		u2.longitude_temp=longitude;
		tx_buffer[0]=u1.temp_array_lat[0];
		tx_buffer[1]=u1.temp_array_lat[1];
		tx_buffer[2]=u1.temp_array_lat[2];
		tx_buffer[3]=u1.temp_array_lat[3];
		tx_buffer[4]=u2.temp_array_lon[0];
		tx_buffer[5]=u2.temp_array_lon[1];
		tx_buffer[6]=u2.temp_array_lon[2];
		tx_buffer[7]=u2.temp_array_lon[3];
		tx_buffer[8]=soilMoistureValue2 >> 8;
		tx_buffer[9]=soilMoistureValue2;
		tx_buffer[10]=_tDataf2 >> 8;
		tx_buffer[11]=_tDataf2;
		tx_buffer[12]=_rhDataf2 >> 8;
		tx_buffer[13]=_rhDataf2;
		tx_buffer[14]=lightValue2;
		u3.x_temp=x;
		tx_buffer[15]=u3.temp_array_x[0];
		tx_buffer[16]=u3.temp_array_x[1];
		tx_buffer[17]=u3.temp_array_x[2];
		tx_buffer[18]=u3.temp_array_x[3];
		u4.y_temp=y;
		tx_buffer[19]=u4.temp_array_y[0];
		tx_buffer[20]=u4.temp_array_y[1];
		tx_buffer[21]=u4.temp_array_y[2];
		tx_buffer[22]=u4.temp_array_y[3];
		u5.z_temp=z;
		tx_buffer[23]=u5.temp_array_z[0];
		tx_buffer[24]=u5.temp_array_z[1];
		tx_buffer[25]=u5.temp_array_z[2];
		tx_buffer[26]=u5.temp_array_z[3];
		tx_buffer[27]=red;
		tx_buffer[28]=green;
		tx_buffer[29]=blue;
		printf(" Payload: ");
		for(int i=0; i<30; i++){
			printf("%02x ",tx_buffer[i]);
		}
		
    //retcode = lorawan.send(MBED_CONF_LORA_APP_PORT, tx_buffer, packet_len,
    //                       MSG_UNCONFIRMED_FLAG);
		retcode = lorawan.send(MBED_CONF_LORA_APP_PORT, tx_buffer, 30,
                           MSG_UNCONFIRMED_FLAG);
		//retcode = lorawan.send(MBED_CONF_LORA_APP_PORT, tx_buffer, packet_len,
    //                       MSG_CONFIRMED_FLAG);

    if (retcode < 0) {
        retcode == LORAWAN_STATUS_WOULD_BLOCK ? printf("send - WOULD BLOCK\r\n")
        : printf("\r\n send() - Error code %d \r\n", retcode);

        if (retcode == LORAWAN_STATUS_WOULD_BLOCK) {
            //retry in 3 seconds
            if (MBED_CONF_LORA_DUTY_CYCLE_ON) {
                ev_queue.call_in(3s, send_message);
            }
        }
        return;
    }

    printf("\r\n %d bytes scheduled for transmission \r\n", retcode);
    memset(tx_buffer, 0, sizeof(tx_buffer));
}

/**
 * Receive a message from the Network Server
 */
static void receive_message()
{
    uint8_t port;
    int flags;
    int16_t retcode = lorawan.receive(rx_buffer, sizeof(rx_buffer), port, flags);
		string command;

    if (retcode < 0) {
        printf("\r\n receive() - Error code %d \r\n", retcode);
        return;
    }

    printf(" RX Data on port %u (%d bytes): ", port, retcode);		
    for (uint8_t i = 0; i < retcode; i++) {
        printf("%02x ", rx_buffer[i]);
    }
    printf("\r\n");
		command= string((char *)rx_buffer);
		printf(" Command: %s \r\n", command.c_str());
    
    memset(rx_buffer, 0, sizeof(rx_buffer));
		
		if (command=="OFF"){
			redLed=1;
			greenLed=1;
			blueLed=1;
		}
		if (command=="Green"){
			redLed=1;
			greenLed=0;
			blueLed=1;
		}
		if (command=="Red"){
			redLed=0;
			greenLed=1;
			blueLed=1;
		}
}

/**
 * Event handler
 */
static void lora_event_handler(lorawan_event_t event)
{	
    switch (event) {
        case CONNECTED:
            printf("\r\n Connection - Successful \r\n");
            if (MBED_CONF_LORA_DUTY_CYCLE_ON) {
                send_message();
            } else {
                ev_queue.call_every(TX_TIMER, send_message);
            }
            break;
        case DISCONNECTED:
            ev_queue.break_dispatch();
            printf("\r\n Disconnected Successfully \r\n");
            break;
        case TX_DONE:
            printf("\r\n Message Sent to Network Server \r\n");
						flaggps=false;
            if (MBED_CONF_LORA_DUTY_CYCLE_ON) {								
								while(!flaggps) {										
									myGPS.read();   //queries the GPS, needs to be sycncronicing all the time to work properly
									//check if we recieved a new message from GPS, if so, attempt to parse it,
									if ( myGPS.newNMEAreceived() ) {										
										if ( !myGPS.parse(myGPS.lastNMEA()) ) {
											flaggps=true;
										}
									}									
								}
								if (myGPS.fixquality > 0) {
									latitude =	myGPS.latitude/100;
									longitude = myGPS.longitude/100;
									if (myGPS.lat=='S') {
										latitude=latitude*(-1);
									}
									if (myGPS.lon=='W') {
										longitude=longitude*(-1);
									}
								}
								else{
									latitude=40.2416f;
									longitude=-3.987f;
								}
                send_message();
            }
            break;
        case TX_TIMEOUT:
        case TX_ERROR:
        case TX_CRYPTO_ERROR:
        case TX_SCHEDULING_ERROR:
            printf("\r\n Transmission Error - EventCode = %d \r\n", event);
            // try again
            if (MBED_CONF_LORA_DUTY_CYCLE_ON) {
                send_message();
            }
            break;
        case RX_DONE:
            printf("\r\n Received message from Network Server \r\n");
            receive_message();
            break;
        case RX_TIMEOUT:
        case RX_ERROR:
            printf("\r\n Error in reception - Code = %d \r\n", event);
            break;
        case JOIN_FAILURE:
            printf("\r\n OTAA Failed - Check Keys \r\n");
            break;
        case UPLINK_REQUIRED:
            printf("\r\n Uplink required by NS \r\n");
            if (MBED_CONF_LORA_DUTY_CYCLE_ON) {
                send_message();
            }
            break;
        default:
            MBED_ASSERT("Unknown Event");
    }
}

// EOF
