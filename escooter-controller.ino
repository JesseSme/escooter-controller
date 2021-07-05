/*
				Name:						escooter-controller.ino
				Author:						Jesse Smedberg
				Description:				Controller code for Arduino MKR1500 to be used
											in conjunction with a VESC4.12.
											Gathers motor data from VESC using UART.
											Gathers temperature data from DHT22 temperature and
											moisture sensors.
											Gathers GPS data using a Adafruit Ultimate GPS Breakout V3 module
											built around MTK3339.
*/

#include <ArduinoJson.h>
#include <ArduinoBearSSL.h>
#include <ArduinoECCX08.h>
#include "ArduinoMqttClient.h"
#include <DHT.h>
#include "SSRelay.h"

#include <Adafruit_GPS.h>

//VescUart libraries
#include <VescUart.h>
#include <datatypes.h>
#include <crc.h>
#include <buffer.h>
#include "wiring_private.h"
#include "arduino_secrets.h"

#include "MKRNB.h"

#define ARDUINOJSON_USE_DOUBLE 1
//Macros
#define DHTTYPE													DHT22		// DHT 22  (AM2302), AM2321

//Pins
#define DHTBATTPIN          6			// Digital pin connected to the DHT sensor
#define DHTOUTPIN           7

//Hacky delays
#define POWERCHECK_DELAY    15000
#define VESC_DELAY          10000
#define TEMPERATURE_DELAY   5000
#define POST_DELAY          10000
#define GPS_DELAY           5000
#define TURN_OFF_DELAY      60000
#define POLL_DELAY          1000

uint32_t turnoff_endtime    = 0;
uint32_t powerCheck_endtime = 0;
uint32_t vesc_endtime       = 0;
uint32_t temp_endtime       = 0;
uint32_t post_endtime       = 0;
uint32_t gps_endtime        = 0;
uint32_t poll_endtime       = 0;

#define NOW																					(uint32_t)millis()
#define RUN_THIS(endtime, d)				(((NOW) - (endtime)) > (d))
//#define SerialGPS															Serial1

//Data variables
//EPOCH
unsigned long epoch = 0;
//VESC
float temp_fet = 0;
float temp_motor = 0;
float average_motorcurrent = 0;
float average_inputcurrent = 0;
float input_voltage = 0;
int rpm = 0;
int tachometer = 0;
//GPS
float lati = 0;
float lngi = 0;
//TEMPS
float temp_out = -1;
float temp_batt = -1;

//States
int powerState = 0;
int lastPowerState = 0;
int requestState = 1;

//Sensors
DHT dhtBatt(DHTBATTPIN, DHTTYPE);
DHT dhtOut(DHTOUTPIN, DHTTYPE);

//Web client stuff. For AWS data POSTing.
const char basepath[]       = "scooter/jesse/";
const char vescpath[]       = "vesc";
const char temppath[]       = "temp";
const char locapath[]       = "location";
const char powerpath[]      = "power";
String datapath             = "scooter/jesse/data";
const char pinnumber[]      = SECRET_PINNUMBER;
const char broker[]         = SECRET_BROKER;
const char* certificate     = SECRET_CERTIFICATE;

NB nbAccess(true);
GPRS gprs;

NBClient nbClient;
BearSSLClient   sslClient(nbClient);
MqttClient  mqttClient(sslClient);

//Adafruit_GPS GPS(&SerialGPS);

//VescUART setup
VescUart SerialVESC;
//Initializes additional serial communications on pins 0 and 1.
Uart VUART(&sercom3, 1, 0, SERCOM_RX_PAD_1, UART_TX_PAD_0); // Create the new UART instance assigning it to pin 1 and 0
void SERCOM3_Handler() {
				VUART.IrqHandler();
}


void setup() {
    Serial.begin(115200);			//For debugging
    while (!Serial);
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);
    switchRelayState(LOW);
    VUART.begin(115200);		//Vesc communication
    pinPeripheral(1, PIO_SERCOM);	//Assign RX function to pin 1
    pinPeripheral(0, PIO_SERCOM);	//Assign TX function to pin 0
    // initialize serial communications:
    SerialVESC.setSerialPort(&VUART);
    //GPS.begin(115200);
    //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
    //GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);

    //Serial.println("Init temps...");
    //dhtBatt.begin();
    //dhtOut.begin();
    if (!ECCX08.begin()) {
        Serial.println("No ECCX08 present");
        while (1);
    }
    ArduinoBearSSL.onGetTime(getTime);
    sslClient.setEccSlot(0, certificate);
    //Initialize some fields.

    
    //mqttClient.setId("JessesScooter1500");
    mqttClient.onMessage(onMessageReceived);
    Serial.println("Setup done...");
}


//Main loop

void loop() {
    if (nbAccess.status() != NB_READY || gprs.status() != GPRS_READY) {
        Serial.print("Disconnected internet... ");
        Serial.println(millis());
        connectNB();
        return;
    }
    /* ... */
    if (!mqttClient.connected()) {
        Serial.println("MQTT connection lost");
        connectMQTT();
        return;
    }
    

				////powerStateRequest and Response should be melded together.
				//if (NOW - powerCheck_endtime > POWERCHECK_DELAY) {
				//				int pSReq = powerStateRequest();
				//				if (pSReq == 1) {
				//								Serial.println("power request sent");
				//				} else {
				//								powerCheck_endtime = NOW;
				//								return;
				//				}
				//				Serial.println("Stuuuuuuuuck here?");
    //    //TODO: powerState has to be checked for change before switching relay.
				//				powerState = powerStateResponse();
				//				int sRSCode = switchRelayState(powerState);
				//				//Serial.print("PowerState: ");
				//				//Serial.println(powerState);
				//				//Serial.println("lastPowerState: ");
				//				//Serial.println(lastPowerState);
				//				Serial.print("Relay code: ");
    //    Serial.println(sRSCode);
				//				powerCheck_endtime = NOW;
				//				return;
				//}
  mqttClient.poll();
    /*
    if (NOW - poll_endtime > POLL_DELAY) {
        Serial.println("Polling...");
        
        Serial.println("Polled...");
        poll_endtime = NOW;
        return;
    }
    */
				//if (NOW - gps_endtime > GPS_DELAY) {
				//				getGPSData();
				//				gps_endtime = NOW;
				//				return;
				//}
				//digitalWrite(LED_BUILTIN, LOW);

				if (NOW - vesc_endtime > VESC_DELAY) {
								getVESCData();
								vesc_endtime = NOW;
								return;
				}
				if (NOW - temp_endtime > TEMPERATURE_DELAY) {
								getTempData();
								temp_endtime = NOW;
								return;
				}
				if (NOW - post_endtime > POST_DELAY && mqttClient.connected()) {
								epoch = nbAccess.getLocalTime();
								if (postData(datapath)) {
												Serial.print("data POST ");
            Serial.println(millis());
								} else {
												Serial.println("data not posted...");
								}
								post_endtime = NOW;
								return;
				}
				//digitalWrite(LED_BUILTIN, HIGH);
    //delay(20);
}

/*
*   Reads location data from the GPS module.
*   Check if 
*/
//void getGPSData() {
//				Serial.print("started getGPSData() ");
//    Serial.println(millis());
//
//				while (SerialGPS.available() > 0) {
//								char c = GPS.read();
//								if (GPS.newNMEAreceived()) {
//												Serial.print("parsing... ");
//            Serial.println(millis());
//												if (!GPS.parse(GPS.lastNMEA())) {
//																return;
//												}
//
//												if (GPS.fix) {
//																lati = (float)GPS.latitude_fixed / 10000000;
//																lngi = (float)GPS.longitude_fixed / 10000000;
//																Serial.println(lati, 7);
//																Serial.println(lngi, 7);
//												}
//								}
//				}
//}

/*
* 
*/
void getVESCData() {
				Serial.print("started getVESCData() ");
    Serial.println(millis());
				if (SerialVESC.getVescValues()) {
								temp_fet = SerialVESC.data.tempFET;
								temp_motor = SerialVESC.data.tempMotor;
								average_motorcurrent = SerialVESC.data.avgMotorCurrent;
								average_inputcurrent = SerialVESC.data.avgInputCurrent;
								input_voltage = SerialVESC.data.inpVoltage;
								rpm = SerialVESC.data.rpm;
								tachometer = SerialVESC.data.tachometer;
				} else {
								Serial.print("Vesc not connected... ");
        Serial.println(millis());
								temp_fet = -1;
								temp_motor = -1;
								average_motorcurrent = -1;
								average_inputcurrent = -1;
								input_voltage = -1;
								rpm = -1;
								tachometer = -1;
				}
}


void getTempData() {
				float outval = dhtOut.readTemperature();
				float battval = dhtBatt.readTemperature();
				Serial.print("outval read...");
    Serial.println(millis());
				if (isnan(outval)) {
								temp_out = -1;
				} else {
								temp_out = outval;
				}
				Serial.print("battval read...");
    Serial.println(millis());
				if (isnan(battval)) {
								temp_batt = -1;
				} else {
								temp_batt = battval;
				}
}


//Post data to AWS server
bool postData(String postpath) {
    Serial.print("Publish begun...");
    Serial.println(millis());
    StaticJsonDocument<256> doc;
    Serial.print("JsonDocument created...");
    Serial.println(millis());
    char output[256];
    doc["identifier"] = SECRET_IMEI;

    JsonObject location = doc.createNestedObject("location");
    location["longitude"] = lngi;
    location["latitude"] = lati;
    doc["temp_out"] = temp_out;
    doc["temp_batt"] = temp_batt;
    doc["temp_fet"] = temp_fet;
    doc["temp_motor"] = temp_motor;
    doc["average_motorcurrent"] = average_motorcurrent;
    doc["average_inputcurrent"] = average_inputcurrent;
    doc["input_voltage"] = input_voltage;
    doc["rpm"] = rpm;
    doc["tachometer"] = tachometer;
    doc["epoch"] = epoch;

    Serial.print("JsonDocument filled...");
    Serial.println(millis());
    mqttClient.beginMessage(postpath);
    // send message, the Print interface can be used to set the message contents
    //mqttClient.print(output);
    serializeMsgPack(doc, mqttClient);
    mqttClient.endMessage();
    Serial.print("Message sent...");
    Serial.println(millis());

    return 1;
}

////Post data to AWS server
//bool postData(String postpath) {
//    Serial.print("Publish begun...");
//    Serial.println(millis());
//    StaticJsonDocument<256> doc;
//    Serial.print("JsonDocument created...");
//    Serial.println(millis());
//    char output[256];
//    doc["identifier"] = SECRET_IMEI;
//
//    JsonObject location = doc.createNestedObject("location");
//    location["longitude"] = lngi;
//    location["latitude"] = lati;
//    doc["temp_out"] = temp_out;
//    doc["temp_batt"] = temp_batt;
//    doc["temp_fet"] = temp_fet;
//    doc["temp_motor"] = temp_motor;
//    doc["average_motorcurrent"] = average_motorcurrent;
//    doc["average_inputcurrent"] = average_inputcurrent;
//    doc["input_voltage"] = input_voltage;
//    doc["rpm"] = rpm;
//    doc["tachometer"] = tachometer;
//    doc["epoch"] = epoch;
//
//    Serial.print("JsonDocument filled...");
//    Serial.println(millis());
//    serializeJson(doc, output);
//    mqttClient.beginMessage(postpath);
//    mqttClient.print(output);
//    mqttClient.endMessage();
//        Serial.print("Message sent...");
//        Serial.println(millis());
//    // send message, the Print interface can be used to set the message contents
//
//    return 1;
//}


void connectNB() {
  Serial.println("Attempting to connect to the cellular network");

  while ((nbAccess.begin(pinnumber) != NB_READY) ||
         (gprs.attachGPRS() != GPRS_READY)) {
    // failed, retry
    Serial.print(".");
    delay(1000);
  }

  Serial.println("You're connected to the cellular network");
  Serial.println();
}


// get the current time from the NB module
unsigned long getTime() {
  return nbAccess.getLocalTime();
}


void connectMQTT() {
  Serial.print("Attempting to connect to the MQTT broker");

  while (!mqttClient.connect(broker, 8883)) {
      // failed, retry
      Serial.println("");
      Serial.print("mqttClient connection error: ");
      Serial.println(mqttClient.connectError());
      delay(5000);
      if (nbAccess.status() != NB_READY || gprs.status() != GPRS_READY) {
          connectNB();
      }
  }
  Serial.println();

  Serial.println("You're connected to the MQTT broker");
  Serial.println();

  // subscribe to a topic
  mqttClient.subscribe("arduino/incoming");
  mqttClient.subscribe(powerpath);
}


void onMessageReceived(int messageSize) {
    // we received a message, print out the topic and contents
    Serial.print("Received a message with topic '");
    Serial.print(mqttClient.messageTopic());
    Serial.print("', length ");
    Serial.print(messageSize);
    Serial.println(" bytes:");

    // use the Stream interface to print the contents
    while (mqttClient.available()) {
        Serial.print((char)mqttClient.read());
    }
    Serial.println();

    Serial.println();
}
