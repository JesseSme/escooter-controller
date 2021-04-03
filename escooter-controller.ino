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
#include <MKRNB.h>
#include <ArduinoHttpClient.h>
#include <ArduinoJson.h>
#include <DHT.h>
#include <TinyGPS++.h>
#include "SSRelay.h"
//#include <ArduinoLowPower.h>

//#include <NMEA_data.h>
//#include <Adafruit_PMTK.h>
#include <Adafruit_GPS.h>

//VescUart libraries
#include <VescUart.h>
#include <datatypes.h>
#include <crc.h>
#include <buffer.h>
#include "wiring_private.h"


//Macros
#define PINNUMBER											"1234"		// Sim pin
#define DHTTYPE													DHT22		// DHT 22  (AM2302), AM2321

//Pins
#define DHTBATTPIN										6			// Digital pin connected to the DHT sensor
#define DHTOUTPIN											7

//Hacky delays
#define POWERCHECK_DELAY				15000
#define VESC_DELAY										10000
#define TEMPERATURE_DELAY			5000
#define POST_DELAY										45000
#define GPS_DELAY											5000
#define TURN_OFF_DELAY						60000

#define LOGGING													1   //Clean up

uint32_t turnoff_endtime				= 0;
uint32_t powerCheck_endtime	= 0;
uint32_t vesc_endtime							= 0;
uint32_t temp_endtime							= 0;
uint32_t post_endtime							= 0;
uint32_t gps_endtime								= 0;

#define NOW																					(uint32_t)millis()
#define RUN_THIS(endtime, d)				(((NOW) - (endtime)) > (d))
#define SerialGPS															Serial1

//Data variables
int temp = 0, hum = 0;
String IMEI = "";

//States
int powerState = 0;
int lastPowerState = 0;
int requestState = 1;

//Sensors
DHT dhtBatt(DHTBATTPIN, DHTTYPE);
DHT dhtOut(DHTOUTPIN, DHTTYPE);

//Web client stuff. For AWS data POSTing.
char server[] = "52.29.232.160";
char datapath[] = "/ipa/controller";
char idpath[] = "/ipa/id";
char powerpath[] = "/ipa/power_state";
int port = 80;

NB nb;
NBClient nbClient;
NBModem nbModem;
GPRS gprs;
IPAddress ip;
HttpClient client = HttpClient(nbClient, server, port);

Adafruit_GPS GPS(&SerialGPS);

//JSON created with ArduinoJson.h
//const int capacity  = JSON_OBJECT_SIZE(2) + JSON_OBJECT_SIZE(3) + JSON_OBJECT_SIZE(4) + 401;
DynamicJsonDocument doc(1024);
JsonObject location = doc.createNestedObject("location");

DynamicJsonDocument id(256);

//VescUART setup
VescUart SerialVESC;
//Initializes additional serial communications on pins 0 and 1.
Uart VUART(&sercom3, 1, 0, SERCOM_RX_PAD_1, UART_TX_PAD_0); // Create the new UART instance assigning it to pin 1 and 0
void SERCOM3_Handler() {
				VUART.IrqHandler();
}


void setup() {

				pinMode(LED_BUILTIN, OUTPUT);
				digitalWrite(LED_BUILTIN, HIGH);
    switchRelayState(LOW);
    VUART.begin(115200);		//Vesc communication
    pinPeripheral(1, PIO_SERCOM);	//Assign RX function to pin 1
    pinPeripheral(0, PIO_SERCOM);	//Assign TX function to pin 0
				// initialize serial communications:
				Serial.begin(9600);			//For debugging
				
				//SerialGPS.begin(9600);
				GPS.begin(9600);
				GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
				GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);



				Serial.println("Init temps...");
				dhtBatt.begin();
				dhtOut.begin();

				Serial.println("Initializing web client...");

				bool notConnected = 1;
				while (notConnected) {
								if (nb.begin(PINNUMBER) == NB_READY
								&& gprs.attachGPRS() == GPRS_READY) {
												Serial.println("Initialized web client...");
												notConnected = false;
								}
				}
				IMEI = nbModem.getIMEI();
				//Initialize some fields.
				Serial.println(IMEI);
				doc["identifier"] = IMEI;
				location["latitude"] = 0;
				location["longitude"] = 0;

				client.connectionKeepAlive();
				client.setHttpResponseTimeout(5000);

				SerialVESC.setSerialPort(&VUART);

}


//Main loop

void loop() {
				
				//powerStateRequest and Response should be melded together.
				if (NOW - powerCheck_endtime > POWERCHECK_DELAY) {
								int pSReq = powerStateRequest();
								if (pSReq == 1) {
												Serial.println("power request sent");
								} else {
												powerCheck_endtime = NOW;
												return;
								}
								Serial.println("Stuuuuuuuuck here?");
        //TODO: powerState has to be checked for change before switching relay.
								powerState = powerStateResponse();
								int sRSCode = switchRelayState(powerState);
								//Serial.print("PowerState: ");
								//Serial.println(powerState);
								//Serial.println("lastPowerState: ");
								//Serial.println(lastPowerState);
								Serial.print("Relay code: ");
        Serial.println(sRSCode);
								powerCheck_endtime = NOW;
								return;
				}

				if (NOW - gps_endtime > GPS_DELAY) {
								getGPSData();
								gps_endtime = NOW;
								return;
				}
				digitalWrite(LED_BUILTIN, LOW);

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
				if (NOW - post_endtime > POST_DELAY) {
								doc["epoch"] = nb.getLocalTime();
								if (postData(doc, datapath)) {
												Serial.println("data POST");
								} else {
												Serial.println("data not posted...");
								}
								post_endtime = NOW;
								return;
				}
				digitalWrite(LED_BUILTIN, HIGH);
}

/*
*			Gets the wanted power state from the server.
* 
*			@param	values						None
*			@return values					1, if success
*																						0, failed
*/
int powerStateRequest() {
				
				Serial.println("Started powerStateRequest()");

				uint32_t errtime = NOW;
				while (client.get(powerpath) > 0 || NOW - errtime > 5000) {
								Serial.println("Error in get...");
								client.stop();
								return 0;
				};
				Serial.print("GET Success...");
				return 1;
}

/*
*			Reads the response from aws server
*			and resets the requestState, if
*			no response arrives in time.
*			
*/
int powerStateResponse() {
				Serial.println("started powerStateResponse()");
				int err = 0;
				uint8_t connectionStatus = client.connected();
				Serial.println(connectionStatus);
				if (connectionStatus) {
								uint32_t test = NOW;
								while (client.available() <= 0 || NOW - test < 5000) { // <-- TODO: Clean up.
												err = client.responseStatusCode();

													if (err == 200) {
																		String response = client.responseBody();
																		int intResponse = response.toInt();
																		Serial.println(err);
																		Serial.print("Power response: ");
																		Serial.println(intResponse);
																		return intResponse;
													}
													Serial.print("HTTP not OK. Code: ");
													Serial.println(err);
													client.flush();
													client.stop();
								}
								Serial.println("Timed out or client not available...");
								return 0;
				} else {
								Serial.println("Disconnected...");
								return 0;
				}
				return 0;
}

/*
*   Reads location data from the GPS module.
*   Check if 
*/
void getGPSData() {
				Serial.println("started getGPSData()");

				while (SerialGPS.available() > 0) {
								char c = GPS.read();
								if (GPS.newNMEAreceived()) {
												Serial.println("parsing...");
												if (!GPS.parse(GPS.lastNMEA())) {
																return;
												}

												if (GPS.fix) {
																float lati = (float)GPS.latitude_fixed / 10000000;
																float lngi = (float)GPS.longitude_fixed / 10000000;
																Serial.println(lati, 7);
																Serial.println(lngi, 7);
																location["latitude"] = lati;
																location["longitude"] = lngi;
												}
								}
				}
}

/*
* 
*/
void getVESCData() {
				Serial.println("started getVESCData()");
				if (SerialVESC.getVescValues()) {
								doc["temp_fet"] = SerialVESC.data.tempFET;
								doc["temp_motor"] = SerialVESC.data.tempMotor;
								doc["average_motorcurrent"] = SerialVESC.data.avgMotorCurrent;
								doc["average_inputcurrent"] = SerialVESC.data.avgInputCurrent;
								doc["input_voltage"] = SerialVESC.data.inpVoltage;
								doc["rpm"] = SerialVESC.data.rpm;
								doc["tachometer"] = SerialVESC.data.tachometer;
				} else {
								Serial.println("Vesc not connected...");
								doc["temp_fet"] = -1;
								doc["temp_motor"] = -1;
								doc["average_motorcurrent"] = -1;
								doc["average_inputcurrent"] = -1;
								doc["input_voltage"] = -1;
								doc["rpm"] = -1;
								doc["tachometer"] = -1;
				}
}


void getTempData() {
				float outval = dhtOut.readTemperature();
				float battval = dhtBatt.readTemperature();
				Serial.println("outval read...");
				if (isnan(outval)) {
								doc["temp_out"] = -1;
				} else {
								doc["temp_out"] = outval;
				}
				Serial.println("battval read...");
				if (isnan(battval)) {
								doc["temp_batt"] = -1;
				} else {
								doc["temp_batt"] = battval;
				}
}


//Post data to AWS server
bool postData(DynamicJsonDocument document, char *postpath) {

				Serial.println("started postData()");
				String contentType = "application/json";
				String data = document.as<String>();
				Serial.println("postData made...");
				uint32_t errtime = NOW;
				while (client.post(postpath, contentType, data) > 0 || NOW - errtime > 5000) { 
								Serial.println("Error in post...");
								client.stop();
								return 0; 
				};

				//If statuscode is anything else than 200, flush the incoming message and return.
				int statusCode = client.responseStatusCode();
				if (statusCode != 200) {
								Serial.println("Status code:");
								Serial.println(statusCode);
								client.flush();
								//client.stop();
								return 0;
				}
				String response = client.responseBody();

				Serial.println("Response:");
				Serial.println(response);
				return 1;
}


//An universal get request. 
//TODO: Move to this in later versions.
String getIP() {
				int    HTTP_PORT = 80;
				String HTTP_METHOD = "GET";
				char   HOST_NAME[] = "api.ipify.org";
				String PATH_NAME = "/";
				String getPayload;

				if (client.connect(HOST_NAME, HTTP_PORT)) {
								// if connected:
								Serial.println("Connected to server");
								// make a HTTP request:
								// send HTTP header
								client.println(HTTP_METHOD + " " + PATH_NAME + " HTTP/1.1");
								client.println("Host: " + String(HOST_NAME));
								client.println("Connection: close");
								client.println(); // end HTTP header

								while (client.connected()) {
												if (client.available()) {
																char readChar = client.read();
																getPayload += readChar;
																// read an incoming byte from the server and print it to serial monitor:            
												}
								}
								int length = getPayload.length();
								getPayload = getPayload.substring(length - 16, length);
								getPayload.trim();
								Serial.println(getPayload);
								client.stop();

								return getPayload;

				}
				return "ERROR: No IP gotten.";
}
