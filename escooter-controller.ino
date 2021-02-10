/*
				Name:											escooter-controller.ino
				Author:									Jesse Smedberg
				Description:				Controller code for Arduino MKR1500 to be used
																				in conjunction with a VESC4.12.
																				Gathers motor data from VESC using UART.
																				Gathers temperature data from DHT22 temperature and
																								moisture sensors.
																				Gathers GPS data using 
*/
#include <MKRNB.h>
#include <ArduinoHttpClient.h>
#include <ArduinoJson.h>
#include <DHT.h>
#include <DHT_U.h>
//#include <TinyGPS++.h>
#include "SSRelay.h"

//VescUart libraries
#include <VescUart.h>
#include <datatypes.h>
#include <crc.h>
#include <buffer.h>
#include "wiring_private.h"


#define PINNUMBER											"1234"		// Sim pin
#define DHTBATTPIN										6							// Digital pin connected to the DHT sensor
#define DHTOUTPIN											7
#define DHTTYPE													DHT22			// DHT 22  (AM2302), AM2321

#define POWERCHECK_DELAY				5000
#define VESCDATA_DELAY						2000
#define TEMPERATURE_DELAY			1000
#define POST_DELAY										5000
#define GPS_DELAY											1000


//States
enum ControllerStates {
        IS_INIT,
        IS_SLEEPING,
        IS_IDLE,
        IS_TRANSMITTING,
        IS_CHECKING_SENSORS
};
ControllerStates controllerState = IS_IDLE;

//Data variables
int temp = 0, hum = 0;
String IMEI = "";

//States
int requestState = 1;

//Time variables
uint32_t startTime = 0;
uint32_t tempEndTime = 0;
uint32_t requestTime = 30000;

uint32_t counterForDebug = 0;

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

//TinyGPSPlus gps;

//JSON created with ArduinoJson.h
//const int capacity  = JSON_OBJECT_SIZE(2) + JSON_OBJECT_SIZE(3) + JSON_OBJECT_SIZE(4) + 401;
DynamicJsonDocument doc(1024);
JsonObject location = doc.createNestedObject("location");

DynamicJsonDocument id(256);

//VescUART setup
VescUart VUART;
//Initializes addiotional serial communications on pins 0 and 1.
Uart SerialVESC(&sercom3, 1, 0, SERCOM_RX_PAD_1, UART_TX_PAD_0); // Create the new UART instance assigning it to pin 1 and 0
void SERCOM3_Handler() {
				SerialVESC.IrqHandler();
}


void setup() {

				pinMode(LED_BUILTIN, OUTPUT);
				digitalWrite(LED_BUILTIN, HIGH);
				// initialize serial communications and wait for port to open:
				Serial.begin(115200);
				SerialVESC.begin(115200);

				pinPeripheral(1, PIO_SERCOM); //Assign RX function to pin 1
				pinPeripheral(0, PIO_SERCOM); //Assign TX function to pin 0
				Serial.println("Init temps...");
				//dht.begin();

				Serial.println("Initializing web client...");

				bool notConnected = 1;
				while (notConnected) {
								//Serial.println("Connecting...");
								if (nb.begin(PINNUMBER) == NB_READY
								&& gprs.attachGPRS() == GPRS_READY) {
												Serial.println("Initialized web client...");
												notConnected = false;
								}
				}
				IMEI = nbModem.getIMEI();
				id["imei"] = IMEI;
				Serial.println(IMEI);

				client.setHttpResponseTimeout(requestTime);

}


//Main loop

//TODO: Does this need a state machine?
//TODO: Acquire data from VESC
//TODO: Remote startup
void loop() {

				//See powerState
				if (requestState) {
								if (powerStateRequest()) {
												requestState = 0;
												Serial.println("power request sent");
								}
				}
				if (powerStateResponse()) {
								requestState = 1;
				}


}

/*
*			Gets the wanted power state from the server.
* 
*			Should be ran every 5s.
* 
*			@param	values						None
*			@return values					1, if success
*																						0, failed
*/
int powerStateRequest() {

				int err = 0;
				err = client.get(powerpath);
				Serial.println(err);
				
				if (err == 0) {
								return 1;
				}
				return 0;
}

/*
*			Reads the response from aws server
*			and resets the requestState, if
*			no response arrives in time.
*			
*/
int powerStateResponse() {

				int err = 0;
				int ava = client.available();
				int con = client.connected();
				Serial.println("Incoming data status: ");
				Serial.println(ava);
				Serial.println("Connection status: ");
				Serial.println(con);
				if (client.available() 
								|| client.connected()) {
								err = client.responseStatusCode();

								if (err == 200) {
												String response = client.responseBody();
												int intResponse = response.toInt();
												Serial.println(err);
												Serial.println(intResponse);
												return 1;
								}
				} else {
								Serial.println("Timed out...");
								client.flush();
								requestState = 1;
								return 0;
				}
				return 0;
}




//Get location data from MKRGPS and save it in a JSON.
//Also gets the time from MKRGPS and MKRNB
int getSensorData() {
				/*
				if (!GPS.available()) {
				//Serial.println("No new GPS data found...");
				return 0;
				};
				*/
				//,
				location["latitude"] = 60.4599453199457;//GPS.latitude();//gps.location.lat();//
				location["longitude"] = 22.28776938556175;//GPS.longitude();//gps.location.lng();
				doc["identifier"] = IMEI;
				doc["epoch"] = nb.getTime();
				doc["temp_out"] = 2323;					//dhtOut.readTemperature();
				doc["temp_batt"] = 32.3;				//dhtBatt.readTemperature();
				doc["temp_fet"] = 311.3;				//vesc.data.tempFET
				doc["temp_motor"] = 45.5;			//vesc.data.tempMotor
				doc["average_motorcurrent"] = 55.6;					//vesc.data.avgMotorCurrent
				doc["average_inputcurrent"] = 445.6;				//vesc.data.avgInputCurrent
				doc["input_voltage"] = 21.4;				//vesc.data.inpVoltage
				doc["rpm"] = 99;																//vesc.data.rpm
				doc["tachometer"] = 22;									//vesc.data.tachometer

}


//Creates a string for easy sending.
String IPToString(IPAddress address) {
				return String() + address[0] + "." + address[1] + "." + address[2] + "." + address[3];
}



//Post data to AWS server
bool postData(DynamicJsonDocument document, char *postpath) {
				String contentType = "application/json";
				String postData = document.as<String>();

				if (client.connect(server, 80)) {
								Serial.println(server);
								Serial.println(postpath);
								client.post(postpath, contentType, postData);

								int statusCode = client.responseStatusCode();
								String response = client.responseBody();
								Serial.println("Status code:");
								Serial.println(statusCode);
								Serial.println("Response:");
								Serial.println(response);
								if (statusCode != 200) {
												return 0;
								}

								return 1;
				}
				Serial.println("Failed to send...");
				return 0;
}


//Uses ipify api to GET own IP. 
//No idea why there isn't a way to get own public ip from the device.
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
