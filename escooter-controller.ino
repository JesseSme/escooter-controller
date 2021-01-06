/*
*
*
*
*
*
*/
#include <MKRNB.h>
#include <Arduino_MKRGPS.h>
#include <ArduinoHttpClient.h>
#include <ArduinoJson.h>

#include <Servo.h>

#define PINNUMBER "1234"


//Variables
uint16_t throttlePin = A1;

uint16_t startTime = 0;
uint16_t endTime = 0;


//Servo
Servo esc;


//States
int powerState = 1;  //Needed? Controlled with the server, or something?
int GPSDataState = 1;  //Get gps data every now and then.
int transferState = 1;  //Transfer the data
bool ledBuiltInState = true;  //For blinking led when needed.
int written = 1;


//Web client stuff. For AWS data POSTing.
char server[] = "52.29.232.160";
char path[] = "/ipa/controller";
int port = 80;

NB nb;
NBClient nbClient;
GPRS gprs;
IPAddress ip;
HttpClient client = HttpClient(nbClient, server, port);


//JSON created with ArduinoJson.h
//const int capacity  = JSON_OBJECT_SIZE(2) + JSON_OBJECT_SIZE(3) + JSON_OBJECT_SIZE(4) + 401;
DynamicJsonDocument doc(1024);
JsonObject location = doc.createNestedObject("location");


void setup() {

				pinMode(LED_BUILTIN, OUTPUT);

				hello();
				esc.attach(5);

				// initialize serial communications and wait for port to open:
				Serial.begin(115200); // Just for debugging.
				while (!Serial);
				Serial.println("Initializing web client...");

				bool notConnected = 1;
				while (notConnected) {
								Serial.println("Connecting...");
								if (nb.begin(PINNUMBER) == NB_READY
												&& gprs.attachGPRS() == GPRS_READY) {
												Serial.println("Initialized web client...");
												notConnected = false;

								}

				}

				Serial.println("Initializing GPS...");
				//Wait for GPS module to initialize.
				while (GPSDataState) {
								if (GPS.begin()) {
												GPSDataState = 0;
												Serial.println("Initialized GPS...");
								}
				}

}


//Main loop

//TODO: Does this need a state machine?
//TODO: Acquire data from VESC
//TODO: Finish servo signal control with the thumb pot  
//TODO: Remote startup
//TODO: Communication with server would probably work best with a websocket
void loop() {
				/*
				esc.write(50);
				int escValue = esc.read();

				Serial.println(escValue);
				*/

				if (transferState) {


								if (getGPSInfo()) {
												;
												Serial.println("Getting GPS info...");
												delay(100);

												Serial.println("Posting GPS info...");

												postGPSInfo();
												delay(100);
								}
								if (millis() - startTime > 50) {
												startTime = millis();
												setPWM();
												Serial.println(esc.readMicroseconds());
								}


				}
				else {

								Serial.println("Data has been sent?");
								delay(1000);

				}
}


void hello() {

				for (int i = 0; i < 3; i++) {
								digitalWrite(LED_BUILTIN, !ledBuiltInState);
								delay(500);
				}

}


//Wake the scooter
int wakeUp() {

				/*
				* Add a way to toggle the battery on the scooter, and wake all the modules from sleep.
				*/

				return 1;
}


//Enters power saving mode.
int putToSleep() {

				/*
				* Reverse the wakeup.
				*/

				return 1;
}


//Set servo control for VESC.
uint16_t setPWM() {
				uint16_t throttleValue = 0;

				if (esc.attached()) {
								throttleValue = analogRead(throttlePin);
								throttleValue = map(throttleValue, 270, 1023, 700, 2000);
								esc.writeMicroseconds(throttleValue);
				}

				return throttleValue;

}


//Get location data from MKRGPS and save it in a JSON.
//Also gets the time from MKRGPS and MKRNB
int getGPSInfo() {

				if (!GPS.available()) {
								return 0;
				};
				//60.4599453199457, 22.28776938556175

				location["latitude"] = GPS.latitude();
				location["longitude"] = GPS.longitude();
				doc["identifier"] = "Controller eScooter";
				doc["epoch"] = nb.getTime();
				//doc["senderip"]    = IPToString(gprs.getIPAddress());
				doc["temp_out"] = 233.2;
				doc["temp_batt"] = 32.3;
				doc["temp_fet"] = 311.3;
				doc["temp_motor"] = 45.5;
				doc["average_motorcurrent"] = 55.6;
				doc["average_inputcurrent"] = 445.6;
				doc["input_voltage"] = 21.4;
				doc["rpm"] = 99;
				doc["tachometer"] = 22;



}


//Creates a string for easy sending.
String IPToString(IPAddress address) {
				return String() + address[0] + "." + address[1] + "." + address[2] + "." + address[3];
}



//Post gps data to AWS server
void postGPSInfo() {
				String contentType = "application/json";
				String postData = doc.as<String>();

				client.post(path, contentType, postData);

				int statusCode = client.responseStatusCode();
				String response = client.responseBody();

				Serial.println("Status code:");
				Serial.println(statusCode);
				Serial.println("Response:");
				Serial.println(response);

				transferState = 0;

}