

/*




*/
#include <MKRNB.h>
#include <Arduino_MKRGPS.h>
#include <ArduinoHttpClient.h>
#include <ArduinoJson.h>
#include <DHT.h>
#include <DHT_U.h>
//#include <TinyGPSPlus.h>
//#include <TinyGPS++.h>
#include <Servo.h>

//#include <SoftwareSerial.h>

#define PINNUMBER "1234"
#define DHTPIN 2     // Digital pin connected to the DHT sensor
#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321

//Data variables
int temp = 0, hum = 0;
int IMEI = 0;

//Variables
uint16_t throttleValue = 0;
uint16_t throttlePin = A1;

uint32_t startTime = 0;
uint32_t tempStartTime = 0;
uint32_t endTime = 0;
uint32_t tempEndTime = 0;


//Servo
Servo esc;


//Sensors
DHT dht(DHTPIN, DHTTYPE);


//States
int powerState = 1;  //Needed? Controlled with the server, or something?
int GPSDataState = 1;  //Get gps data every now and then.
int transferState = 1;  //Transfer the data
int ledBuiltInState = HIGH;  //For blinking led when needed.
int written = 1;


//Web client stuff. For AWS data POSTing.
char server[] = "52.29.232.160";
char path[] = "/ipa/controller";
int port = 80;

NB nb;
NBClient nbClient;
NBModem nbModem;
GPRS gprs;
IPAddress ip;
HttpClient client = HttpClient(nbClient, server, port);
//TinyGPSPlus gps;
//SoftwareSerial ss(9, 8);

//JSON created with ArduinoJson.h
//const int capacity  = JSON_OBJECT_SIZE(2) + JSON_OBJECT_SIZE(3) + JSON_OBJECT_SIZE(4) + 401;
DynamicJsonDocument doc(1024);
JsonObject location = doc.createNestedObject("location");


void setup() {

				pinMode(LED_BUILTIN, OUTPUT);
				digitalWrite(LED_BUILTIN, HIGH);

				hello();
				esc.attach(5);
				pinMode(5, OUTPUT);
				tone(5, 1000);
				delay(100);
				noTone(5);
				pinMode(5, INPUT);
				// initialize serial communications and wait for port to open:
				Serial.begin(115200); // Just for debugging.
        Serial.println("Init temps...");
        dht.begin();

				Serial.println("Initializing web client...");

				bool notConnected = 1;
				while (notConnected) {
								//Serial.println("Connecting...");
								if (nb.begin(PINNUMBER) == NB_READY
												&& gprs.attachGPRS() == GPRS_READY
												&& nbModem.begin()) {
												Serial.println("Initialized web client...");
												notConnected = false;

								}

				}
        IMEI = nbModem.getIMEI();
       Serial.println(IMEI);

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
											//Serial.println("Getting GPS info...");
                        
											Serial.println("Posting GPS info...");
											postGPSInfo();
							}
							if (millis() - startTime > 500) {
											startTime = millis();
											digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
											setPWM();
											//Serial.println(esc.readMicroseconds());
							}
             if (millis() - tempStartTime > 2000) {
                  tempStartTime = millis();
                  hum = dht.readHumidity();
                  temp = dht.readTemperature();
                  Serial.print("Humidity: ");
                  Serial.println(hum);
                  Serial.print("Temperature: ");
                  Serial.println(temp);
             }

				}
				else {

							Serial.println("Data has been sent?");
							delay(1000);

				}
}

void hello() {

				for (int i = 0; i < 3; i++) {
								digitalWrite(LED_BUILTIN, i % 2);
								delay(500);
				}

}


//Wake the scooter
int wakeUp() {

				/*
						Add a way to toggle the battery on the scooter, and wake all the modules from sleep.
				*/

				return 1;
}


//Enters power saving mode.
int putToSleep() {

				/*
						Reverse the wakeup.
				*/

				return 1;
}


//Set servo control for VESC.
uint16_t setPWM() {

				if (esc.attached()) {
								//Serial.println("Setting PWM...");
								throttleValue = 0;
								throttleValue = analogRead(throttlePin);
								throttleValue = map(throttleValue, 270, 1023, 700, 2000);
								esc.writeMicroseconds(throttleValue);
				}

				return throttleValue;

}


//Get location data from MKRGPS and save it in a JSON.
//Also gets the time from MKRGPS and MKRNB
int getGPSInfo() {
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
				doc["senderip"]    = IPToString(gprs.getIPAddress());
				doc["temp_out"] = temp;
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
