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

#include <Scheduler.h>

#include <Servo.h>

#define PINNUMBER "1234"


//Variables
uint16_t throttleValue = 0;
uint16_t throttlePin   = A1;

uint16_t startTime     = 0;
uint16_t endTime       = 0;


//Servo
Servo esc;


//States
int powerState      = 1;  //Needed? Controlled with the server, or something?
int GPSDataState    = 1;  //Get gps data every now and then.
int transferState   = 1;  //Transfer the data
int ledBuiltInState = 1;  //For blinking led when needed.


//Web client stuff. For AWS data POSTing.
char server[] = "52.29.232.160";
char path[]   = "/ipa/location";
int port      = 80;

NB nb;
NBClient nbClient;
GPRS gprs;
IPAddress ip;
HttpClient client = HttpClient(nbClient, server, port);


//JSON created with ArduinoJson.h
//const int capacity  = JSON_OBJECT_SIZE(2) + JSON_OBJECT_SIZE(3) + JSON_OBJECT_SIZE(4) + 401;
DynamicJsonDocument doc(1024);
JsonObject location = doc.createNestedObject("location");
JsonObject epoch    = doc.createNestedObject("epoch");
JsonObject senderip = doc.createNestedObject("senderip");



void setup() {

  pinMode(LED_BUILTIN, OUTPUT);
  esc.attach(5);
  
  // initialize serial communications and wait for port to open:
  Serial.begin(115200); // Just for debugging.
  while (!Serial);

  bool notConnected = 1;
  while (notConnected) {

    if (nb.begin(PINNUMBER) == NB_READY 
     && gprs.attachGPRS()   == GPRS_READY) {

      notConnected = false;
      Serial.println("");
      
    }
    
  }


  //Wait for GPS module to initialize.
  if (!GPS.begin()) {
    while (1) {
      digitalWrite(LED_BUILTIN, !ledBuiltInState);
      delay(500);
    }
  }

  //Scheduler.startLoop(dataLoop);
}


//Main loop
/*
void loop() {

  if (powerState) {

    Serial.println("Setting PWM");
    delayMicroseconds(setPWM());
    
    
  } else {

    delay(20);
    
  }
  
}
*/
void Loop() {

  if (transferState) {

    Serial.println("Getting GPS info...");

    getGPSInfo();
    delay(100);

    Serial.println("Posting GPS info...");
  
    postGPSInfo();
    delay(100);
  
  } else {

    Serial.println("Data has been sent?");
    delay(1000);
    
  }
  //yield();
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
  
  if (esc.attached()) {
    throttleValue = 0;
    throttleValue = analogRead(throttlePin);
    //throttleValue = map(throttleValue, TODO, TODO, 1000, 2000);
    esc.writeMicroseconds(throttleValue);
  }

  return 20000 - throttleValue;
  
}


//Get location data from MKRGPS and save it in a JSON.
//Also gets the time from MKRGPS and MKRNB
void getGPSInfo() {

  GPS.wakeup();

  while (!GPS.available()) {
    delay(100);
  }

  location["latitude"]    = GPS.latitude();
  location["longitude"]   = GPS.longitude();
  location["speed"]       = GPS.speed();
  location["satellites"]  = GPS.satellites();

  epoch["epoch-gps"]      = GPS.getTime();
  epoch["epoch-nb"]       = nb.getTime();

  senderip["senderip"]    = IPToString(gprs.getIPAddress());

  GPS.standby();

}


String IPToString(IPAddress address) {
  return String() + address[0] + "." + address[1] + "." + address[2] + "." + address[3];
}



//Post gps data to AWS server
void postGPSInfo() {

  String contentType = "application/json";
  String postData = doc.as<String>();

  client.post("/ipa/location/", contentType, postData);

  int statusCode = client.responseStatusCode();
  String response = client.responseBody();

  Serial.println("Status code:");
  Serial.println(statusCode);
  Serial.println("Response:");
  Serial.println(response);

  transferState = 0;

  
  
}
