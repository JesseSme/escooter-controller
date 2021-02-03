

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
char datapath[] = "/ipa/controller";
char idpath[] = "/ipa/id";
int port = 80;
String localIP = "";

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

DynamicJsonDocument id(256);


void setup() {

        pinMode(LED_BUILTIN, OUTPUT);
        digitalWrite(LED_BUILTIN, HIGH);

        hello();
        /*
        pinMode(5, OUTPUT);
        tone(5, 1000);
        delay(100);
        noTone(5);
        pinMode(5, INPUT);
        */
        // initialize serial communications and wait for port to open:
        Serial.begin(115200); // Just for debugging.
        Serial.println("Init temps...");
        //dht.begin();

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
        id["imei"] = IMEI;
        Serial.println(IMEI);

        Serial.println("Initializing GPS...");
        //Wait for GPS module to initialize.


        while (GPSDataState) {
                if (GPS.begin(GPS_MODE_SHIELD)) {
                        GPSDataState = 0;
                        Serial.println("Initialized GPS...");
                }
        }

        localIP = getIP();
        getSensorData();
        postData(doc, datapath);
}


//Main loop

//TODO: Does this need a state machine?
//TODO: Acquire data from VESC
//TODO: Finish servo signal control with the thumb pot
//TODO: Remote startup
void loop() {
        /*
            esc.write(50);
            int escValue = esc.read();

            Serial.println(escValue);
        */
        switch (controllerState) {
                case IS_IDLE:
                        
                        break;
                case IS_CHECKING_SENSORS:
                        break;
                case IS_TRANSMITTING:
                        break;
                case IS_SLEEPING:
                        if (wakeUp()) controllerState = IS_IDLE;
                        break;
        }


        /*
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
*/
}

void hello() {

        for (int i = 0; i < 3; i++) {
                digitalWrite(LED_BUILTIN, i % 2);
                delay(500);
        }

}


int startUp() {

        String inData;
        
        while(client.available()) {
                char readChar = client.read();
                inData += readChar;
        }
        
        Serial.println(inData);
        
        return 0;
}


//Wake the scooter
int wakeUp() {

        if (client.available()) {
                char c = client.read();
                Serial.print(c);
        }

        /*
            Add a way to toggle the battery on the scooter, and wake all the modules from sleep.
        */

        return 0;
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
        doc["senderip"] = localIP;
        doc["temp_out"] = 2323; //temp;
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



//Post data to AWS server
bool postData(DynamicJsonDocument document, char *postpath) {
        String contentType = "application/json";
        String postData = document.as<String>();

        if (client.connect(server, 80)) {
                Serial.println(server);
                Serial.println(postpath);
                client.post(postpath, contentType, postData);

                int statusCode = client.responseStatusCode();
                Serial.println("TEst 1");
                String response = client.responseBody();
                Serial.println("TEst 2");
                Serial.println("Status code:");
                Serial.println(statusCode);
                Serial.println("Response:");
                Serial.println(response);
                if (statusCode != 200) {
                        return 0;
                }

                transferState = 0;

                return 1;
        }
        Serial.println("Failed to send...");
        return 0;
}


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
