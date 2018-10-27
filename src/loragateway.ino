// Adafruit Feather Huzzah ESP8266 LoRa receiver for data relay to Raspberry Pi IoT Gateway
// Author : Tanmoy Dutta
// 28 Oct 2017 
//works
#include <WiFi.h>
#include <PubSubClient.h>
//#include <ESP8266WiFiMulti.h>
//#include <ESP8266HTTPClient.h>

#include <RH_RF95.h>
#include "RadioSettings.h"
#include <ArduinoJson.h>
#include <Adafruit_NeoPixel.h>


//#include "Wire.h"//th
//#include "UbidotsMicroESP8266.h"
// WiFi access credentials
#define  WIFI_SSID         "Penryn"         // WiFi SSID
#define  WIFI_PASSWORD     "hoooverr"         // WiFI Password


#define TOKEN  "A1E-0V3Qu4hZfmUA0uOtVMt4rFPtVaz171"  // Put here your Ubidots TOKEN
#define ID_1 "59d85600c03f97202c9ff2c0" // Put your variable ID here

WiFiClient wificlient;

/****************************************
 * MQTT
 ****************************************/
void ubidotsmqttSingle(char const* varable, char const* value);
void ubidotsmqttJson(char* varable1, int value1, char* varable2, int value2, char* varable3, int value3);
#define VARIABLE_LABEL "MQTTsensor2" // Assing the variable label
#define DEVICE_LABEL "esp" // Assig the device label
#define MQTT_CLIENT_NAME "OLtBrXVH6i" // MQTT client Name, please enter your own 8-12 alphanumeric character ASCII string;
char mqttBroker[]  = "things.ubidots.com";
char payload[100];
char topic[150];
// Space to store values to send
char str_sensor[10];

PubSubClient client(wificlient);

void callback(char* topic, byte* payload, unsigned int length);
void reconnect();
/*****************************************/
void _initLoRa();
void _initWiFi();
void _checkWifi_mqtt();
void rainbow(uint8_t wait);
void theaterChaseRainbow(uint8_t wait);
//int32_t Wheel(byte WheelPos);

long batteryVoltageDecompress (byte batvoltage);
float temperatureDecompress(byte temperature);
unsigned int Combine2bytes(byte x_high, byte x_low);
int calculate_tank_level(int sensor1, int sensor2, int sensor3);

int tank_level=0;//derived from multiple sensors

//Ubidots client2(TOKEN,"client2");

// JSON Buffer
DynamicJsonBuffer jsonBuffer;

#define RFM95_CS 18
#define RFM95_RST 14
#define RFM95_INT 26

// Blinky on receipt
#define BUILTIN_BLUE_LED 2

// Set radio frequency
#define RF95_FREQ 434.0

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

static const RH_RF95::ModemConfig radiosetting = {
    BW_SETTING<<4 | CR_SETTING<<1 | ImplicitHeaderMode_SETTING,
    SF_SETTING<<4 | CRC_SETTING<<2,
    LowDataRateOptimize_SETTING<<3 | ACGAUTO_SETTING<<2};

struct payloadDataStruct{
  byte nodeID;
  byte rssi;
  byte voltage;
  byte temperature;
  byte capsensor1Lowbyte;
  byte capsensor1Highbyte;
  byte capsensor2Lowbyte;
  byte capsensor2Highbyte;
  byte capsensor3Lowbyte;
  byte capsensor3Highbyte;
}rxpayload;

#define Neopixel_PIN 21
Adafruit_NeoPixel strip = Adafruit_NeoPixel(8, Neopixel_PIN, NEO_GRB + NEO_KHZ800);


/*struct payloadDataStruct{
int voltage;

  byte rssi;
  byte nodeID;
}rxpayload;
*/
/*----------------------------------------------------------------------------
Function : setup()
Description :
------------------------------------------------------------------------------*/
void setup() {
strip.begin();
  rainbow(2);
  theaterChaseRainbow(50);
 strip.setPixelColor(7, strip.Color(0,0,255));
  strip.setPixelColor(2, strip.Color(0,0,255));
  strip.setPixelColor(0, strip.Color(255,0,255));
  strip.show();
//client2.setDebug(true); // Uncomment this line to set DEBUG on
//set up 1HZ pwm on LED
pinMode(BUILTIN_BLUE_LED, OUTPUT);
ledcSetup(0, 1, 8);//pwm channel 0. 1Hz, 8 bit resolution
ledcAttachPin(BUILTIN_BLUE_LED, 0);//attach LED to PWN channel 0
ledcWrite(0, 0);//led off

  Serial.begin(57600);
  delay(1000);
  _initLoRa();

  rf95.setModemRegisters(&radiosetting);
  _initWiFi();

  Serial.print(" CPU");Serial.print(F_CPU/1000000,DEC);
  Serial.println(F(" MHz"));

  client.setServer(mqttBroker, 1883);
  client.setCallback(callback);
  _checkWifi_mqtt();

}


/*----------------------------------------------------------------------------
Function : loop()
Description : Main program loop
------------------------------------------------------------------------------*/
void loop() {
//client.publish("/v1.6/devices/esp","{\"voltage\":2660,\"rssi\":0,\"temp\":18}");
//rf95.printRegisters(); //th

  if (rf95.available()) {
    // Should be a message for us now
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);
    if (rf95.recv(buf, &len)) {
      delay(10);

      char bufChar[len];
      for(int i = 0; i < len; i++) {
        bufChar[i] = char(buf[i]);
        Serial.print(String(bufChar[i]));
      }

      Serial.println();
      Serial.println("Len==" + String(len));
      Serial.println("String(bufChar)==" + String(bufChar));

      memcpy(&rxpayload, buf, sizeof(rxpayload));

      tank_level= calculate_tank_level((int)Combine2bytes(rxpayload.capsensor1Highbyte,rxpayload.capsensor1Lowbyte),(int)Combine2bytes(rxpayload.capsensor2Highbyte,rxpayload.capsensor2Lowbyte),(int)Combine2bytes(rxpayload.capsensor3Highbyte,rxpayload.capsensor3Lowbyte));
      
      //rxpayload.voltage=batteryVoltageDecompress(rxpayload.voltage);
      Serial.print(" nodeID = ");Serial.print(rxpayload.nodeID);
      Serial.print(" remote voltage comp = ");Serial.print((rxpayload.voltage));
      Serial.print(" remote voltage = ");Serial.print(batteryVoltageDecompress(rxpayload.voltage));
      Serial.print(" remote rssi = ");Serial.print(rxpayload.rssi);
      Serial.print(" Local RSSI: ");Serial.print(rf95.lastRssi(), DEC);
      Serial.print(" Local SNR: ");Serial.print(rf95.lastSNR(), DEC);
      Serial.print(" cap1= ");Serial.print(Combine2bytes(rxpayload.capsensor1Highbyte,rxpayload.capsensor1Lowbyte));
      Serial.print(" cap2= ");Serial.print(Combine2bytes(rxpayload.capsensor2Highbyte,rxpayload.capsensor2Lowbyte));
      Serial.print(" cap3= ");Serial.print(Combine2bytes(rxpayload.capsensor3Highbyte,rxpayload.capsensor3Lowbyte));
      Serial.print(" tank level= ");Serial.print(tank_level);
      ///////////////////////MQTT Code
      //sendMessage(String(bufChar));


      //Flash LED
      if (tank_level<= 1)  {
        theaterChaseRainbow(50);
        strip.setPixelColor(1, strip.Color(255,0,0));
        strip.setPixelColor(2, strip.Color(255,0,0));
        strip.setPixelColor(3, strip.Color(255,0,0));
        strip.show();

        ledcWrite(0, 128);}//builtin LED flash (pwm channel 0)
       else if (tank_level<=50) {
         theaterChaseRainbow(10);
         ledcWrite(0, 128);//LED flash (pwm channel 0)
         strip.setPixelColor(1, strip.Color(255,255,0));
         strip.setPixelColor(2, strip.Color(255,0,0));
         strip.setPixelColor(3, strip.Color(255,0,0));
         strip.show();
       }
       else if (tank_level<=80) {
         theaterChaseRainbow(2);
         ledcWrite(0, 128);//LED off (pwm channel 0)
         strip.setPixelColor(1, strip.Color(255,255,0));
         strip.setPixelColor(2, strip.Color(255,255,0));
         strip.setPixelColor(3, strip.Color(255,0,0));
         strip.show();
       }
       else {
         ledcWrite(0, 0);//LED off (pwm channel 0)
         strip.setPixelColor(1, strip.Color(0,255,0));
         strip.setPixelColor(2, strip.Color(0,0,0));
         strip.setPixelColor(3, strip.Color(0,0,0));
         strip.show();

       }



       // Send a reply to sensor
       //uint8_t outgoingData[] = "{\"Status\" : \"Ack\"}";
       uint8_t ack[1];
       // the acknolement consists of an ack identiferer followed by nodeID
       ack[0]=(uint8_t)170; //ack identifier 170=10101010
       ack[1]= (uint8_t)rxpayload.nodeID;
       rf95.send(ack, 2);
       //rf95.send(outgoingData, sizeof(outgoingData));
       rf95.waitPacketSent();

      //MQTT SEND/////////////////////////////////
      _checkWifi_mqtt();
      float sensor = temperatureDecompress(rxpayload.temperature);
      dtostrf(sensor, 4, 3, str_sensor); /* 4 is mininum width, 2 is precision; float value is copied onto str_sensor*/
      ubidotsmqttSingle("temp", str_sensor);
      delay(1000);
      ubidotsmqttJson("local-rssi", (int)rf95.lastRssi(),
        "rssi", -rxpayload.rssi, "voltage", batteryVoltageDecompress(rxpayload.voltage));

      delay(3000);
      ubidotsmqttJson("level", (int)Combine2bytes(rxpayload.capsensor1Highbyte,rxpayload.capsensor1Lowbyte),
         "level-2", (int)Combine2bytes(rxpayload.capsensor2Highbyte,rxpayload.capsensor2Lowbyte), "level-3",(int)Combine2bytes(rxpayload.capsensor3Highbyte,rxpayload.capsensor3Lowbyte));

      delay(1000);
      sensor = rf95.frequencyError();
      dtostrf(sensor, 4, 3, str_sensor); /* 4 is mininum width, 2 is precision; float value is copied onto str_sensor*/
      ubidotsmqttSingle("frequency-error", str_sensor);

      
      dtostrf(tank_level, 4, 3, str_sensor); /* 4 is mininum width, 2 is precision; float value is copied onto str_sensor*/
      ubidotsmqttSingle("CalcLevel",str_sensor);

     /*client2.add("59d864b6c03f972cdb9e33e6", -rxpayload.rssi);
     client2.add("59dee274c03f976a87c2594b", (int)rf95.lastRssi());
     client2.add("59d864a1c03f972cdb9e33e5", batteryVoltageDecompress(rxpayload.voltage));
     client2.add("59d85600c03f97202c9ff2c0",temperatureDecompress(rxpayload.temperature))  ;
     client2.sendAll(false);
     client2.add("59e7b649c03f972d175ee2e2",(int)Combine2bytes(rxpayload.capsensor1Highbyte,rxpayload.capsensor1Lowbyte));
     client2.add("5a654f7cc03f9724e9db682c",(int)Combine2bytes(rxpayload.capsensor2Highbyte,rxpayload.capsensor2Lowbyte));
     client2.sendAll(false);*/


    }
    else {
      //Serial.println("Receive failed");
    }
  }
  client.loop();//Mqtt
  /*esp_sleep_enable_timer_wakeup(10000000); //10 seconds
  //erial.printf("start light_sleep: %d\n");
   int ret = esp_light_sleep_start();
   Serial.printf("light_sleep: %d\n", ret);*/
delay(10000);

}


void donothing() {
}

/*----------------------------------------------------------------------------
Function : _initWiFi()
Description : Connect to WiFi access point
------------------------------------------------------------------------------*/
void _initWiFi() {
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
Serial.println("WiFi Connecting");
for (int waiting=0; waiting <= 6; waiting++){
  if (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println(" Waiting for WiFi...");
    Serial.println(waiting);
  }

  delay(1000);
}

if (WiFi.status() == WL_CONNECTED) {
  Serial.println("WiFi Connected!");
}
}


void _checkWifi_mqtt() {
if (WiFi.status() != WL_CONNECTED) {
  _initWiFi();
  delay(1000);
  //client.connect(MQTT_CLIENT_NAME, TOKEN, "")
}
if (!client.connected()) {
    Serial.println("reconecting/connecting mqtt");
    reconnect();
   if (!client.connected()) {
      strip.setPixelColor(7, strip.Color(0,0,255));//turn error LED on
      strip.show();
    }
}  
else {strip.setPixelColor(7, strip.Color(0,0,0));  
     strip.show();}
}
/*----------------------------------------------------------------------------
Function : _initLoRa()
Description : Connect to WiFi access point
------------------------------------------------------------------------------*/
void _initLoRa() {
  attachInterrupt(digitalPinToInterrupt(RFM95_INT), donothing, CHANGE);
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  Serial.println("LoRa RX WiFi Repeater");
  delay(1000);

  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(100);

  while (!rf95.init()) {
    Serial.println("LoRa radio init failed");
    //while (1);//th
  }
  Serial.println("LoRa radio init OK!");
  delay(1000);

  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    while (1);
  }
  Serial.println("Freq set");
  delay(1000);

  rf95.setTxPower(17, false);
  rf95.setModemConfig(RH_RF95::Bw125Cr48Sf4096);//
  delay(10);
  rf95.setModemRegisters(&radiosetting);//this is where we apply our custom settings
  rf95.printRegisters();



  Serial.println("LoRa Listening...");
  delay(1000);
}

long batteryVoltageDecompress (byte batvoltage) {
//decompress voltage from 1 byte
long result2;
result2 =  (batvoltage*8)+1300;
return (result2);
}

float temperatureDecompress(byte temperature){
  //decomcompress temperature from 1 byte
  // allowable temperature range is 0 to 51 degree C
float result2;
result2=((float)temperature)/5;
//result2=(float)temperature;
return result2;
}

unsigned int Combine2bytes(byte x_high, byte x_low)
//convert high and low bytes to unsigned int (0-65,535 (2^16) - 1)
{
unsigned int combined;
  if ((x_high >=256) & (x_low>=256)) combined =0; ///out of range
  combined = x_low | x_high << 8;
  return combined;
}


void callback(char* topic, byte* payload, unsigned int length) {
  char p[length + 1];
  memcpy(p, payload, length);
  p[length] = NULL;
  String message(p);
  Serial.write(payload, length);
  Serial.println(topic);
}

void reconnect() {
  // Loop a few times until we're reconnected
  int attempts=10;
  while (!client.connected()) {
    if (attempts<=0) break;
    attempts = attempts-1;
    Serial.println("Attempting MQTT connection...");

    // Attemp to connect MQTT
    if (client.connect(MQTT_CLIENT_NAME, TOKEN, "")) {
      Serial.println("Connected");
    } else {
      Serial.print("Failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 2 seconds");
      // Wait 2 seconds before retrying
      delay(2000);
    }
  }
}

void ubidotsmqttSingle(char const* varable, char const* value){
//format topic and payload, then publish single data point
sprintf(topic, "%s%s", "/v1.6/devices/", DEVICE_LABEL);

sprintf(payload, "%s", ""); // Cleans the payload
sprintf(payload, "%s {\"%s\": %s}", payload, varable, value);// Adds the value
Serial.print("Publishing data to Ubidots Cloud");
Serial.print(" topic= ");Serial.println(topic);
Serial.println(" payload= ");Serial.println(payload);
client.publish(topic, payload); //eg client.publish(/v1.6/devices/esp,{"MQTTsensor": {"value": 3.10}})

//example client.publish(topic, "{\"temperature\": 10, \"humidity\": 50}");

}


void ubidotsmqttTriple(char* varable1, int value1, char* varable2, char* value2, char* varable3, char* value3){
//format topic and payload, then publish single data point
sprintf(topic, "%s%s", "/v1.6/devices/", DEVICE_LABEL);
char str_value1[10];
char str_value2[10];
char str_value3[10];
dtostrf(value1, 4, 2, str_value1);

sprintf(payload, "%s", ""); // Cleans the payload
//sprintf(payload, "%s {\"%s\": %s}", payload, varable, value);// Adds the value
sprintf(payload, "%s {\"%s\": %s\", \"%s\": %s, \"%s\": %s}", payload, varable1, value1, varable2, value2, varable3, value3);// Adds the value
Serial.print("Publishing data to Ubidots Cloud");
Serial.print(" topic= ");Serial.println(topic);
Serial.println(" payload= ");Serial.println(payload);
client.publish(topic, payload); //eg client.publish(/v1.6/devices/esp,{"MQTTsensor": {"value": 3.10}})

//example client.publish(topic, "{\"temperature\": 10, \"humidity\": 50}");

}

void ubidotsmqttJson(char* varable1, int value1, char* varable2, int value2, char* varable3, int value3){

  StaticJsonBuffer<100> jsonBuffer;
  char JSONmessageBuffer[100];
  JsonObject& root = jsonBuffer.createObject();

  root[varable1] = value1;
  root[varable2] = value2;
  root[varable3] = value3;

root.printTo(JSONmessageBuffer, sizeof(JSONmessageBuffer));
Serial.println(JSONmessageBuffer);
sprintf(topic, "%s%s", "/v1.6/devices/", DEVICE_LABEL);
Serial.print("Publishing data to Ubidots Cloud");
Serial.print(" topic= ");Serial.println(topic);
Serial.println(" payload= ");Serial.println(JSONmessageBuffer);
client.publish(topic, JSONmessageBuffer);
}


  void rainbow(uint8_t wait) {
    uint16_t i, j;

    for(j=0; j<256; j++) {
      for(i=0; i<strip.numPixels(); i++) {
        strip.setPixelColor(i, Wheel((i+j) & 255));
      }
      strip.show();
      delay(wait);
    }
  }

  //Theatre-style crawling lights with rainbow effect
  void theaterChaseRainbow(uint8_t wait) {
    for (int j=0; j < 256; j++) {     // cycle all 256 colors in the wheel
      for (int q=0; q < 3; q++) {
        for (int i=0; i < strip.numPixels(); i=i+3) {
          strip.setPixelColor(i+q, Wheel( (i+j) % 255));    //turn every third pixel on
        }
        strip.show();

        delay(wait);

        for (int i=0; i < strip.numPixels(); i=i+3) {
          strip.setPixelColor(i+q, 0);        //turn every third pixel off
        }
      }
    }
  }

  // Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t Wheel(byte WheelPos) {
  WheelPos = 255 - WheelPos;
  if(WheelPos < 85) {
    return strip.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  }
  if(WheelPos < 170) {
    WheelPos -= 85;
    return strip.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
  WheelPos -= 170;
  return strip.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
}

int calculate_tank_level(int sensor1, int sensor2, int sensor3){
//based on the three sensor readings estimate the level
//valid outputs 100, 80, 50, 20
//output on error is 0

const int threshold1 = 250;
const int threshold2 = 300;
const int threshold3 = 240;

int calculated_level=0;

if (sensor3>threshold3 && sensor2>threshold2 && sensor1>threshold1) calculated_level=100;
if (sensor3<threshold3 && sensor2>threshold2 && sensor1>threshold1) calculated_level=80;
if (sensor3<threshold3 && sensor2<threshold2 && sensor1>threshold1) calculated_level=50;
if (sensor3<threshold3 && sensor2<threshold2 && sensor1<threshold1) calculated_level=20;

return calculated_level;
}