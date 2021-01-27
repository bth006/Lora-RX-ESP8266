// Adafruit Feather Huzzah ESP8266 LoRa receiver for data relay to Raspberry Pi IoT Gateway
// Author : Tanmoy Dutta
// 24 Jan 2021
//works

/////LEDs
//0 power LEDalways green
//1+2+3 tank full->off, tank full->80% yellow ,tank 50% full->orange, tank 40% full->red
//6 blue is no recieving lora messages
//7 white if wifi/mqtt loss of connection
//7 pink if adafruit wifi/mqtt loss of connection
/////////

//ENABLE FEATURES
//#define EnableUbidots // whether to use ubidots


#include <WiFi.h>
#include <PubSubClient.h>
#include <RH_RF95.h> //ver 1.89
#include "RadioSettings.h"
#include <ArduinoJson.h>
#include <NeoPixelBus.h> //https://github.com/Makuna/NeoPixelBus/

//PINS
#define RFM95_CS 18
#define RFM95_RST 14
#define RFM95_INT 26
#define PIXEL_PIN1 21
#define BUILTIN_BLUE_LED 2


// WiFi access credentials
#define WIFI_SSID "Penryn"                         // WiFi SSID
#define WIFI_PASSWORD "hoooverr"                   // WiFI Password


//WiFiClient wificlient; // general use client
#if defined (EnableUbidots)
WiFiClient wificlientUbidots;
#endif
WiFiClient wificlientAdafruitIO;
WiFiClient homeAssistant;


/****************************************
 * MQTT
 ****************************************/
void ubidotsmqttSingle(char const *varable, char const *value);
void ubidotsmqttJson(char *varable1, int value1, char *varable2, int value2, char *varable3, int value3);
#define VARIABLE_LABEL "MQTTsensor2"  // Assing the variable label
#define DEVICE_LABEL "esp"            // Assig the device label
#define MQTT_CLIENT_NAME "OLtBrXVH6i" // MQTT client Name, please enter your own 8-12 alphanumeric character ASCII string;
#define UBIDOTSTOKEN "A1E-0V3Qu4hZfmUA0uOtVMt4rFPtVaz171" // Ubidots TOKEN
#define HATOKEN "koh5eixie2pae8vah5ton1saay7AQuaph4aejoaLae5toh6Bahx6aePheiseiMiP"
char UbidotsmqttBroker[] = "things.ubidots.com";
char HomeAssistantBroker[]= "192.168.1.198";
void adfruitiomqttSingle(char const *varable, char const *value);
void adfruitiomqttJson(char *varable1, int value1, char *varable2, int value2, char *varable3, int value3);
void reconnectHomeAssistBroker();
void HAmqttSingle(char const *varable, char const *value);
String clientId = "ESP32Client-cc5246fdcz";
  
#define ADAFRUIT_MQTT_CLIENT_NAME "distiller" // 
#define ADAFRUITIOTOKEN "e3758c24375742ccb361c88e38d638f3" // ada TOKEN
char AdafruitiomqttBroker[] = "io.adafruit.com";
char payload[100];
char topic[150];
char str_sensor[10];

#if defined (EnableUbidots)
PubSubClient Ubidotsclient(wificlientUbidots);
#endif
PubSubClient Adafruitioclient(wificlientAdafruitIO);
PubSubClient HomeAssistantclient(homeAssistant);

/**********************************************/
//FUNCTION DECLARATIONS
void callback(char *topic, byte *payload, unsigned int length);
void reconnect();
void _initLoRa();
void _initWiFi();
void _checkWifi_mqtt();
void AdafruitSendAll ();
void UbidotsSendAll();
void AdafruitiomqttSingle(char const *varable, char const *value);
void neopixel_clear();
void rainbow(uint8_t wait);
void theaterChase(uint32_t color, int wait);
void theaterChaseRainbow(uint8_t wait, uint8_t pixLenth);
void interruptReboot();// reboot if watchdog times out
void makeIFTTTRequest();
void ProcessTanKPacket();
float ProcessIspindlePacket();
RgbColor Wheel();
long batteryVoltageDecompress(byte batvoltage);
float temperatureDecompress(byte temperature);
unsigned int Combine2bytes(byte x_high, byte x_low);
int calculate_tank_level(int sensor1, int sensor2, int sensor3);
/*****************************************/




int tank_level = 0; //derived from multiple sensors
unsigned long LoraMessageTimer; //used to calculate time since last lora message rxed
unsigned long timeSinceLastLoraMessage;//time since last lora message rxed
DynamicJsonBuffer jsonBuffer; // JSON Buffer

struct payloadDataStruct
{
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
} rxpayload;




///*****LORA
// Set radio frequency
#define RF95_FREQ 434.0

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

static const RH_RF95::ModemConfig radiosetting = {
    BW_SETTING << 4 | CR_SETTING << 1 | ImplicitHeaderMode_SETTING,
    SF_SETTING << 4 | CRC_SETTING << 2,
    LowDataRateOptimize_SETTING << 3 | ACGAUTO_SETTING << 2};


////neopixel
const uint16_t NUM_PIXELS = 8; // How many pixels you want to drive (could be set individualy)
NeoPixelBus<NeoGrbFeature, NeoEsp32I2s1800KbpsMethod> strip(NUM_PIXELS, PIXEL_PIN1);
#define colorSaturation 128 //max255
RgbColor red(colorSaturation, 0, 0);
RgbColor redfull(255, 0, 0);
RgbColor green(0, colorSaturation, 0);
RgbColor greenlow(0, colorSaturation/4, 0);
RgbColor blue(0, 0, colorSaturation);
RgbColor white(colorSaturation/2);
RgbColor pink(colorSaturation,colorSaturation/3,colorSaturation/3);
RgbColor black(0);
RgbColor orange(colorSaturation * 0.8, colorSaturation * 0.35, 0);
RgbColor yellow(colorSaturation * 0.75, colorSaturation * 0.75, 0);


//IFTT
const char* resource = "/trigger/water_level/with/key/daSd2eiJFlysvFuIf-QZX8";
const char* server = "maker.ifttt.com";
unsigned long startMillis = 0;//used for scheduling IFTTT posts
unsigned long IFTTinterval = 60*30 ;// interval in seconds between IFTTT posts

//watchdog timer
hw_timer_t *watchdogTimer = NULL;
hw_timer_t *timerread = NULL;
int watchInt = 480000000; //set time in uS must be fed within this time or reboot


//Thermister constants and varables
#define NUMSAMPLES 10
#define ACOEFFICIENT 0.8483803702e-3
#define BCOEFFICIENT 2.572522990e-4
#define CCOEFFICIENT 1.703807621e-7
#define RESISTOR 10060  
//long ADCtherm;


/*----------------------------------------------------------------------------
Function : setup()
Description :
------------------------------------------------------------------------------*/
void setup()
{
  strip.Begin();
  rainbow(2);
  theaterChase(10000, 200);
  neopixel_clear();
  strip.SetPixelColor(0, orange); //power indicator
  strip.Show();

 //setup watchdog
  watchdogTimer = timerBegin(0, 80, true); //timer 0 divisor 80
  timerAlarmWrite(watchdogTimer, watchInt, false); // set time in uS must be fed within this time or reboot
  timerAttachInterrupt(watchdogTimer, & interruptReboot, true);
  timerAlarmEnable(watchdogTimer);  // enable interrupt

  //client2.setDebug(true); // Uncomment this line to set DEBUG on
  //set up 1HZ pwm on LED
  pinMode(BUILTIN_BLUE_LED, OUTPUT);
  ledcSetup(0, 1, 8);                 //pwm channel 0. 1Hz, 8 bit resolution
  ledcAttachPin(BUILTIN_BLUE_LED, 0); //attach LED to PWN channel 0
  ledcWrite(0, 0);                    //led off

  Serial.begin(57600);
  delay(1000);
  ///////////////////////////////////////need to reanable lora after testing
  //_initLoRa();

  rf95.setModemRegisters(&radiosetting);
  _initWiFi();

  Serial.print(" CPU");
  Serial.print(F_CPU / 1000000, DEC);
  Serial.println(F(" MHz"));
#if defined (EnableUbidots)
  Ubidotsclient.setServer(UbidotsmqttBroker, 1883);
    Ubidotsclient.setCallback(callback);
 #endif   
  Adafruitioclient.setServer(AdafruitiomqttBroker, 1883);
  HomeAssistantclient.setServer(HomeAssistantBroker,1883);
  _checkWifi_mqtt();
  LoraMessageTimer = millis(); //initialise timer (to calculate time since last lora message was received)
  delay(2000);
  AdafruitiomqttSingle("messages", "booting");

  reconnectHomeAssistBroker();
  dtostrf(2.5 , 4, 3, str_sensor); /* 4 is mininum width, 2 is precision; float value is copied onto str_sensor*/
  HAmqttSingle("/edvices/ESP32", str_sensor);
  
}

/*----------------------------------------------------------------------------
Function : loop()
Description : Main program loop
------------------------------------------------------------------------------*/
void loop()
{
  //client.publish("/v1.6/devices/esp","{\"voltage\":2660,\"rssi\":0,\"temp\":18}");
  //rf95.printRegisters(); //th
  timerWrite(watchdogTimer, 0); // reset timer feed dog
  strip.SetPixelColor(0, greenlow); //power indicator
  strip.Show();

  if (millis() - LoraMessageTimer >= 1800000)
  { //turn led on if no lora message for 30 minutes
    strip.SetPixelColor(6, blue);
  }
  else
    strip.SetPixelColor(6, black);

  if (rf95.available())
  {
    // Should be a message for us now
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);
    timeSinceLastLoraMessage = millis()- LoraMessageTimer;
    LoraMessageTimer = millis(); //reset timer (to calculate time since last lora message was received)

    if (rf95.recv(buf, &len))
    {
      delay(10);

       if (rxpayload.nodeID==1)
        {ProcessTanKPacket();}
       else if (rxpayload.nodeID==2)
        {ProcessIspindlePacket();}
       else Serial.println("NodeID not found");
       
       
      char bufChar[len];
      for (int i = 0; i < len; i++)
      {
        bufChar[i] = char(buf[i]);
        Serial.print(String(bufChar[i]));
      }

      Serial.println();
      Serial.println("Len==" + String(len));
      Serial.println("String(bufChar)==" + String(bufChar));

      memcpy(&rxpayload, buf, sizeof(rxpayload));





    }
    else
    {
      //Serial.println("Receive failed");
    }
  }
  //needed if you want to receive mqtt:
  //Ubidotsclient.loop(); //Mqtt
  //Adafruitioclient.loop(); //Mqtt
  
  if (millis() - startMillis >= IFTTinterval*1000) {//time to send IFTT ?
     startMillis=millis();
     makeIFTTTRequest();}
  delay(8000);


}

void donothing()
{
}

/*----------------------------------------------------------------------------
Function : _initWiFi()
Description : Connect to WiFi access point
------------------------------------------------------------------------------*/
void _initWiFi()
{
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.println("WiFi Connecting");
  for (int waiting = 0; waiting <= 6; waiting++)
  {
    if (WiFi.status() != WL_CONNECTED)
    {
      delay(500);
      Serial.println(" Waiting for WiFi...");
      Serial.println(waiting);
    }

    delay(1000);
  }

  if (WiFi.status() == WL_CONNECTED)
  {
    Serial.println("WiFi Connected!");
  }
}

void _checkWifi_mqtt()
{
  if (WiFi.status() != WL_CONNECTED)
  {
    _initWiFi();
    delay(1000);
  }
  strip.SetPixelColor(7, black);
  strip.Show();
#if defined (EnableUbidots)
  if (!Ubidotsclient.connected())
  {
    Serial.println("reconecting/connecting Ubidots mqtt");
    reconnect();
    if (!Ubidotsclient.connected())
    {
      strip.SetPixelColor(7, white); //turn error LED on
      strip.Show();
    }
  }
#endif
  if (!Adafruitioclient.connected())
  {
    Serial.println("reconecting/connecting Adafruit mqtt");
    reconnect();
    if (!Adafruitioclient.connected())
    {
      strip.SetPixelColor(7, pink); //turn error LED on
      strip.Show();
    }
  }
  
}


void reconnectHomeAssistBroker()
{
  // Loop a few times until we're reconnected
  int attempts =0;
  int MaxAttempts = 4;
  char text1[30];
  char text2[30];
  char text3[30];

while (!HomeAssistantclient.connected())
    {
    if (attempts > MaxAttempts)
      break;
    attempts = attempts + 1;
    Serial.println("Attempting HA MQTT connection...");

    // Attemp to connect MQTT
    if    (HomeAssistantclient.connect("esp32 therm","homeassistant",HATOKEN))// password needed??/
     {
      Serial.print("HA MQTT Connected attempt:");Serial.println(attempts -1);
      //AdafruitiomqttSingle("messages", "Ada MQTT Connected");
      //AdafruitiomqttSingle("messages", attempts);
      //sprintf(text1, "Ada Mqtt conn attempts %d", attempts -1);
      //AdafruitiomqttSingle("messages", text1);
     }
    else
     {
      Serial.print("Failed, rc=");
      Serial.print(HomeAssistantclient.state());
      Serial.println(" HA Mqtt try again in 2 seconds");
      // Wait  before retrying
      delay(2000*attempts);
     }
    }
}
void reconnect()
{
  // Loop a few times until we're reconnected
  int attempts =0;
  int MaxAttempts = 4;
  char text1[30];
  char text2[30];
  char text3[30];

while (!Adafruitioclient.connected())
    {
    if (attempts > MaxAttempts)
      break;
    attempts = attempts + 1;
    Serial.println("Attempting Ada MQTT connection...");

    // Attemp to connect MQTT
    if    (Adafruitioclient.connect(clientId.c_str(), ADAFRUIT_MQTT_CLIENT_NAME, ADAFRUITIOTOKEN))
     {
      Serial.print("Ada Connected attempt:");Serial.println(attempts -1);
      //AdafruitiomqttSingle("messages", "Ada MQTT Connected");
      //AdafruitiomqttSingle("messages", attempts);
      //sprintf(text1, "Ada Mqtt conn attempts %d", attempts -1);
      //AdafruitiomqttSingle("messages", text1);
     }
    else
     {
      Serial.print("Failed, rc=");
      Serial.print(Adafruitioclient.state());
      Serial.println(" Ada Mqtt try again in 2 seconds");
      // Wait  before retrying
      delay(2000*attempts);
     }
    }
  
  
#if defined (EnableUbidots)
  attempts = 1;

  while (!Ubidotsclient.connected())
  {
    if (attempts > MaxAttempts)
      break;
    attempts = attempts + 1;
    Serial.println("Attempting Ubidots MQTT connection...");

    // Attemp to connect MQTT
    if (Ubidotsclient.connect(MQTT_CLIENT_NAME, UBIDOTSTOKEN, ""))
    {
      Serial.println("Ubidots Connected");
      AdafruitiomqttSingle("messages", "Ubidots MQTT Connected");
      sprintf(text2, "Ubi Mqtt connect attempts %d", attempts - 1);
        }
    else
    {
      Serial.print("Failed, rc=");
      Serial.print(Ubidotsclient.state());

      Serial.println(" Ubidots try again in 2 seconds");
      // Wait 2 seconds before retrying
      delay(2000);
    }
  }
  sprintf(text3, "Ubi Mqtt state %d", Ubidotsclient.state() );
  AdafruitiomqttSingle("messages", text3);
#endif

}


/*----------------------------------------------------------------------------
Function : _initLoRa()
Description : Connect to WiFi access point
------------------------------------------------------------------------------*/
void _initLoRa()
{
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

  while (!rf95.init())
  {
    Serial.println("LoRa radio init failed");
    //while (1);//th
  }
  Serial.println("LoRa radio init OK!");
  delay(1000);

  if (!rf95.setFrequency(RF95_FREQ))
  {
    Serial.println("setFrequency failed");
    while (1)
      ;
  }
  Serial.println("Freq set");
  delay(1000);

  rf95.setTxPower(17, false);
  rf95.setModemConfig(RH_RF95::Bw125Cr48Sf4096); //
  delay(10);
  rf95.setModemRegisters(&radiosetting); //this is where we apply our custom settings
  rf95.printRegisters();
  Serial.println("LoRa Listening...");
  delay(1000);
}

long batteryVoltageDecompress(byte batvoltage)
{
  //decompress voltage from 1 byte
  long result2;
  result2 = (batvoltage * 8) + 1300;
  return (result2);
}

float temperatureDecompress(byte temperature)
{
  //decomcompress temperature from 1 byte
  // allowable temperature range is 0 to 51 degree C
  float result2;
  result2 = ((float)temperature) / 5;
  //result2=(float)temperature;
  return result2;
}

unsigned int Combine2bytes(byte x_high, byte x_low)
//convert high and low bytes to unsigned int (0-65,535 (2^16) - 1)
{
  unsigned int combined;
  if ((x_high >= 256) & (x_low >= 256))
    combined = 0; ///out of range
  combined = x_low | x_high << 8;
  return combined;
}

void callback(char *topic, byte *payload, unsigned int length)
{
  char p[length + 1];
  memcpy(p, payload, length);
  p[length] = NULL;
  String message(p);
  Serial.write(payload, length);
  Serial.println(topic);
}


void ubidotsmqttSingle(char const *varable, char const *value)
{
  //format topic and payload, then publish single data point
  sprintf(topic, "%s%s", "/v1.6/devices/", DEVICE_LABEL);

  sprintf(payload, "%s", "");                                   // Cleans the payload
  sprintf(payload, "%s {\"%s\": %s}", payload, varable, value); // Adds the value
  Serial.print("Publishing data to Ubidots Cloud");
  Serial.print(" topic= ");
  Serial.println(topic);
  Serial.println(" payload= ");
  Serial.println(payload);
#if defined (EnableUbidots)
  Ubidotsclient.publish(topic, payload); //eg client.publish(/v1.6/devices/esp,{"MQTTsensor": {"value": 3.10}})
#endif
  //example client.publish(topic, "{\"temperature\": 10, \"humidity\": 50}");
}

void ubidotsmqttTriple(char *varable1, int value1, char *varable2, char *value2, char *varable3, char *value3)
{
  //format topic and payload, then publish single data point
  sprintf(topic, "%s%s", "/v1.6/devices/", DEVICE_LABEL);
  char str_value1[10];
  char str_value2[10];
  char str_value3[10];
  dtostrf(value1, 4, 2, str_value1);

  sprintf(payload, "%s", ""); // Cleans the payload
  //sprintf(payload, "%s {\"%s\": %s}", payload, varable, value);// Adds the value
  sprintf(payload, "%s {\"%s\": %s\", \"%s\": %s, \"%s\": %s}", payload, varable1, value1, varable2, value2, varable3, value3); // Adds the value
  Serial.print("Publishing data to Ubidots Cloud");
  Serial.print(" topic= ");
  Serial.println(topic);
  Serial.println(" payload= ");
  Serial.println(payload);
  #if defined (EnableUbidots)
  Ubidotsclient.publish(topic, payload); //eg client.publish(/v1.6/devices/esp,{"MQTTsensor": {"value": 3.10}})
  #endif
  //example client.publish(topic, "{\"temperature\": 10, \"humidity\": 50}");
}

void ubidotsmqttJson(char *varable1, int value1, char *varable2, int value2, char *varable3, int value3)
{

  StaticJsonBuffer<100> jsonBuffer;
  char JSONmessageBuffer[100];
  JsonObject &root = jsonBuffer.createObject();

  root[varable1] = value1;
  root[varable2] = value2;
  root[varable3] = value3;

  root.printTo(JSONmessageBuffer, sizeof(JSONmessageBuffer));
  Serial.println(JSONmessageBuffer);
  sprintf(topic, "%s%s", "/v1.6/devices/", DEVICE_LABEL);
  Serial.print("Publishing data to Ubidots Cloud");
  Serial.print(" topic= ");
  Serial.println(topic);
  Serial.println(" payload= ");
  Serial.println(JSONmessageBuffer);
  #if defined (EnableUbidots)
  Ubidotsclient.publish(topic, JSONmessageBuffer);
  #endif
}
void AdafruitiomqttSingle(char const *varable, char const *value)
{
  //format topic and payload, then publish single data point
  sprintf(topic,"distiller/feeds/%s", varable);
  Serial.print("Publishing data to ada Cloud");
  Serial.print(" topic= ");
  Serial.println(topic);
  Serial.println(" payload= ");
  Serial.println(value);
  Adafruitioclient.publish(topic, value); 
  delay(2500);
}

void HAmqttSingle(char const *varable, char const *value)
{
  //format topic and payload, then publish single data point
  sprintf(topic,"%s", varable);
  Serial.print("Publishing data to HA");
  Serial.print(" topic= ");
  Serial.println(topic);
  Serial.println(" payload= ");
  Serial.println(value);
  HomeAssistantclient.publish(topic, value, true); 
  delay(2500);
}


void UbidotsSendAll(){

    float sensor = temperatureDecompress(rxpayload.temperature);
      
          dtostrf(sensor, 4, 3, str_sensor); /* 4 is mininum width, 2 is precision; float value is copied onto str_sensor*/
      ubidotsmqttSingle("temp", str_sensor);
      delay(1000);
      ubidotsmqttJson("local-rssi", (int)rf95.lastRssi(),
                      "rssi", -rxpayload.rssi, "voltage", batteryVoltageDecompress(rxpayload.voltage));

      delay(3000);
      ubidotsmqttJson("level", (int)Combine2bytes(rxpayload.capsensor1Highbyte, rxpayload.capsensor1Lowbyte),
                      "level-2", (int)Combine2bytes(rxpayload.capsensor2Highbyte, rxpayload.capsensor2Lowbyte), "level-3", (int)Combine2bytes(rxpayload.capsensor3Highbyte, rxpayload.capsensor3Lowbyte));

      delay(1000);
      sensor = rf95.frequencyError();
      dtostrf(sensor, 4, 3, str_sensor); /* 4 is mininum width, 2 is precision; float value is copied onto str_sensor*/
      ubidotsmqttSingle("frequency-error", str_sensor);

      dtostrf(tank_level, 4, 3, str_sensor); /* 4 is mininum width, 2 is precision; float value is copied onto str_sensor*/
      ubidotsmqttSingle("CalcLevel", str_sensor);
      #if defined (EnableUbidots)
      Ubidotsclient.disconnect();
      #endif
      
}

void AdafruitSendAll ()
{
dtostrf((int)Combine2bytes(rxpayload.capsensor3Highbyte, rxpayload.capsensor3Lowbyte), 3, 0, str_sensor); /* 3 is mininum width, 0 is precision; float value is copied onto str_sensor*/
AdafruitiomqttSingle("level-3", str_sensor);

dtostrf((int)Combine2bytes(rxpayload.capsensor2Highbyte, rxpayload.capsensor2Lowbyte), 3, 0, str_sensor); 
AdafruitiomqttSingle("level-2", str_sensor);

dtostrf((int)Combine2bytes(rxpayload.capsensor1Highbyte, rxpayload.capsensor1Lowbyte), 3, 0, str_sensor); 
AdafruitiomqttSingle("level-1", str_sensor);

float sensor2 = temperatureDecompress(rxpayload.temperature);
dtostrf(sensor2, 4, 3, str_sensor); 
AdafruitiomqttSingle("temperature", str_sensor);

dtostrf(tank_level, 3, 0, str_sensor); 
AdafruitiomqttSingle("tank_level", str_sensor);

dtostrf((int)rf95.lastRssi(), 4, 0, str_sensor);
AdafruitiomqttSingle("rx-RSSI", str_sensor);

dtostrf(batteryVoltageDecompress(rxpayload.voltage), 4, 0, str_sensor);
AdafruitiomqttSingle("voltage", str_sensor);

//dtostrf(timeSinceLastLoraMessage/1000, 4, 3, str_sensor);
//AdafruitiomqttSingle("messages", str_sensor);
delay(100);
Adafruitioclient.disconnect();


}

void rainbow(uint8_t wait)
{
  uint16_t i, j;

  for (j = 0; j < 256; j++)
  {
    for (i = 0; i < NUM_PIXELS; i++)
    {
      strip.SetPixelColor(i, Wheel((i + j) & 255));
    }
    strip.Show();
    delay(wait);
  }
}


void theaterChase(uint32_t color, int wait) {
  for(int a=0; a<10; a++) {  // Repeat 10 times...
    for(int b=0; b<3; b++) { //  'b' counts from 0 to 2...
      neopixel_clear();         //   Set all pixels in RAM to 0 (off)
      // 'c' counts up from 'b' to end of strip in steps of 3...
      for(int c=b; c< NUM_PIXELS; c += 3) {
        strip.SetPixelColor(c, color); // Set pixel 'c' to value 'color'
      }
      strip.Show(); // Update strip with new contents
      delay(wait);  // Pause for a moment
    }
  }
}


//Theatre-style crawling lights with rainbow effect
void theaterChaseRainbow(uint8_t wait, uint8_t pixLenth)
{
  for (int j = 0; j < 256; j++)
  { // cycle all 256 colors in the wheel
    for (int q = 0; q < 3; q++)
    {
      for (int i = 0; i < pixLenth; i = i + 3)
      {
        strip.SetPixelColor(i + q, Wheel((i + j) % 255)); //turn every third pixel on
      }
      strip.Show();

      delay(wait);

      for (int i = 0; i < pixLenth; i = i + 3)
      {
        strip.SetPixelColor(i + q, 0); //turn every third pixel off
      }
    }
  }
}

// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
RgbColor Wheel(uint8_t WheelPos)
{
  WheelPos = 255 - WheelPos;
  if (WheelPos < 85)
  {
    return RgbColor(255 - WheelPos * 3, 0, WheelPos * 3);
  }
  else if (WheelPos < 170)
  {
    WheelPos -= 85;
    return RgbColor(0, WheelPos * 3, 255 - WheelPos * 3);
  }
  else
  {
    WheelPos -= 170;
    return RgbColor(WheelPos * 3, 255 - WheelPos * 3, 0);
  }
}

int calculate_tank_level(int sensor1, int sensor2, int sensor3)
{
  //based on the three sensor readings estimate the level
  //valid outputs 100, 80, 50, 20
  //output on error is 0

  const int threshold1 = 250;
  const int threshold2 = 300;
  const int threshold3 = 240;

  int calculated_level = 0;

  if (sensor3 > threshold3 && sensor2 > threshold2 && sensor1 > threshold1)
    calculated_level = 100;
  if (sensor3 < threshold3 && sensor2 > threshold2 && sensor1 > threshold1)
    calculated_level = 80;
  if (sensor3 < threshold3 && sensor2 < threshold2 && sensor1 > threshold1)
    calculated_level = 50;
  if (sensor3 < threshold3 && sensor2 < threshold2 && sensor1 < threshold1)
    calculated_level = 20;

  return calculated_level;
}

void neopixel_clear()
{
  for (uint16_t i = 0; i < NUM_PIXELS; i++)
  {
    strip.SetPixelColor(i, black);
  }
  strip.Show();
}


void interruptReboot()
{
  Serial.print("Rebooting .. \n\n");
  //esp_restart_noos();removed jan 2021 because command deprecated
   esp_restart();// also see https://iotassistant.io/esp32/enable-hardware-watchdog-timer-esp32-arduino-ide/ and https://github.com/espressif/arduino-esp32/issues/2304
}

// Make an HTTP request to the IFTTT web service
void makeIFTTTRequest() {
  Serial.print("Connecting to "); 
  Serial.print(server);
  
  WiFiClient client;
  int retries = 5;
  while(!!!client.connect(server, 80) && (retries-- > 0)) {
    Serial.print(".");
  }
  Serial.println();
  if(!!!client.connected()) {
    Serial.println("Failed to connect...");
  }
  
  Serial.print("Request resource: "); 
  Serial.println(resource);

  // get capacitance
  unsigned int valonepoint1 = Combine2bytes(rxpayload.capsensor1Highbyte, rxpayload.capsensor1Lowbyte);
  unsigned int valonepoint2 = Combine2bytes(rxpayload.capsensor2Highbyte, rxpayload.capsensor2Lowbyte);
  unsigned int valonepoint3 = Combine2bytes(rxpayload.capsensor3Highbyte, rxpayload.capsensor3Lowbyte);
  
  
  String jsonObject = String("{\"value1\":\"") + valonepoint1 + ", " + valonepoint2 + ", " + valonepoint3 +  "\",\"value2\":\"" + ESP.getFreeHeap()+ ", " + millis()/1000
                      + "\",\"value3\":\"" + batteryVoltageDecompress(rxpayload.voltage) + ", " + temperatureDecompress(rxpayload.temperature) + "\"}";
  Serial.println(jsonObject);
                      
  // Comment the previous line and uncomment the next line to publish temperature readings in Fahrenheit                    
  /*String jsonObject = String("{\"value1\":\"") + (1.8 * bme.readTemperature() + 32) + "\",\"value2\":\"" 
                      + (bme.readPressure()/100.0F) + "\",\"value3\":\"" + bme.readHumidity() + "\"}";*/
                      
  client.println(String("POST ") + resource + " HTTP/1.1");
  client.println(String("Host: ") + server); 
  client.println("Connection: close\r\nContent-Type: application/json");
  client.print("Content-Length: ");
  client.println(jsonObject.length());
  client.println();
  client.println(jsonObject);
        
  int timeout = 5 * 10; // 5 seconds             
  while(!!!client.available() && (timeout-- > 0)){
    delay(100);
  }
  if(!!!client.available()) {
    Serial.println("No response...");
  }
  while(client.available()){
    Serial.write(client.read());
  }
  
  Serial.println("\nclosing connection");
  client.stop(); 
}

void ProcessTanKPacket(){
      tank_level = calculate_tank_level((int)Combine2bytes(rxpayload.capsensor1Highbyte, rxpayload.capsensor1Lowbyte), (int)Combine2bytes(rxpayload.capsensor2Highbyte, rxpayload.capsensor2Lowbyte), (int)Combine2bytes(rxpayload.capsensor3Highbyte, rxpayload.capsensor3Lowbyte));

      //rxpayload.voltage=batteryVoltageDecompress(rxpayload.voltage);
      Serial.print(" nodeID = ");
      Serial.print(rxpayload.nodeID);
      Serial.print(" remote voltage comp = ");
      Serial.print((rxpayload.voltage));
      Serial.print(" remote voltage = ");
      Serial.print(batteryVoltageDecompress(rxpayload.voltage));
      Serial.print(" remote rssi = ");
      Serial.print(rxpayload.rssi);
      Serial.print(" Local RSSI: ");
      Serial.print(rf95.lastRssi(), DEC);
      Serial.print(" Local SNR: ");
      Serial.print(rf95.lastSNR(), DEC);
      Serial.print(" cap1= ");
      Serial.print(Combine2bytes(rxpayload.capsensor1Highbyte, rxpayload.capsensor1Lowbyte));
      Serial.print(" cap2= ");
      Serial.print(Combine2bytes(rxpayload.capsensor2Highbyte, rxpayload.capsensor2Lowbyte));
      Serial.print(" cap3= ");
      Serial.print(Combine2bytes(rxpayload.capsensor3Highbyte, rxpayload.capsensor3Lowbyte));
      Serial.print(" tank level= ");
      Serial.print(tank_level);
      ///////////////////////MQTT Code
      //sendMessage(String(bufChar));

      //set LEDs based on tank level
      if (tank_level <= 1)
      {
        //theaterChaseRainbow(30, 6);
        strip.SetPixelColor(0, greenlow); //power indicator
        strip.SetPixelColor(1, redfull);
        strip.SetPixelColor(2, redfull);
        strip.SetPixelColor(3, redfull);
        strip.Show();

        ledcWrite(0, 128);
      } //builtin LED flash (pwm channel 0)
      else if (tank_level <= 50)
      {
        //theaterChaseRainbow(10,6);
        ledcWrite(0, 128); //LED flash (pwm channel 0)
        strip.SetPixelColor(0, greenlow); //power indicator
        strip.SetPixelColor(1, orange);
        strip.SetPixelColor(2, orange);
        strip.SetPixelColor(3, orange);
        strip.Show();
      }
      else if (tank_level <= 80)
      {
        //theaterChaseRainbow(2,6);
        ledcWrite(0, 128); //LED off (pwm channel 0)
        strip.SetPixelColor(0, greenlow); //power indicator
        strip.SetPixelColor(1, yellow);
        strip.SetPixelColor(2, yellow);
        strip.SetPixelColor(3, yellow);
        strip.Show();
      }
      else
      {
        ledcWrite(0, 0); //LED off (pwm channel 0)
        strip.SetPixelColor(0, greenlow); //power indicator
        strip.SetPixelColor(1, black);
        strip.SetPixelColor(2, black);
        strip.SetPixelColor(3, black);
        strip.Show();
      }

      // Send a reply to sensor
      //uint8_t outgoingData[] = "{\"Status\" : \"Ack\"}";
      uint8_t ack[1];
      // the acknolement consists of an ack identiferer followed by nodeID
      ack[0] = (uint8_t)170; //ack identifier 170=10101010
      ack[1] = (uint8_t)rxpayload.nodeID;
      rf95.send(ack, 2);
      //rf95.send(outgoingData, sizeof(outgoingData));
      rf95.waitPacketSent();

      //MQTT SEND/////////////////////////////////
      _checkWifi_mqtt();
      
      AdafruitSendAll ();
      //wificlientAdafruitIO.stop();
      delay(2000);
      _checkWifi_mqtt();
      #if defined (EnableUbidots)
      UbidotsSendAll();      
      Ubidotsclient.loop(); //Mqtt
      #endif
      delay(1000);
      Adafruitioclient.loop(); //Mqtt
      delay(1000);
      }


      float ProcessIspindlePacket(){
    int TemperatureADC = 0;

float lnR;
float ThermResistance;
float ThermTemperature;
//float resistor = RESISTOR;
delay(1);

 TemperatureADC = (int)Combine2bytes(rxpayload.capsensor1Highbyte, rxpayload.capsensor1Lowbyte);
    Serial.print("TemperatureADC "); Serial.println(TemperatureADC);
  


 ThermResistance = (1023/ TemperatureADC) -1;
 ThermResistance = RESISTOR / ThermResistance;
 lnR = log(ThermResistance);
 ThermTemperature = 1 / (ACOEFFICIENT + BCOEFFICIENT * lnR + CCOEFFICIENT * lnR* lnR *lnR) - 273.15;
return ThermTemperature;


      }