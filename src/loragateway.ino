// 13 april 2023
//works!!!

/////LEDs
//0 power LEDalways green
//1+2+3 tank full->off, tank full->80% yellow ,tank 50% full->orange, tank 40% full->red
//6 blue is no recieving lora messages
//7 white if wifi/mqtt loss of connection
//7 pink if adafruit wifi/mqtt loss of connection
/////////


#include <Arduino.h>
#include <esp_task_wdt.h> // https://iotassistant.io/esp32/enable-hardware-watchdog-timer-esp32-arduino-ide/
#include <WiFi.h>
#include <PubSubClient.h>
#include <RH_RF95.h> //ver 1.89
#include "RadioSettings.h"
#include <ArduinoJson.h> // careful https://arduinojson.org/v6/doc/upgrade/
#include <NeoPixelBus.h> //https://github.com/Makuna/NeoPixelBus/
#include "RunningMedian.h"
#include <AsyncTCP.h>//ota https://randomnerdtutorials.com/esp32-ota-over-the-air-vs-code/
#include <ESPAsyncWebServer.h>//ota
#include <AsyncElegantOTA.h>//ota
#include "AsyncJson.h"//for web server
#include <WebSerial.h>

//PINS
#define RFM95_CS 18
#define RFM95_RST 14
#define RFM95_INT 26
#define PIXEL_PIN1 21
#define BUILTIN_BLUE_LED 2


// WiFi access credentials
//#define WIFI_SSID "Penryn" // WiFi SSID
#define WIFI_SSID "Starlink" // WiFi SSID
#define WIFI_PASSWORD "hoooverr" // WiFI Password
WiFiClient wificlientAdafruitIO;
WiFiClient homeAssistant;

/****************************************
 * MQTT
 ****************************************/

char HomeAssistantBroker[] = "192.168.1.198";
void adfruitiomqttSingle(char
  const * varable, char
  const * value);
void adfruitiomqttJson(char * varable1, int value1, char * varable2, int value2, char * varable3, int value3);
void reconnectHomeAssistBroker();
void HAmqttSingle(char
  const * varable, char
  const * value);
String clientId = "ESP32Client-cc5246fdcz";
#define HATOKEN "koh5eixie2pae8vah5ton1saay7AQuaph4aejoaLae5toh6Bahx6aePheiseiMiP"
#define ADAFRUIT_MQTT_CLIENT_NAME "distiller" // 
#define ADAFRUITIOTOKEN "e3758c24375742ccb361c88e38d638f3" // ada TOKEN
char AdafruitiomqttBroker[] = "io.adafruit.com";
char payload[100];
char topic[150];
char str_sensor[10];
char string1[35] = "";//web
char string2[10];//web

PubSubClient Adafruitioclient(wificlientAdafruitIO);
PubSubClient HomeAssistantclient(homeAssistant);

/**********************************************/
//FUNCTION DECLARATIONS
void callback(char * topic, byte * payload, unsigned int length);
void reconnect();
void _initLoRa();
void _initWiFi();
void _checkWifi_mqtt();
void AdafruitSendAll();
void AdafruitiomqttSingle(char
  const * varable, char
  const * value);
void neopixel_clear();
void rainbow(uint8_t wait);
void theaterChase(uint32_t color, int wait);
void theaterChaseRainbow(uint8_t wait, uint8_t pixLenth);
void ProcessTanKPacket();
float ProcessIspindlePacket();
RgbColor Wheel(byte WheelPos);
long batteryVoltageDecompress(byte batvoltage);
float temperatureDecompress(byte temperature);
unsigned int Combine2bytes(byte x_high, byte x_low);
int calculate_tank_level(int sensor1, int sensor2, int sensor3);
void verbose_print_reset_reason(int reason);
void set_fermentor_temp_led(float temperature);

/*****************************************/
//VARABLES
int tank_level = 0; //derived from multiple sensors	
unsigned long LoraMessageTimer; //used to calculate time since last lora message rxed	
unsigned long timeSinceLastNode1LoraMessage; //time since last lora message rxed	
float Fermenter_temperature = 1.11;
DynamicJsonBuffer jsonBuffer; // JSON Buffer	
struct payloadDataStruct {	
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
}
rxpayload;

///*****LORA
// Set radio frequency
#define RF95_FREQ 434.0

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

static
const RH_RF95::ModemConfig radiosetting = {
  BW_SETTING << 4 | CR_SETTING << 1 | ImplicitHeaderMode_SETTING,
  SF_SETTING << 4 | CRC_SETTING << 2,
  LowDataRateOptimize_SETTING << 3 | ACGAUTO_SETTING << 2
};

////neopixel
const uint16_t NUM_PIXELS = 8; // How many pixels you want to drive (could be set individualy)
NeoPixelBus < NeoGrbFeature, NeoEsp32I2s1800KbpsMethod > strip(NUM_PIXELS, PIXEL_PIN1);
#define colorSaturation 128 //max255
RgbColor red(colorSaturation, 0, 0);	
RgbColor redfull(255, 0, 0);	
RgbColor green(0, colorSaturation, 0);	
RgbColor greenlow(0, colorSaturation / 4, 0);	
RgbColor blue(0, 0, colorSaturation);	
RgbColor white(colorSaturation / 2);	
RgbColor pink(colorSaturation, colorSaturation / 3, colorSaturation / 3);	
RgbColor black(0);	
RgbColor orange(colorSaturation * 0.8, colorSaturation * 0.35, 0);	
RgbColor yellow(colorSaturation * 0.75, colorSaturation * 0.75, 0);	


//Thermister constants and varables
#define NUMSAMPLES 10
#define ACOEFFICIENT 0.8483803702e-3
#define BCOEFFICIENT 2.572522990e-4
#define CCOEFFICIENT 1.703807621e-7
#define RESISTOR 10060

//Ispindle Tilt
RunningMedian TiltSamples = RunningMedian(4);

//ota
AsyncWebServer server2(80);

//Print last reset reason of ESP32
#ifdef ESP_IDF_VERSION_MAJOR // IDF 4+
#if CONFIG_IDF_TARGET_ESP32 // ESP32/PICO-D4
#include "esp32/rom/rtc.h"
#else 
#error Target CONFIG_IDF_TARGET is not supported
#endif
#else // ESP32 Before IDF 4.0
#include "rom/rtc.h"
#endif
#define uS_TO_S_FACTOR 1000000  /* Conversion factor for micro seconds to seconds */


/*----------------------------------------------------------------------------
Function : setup()
Description :
------------------------------------------------------------------------------*/
void setup() {
  //setup watchdog
  esp_task_wdt_init(1200, true); //enable panic so ESP32 restarts, timeout 20 minutes
  esp_task_wdt_add(NULL); //add current thread to WDT watch
  
  delay(400);
  strip.Begin();
  rainbow(2);
  theaterChase(10000, 200);
  neopixel_clear();
  strip.SetPixelColor(0, orange); //power indicator
  strip.Show();
  
  //client2.setDebug(true); // Uncomment this line to set DEBUG on
  //set up 1HZ pwm on LED
  pinMode(BUILTIN_BLUE_LED, OUTPUT);
  ledcSetup(0, 5000, 8); //pwm channel 0. 5000Hz, 8 bit resolution
  ledcAttachPin(BUILTIN_BLUE_LED, 0); //attach LED to PWN channel 0
  ledcWrite(0, 0); //led off

  Serial.begin(115200);
  delay(1000);
  WebSerial.begin(&server2);
  Serial.println("CPU0 reset reason:");
  verbose_print_reset_reason(rtc_get_reset_reason(0));
  Serial.println("CPU1 reset reason:");
  verbose_print_reset_reason(rtc_get_reset_reason(1));

  _initLoRa();
  rf95.setModemRegisters( & radiosetting);
  _initWiFi();

  Serial.print(" CPU");
  Serial.print(F_CPU / 1000000, DEC);
  Serial.println(F(" MHz"));
  Adafruitioclient.setServer(AdafruitiomqttBroker, 1883);
  HomeAssistantclient.setServer(HomeAssistantBroker, 1883);
  _checkWifi_mqtt();
  LoraMessageTimer = millis(); //initialise timer (to calculate time since last lora message was received)
  delay(2000);

  dtostrf(rtc_get_reset_reason(0), 4, 0, str_sensor);
  AdafruitiomqttSingle("messages", "booting");
  AdafruitiomqttSingle("messages", str_sensor);

  reconnectHomeAssistBroker();

//OTA
AsyncElegantOTA.begin(&server2);// Start ElegantOTA
server2.begin();
Serial.println("HTTP server2 started");


server2.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/plain", string1);//string1 is set in main loop
  });

delay(2000);
}

/*----------------------------------------------------------------------------
Function : loop()
Description : Main program loop
------------------------------------------------------------------------------*/
void loop() {

  //client.publish("/v1.6/devices/esp","{\"voltage\":2660,\"rssi\":0,\"temp\":18}");
  esp_task_wdt_reset();// reset timer feed dog
  strip.SetPixelColor(0, greenlow); //power indicator
  strip.Show();
  if (millis() - LoraMessageTimer >= 1800000) { //turn led on if no lora Node1 message for 30 minutes
    strip.SetPixelColor(6, blue);
  } else
    strip.SetPixelColor(6, black);

  if (rf95.available()) {
    // Should be a message for us now
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);

    if (rf95.recv(buf, & len)) {
      delay(10);
      char bufChar[len];
      for (int i = 0; i < len; i++) {
        bufChar[i] = char(buf[i]);
        Serial.print(String(bufChar[i]));
      }

      Serial.println();
      Serial.println("Len==" + String(len));
      Serial.println("String(bufChar)==" + String(bufChar));

      memcpy( & rxpayload, buf, sizeof(rxpayload));
      Serial.println("packet rx with nodeid ");Serial.println(rxpayload.nodeID);
      
      if (rxpayload.nodeID == 1) {
        ProcessTanKPacket();
      } else if (rxpayload.nodeID == 2) {
        Fermenter_temperature = ProcessIspindlePacket();
      } else Serial.println("NodeID not found");

    } else {
      //Serial.println("Receive failed");
    }
  }
  //needed if you want to receive mqtt:

  //Adafruitioclient.loop(); //Mqtt
  
  //web
  dtostrf(Fermenter_temperature, 4, 2, string1); /* 4 is mininum width, 2 is precision; float value is copied onto str_sensor*/
  dtostrf(esp_timer_get_time()/60000000, 9, 0, string2);//uptime in minutes
  strcat(string1, " <--temp   uptime--> ");
  strcat(string1, string2);
  //WebSerial.println(string1);
 
  
  delay(8000);

}

void donothing() {}

/*----------------------------------------------------------------------------
Function : _initWiFi()
Description : Connect to WiFi access point
------------------------------------------------------------------------------*/
void _initWiFi() {

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
 
  Serial.println("WiFi Connecting");
  for (int waiting = 0; waiting <= 8; waiting++) {
    
    if (WiFi.waitForConnectResult() != WL_CONNECTED) {


    //if (WiFi.status() != WL_CONNECTED) {
      delay(1000);
      Serial.print(" Waiting for WiFi...");
         Serial.print(WiFi.status());Serial.print(" ");
      Serial.println(waiting);
    }

    delay(1000);
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("WiFi Connected!");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  }
}

void _checkWifi_mqtt() {
  if (WiFi.status() != WL_CONNECTED) {
    _initWiFi();
    delay(1000);
  }
  strip.SetPixelColor(7, black);
  strip.Show();
 
  if (!Adafruitioclient.connected()) {
    Serial.println("reconecting/connecting Adafruit mqtt");
    reconnect();
    if (!Adafruitioclient.connected()) {
      strip.SetPixelColor(7, pink); //turn error LED on
      strip.Show();
    }
  }

}

void reconnectHomeAssistBroker() {
  // Loop a few times until we're reconnected
  int attempts = 1;
  int MaxAttempts = 2;
  char text1[30];
  char text2[30];
  char text3[30];

  while (!HomeAssistantclient.connected()) {
    if (attempts > MaxAttempts)
      break;
    attempts = attempts + 1;
    Serial.println("Attempting HA MQTT connection...");

    // Attemp to connect MQTT
    if (HomeAssistantclient.connect("esp32 therm", "homeassistant", HATOKEN)) // password needed??/
    {
      Serial.print("HA MQTT Connected attempt:");
      Serial.println(attempts - 1);
      //AdafruitiomqttSingle("messages", "Ada MQTT Connected");
      //AdafruitiomqttSingle("messages", attempts);
      //sprintf(text1, "Ada Mqtt conn attempts %d", attempts -1);
      //AdafruitiomqttSingle("messages", text1);
    } else {
      Serial.print("Failed, rc=");
      Serial.print(HomeAssistantclient.state());
      Serial.println(" HA Mqtt try again in 0.5 seconds");
      // Wait  before retrying
      delay(500);
    }
  }
}
void reconnect() {
  // Loop a few times until we're reconnected
  int attempts = 1;
  int MaxAttempts = 3;
  char text1[30];
  char text2[30];
  char text3[30];

  while (!Adafruitioclient.connected()) {
    if (attempts > MaxAttempts)
      break;
    attempts = attempts + 1;
    Serial.println("Attempting Ada MQTT connection...");

    // Attemp to connect MQTT
    if (Adafruitioclient.connect(clientId.c_str(), ADAFRUIT_MQTT_CLIENT_NAME, ADAFRUITIOTOKEN)) {
      Serial.print("Ada Connected attempt:");
      Serial.println(attempts - 1);
      //AdafruitiomqttSingle("messages", "Ada MQTT Connected");
      //AdafruitiomqttSingle("messages", attempts);
      //sprintf(text1, "Ada Mqtt conn attempts %d", attempts -1);
      //AdafruitiomqttSingle("messages", text1);
    } else {
      Serial.print("Failed, rc=");
      Serial.print(Adafruitioclient.state());
      Serial.println(" Ada Mqtt try again in 2 seconds");
      // Wait  before retrying
      delay(2000 * attempts);
    }
  }



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
    while (1)
    ;
  }
  Serial.println("Freq set");
  delay(1000);

  rf95.setTxPower(17, false);
  rf95.setModemConfig(RH_RF95::Bw125Cr48Sf4096); //
  delay(10);
  rf95.setModemRegisters( & radiosetting); //this is where we apply our custom settings
  rf95.printRegisters();
  Serial.println("LoRa Listening...");
  delay(1000);
}

long batteryVoltageDecompress(byte batvoltage) {
  //decompress voltage from 1 byte
  long result2;
  result2 = (batvoltage * 8) + 1300;
  return (result2);
}

float temperatureDecompress(byte temperature) {
  //decomcompress temperature from 1 byte
  // allowable temperature range is 0 to 51 degree C
  float result2;
  result2 = ((float) temperature) / 5;
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

void callback(char * topic, byte * payload, unsigned int length) {
  char p[length + 1];
  memcpy(p, payload, length);
  p[length] = NULL;
  String message(p);
  Serial.write(payload, length);
  Serial.println(topic);
}


void AdafruitiomqttSingle(char
  const * varable, char
  const * value) {
  //format topic and payload, then publish single data point
  sprintf(topic, "distiller/feeds/%s", varable);
  Serial.print("Publishing data to ada Cloud");
  Serial.print(" topic= ");
  Serial.println(topic);
  Serial.println(" payload= ");
  Serial.println(value);
  Adafruitioclient.publish(topic, value);
  delay(500);//was 2500
}

void HAmqttSingle(char
  const * varable, char
  const * value) {
  //format topic and payload, then publish single data point
  sprintf(topic, "%s", varable);
  Serial.print("Publishing data to HA");
  Serial.print(" topic= ");
  Serial.println(topic);
  Serial.println(" payload= ");
  Serial.println(value);
  reconnectHomeAssistBroker();
  if (HomeAssistantclient.connected()) {
    HomeAssistantclient.publish(topic, value, true);
    delay(200);}//was 2500
}



void AdafruitSendAll() {
  dtostrf((int) Combine2bytes(rxpayload.capsensor3Highbyte, rxpayload.capsensor3Lowbyte), 3, 0, str_sensor); /* 3 is mininum width, 0 is precision; float value is copied onto str_sensor*/
  AdafruitiomqttSingle("level-3", str_sensor);

  dtostrf((int) Combine2bytes(rxpayload.capsensor2Highbyte, rxpayload.capsensor2Lowbyte), 3, 0, str_sensor);
  AdafruitiomqttSingle("level-2", str_sensor);

  dtostrf((int) Combine2bytes(rxpayload.capsensor1Highbyte, rxpayload.capsensor1Lowbyte), 3, 0, str_sensor);
  AdafruitiomqttSingle("level-1", str_sensor);

  float sensor2 = temperatureDecompress(rxpayload.temperature);
  dtostrf(sensor2, 4, 3, str_sensor);
  AdafruitiomqttSingle("temperature", str_sensor);

  dtostrf(tank_level, 3, 0, str_sensor);
  AdafruitiomqttSingle("tank_level", str_sensor);

  //dtostrf((int) rf95.lastRssi(), 4, 0, str_sensor);
  //AdafruitiomqttSingle("rx-RSSI", str_sensor);

  dtostrf(batteryVoltageDecompress(rxpayload.voltage), 4, 0, str_sensor);
  AdafruitiomqttSingle("voltage", str_sensor);

  //dtostrf(timeSinceLastLoraMessage/1000, 4, 3, str_sensor);
  //AdafruitiomqttSingle("messages", str_sensor);
  delay(100);
  Adafruitioclient.disconnect();

}

void rainbow(uint8_t wait) {
  uint16_t i, j;

  for (j = 0; j < 256; j++) {
    for (i = 0; i < NUM_PIXELS; i++) {
      strip.SetPixelColor(i, Wheel((i + j) & 255));
    }
    strip.Show();
    delay(wait);
  }
}

void theaterChase(uint32_t color, int wait) {
  for (int a = 0; a < 10; a++) { // Repeat 10 times...
    for (int b = 0; b < 3; b++) { //  'b' counts from 0 to 2...
      neopixel_clear(); //   Set all pixels in RAM to 0 (off)
      // 'c' counts up from 'b' to end of strip in steps of 3...
      for (int c = b; c < NUM_PIXELS; c += 3) {
        strip.SetPixelColor(c, color); // Set pixel 'c' to value 'color'
      }
      strip.Show(); // Update strip with new contents
      delay(wait); // Pause for a moment
    }
  }
}

//Theatre-style crawling lights with rainbow effect
void theaterChaseRainbow(uint8_t wait, uint8_t pixLenth) {
  for (int j = 0; j < 256; j++) { // cycle all 256 colors in the wheel
    for (int q = 0; q < 3; q++) {
      for (int i = 0; i < pixLenth; i = i + 3) {
        strip.SetPixelColor(i + q, Wheel((i + j) % 255)); //turn every third pixel on
      }
      strip.Show();

      delay(wait);

      for (int i = 0; i < pixLenth; i = i + 3) {
        strip.SetPixelColor(i + q, 0); //turn every third pixel off
      }
    }
  }
}

// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
RgbColor Wheel(uint8_t WheelPos) {
  WheelPos = 255 - WheelPos;
  if (WheelPos < 85) {
    return RgbColor(255 - WheelPos * 3, 0, WheelPos * 3);
  } else if (WheelPos < 170) {
    WheelPos -= 85;
    return RgbColor(0, WheelPos * 3, 255 - WheelPos * 3);
  } else {
    WheelPos -= 170;
    return RgbColor(WheelPos * 3, 255 - WheelPos * 3, 0);
  }
}

int calculate_tank_level(int sensor1, int sensor2, int sensor3) {
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

void neopixel_clear() {
  for (uint16_t i = 0; i < NUM_PIXELS; i++) {
    strip.SetPixelColor(i, black);
  }
  strip.Show();
}




void ProcessTanKPacket() {
  timeSinceLastNode1LoraMessage = millis() - LoraMessageTimer;
  LoraMessageTimer = millis(); //reset timer (to calculate time since last lora message was received)
  tank_level = calculate_tank_level((int) Combine2bytes(rxpayload.capsensor1Highbyte, rxpayload.capsensor1Lowbyte), (int) Combine2bytes(rxpayload.capsensor2Highbyte, rxpayload.capsensor2Lowbyte), (int) Combine2bytes(rxpayload.capsensor3Highbyte, rxpayload.capsensor3Lowbyte));

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
  if (tank_level <= 1) {
    //theaterChaseRainbow(30, 6);
    strip.SetPixelColor(0, greenlow); //power indicator
    strip.SetPixelColor(1, redfull);
    strip.SetPixelColor(2, redfull);
    strip.SetPixelColor(3, redfull);
    strip.Show();

    ledcWrite(0, 128);
  } //builtin LED flash (pwm channel 0)
  else if (tank_level <= 50) {
    //theaterChaseRainbow(10,6);
    ledcWrite(0, 128); //LED flash (pwm channel 0)
    strip.SetPixelColor(0, greenlow); //power indicator
    strip.SetPixelColor(1, orange);
    strip.SetPixelColor(2, orange);
    strip.SetPixelColor(3, orange);
    strip.Show();
  } else if (tank_level <= 80) {
    //theaterChaseRainbow(2,6);
    ledcWrite(0, 128); //LED off (pwm channel 0)
    strip.SetPixelColor(0, greenlow); //power indicator
    strip.SetPixelColor(1, yellow);
    strip.SetPixelColor(2, yellow);
    strip.SetPixelColor(3, yellow);
    strip.Show();
  } else {
    ledcWrite(0, 0); //LED off (pwm channel 0)
    strip.SetPixelColor(0, greenlow); //power indicator
    strip.SetPixelColor(1, black);
    strip.SetPixelColor(2, black);
    strip.SetPixelColor(3, black);
    strip.Show();
  }
  uint8_t ack[1];
  // the acknolement consists of an ack identiferer followed by nodeID
  ack[0] = (uint8_t) 170; //ack identifier 170=10101010
  ack[1] = (uint8_t) rxpayload.nodeID;
  rf95.send(ack, 2);
  //rf95.send(outgoingData, sizeof(outgoingData));
  rf95.waitPacketSent();

  //MQTT SEND/////////////////////////////////
  _checkWifi_mqtt();
  AdafruitSendAll();
  delay(2000);
  _checkWifi_mqtt();
 
  delay(1000);
  Adafruitioclient.loop(); //Mqtt
  delay(1000);
}

float ProcessIspindlePacket() {
  int TemperatureADC = 0;
  float lnR;
  float ThermResistance;
  float ThermTemperature;
  int lastRSSIlevel;
  float degreesTilt;

  lastRSSIlevel=(int) rf95.lastRssi();
  delay(1);
  Serial.print("L byte "); Serial.print(rxpayload.capsensor1Lowbyte);Serial.print("Hbyte ");Serial.println(rxpayload.capsensor1Highbyte);
  TemperatureADC = (int) Combine2bytes(rxpayload.capsensor1Highbyte, rxpayload.capsensor1Lowbyte);
  Serial.print("TemperatureADC ");
  Serial.println(TemperatureADC);
  ThermResistance = (1023 / (float)TemperatureADC) - 1;
  ThermResistance = RESISTOR / ThermResistance;
  lnR = log(ThermResistance);
  ThermTemperature = 1 / (ACOEFFICIENT + BCOEFFICIENT * lnR + CCOEFFICIENT * lnR * lnR * lnR) - 273.15;
  Serial.print("Temperature");Serial.println(ThermTemperature);

  Serial.print(" tilt= ");
  Serial.print(Combine2bytes(rxpayload.capsensor2Highbyte, rxpayload.capsensor2Lowbyte));
  degreesTilt=float(Combine2bytes(rxpayload.capsensor2Highbyte, rxpayload.capsensor2Lowbyte))/700;//dividing by 700 to convert to degrees
  
  
  dtostrf(degreesTilt, 4, 3, str_sensor); /*  4 is mininum width, 2 is precision; float value is copied onto str_sensor*/
  HAmqttSingle("/devices/tilt", str_sensor);
 
  dtostrf(temperatureDecompress(rxpayload.temperature), 4, 3, str_sensor); /* 4 is mininum width, 2 is precision; float value is copied onto str_sensor*/
  HAmqttSingle("/devices/AtmelTemp", str_sensor);
  
  dtostrf(batteryVoltageDecompress(rxpayload.voltage), 4, 3, str_sensor); /* 4 is mininum width, 2 is precision; float value is copied onto str_sensor*/
  HAmqttSingle("/devices/AtmelVolt", str_sensor);

  dtostrf(lastRSSIlevel, 4, 0, str_sensor); /* 4 is mininum width, 2 is precision; float value is copied onto str_sensor*/
  HAmqttSingle("/devices/Node2RSSI", str_sensor);

  dtostrf(ThermTemperature, 4, 3, str_sensor); /* 4 is mininum width, 2 is precision; float value is copied onto str_sensor*/
  HAmqttSingle("/devices/ESP32", str_sensor);
  HomeAssistantclient.disconnect();
  delay(1000);
  Adafruitioclient.connect(clientId.c_str(), ADAFRUIT_MQTT_CLIENT_NAME, ADAFRUITIOTOKEN);delay(100);
  AdafruitiomqttSingle("ispindeltemp", str_sensor);delay(100);
  
  TiltSamples.add(degreesTilt);

  dtostrf(TiltSamples.getMedian(), 4, 3, str_sensor); /*  4 is mininum width, 2 is precision; float value is copied onto str_sensor*/
  AdafruitiomqttSingle("ispindeltilt", str_sensor);delay(500);

  dtostrf(batteryVoltageDecompress(rxpayload.voltage), 4, 0, str_sensor); /* 4 is mininum width, 2 is precision; float value is copied onto str_sensor*/
  AdafruitiomqttSingle("ispindelvoltage", str_sensor);delay(500);

  set_fermentor_temp_led(ThermTemperature);
  
  return ThermTemperature;
}

void verbose_print_reset_reason(int reason)
{
  switch ( reason)
  {
    case 1  : Serial.println ("Vbat power on reset");break;
    case 3  : Serial.println ("Software reset digital core");break;
    case 4  : Serial.println ("Legacy watch dog reset digital core");break;
    case 5  : Serial.println ("Deep Sleep reset digital core");break;
    case 6  : Serial.println ("Reset by SLC module, reset digital core");break;
    case 7  : Serial.println ("Timer Group0 Watch dog reset digital core");break;
    case 8  : Serial.println ("Timer Group1 Watch dog reset digital core");break;
    case 9  : Serial.println ("RTC Watch dog Reset digital core");break;
    case 10 : Serial.println ("Instrusion tested to reset CPU");break;
    case 11 : Serial.println ("Time Group reset CPU");break;
    case 12 : Serial.println ("Software reset CPU");break;
    case 13 : Serial.println ("RTC Watch dog Reset CPU");break;
    case 14 : Serial.println ("for APP CPU, reseted by PRO CPU");break;
    case 15 : Serial.println ("Reset when the vdd voltage is not stable");break;
    case 16 : Serial.println ("RTC Watch dog reset digital core and rtc module");break;
    default : Serial.println ("NO_MEAN");
  }
}

void set_fermentor_temp_led(float temperature)
{
  
 if (temperature <= 25) {

    strip.SetPixelColor(4, blue);
    strip.Show();

      } //builtin LED flash (pwm channel 0)
    else if (temperature <= 26.5) {
    strip.SetPixelColor(4, blue); 
      strip.Show();
  } else if (temperature <= 28.5) {
    //theaterChaseRainbow(2,6);
    strip.SetPixelColor(4, greenlow); 
    strip.Show();
  } else {
    strip.SetPixelColor(4, red); 

    strip.Show();
  }

  
}

/*
Dependency Graph
|-- PubSubClient @ 2.8.0
|-- AsyncElegantOTA @ 2.2.7
|   |-- AsyncTCP @ 1.1.1
|   |-- FS @ 2.0.0
|   |-- Update @ 2.0.0
|   |-- WiFi @ 2.0.0
|-- AsyncTCP @ 1.1.1
|-- ESPAsyncWebServer-esphome @ 3.0.0
|   |-- AsyncTCP-esphome @ 2.0.0
|   |-- ArduinoJson @ 5.13.3
|   |-- AsyncElegantOTA @ 2.2.7
|   |   |-- AsyncTCP @ 1.1.1
|   |   |-- FS @ 2.0.0
|   |   |-- Update @ 2.0.0
|   |   |-- WiFi @ 2.0.0
|   |-- FS @ 2.0.0
|   |-- AsyncTCP @ 1.1.1
|   |-- WiFi @ 2.0.0
|-- RunningMedian @ 0.1.15
|-- ArduinoJson @ 5.13.3
|-- NeoPixelBus @ 2.3.4
|   |-- SPI @ 2.0.0
|-- RadioHead @ 1.89
|   |-- SPI @ 2.0.0
|-- WiFi @ 2.0.0
*/

