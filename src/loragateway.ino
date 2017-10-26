// Adafruit Feather Huzzah ESP8266 LoRa receiver for data relay to Raspberry Pi IoT Gateway
// Author : Tanmoy Dutta
// March 2017

#include <ESP8266WiFi.h>
#include <ESP8266WiFiMulti.h>
#include <ESP8266HTTPClient.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <RH_RF95.h>
#include <ArduinoJson.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
#include "Wire.h"//th
#include "UbidotsMicroESP8266.h"
// WiFi access credentials
#define  WIFI_SSID         "Penryn"         // WiFi SSID
#define  WIFI_PASSWORD     "hoooverr"         // WiFI Password

#define MQTT_PI_SERVER      "192.168.2.254"           // MQTT Queue Manager IP address
#define MQTT_PI_SERVER_PORT  1883                    // MQTT Port, use 8883 for SSL

#define TOKEN  "A1E-0V3Qu4hZfmUA0uOtVMt4rFPtVaz171"  // Put here your Ubidots TOKEN
#define ID_1 "59d85600c03f97202c9ff2c0" // Put your variable ID here

long batteryVoltageDecompress (byte batvoltage);
float temperatureDeompress(byte temperature);
unsigned int Combine2bytes(byte x_high, byte x_low);

WiFiClient client;

Ubidots client2(TOKEN,"client2");

Adafruit_MQTT_Client mqtt(&client, MQTT_PI_SERVER, MQTT_PI_SERVER_PORT);          // MQTT Client initialization
Adafruit_MQTT_Publish telemetry = Adafruit_MQTT_Publish(&mqtt, "lora/temp");  // MQTT Topic setup in publish mode


// JSON Buffer
DynamicJsonBuffer jsonBuffer;


#define OLED_RESET 2
// Instance of the display
Adafruit_SSD1306 display(OLED_RESET);

#if (SSD1306_LCDHEIGHT != 64)
//#error("Height incorrect, please fix Adafruit_SSD1306.h!");//th deleted
#endif

#define RFM95_CS 16
#define RFM95_RST 0
#define RFM95_INT 15

// Blinky on receipt
#define LED 5

// Set radion frequency
#define RF95_FREQ 434.0

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);


struct payloadDataStruct{
  byte nodeID;
  byte rssi;
  byte voltage;
  byte temperature;
  byte capsensorLowbyte;
  byte capsensorHighbyte;
}rxpayload;

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
client2.setDebug(true); // Uncomment this line to set DEBUG on

  Serial.begin(57600);
  _initOLED();
  _initWiFi();
  _initMQTT();
  _initLoRa();
}


/*----------------------------------------------------------------------------
Function : loop()
Description : Main program loop
------------------------------------------------------------------------------*/
void loop() {
  if (rf95.available()) {
    // Should be a message for us now
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);

    if (rf95.recv(buf, &len)) {
      digitalWrite(LED, HIGH);
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
      //rxpayload.voltage=batteryVoltageDecompress(rxpayload.voltage);
      Serial.print(" nodeID = ");Serial.print(rxpayload.nodeID);
      Serial.print(" remote voltage comp = ");Serial.print((rxpayload.voltage));
      Serial.print(" remote voltage = ");Serial.print(batteryVoltageDecompress(rxpayload.voltage));
      Serial.print(" remote rssi = ");Serial.print(rxpayload.rssi);
      Serial.print(" Local RSSI: ");Serial.print(rf95.lastRssi(), DEC);
      Serial.print(" cap= ");Serial.print(Combine2bytes(rxpayload.capsensorHighbyte,rxpayload.capsensorLowbyte));

      ///////////////////////MQTT Code
      //sendMessage(String(bufChar));
      sendMessage(String(temperatureDeompress(rxpayload.temperature)));

      //////////////////////////////////
     client2.add("59d864b6c03f972cdb9e33e6", -rxpayload.rssi);
     client2.add("59dee274c03f976a87c2594b", (int)rf95.lastRssi());
     client2.add("59d864a1c03f972cdb9e33e5", batteryVoltageDecompress(rxpayload.voltage));
     client2.add("59d85600c03f97202c9ff2c0",temperatureDeompress(rxpayload.temperature))  ;
     client2.add("59e7b649c03f972d175ee2e2",(int)Combine2bytes(rxpayload.capsensorHighbyte,rxpayload.capsensorLowbyte));
     client2.sendAll(false);

      // Send a reply
      //uint8_t outgoingData[] = "{\"Status\" : \"Ack\"}";

      uint8_t ack[1];
      // the acknolement consists of an ack identiferer followed by nodeID
      ack[0]=(uint8_t)170; //ack identifier 170=10101010
      ack[1]= (uint8_t)rxpayload.nodeID;


      //uint8_t outgoingData[] = {rxpayload.nodeID};
      rf95.send(ack, 2);
      //rf95.send(outgoingData, sizeof(outgoingData));
      rf95.waitPacketSent();
      //Serial.println("Sent a reply");
      digitalWrite(LED, LOW);
    }
    else {
      //Serial.println("Receive failed");
    }
  }
}

/*----------------------------------------------------------------------------
Function : displayOnOLED()
Description : Subroutine to display text on the OLED
------------------------------------------------------------------------------*/
void displayOnOLED (String data) {
  display.setCursor(0,0);
  display.clearDisplay();
  display.print(data);
  display.display();
}

/*----------------------------------------------------------------------------
Function : _initOLED ()
Description : Subroutine to initialize the OLED Display
------------------------------------------------------------------------------*/
void _initOLED () {
  //Initialize the display
  display.begin(SSD1306_SWITCHCAPVCC, 0x3D);  // initialize with the I2C addr 0x3D (for the 128x64)
  display.setRotation(2);
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.clearDisplay();
}

void donothing() {
}

/*----------------------------------------------------------------------------
Function : _initWiFi()
Description : Connect to WiFi access point
------------------------------------------------------------------------------*/
void _initWiFi() {
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  display.setCursor(0,0);
  display.clearDisplay();

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("Waiting for WiFi...");
  }
  Serial.println("WiFi Connected!");
  delay(1000);

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
  delay(10);

  while (!rf95.init()) {
    Serial.println("LoRa radio init failed");
    while (1);
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
  rf95.setModemConfig(RH_RF95::Bw125Cr48Sf4096);//th
  rf95.printRegisters(); //th



  Serial.println("LoRa Listening...");
  delay(1000);
}

/*----------------------------------------------------------------------------
Function : _initMQTT()
Description : Connect to MQTT Queue Manager
------------------------------------------------------------------------------*/
void _initMQTT() {
  int8_t ret;

  // Stop if already connected.
  if (mqtt.connected()) {
    return;
  }

  Serial.println("Connecting to MQTT... ");

  uint8_t retries = 3;
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
       mqtt.disconnect();
       delay(5000);  // wait 5 seconds
       retries--;
       if (retries == 0) {
         // basically die and wait for WDT to reset me
         while (1);
       }
  }
  Serial.println("MQTT Connected!");
  delay(1000);
}

/*----------------------------------------------------------------------------
Function : sendMessage()
Description : Send message to the MQTT Server
------------------------------------------------------------------------------*/
void sendMessage(String msg) {
  String s;
  //String s = "{ \"M\":";
  s += msg;
  //s += " }}";

  int len = s.length();
  char charBuf[len];
  s.toCharArray(charBuf, len);

  if (! telemetry.publish(charBuf)) {
    Serial.println("Sending failed :(");
    Serial.print("mqtt connected=");Serial.println(mqtt.connected());
  } else {
    Serial.println(" mqtt sent :)");
  }
}
long batteryVoltageDecompress (byte batvoltage) {
//decompress voltage from 1 byte
long result2;
result2 =  (batvoltage*8)+1300;
return (result2);
}

float temperatureDeompress(byte temperature){
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
