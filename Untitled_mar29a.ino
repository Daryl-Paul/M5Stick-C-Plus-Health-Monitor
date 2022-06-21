#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <UniversalTelegramBot.h>
#include <ArduinoJson.h>
#include <M5StickCPlus.h>
#include "arduino_secrets.h"
#include "thingProperties.h"
#define USE_ARDUINO_INTERRUPTS false
#include <PulseSensorPlayground.h>

//Telegram parameters
const char* ssid = "Wifi name";
const char* password = "Wifi password";
#define BOTtoken "Obtain token from Bot Father"  // your Bot Token (Get from Botfather)
#define CHAT_ID "Chat ID obtained from IDBot"

//pulseSensor parameters
const int OUTPUT_TYPE = SERIAL_PLOTTER;
const int PULSE_INPUT = 33;
const int THRESHOLD = 550;   // Adjust this number to avoid noise when idle
byte samplesUntilReport;
const byte SAMPLES_PER_SERIAL_SAMPLE = 10;
int myBPM;
unsigned long sendDuration = 1000*10;
unsigned long startPulse;
PulseSensorPlayground pulseSensor;
//Warning, constant threshold may cause inaccuracies

//IMU parameter
//These param are for acceleration
float accX = 0.0F;
float accY = 0.0F;
float accZ = 0.0F;
float netAccel = 0.0F;
const float accelThresh = 3.1;  //net acceleration threshold
//These param are for angular velocity, figure out which is roll, pitch and yaw of the device
float gyroX = 0.0F;
float gyroY = 0.0F;
float gyroZ = 0.0F;
float netAVel = 0.0F;
//Angular velocity threshold in degrees per second
const float AVelThresh = 187.62;

WiFiClientSecure client;
UniversalTelegramBot bot(BOTtoken, client);

//falseAlarm parameter
unsigned long startWarningTime;
unsigned long waitTime = 5000;
bool ALARM = false;

//buzzer parameter
const int servo_pin = 26;
int freq = 50;
int ledChannel = 0;
int resolution = 10;

void setup() {
  //serial baud rate
  Serial.begin(115200);
  M5.begin(115200);
  M5.Imu.Init();
  // Defined in thingProperties.h
  initProperties();
  // Connect to Arduino IoT Cloud
  ArduinoCloud.begin(ArduinoIoTPreferredConnection);

  Serial.print("Connecting Wifi: ");
  Serial.println(ssid);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  client.setCACert(TELEGRAM_CERTIFICATE_ROOT);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  Serial.println("");
  Serial.println("Connected to Wifi");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  bot.sendMessage(CHAT_ID, "Bot started up", "");

  ledcSetup(ledChannel, freq, resolution);
  ledcAttachPin(servo_pin, ledChannel);

  M5.Lcd.setRotation(1);
  M5.Lcd.setTextSize(2); 
  M5.Lcd.print("Press A for fall detection, B for heart rate.");
  /*
     The following function allows you to obtain more information
     related to the state of network and IoT Cloud  and errors
     the higher number the more granular information you’ll get.
     The default is 0 (only errors).
     Maximum is 4
 */
  pulseSensor.analogInput(PULSE_INPUT);
  pulseSensor.setSerial(Serial);
  pulseSensor.setThreshold(THRESHOLD);
  samplesUntilReport = SAMPLES_PER_SERIAL_SAMPLE;
  if (!pulseSensor.begin()) {
      Serial.println("Sensor not working");  
  } 
  setDebugMessageLevel(2);
  ArduinoCloud.printDebugInfo();
}

void loop(){
  //update to cloud
  ArduinoCloud.update();
  //takes button input
  M5.update();
  if(M5.BtnA.wasReleased()){
    //Press button a for fall detection mode
    M5.Lcd.fillScreen(BLACK);
    while(!M5.BtnB.wasReleased()){
      M5.update();
      M5.Lcd.setCursor(0,0);
      M5.Lcd.print("Fall Detection Mode");
      fallIMU();
    }
  }
  else if(M5.BtnB.wasReleased()){
    //Press button B for heart monitor mode
    M5.Lcd.fillScreen(BLACK);
    Serial.println("Begin measuring pulse");
    while(!M5.BtnA.wasReleased()){
      startPulse = millis();
      Serial.println("Loop started");
      while((millis() - startPulse) <= sendDuration){
        M5.update();
        if(M5.BtnA.wasReleased()){
          Serial.println("Exiting loop");
          break;
        }
        else{
          pulseMeas();
          M5.Lcd.setCursor(0,0);
          M5.Lcd.print("Heart rate is: ");
          M5.Lcd.setCursor(20,20);
          M5.Lcd.print(myBPM);
        }
      }
      //heart rate sent to IoT cloud
      heart = myBPM;
      ArduinoCloud.update();
      Serial.println("Sending data");
    }
  Serial.println("loop exited");
  delay(2000);
  }
}

void onHeartRateChange(){//Function runs when detecting a change in BPM
  Serial.println("Updating BPM");
}

void pulseMeas(){//Function for heart rate monitor
  myBPM = pulseSensor.getBeatsPerMinute();
  if (pulseSensor.sawNewSample()) {

    if (--samplesUntilReport == (byte) 0) {
      samplesUntilReport = SAMPLES_PER_SERIAL_SAMPLE;

      pulseSensor.outputSample();
      Serial.println(myBPM);

      if (pulseSensor.sawStartOfBeat()) {
        pulseSensor.outputBeat();
       
      }
    }
  }
}

void showAccel(){//This function is to monitor IMU in real time via serial monitor
    Serial.printf("Net Accel: %f\n", netAccel);
    Serial.println("Potential fall detected");
    Serial.printf("Net Angular Velocity: %f\n",netAVel);
}

void falseAlarm(){
  startWarningTime = millis();
  while((millis() - startWarningTime) <= waitTime){
    M5.update();//To catch the button status (1 for pressed, 0 for released)
    ledcWriteTone(ledChannel, 1250);//piezo starts buzzing
    delay(100);
    if(M5.BtnA.wasPressed()){ //if button is pressed within 5 seconds, then alarm is false, exit loop and proceed with monitoring
      ALARM = false;
      //Serial.println("Pressed");
      break;
    }
    else if(!M5.BtnA.wasReleased()){  //if button is not pressed within 5 seconds, alarm is true and exit loop
      ALARM = true;
    }
  }
  ledcWriteTone(ledChannel, 0); //pizeo stops buzzing
  if(ALARM == true){  //Nothing special here, just for our Serial monitor
    Serial.println("Fall detected");
  }
  else if(ALARM == false){
    Serial.println("False Alarm");
  }
}

void fallIMU(){
  M5.IMU.getAccelData(&accX,&accY,&accZ);
  M5.IMU.getGyroData(&gyroX,&gyroY,&gyroZ);
  
  netAccel = sqrt(sq(accX)+sq(accY)+sq(accZ)); //This is the equatioin for net acceleration
  netAVel = sqrt(sq(gyroY)+sq(gyroZ));  //This is the equatioin for net angular velocity
  if(netAccel > accelThresh){ //When net acceleration from the accelero exceeds the threshold set on top
    if(netAVel > AVelThresh){  
      showAccel();  //This is just for our serial monitor, nothing special
      falseAlarm(); //calls the false alarm delay timer func
      if(ALARM == true){  //if the fall is real send a message to Bot
        bot.sendMessage(CHAT_ID, "Fall detected!!", "");
        bot.sendMessage(CHAT_ID, "Peak Accel is: " + String(netAccel) + "Gs");
      }
    }
  }  
}
