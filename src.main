#include <LiquidCrystal.h>
#include <PubSubClient.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <ESP32Servo.h>
#include <ArduinoJson.h>

#define buttonPinFan    5
#define buttonPinPump    18
#define buttonPinDoor    19
#define buttonPinMode   21

#define SENSOR_MQ2  35
#define SENSOR_FIRE   34
#define SENSOR_FIRE_ON    0
#define SENSOR_FIRE_OFF   1

#define BUZZER  23
#define BUZZER_ON 1
#define BUZZER_OFF 0

#define AUTO_MODE 1
#define MANUAL_MODE 0
#define ON 1
#define OFF 0

#define RELAY1  22
#define RELAY2  32
#define SERVO1 33
#define SERVO2 25

#define LCD_RS  15
#define LCD_EN  13
#define LCD_D4  12
#define LCD_D5  14
#define LCD_D6  27
#define LCD_D7  26
LiquidCrystal My_LCD(15, 13, 12, 14, 27, 26);

const int buttonPins[] = {5, 19, 18, 21};
const int numButtons = sizeof(buttonPins) / sizeof(buttonPins[0]);  

bool relay1State = OFF;
bool relay2State = OFF;

int mq2Thresshold = 1000; 

Servo myservo1; 
Servo myservo2; 
int window = 0;

volatile bool flagMQ2 = 0;
volatile bool flagFire = 0;

const char* ssid = "GIAHUNG-406-6VTK";
const char* pass = "GIAHUNG1368";
const char* mqtt_server = "192.168.46.61";
const int mqtt_port = 1883;

const char* subFan_topic = "Devices/Fan";
const char* subPump_topic = "Devices/Pump";
const char* subDoor_topic = "Devices/Doors";
const char* subMode_topic = "Devices/Mode";

volatile int status_thiet_bi = OFF, status_mode = MANUAL_MODE;

TaskHandle_t TaskButton_handle = NULL;
TaskHandle_t TaskMqtt_handle = NULL;

void esp_sub();
void pubMQTT(const char * topic, const char* payload);
void sendDatatoMQTT(int gas_value, int fire_status);

void TaskMqtt(void *pvParameters);
void TaskButton(void *pvParameters); 

void openWindow();
void closeWindow();
void controlWindow(int onoff);

int readFireSensor();
int readMQ2();

void LCD1602_Init();
void LCDPrint(int hang, int cot, char *text, int clearOrNot);
void callback(char* topic, byte* message, unsigned int length);

void controlRelay(int relay, int state);

WiFiClient espClient; 
PubSubClient client(espClient);

void setup() {
  Serial.begin(115200);
  LCD1602_Init();
  
  //---------- Khai báp WiFi ---------
  WiFi.begin(ssid, pass); 
  while(WiFi.status() != WL_CONNECTED){
    delay(500); 
    Serial.print(".");
  }
  Serial.println("\nWiFi Connected!");
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
  if(client.connect("ESP32_Client")){
    Serial.println(" Kết nối thành công MQTT");
    esp_sub();
    pubMQTT("Devices/Fan", "off");
    pubMQTT("Devices/Pump", "off");
    pubMQTT("Devices/Doors", "off");
    pubMQTT("Devices/Mode", "Manual");
  }else{
    Serial.println("Lỗi kết nối MQTT");
  }

  pinMode(RELAY1, OUTPUT);                 
  pinMode(RELAY2, OUTPUT);


  pinMode(BUZZER, OUTPUT);  
  pinMode(SENSOR_FIRE, INPUT_PULLUP);  
  digitalWrite(BUZZER, BUZZER_OFF);

  for (int i = 0; i < numButtons; i++) {
        pinMode(buttonPins[i], INPUT_PULLUP);  
    }
    
  
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  myservo1.setPeriodHertz(50);  
  myservo2.setPeriodHertz(50);  
  myservo1.attach(SERVO1, 500, 2400);
  myservo2.attach(SERVO2, 500, 2400);
  closeWindow();
  
  
  xTaskCreatePinnedToCore(TaskMqtt,"TaskMqtt" , 1024*2 ,  NULL,  5 ,  &TaskMqtt_handle , 0);
  xTaskCreatePinnedToCore(TaskButton,"TaskButton" ,1024*4 ,  NULL,  5 ,  &TaskButton_handle , 0);

}
void loop() {
}

void esp_sub(){
    client.subscribe(subFan_topic);
    client.subscribe(subPump_topic); 
    client.subscribe(subDoor_topic);
    client.subscribe(subMode_topic);
}

void pubMQTT(const char * topic, const char* payload){
  if(client.connected()){
    client.publish(topic, payload);
  }
}

void sendDatatoMQTT(int gas_value, int fire_status) {

    // Chuyển JSON thành chuỗi
    char gasBuffer[10];
    char fireBuffer[2];
    
    sprintf(gasBuffer, "%d", gas_value);
    sprintf(fireBuffer, "%d", fire_status);
    
    pubMQTT("esp32/sensor/Gas", gasBuffer);  
    pubMQTT("esp32/sensor/Flame",fireBuffer);
    
    // In ra Serial để kiểm tra
    Serial.print("Gửi MQTT - Gas: ");
    Serial.println(gasBuffer);

    Serial.print("Gửi MQTT - Fire: ");
    Serial.println(fireBuffer);
}


void callback(char* topic, byte* message, unsigned int length) {
    Serial.print("Nhận dữ liệu từ topic:");
    Serial.println(topic);
    Serial.print(". Message: ");
    String messageTemp;
    
    for (int i = 0; i < length; i++) {
      Serial.print((char)message[i]);
      messageTemp += (char)message[i];
    }
    Serial.println();

    if(messageTemp == "Auto" && String(topic) == "Devices/Mode"){
      status_mode = AUTO_MODE;
    }
    else if(messageTemp == "Manual" && String(topic) == "Devices/Mode"){
      status_mode = MANUAL_MODE;
    }
    
    if(status_mode == MANUAL_MODE){
      if(messageTemp == "on"){
        status_thiet_bi = 1;
      } else if(messageTemp == "off"){
        status_thiet_bi = 0;
      } 
      if(String(topic) == "Devices/Fan") {
        digitalWrite(RELAY1, status_thiet_bi ? HIGH : LOW);
        
        Serial.println(status_thiet_bi ? "Quạt BẬT" : "Quạt TẮT");
      }
      else if(String(topic) == "Devices/Pump"){
        digitalWrite(RELAY2, status_thiet_bi ? HIGH : LOW);
        Serial.println(status_thiet_bi ? "Máy bơm BẬT" : "Máy bơm TẮT");
      }
      else if(String(topic) == "Devices/Doors"){
        controlWindow(status_thiet_bi);
        Serial.println(status_thiet_bi ? "Cửa Mở" : "Cửa Đóng");
      }
    }
}

void TaskMqtt(void *pvParameters) {
    while (1) {
        if (!client.connected()) {  
            Serial.println("MQTT Disconnected! Reconnecting...");
            if (client.connect("ESP32_Client")) {
                Serial.println("MQTT Connected!");
            }
        }
        Serial.print("task_mqtt status_mode");
        Serial.println(status_mode);
        client.loop();
        int gas_value = readMQ2();
        int fire_status = readFireSensor();

        if(status_mode == 1){
           int windowState = 0;
           if(gas_value >= mq2Thresshold ) {
                 relay1State = ON;  relay2State = OFF; 
                 controlRelay(RELAY1, relay1State);
                 controlRelay(RELAY2, relay2State);  
                 windowState = 1; controlWindow(windowState);
                 if(flagMQ2 == 0){
                  pubMQTT("Devices/Fan", "on");
                  pubMQTT("Devices/Doors", "on");
                  flagMQ2 = 1;
                 }
                 delay(100);
              }
            else if(fire_status != SENSOR_FIRE_ON) {
               relay1State = OFF;  relay2State = ON; 
               //windowState = 0; controlWindow(windowState);
               controlRelay(RELAY1, relay1State);
               controlRelay(RELAY2, relay2State); 
               if(flagFire == 0){
                flagFire = 1;
                pubMQTT("Devices/Pump", "on");
               }
               delay(100);
            }
            else if(gas_value < mq2Thresshold  || fire_status != SENSOR_FIRE_OFF ) {
               if(flagMQ2) {
                 pubMQTT("Devices/Fan", "off");
                 pubMQTT("Devices/Doors", "off");
                 flagMQ2 = 0;
               }
               if(flagFire) {                 
                 pubMQTT("Devices/Pump", "off");
                 flagFire = 0;
               }
               relay1State = OFF;  relay2State = OFF; 
               controlRelay(RELAY1, relay1State);
               controlRelay(RELAY2, relay2State);  
               windowState = 0; controlWindow(windowState);
            }
        }
        sendDatatoMQTT(gas_value, fire_status);  
        vTaskDelay(pdMS_TO_TICKS(500));  
    }
}

void TaskButton(void *pvParameters) {

    bool lastStateBTN1 = HIGH;
    bool lastStateBTN2 = HIGH;
    bool lastStateBTN3 = HIGH;

    
    while (1) {
        bool currentStateBTN1 = digitalRead(buttonPinFan);
        bool currentStateBTN2 = digitalRead(buttonPinPump);
        bool currentStateBTN3 = digitalRead(buttonPinDoor);
        
        if(status_mode == MANUAL_MODE){
          if (currentStateBTN1 == LOW && lastStateBTN1 == HIGH) {
            relay1State = !relay1State;
            digitalWrite(RELAY1, relay1State ? HIGH : LOW);
            digitalWrite(2, relay1State ? HIGH : LOW);
    
            Serial.print("RELAY1: ");
            Serial.println(relay1State ? "ON" : "OFF");
            
            pubMQTT("Devices/Fan", relay1State ? "on" : "off");
          }
          lastStateBTN1 = currentStateBTN1;
  
          if (currentStateBTN2 == LOW && lastStateBTN2 == HIGH) {
            relay2State = !relay2State;
            digitalWrite(RELAY2, relay2State ? HIGH : LOW);
            digitalWrite(2, relay1State ? HIGH : LOW);
           
            Serial.print("RELAY2: ");
            Serial.println(relay2State ? "ON" : "OFF");
            
            pubMQTT("Devices/Pump", relay2State ? "on" : "off");
          }
          lastStateBTN2 = currentStateBTN2;

          if (currentStateBTN3 == LOW && lastStateBTN3 == HIGH) {
            vTaskDelay(pdMS_TO_TICKS(20));
            if (currentStateBTN3 == LOW && lastStateBTN3 == HIGH) {
              
              window = !window;
              controlWindow(window);
              
              pubMQTT("Devices/Doors", window ? "on" : "off");
            }
          }    
          lastStateBTN3 = currentStateBTN3;   
         }
         vTaskDelay(pdMS_TO_TICKS(100)); 
      }
}

int readFireSensor() {
    int Fire_Value = !digitalRead(SENSOR_FIRE);
    if(Fire_Value == 1){
      digitalWrite(BUZZER,BUZZER_ON);
    }
    else {
      digitalWrite(BUZZER,BUZZER_OFF);
    }
//    Serial.println(Fire_Value);
    return Fire_Value;
}

int readMQ2(){
    float MQ2_Value = analogRead(SENSOR_MQ2);
    MQ2_Value = map(MQ2_Value, 0 , 4095, 0, 10000 );
//    Serial.println(MQ2_Value);
    return MQ2_Value;
}

void LCD1602_Init() {
   My_LCD.begin(16, 2);
   My_LCD.clear();
   LCDPrint(0,2,"Gas Detection",0);
   LCDPrint(1,5,"System",0);
}

void LCDPrint(int hang, int cot, char *text, int clearOrNot) {
   if(clearOrNot == 1)
      My_LCD.clear();
   My_LCD.setCursor(cot, hang);
   My_LCD.print(text);
}

void openWindow() {
    myservo1.write(0); 
    myservo2.write(180);
}

void closeWindow() {
    myservo1.write(90); 
    myservo2.write(90);

}
void controlWindow(int onoff) {
    if(onoff == 0)
      closeWindow();
    else
      openWindow();
}
void controlRelay(int relay, int state) {
    digitalWrite(relay,state);
}
