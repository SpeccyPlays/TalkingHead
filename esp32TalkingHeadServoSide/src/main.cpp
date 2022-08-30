/*
 * Server to receieve microphone samples via TCP
 * These will make a servo move between max and min values of samples
 * 
 * 
 */

#include <WiFi.h>
#include <ESP32Servo.h>
//wifi
const char* ssid = "talkingHead";
const char* password = "talkingHead";
const uint16_t port = 8090;
//storage
uint8_t receivedSample[1024];
uint8_t servoSample[1024];
const uint8_t whiteNoiseStart = 77;
const uint8_t whiteNoiseEnd = 87;
//servo
Servo servo;
//servo actual range = 0-180 but can cause issues at end of ranges
const uint8_t servoMinAngle = 5;
const uint8_t servoMaxAngle = 175;
const uint8_t servoMiddleAngle = 180 / 2; 
uint8_t servoPosition = servoMiddleAngle; //we want the servo to start in the middle
const uint8_t servoPin = 25;

//setup TCPserver
WiFiServer server(port);
//unsigned long previousMillis = 0;
//unsigned long reconnectInterval = 3000;

//set up tasks
void runNetwork(void *pvParameters);
void runServo(void *pvParameters);
TaskHandle_t network;
TaskHandle_t servoTask;


void setup() {
  Serial.begin(115200);
  //Wifi setup
  Serial.print("Starting AP");
  WiFi.softAP(ssid, password);
  IPAddress IP = WiFi.softAPIP();
  Serial.print("Access Point address: ");
  Serial.println(IP);
  server.begin();
  delay(100);
  //Servo setup
  // Allow allocation of all timers
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  servo.setPeriodHertz(50);
  servo.attach(servoPin, 500, 2500);

  //setup tasks to run on cores. If one isn't priority 0 then the watchdog will trigger
  xTaskCreatePinnedToCore(runNetwork, "network", 10000, (void*) NULL, 0, &network, 0);
  xTaskCreatePinnedToCore(runServo, "servoTask", 10000, (void*) NULL, 1, &servoTask, 1);
}
void runNetwork( void *pvParameters ){
  /*
  Simply receive data over wifi and store in the sample array
  */
  for (;;){
    WiFiClient client = server.available();
    if(client){
      while (client.connected()) {
        client.read(receivedSample, sizeof(receivedSample));
      }
    }
  }
  vTaskDelay(10);
}
void runServo(void *pvParameters){
  /*
  Run through the sample to get the max/min values then map that to servo rotation
  Move servo between max and min
  */
  for (;;){
    memcpy(servoSample, receivedSample, sizeof(receivedSample));
    int max = servoMiddleAngle;//0;
    int min = servoMiddleAngle;//0;
    for (uint16_t i = 0; i < sizeof(servoSample); i++){
      if (servoSample[i] < whiteNoiseStart || servoSample[i] > whiteNoiseEnd){
        if (servoSample[i] > max){
          max = servoSample[i];
        }
        if (servoSample[i] < min){
          min = servoSample[i];
        }
      }
    } //end for loop
    servoPosition = map(max, 0, 255, servoMinAngle, servoMaxAngle);
    //Serial.println(servoPosition);
    servo.write(servoPosition);
    delay(15);
    servoPosition = map(min, 0, 255, servoMinAngle, servoMaxAngle);
    servo.write(servoPosition);
    delay(15);
  } //end for loop
}
void loop() {
} // end loop loop
