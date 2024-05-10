
#define DEBUG 0

#define SENSOR_1_PIN 18
#define SENSOR_2_PIN 21

#define LED_PIN 22
#define LED_COUNT 29

#define WARM 255,125,0

#define SerialToPC Serial
#define DebugSerial Serial

#define BAUDRATE_PC_SERIAL 9600
#define BAUDRATE_DEBUG_SERIAL 115200


#define COMMAND_FOR_PRESS SerialToPC.println("P")
#define COMMAND_FOR_RIGHT SerialToPC.println("R")
#define COMMAND_FOR_LEFT SerialToPC.println("L")

#define COMMAND_FOR_CARD_REMOVE SerialToPC.println("Z")

#define ENCODER_P_PIN 33
#define ENCODER_L_PIN 26
#define ENCODER_R_PIN 25

#define SENSITIVITY 2

#define DELAY_TIME_IN_ms 500


#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

uint8_t encoder_l_counter = 0;
uint8_t encoder_r_counter = 0;

bool currentState;
bool lastState;

Adafruit_NeoPixel pixels(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

bool lastSwitchState =0;

TaskHandle_t Task1;
TaskHandle_t Task2;

 void read_encoder() 
 { 
   currentState = digitalRead(ENCODER_L_PIN);
   if (currentState != lastState)
    {     
     if (digitalRead(ENCODER_R_PIN) != currentState) { encoder_l_counter++; encoder_r_counter =0; if(encoder_l_counter > SENSITIVITY){ COMMAND_FOR_LEFT; encoder_l_counter=0; delay(DELAY_TIME_IN_ms);} } 
     else {encoder_r_counter ++; encoder_l_counter =0; if(encoder_r_counter > SENSITIVITY) { COMMAND_FOR_RIGHT; encoder_r_counter =0; delay(DELAY_TIME_IN_ms);} }
    }
  //  else if(!digitalRead(ENCODER_P_PIN)){ COMMAND_FOR_PRESS; while(!digitalRead(ENCODER_P_PIN)){yield();}}
   lastState = currentState; 
 }

//Read new tag if available


void readMagnet(){
  if (digitalRead(SENSOR_1_PIN) == LOW || digitalRead(SENSOR_2_PIN) == LOW){
    if (lastSwitchState == 0){
      lastSwitchState = 1;
    }
  }
  else if (digitalRead(SENSOR_1_PIN) == HIGH && digitalRead(SENSOR_2_PIN) == HIGH){
    if (lastSwitchState == 1){
    lastSwitchState = 0; COMMAND_FOR_CARD_REMOVE;
    }
    
  }
  
  if (DEBUG){Serial.println(String(lastSwitchState) + "\t" + String(digitalRead(SENSOR_1_PIN)) + "\t" + String(digitalRead(SENSOR_2_PIN) ));}

}

void led_on(uint8_t red, uint8_t green, uint8_t blue)
{
  for (int i = 0; i < LED_COUNT; i++)
  {
    pixels.setPixelColor(i, pixels.Color(red, green, blue));
  }
   pixels.show();
}

void core_zero_loop()
{
  readMagnet();
  delay(1);
}

void Task1code( void * pvParameters ){ for(;;){ core_zero_loop(); yield();} }
void Task2code( void * pvParameters ){ for(;;){yield();} }


void dual_core_setup()
{
  delay(500);
  xTaskCreatePinnedToCore(Task1code,"Task1",10000,NULL,1,&Task1,0);                       
  delay(500); 
  xTaskCreatePinnedToCore(Task2code,"Task2",10000,NULL,1,&Task2,1);      
  delay(500); 
}

void setup() 
{
  // Initiating

  SerialToPC.begin(BAUDRATE_PC_SERIAL);

  pinMode(ENCODER_P_PIN,INPUT_PULLUP);
  pinMode(ENCODER_L_PIN,INPUT_PULLUP);
  pinMode(ENCODER_R_PIN,INPUT_PULLUP);
  pinMode(SENSOR_1_PIN,INPUT_PULLUP);
  pinMode(SENSOR_2_PIN,INPUT_PULLUP);

  pixels.begin();
  dual_core_setup();

  led_on(WARM);
  
}



void loop() 
{
  read_encoder();
}

