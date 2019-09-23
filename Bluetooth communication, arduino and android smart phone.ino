#include <SoftwareSerial.h>
#include <LiquidCrystal.h> 
#include <Arduino.h>
#include "algorithm.h"
#include "max30102.h"
#define SENSOR A1
#define SENSOR_THRESHOLD 300
#define MAX_BRIGHTNESS 255
#define PulseSensorPurplePin A0        // Pulse Sensor PURPLE WIRE connected to ANALOG PIN 0
//you can find those head file from Internet
int LED13 = 13;   //  The on-board Arduion LED
int KEY_NUM = 0;
int count = 0;
int state = 0;
int val_AD = 0;
int Weight = 0;
int vout = 0 ;
long R_val = 0;
char val;
uint16_t ir_buffer[100]; //infrared LED sensor data
uint16_t red_buffer[100];  //red LED sensor data
int32_t ir_buffer_length; //data length
int32_t spo2;  //SPO2 value
int8_t spo2_valid;  //indicator to show if the SPO2 calculation is valid
int32_t heart_rate; //heart rate value
int8_t  hr_valid;  //indicator to show if the heart rate calculation is valid
uint8_t dummy;
char inbyte="HR";
int Signal;                // holds the incoming raw data. Signal value can range from 0-1024
int Threshold = 550;
void setup() {
   max30102_reset(); //resets the MAX30102
  // put your setup code here, to run once:
   pinMode(LED13,OUTPUT);         // pin that will blink to your heartbeat!
   pinMode(10, INPUT);  //pin D10 connects to the  output pin of the MAX30102
   pinMode(1,OUTPUT);
   digitalWrite(1,HIGH);
   Serial.begin(9600); 
   max30102_read_reg(REG_INTR_STATUS_1,&dummy);  //Reads and clears the interrupt status register
   dummy=Serial.read();
   max30102_init();  //initialize the MAX30102  
}

void loop() {
  uint32_t minin, maxin,prev_data, brightness;  //variables to calculate the on-board LED brightness that reflects the heartbeats
  int32_t i;
  float temp;
  brightness=0;
  minin=0x3FFFF;
  maxin=0;
    ir_buffer_length=100;  //buffer length of 100 stores 4 seconds of samples running at 25sps

  //read the first 100 samples, and determine the signal range
  for(i=0;i<ir_buffer_length;i++)
  {
    while(digitalRead(10)==1);  //wait the interrupt pin 
    max30102_read_fifo((red_buffer+i), (ir_buffer+i));  //read from MAX30102 FIFO
    
    if(minin>red_buffer[i])
      minin=red_buffer[i];  //update signal min value 
    if(maxin<red_buffer[i])
      maxin=red_buffer[i];  //update signal max
  }
   prev_data=red_buffer[i];
  //calculate heart rate and SpO2 after first 100 samples (first 4 seconds of samples)
   maxim_heart_rate_and_oxygen_saturation(ir_buffer, ir_buffer_length, red_buffer, &spo2, &spo2_valid, &heart_rate, &hr_valid); 
  while(1)
  {
    i=0;
    minin=0x3FFFF;//set the min and max value 
    maxin=0;

    //dumping the first 25 sets of samples in the memory and shift the last 75 sets of samples to the top
    for(i=25;i<100;i++)
    {
      red_buffer[i-25]=red_buffer[i];//0=25........74=99
      ir_buffer[i-25]=ir_buffer[i];

      //update the signal min and max
      if(minin>red_buffer[i])
        minin=red_buffer[i];
      if(maxin<red_buffer[i])
        maxin=red_buffer[i];
    }

    //take 25 sets of samples before calculating the heart rate.
    for(i=75;i<100;i++)
    {
      prev_data=red_buffer[i-1];
      while(digitalRead(10)==1);
      digitalWrite(9, !digitalRead(9));
      max30102_read_fifo((red_buffer+i), (ir_buffer+i));
      //Serial.println(spo2);
      delay(100);
      
    }
        maxim_heart_rate_and_oxygen_saturation(ir_buffer, ir_buffer_length, red_buffer, &spo2, &spo2_valid, &heart_rate, &hr_valid); 
  }
    
  
  
  while(Serial.available()>0){
    inbyte=Serial.read();
    if(inbyte=='1'){
      Serial.print(inbyte);
  
      Signal = spo2; 
      Serial.print('s');
     // Serial.print(inbyte);
      Serial.print(Signal);
      Serial.print('s');
      Serial.print("\n");
      delay(40);

      }
    else if(inbyte=='2'){
       Signal = analogRead(PulseSensorPurplePin); 
       Serial.print('s');
       Serial.print(Signal);
       Serial.print('s');
       Serial.print("\n");
       delay(40);
      }
    else if(inbyte=='3'){
      val_AD = 1023 - analogRead(SENSOR);
       Serial.print('s');
       Serial.print(val_AD); 
       Serial.print("\n");
       delay(200);
      }
      else if (inbyte=='4'){
      // val_AD = 1023 - analogRead(SENSOR);
       Serial.print('s');
       Serial.print(98); 
       Serial.print("\n");
       delay(200);
        }
      else if (inbyte=='5'){
      //val_AD = 1023 - analogRead(SENSOR);
       Serial.print('s');
       Serial.print(69); 
       Serial.print("\n");
       delay(200);
        }
     else if (inbyte=='6'){
        
  //Signal = analogRead(PulseSensorPurplePin); 
      Serial.print('s');
      Serial.print(16);
      Serial.print('s');
      Serial.print("\n");
      delay(40);
        }
        
    
   // Serial.println(inbyte);
    
    }
  
  // put your main code here, to run repeatedly:

}
