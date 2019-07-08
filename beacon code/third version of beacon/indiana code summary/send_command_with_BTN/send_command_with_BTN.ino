#include "SPI.h"
#include "FS.h"
#include "RTClib.h"

//Node definition
uint8_t node_ID = 255;

typedef union
{
  double coordinate;
  uint8_t bytes[8];
} 
DOUBLEUNION_t;

//RF definition
//const int Rx_int_pin = 15;
//const int Tx_int_pin = 39;
//const int busy_pin = 32;
//const int chipSelectPin_915 = 22;
//const int nreset_pin = 14;

//const int Rx_int_pin_434 = 12; //DIO1
//const int Tx_int_pin_434 = 13; //DIO2
//const int busy_pin_434 = 27;
//const int chipSelectPin_434 = 4;
//const int nreset_pin_434 = 33;

const int Rx_int_pin_434 = 33;
const int Tx_int_pin_434 = 27;
const int busy_pin_434 = 15;
const int chipSelectPin_434 = 4;
const int nreset_pin_434 = 32;

const int LED = 14;
const int off_botton = 13;
const int data_botton = 21;

bool incoming_data_flag_434 = false;
bool transmit_done_flag_434 = false;
bool reset_flag_434 = false;

long int lasttimeCheck = 0;

//test environment;
uint8_t incomingbyte = 0;

//Node definition
#define OFF 1
#define DAT 2
#define IDLE 3
int command = 0;

void setup() {

  // connect at 115200 so we can read the GPS fast enough and echo without dropping chars
  Serial.begin(115200);
  
  ////start RF chip
//  pinMode(Tx_int_pin,INPUT);
//  pinMode(Rx_int_pin,INPUT);//not needed if no receiving
//  pinMode(busy_pin,INPUT);
//  pinMode(nreset_pin,OUTPUT);
//  pinMode(chipSelectPin_915,OUTPUT);
  
  pinMode(Tx_int_pin_434,INPUT);
  pinMode(Rx_int_pin_434,INPUT);//not needed if no receiving
  pinMode(busy_pin_434,INPUT);
  pinMode(nreset_pin_434,OUTPUT);
  pinMode(chipSelectPin_434,OUTPUT);

  digitalWrite(nreset_pin_434,LOW);
  SPI.begin(5, 19, 18, chipSelectPin_434); //SCLK, MISO, MOSI, SS
  
  delay(200);

  RF_reset_434();

  init_434M();
  pinMode(LED,OUTPUT);
  pinMode(off_botton,OUTPUT);
  pinMode(data_botton,OUTPUT);
  digitalWrite(LED, 1);
  delay(500);
}



void loop() {
/*
 *serial port:
 */
 

  if(reset_flag_434){
    RF_reset_434();
    init_434M();
    receive_434(0x00,0x00,0x00);
    reset_flag_434 = false;
  }

  
  //check status every 10 seconds
  if ((millis()-lasttimeCheck) >=10000) {
    lasttimeCheck=millis();
    
      uint8_t * ptr1 = report_status_434();
      if (*ptr1 ==0 |*ptr1 ==1| *(ptr1+1) ==3 |*(ptr1+1) ==4|*(ptr1+1) ==5){
        Serial.println("Problem occured.");
        RF_reset_434();
        init_434M();
      }else{
        Serial.println("Good so far"); 
      }
    }

    if(digitalRead(off_botton) == 1){
      uint8_t off_command[8] = {87, 84, 82, 67, 79,0x4F,0x46,0x46};
      digitalWrite(LED, 1);
      delay(300);
      transmit_434(off_command,8);
      Serial.println("sending OFF!");
    }
    else if(digitalRead(data_botton) == 1){
      digitalWrite(LED, 1);
      delay(150);
      digitalWrite(LED, 0);
      delay(150);
      uint8_t dat_command[8] = {87, 84, 82, 67, 79,0x44,0x41,0x54};
      transmit_434(dat_command,8);
      Serial.println("sending DAT!");
    }else{
      digitalWrite(LED, 0);
    }
    
  

}
