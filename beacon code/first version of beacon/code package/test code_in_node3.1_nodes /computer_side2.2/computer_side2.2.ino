#include "SPI.h"
#include "FS.h"
#include "RTClib.h"

//Node definition
const uint8_t node_ID = 0;

//GPS definition
#include <Adafruit_GPS.h>
#include <TinyGPS++.h>
#define GPSSerial Serial2
Adafruit_GPS GPS(&GPSSerial);
typedef union
{
  double coordinate;
  uint8_t bytes[8];
} 
DOUBLEUNION_t;

DOUBLEUNION_t computer_longtitude;
DOUBLEUNION_t computer_latitude;


uint8_t greenwich_hour;
uint8_t greenwich_minute;
uint8_t greenwich_seconds;
uint8_t rbt_pass_hour = 0;
uint8_t rbt_pass_minute = 0;
uint8_t rbt_pass_seconds = 0;
long int lasttimeGPSUpdate = 0;

//RF definition
const int Rx_int_pin = 15;
const int Tx_int_pin = 39;
const int busy_pin = 32;
const int chipSelectPin = 22;
const int nreset_pin = 14;
const int Rx_int_pin_434 = 12;
const int Tx_int_pin_434 = 13;
const int busy_pin_434 = 27;
const int chipSelectPin_434 = 4;
const int nreset_pin_434 = 33;


bool incoming_data_flag = false;
bool transmit_done_flag = false;
bool another_receive_flag = false;
bool reset_flag = false;
bool ACK_flag = false;

long int lasttimeCheck = 0;
long int lasttimeCheck_unicast = 0;
long int lasttimeReport = 0;
int reset_count = 0;
int unicast_count = 0;

//incomming packet information
DOUBLEUNION_t latitude_data[20];
DOUBLEUNION_t longtitude_data[20];
uint8_t robot_passed_flag[20] = {};
uint8_t robot_passed_hour[20] = {};
uint8_t robot_passed_minute[20] = {};
uint8_t robot_passed_seconds[20] = {};

uint8_t packet_dest_ID;
uint8_t packet_source_ID;
String packet_type;
uint8_t * packet_data;

//outcomming unicast message
uint8_t unicast_packet_array[5][255];
uint8_t unicast_len[5];
uint8_t unicast_pointer = 0;

//test environment;
int message_base = 0;
uint8_t incomingByte;
void setup() {

  // connect at 115200 so we can read the GPS fast enough and echo without dropping chars
  Serial.begin(115200);
  
  ////start RF chip
  pinMode(Tx_int_pin,INPUT);
  pinMode(Rx_int_pin,INPUT);//not needed if no receiving
  pinMode(busy_pin,INPUT);
  pinMode(nreset_pin,OUTPUT);
  pinMode(chipSelectPin,OUTPUT);
  
  pinMode(Tx_int_pin_434,INPUT);
  pinMode(Rx_int_pin_434,INPUT);//not needed if no receiving
  pinMode(busy_pin_434,INPUT);
  pinMode(nreset_pin_434,OUTPUT);
  pinMode(chipSelectPin_434,OUTPUT);

  digitalWrite(nreset_pin,LOW);
  digitalWrite(nreset_pin_434,LOW);
  
  SPI.begin(5, 19, 18, chipSelectPin); //SCLK, MISO, MOSI, SS

  delay(1000);
  
  RF_reset();

  init_915M();

  GPS_setup();
  
  receive(0xFF,0x00,0x00);
}



void loop() {
  //test broad cast assigning ID
  if (Serial.available() > 0) {
  
    // read the incoming byte:
    incomingByte = Serial.read();
    //Serial.print(char(incomingByte));
  }
  if(char(incomingByte) == '1'){
    Serial.println("1:check status:");
    report_status();
    
  }
  if(char(incomingByte) == '2'){
    Serial.println("2:check the data:");
    for(int i = 1;i<=5;i++){
      Serial.print("Node:");Serial.println(i);
      Serial.print("latitude:"); 
      Serial.println(latitude_data[i].coordinate,6);
      Serial.print("longtitude:");
      Serial.println(longtitude_data[i].coordinate,6);
      Serial.print("Passed?:");
      Serial.println(robot_passed_flag[i]);
      Serial.print("time:");
      Serial.print(robot_passed_hour[i]);
      Serial.print(":");
      Serial.print(robot_passed_minute[i]);
      Serial.print(":");
      Serial.println(robot_passed_seconds[i]);
    }
  }
/*
 * end test code
 */
  
  if(another_receive_flag){
    another_receive_flag = false;
    receive(0xFF,0x00,0x00);
  }

  if(ACK_flag){
    send_ACK(packet_source_ID);
    ACK_flag = false;
    receive(0xFF,0x00,0x00);
  }

  if(reset_flag){
    RF_reset();
    init_915M();
    receive(0xFF,0x00,0x00);
    reset_flag = false;
  }
  

  
  //update gps information 1 second
  if ((millis()-lasttimeGPSUpdate) >=1000) {
    GPS_update();
  }
  
  //check status and display GPS information every 10 seconds
  if ((millis()-lasttimeCheck) >=10000) {
    lasttimeCheck=millis();
    
    //GPS_displayData();
    
      uint8_t * ptr1 = report_status();
      if (*ptr1 ==0 |*ptr1 ==1| *(ptr1+1) ==3 |*(ptr1+1) ==4|*(ptr1+1) ==5){
        Serial.println("915 Mhz Problem occured.");
        RF_reset();
        init_915M();
        GPS_setup();
        receive(0xFF,0x00,0x00);
      }else{
        //Serial.println(" 915 Mhz Good so far"); 
      } 
    }
  
    if(incoming_data_flag == true){
      uint8_t receive_buffer[40];
      uint8_t len;
      extract_rx_data(receive_buffer,&len); 
      validate_packet(receive_buffer,len);
      incoming_data_flag = false;
      receive(0xFF,0x00,0x00);
    }
}
