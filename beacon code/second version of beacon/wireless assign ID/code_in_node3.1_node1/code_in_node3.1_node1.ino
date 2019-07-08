#include "SPI.h"
#include "FS.h"
#include "RTClib.h"

//Node definition
uint8_t node_ID = 1;
uint8_t num_of_node = 1;

//GPS definition
#include <Adafruit_GPS.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>
HardwareSerial GPSSerial(2);
//#define GPSSerial Serial2
Adafruit_GPS GPS(&GPSSerial);
typedef union
{
  double coordinate;
  uint8_t bytes[8];
} 
DOUBLEUNION_t;

DOUBLEUNION_t node_longtitude;
DOUBLEUNION_t node_latitude;
DOUBLEUNION_t BDC_longtitude;
DOUBLEUNION_t BDC_latitude;

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
const int chipSelectPin_915 = 22;
const int nreset_pin = 14;

const int Rx_int_pin_434 = 12;
const int Tx_int_pin_434 = 13;
const int busy_pin_434 = 27;
const int chipSelectPin_434 = 4;
const int nreset_pin_434 = 33;

bool incoming_data_flag_915 = false;
bool transmit_done_flag = false;

bool incoming_data_flag_434 = false;
bool transmit_done_flag_434 = false;

bool reset_flag_915 = false;
bool reset_flag_434 = false;

bool ACK_flag = false;

bool robot_passed_flag = false;

bool wait_echo_flag = false;
bool succ_echo_flag = false;


long int lasttimeCheckGPS = 0;
long int lasttimeCheck_unicast = 0;
long int lasttimeReport = 0;
long int lasttimeCheckACK = 0;
long int lasttimeSendEcho = 0;
int reset_count = 0;
int unicast_count = 0;

//incomming packet information
uint8_t packet_dest_ID;
uint8_t packet_source_ID;
String packet_type;
uint8_t * packet_data;

//outcomming unicast message
uint8_t unicast_packet_array[5][255];
uint8_t unicast_len[5];
uint8_t unicast_pointer = 0;
uint8_t unicast_destination = 0;

// exponential backoff information
// a S curve [300,1200]
float boundry[19] = {442.5, 501.4, 546.6, 584.7, 
                     618.3, 648.6, 676.60, 702.5, 
                     727, 750, 773.1, 797.6, 
                     823.6, 851.5, 881.9, 915.5, 
                     953.6, 998.9, 1057.9};
                     
int time_slot_length = 400;
int backoff_slot = 20;
long int backoff_start = 0;
bool broadcast_flag = 0;
uint8_t broadcast_source_ID = 0;

//test environment;
int message_base = 0;
uint8_t incomingByte = 0;


void setup() {

  // connect at 115200 so we can read the GPS fast enough and echo without dropping chars
  Serial.begin(115200);
  GPSSerial.begin(9600, SERIAL_8N1, 16, 17);
  
  ////start RF chip
  pinMode(Tx_int_pin,INPUT);
  pinMode(Rx_int_pin,INPUT);//not needed if no receiving
  pinMode(busy_pin,INPUT);
  pinMode(nreset_pin,OUTPUT);
  pinMode(chipSelectPin_915,OUTPUT);

  pinMode(Tx_int_pin_434,INPUT);
  pinMode(Rx_int_pin_434,INPUT);//not needed if no receiving
  pinMode(busy_pin_434,INPUT);
  pinMode(nreset_pin_434,OUTPUT);
  pinMode(chipSelectPin_434,OUTPUT);

  digitalWrite(nreset_pin,LOW);
  digitalWrite(nreset_pin_434,LOW);
  
  //SPI.begin(5, 19, 18, chipSelectPin_915); //SCLK, MISO, MOSI, SS
  SPI.begin(5, 19, 18, chipSelectPin_434); //SCLK, MISO, MOSI, SS
  
  delay(1000);
  
  RF_reset_915();
  RF_reset_434();

  init_434M();
  init_915M();

  GPS_setup();
  
  receive_915(0x00,0x00,0x00);
  receive_434(0x00,0x00,0x00);

  // used for testing indoor 
//  node_longtitude.coordinate = 86.053978;
//  node_latitude.coordinate = 38.001604;
unicast_destination = node_ID - 1;
}



void loop() {
/*
 * code for test :
 */
  if (Serial.available() > 0) {
  
    // read the incoming byte:
    incomingByte = Serial.read();
    //Serial.print(char(incomingByte));
  }
  if(char(incomingByte) == '1'){
    for(int i = 0; i<5 ; i++){
      for(int j = 0; j<26 ; j++){
        Serial.print(unicast_packet_array[i][j]);
      }
      Serial.println("");
    }
    
  }

/*
 * end test code
 */
 if(wait_echo_flag){
  if(millis() - lasttimeSendEcho >3000){
    uint8_t echo_message[4] = {};
    // ECB 1/0
    echo_message[0] = 0x45;
    echo_message[1] = 0x43;
    echo_message[2] = 0x42;
    echo_message[3] = succ_echo_flag;
    transmit_434(echo_message,4);
    succ_echo_flag = false;
    wait_echo_flag = false;
    receive_434(0x00,0x00,0x00);
  }
 }
  
 
  if(reset_flag_915){
    RF_reset_915();
    init_915M();
    receive_915(0x00,0x00,0x00);
    reset_flag_915 = false;
  }
  if(reset_flag_434){
    RF_reset_434();
    init_434M();
    receive_434(0x00,0x00,0x00);
    reset_flag_434 = false;
  }
  

  
////update gps information and calibrate time as frequent as possible
//  if ((millis()-lasttimeGPSUpdate) >=500) {
//    lasttimeGPSUpdate=millis();
    GPS_update();
    //Serial.println(millis()-lasttimeGPSUpdate);//check how fast gps and time updates
    //lasttimeGPSUpdate=millis();
  //}
  if ((millis()-lasttimeCheckACK) >=1000) {
    if(ACK_flag){
      lasttimeCheckACK = millis();
      send_ACK(packet_source_ID);
      ACK_flag = false;
      receive_915(0x00,0x00,0x00);
    }   
  }
  
////check status of GPS and both RF chips every 10 seconds
  if ((millis()-lasttimeCheckGPS) >=10000) {
    lasttimeCheckGPS=millis();
    
    GPS_displayData();
    
      uint8_t * ptr1 = report_status();
      if (*ptr1 ==0 |*ptr1 ==1| *(ptr1+1) ==3 |*(ptr1+1) ==4|*(ptr1+1) ==5){
        Serial.println("915 Mhz Problem occured.");
        RF_reset_915();
        init_915M();
      }else{
        //Serial.println(" 915 Mhz Good so far"); 
      }
      uint8_t * ptr2 = report_status_434();
      if (*ptr2 ==0 |*ptr2 ==1| *(ptr2+1) ==3 |*(ptr2+1) ==4|*(ptr2+1) ==5){
        Serial.println("434 Mhz Problem occured.");
        RF_reset_434();
        init_434M();
      }else{
        //Serial.println("434 Mhz Good so far"); 
      }
    }
////check if there are incoming data to RF915; if messages are being relayed from another node to this one
    if(incoming_data_flag_915 == true){
      uint8_t receive_buffer[40];
      uint8_t len;
      extract_rx_data(receive_buffer,&len); 
      validate_packet(receive_buffer,len);
      incoming_data_flag_915 = false;
      receive_915(0x00,0x00,0x00);
    }
////check if there are incoming data to RF433; if robot is here
    if(incoming_data_flag_434 == true){
      uint8_t receive_buffer[40];
      uint8_t len;
      extract_rx_data_434(receive_buffer,&len); 
      validate_packet_434(receive_buffer,len);
      incoming_data_flag_434 = false;
      //then no recieve
    }

////check if the RF915 has errors
    if(reset_flag_915){
    RF_reset_915();
    init_915M();
    receive_915(0x00,0x00,0x00);
    reset_flag_915 = false;
    }

////check if the RF433 has errors
    if(reset_flag_434){
      RF_reset_434();
      init_434M();
      receive_434(0x00,0x00,0x00);
      reset_flag_434 = false;
    }

//every cycle generate unicast message to the computer
    if ((millis()-lasttimeReport) >=1000) {
    lasttimeReport=millis();
//    Serial.print("greenwich_minute:"); Serial.println(greenwich_minute);
//    Serial.print("greenwich_seconds:"); Serial.println(greenwich_seconds);
      if(greenwich_minute%num_of_node == (node_ID-1) & greenwich_seconds ==30){
         generate_unicast_packet();
      }
    }
//
////for debug, comment out the above one and use this
//      if ((millis()-lasttimeReport) >=7000) {
//         lasttimeReport=millis();
//              generate_unicast_packet();
//          }


////try unicast every 3000ms, give time to get ACK
  if ((millis()-lasttimeCheck_unicast) >=3000) {
    lasttimeCheck_unicast =millis();
    Serial.print("unicast pointer:");Serial.println(unicast_pointer);
    //in case for buffer overflow
    if(unicast_pointer > 5){
      unicast_pointer = 1;
    }
    if(unicast_pointer > 0){
      if(unicast_count>2){
        //one hop attempt failed, try two hop
        if(unicast_packet_array[0][0]>0 & unicast_packet_array[0][0] == unicast_packet_array[0][1]-1){
          unicast_packet_array[0][0] = unicast_packet_array[0][0]-1;
          unicast_count = 0;
        }else{
          for(int i = 0;i<4;i++){
            memcpy(unicast_packet_array[i],unicast_packet_array[i+1],unicast_len[i+1]);
            unicast_len[i] = unicast_len[i+1];
          }
          unicast_pointer--;
          unicast_count = 0;
        }
        
      }else{
        transmit(unicast_packet_array[0],unicast_len[0]);
        unicast_count++;
        receive_915(0x00,0x00,0x00);
            for(int i = 0;i <unicast_len[0];i++){
                Serial.print(unicast_packet_array[0][i]);Serial.print(" ");
            }
            Serial.println("");
      }
       
    }
  }
}
