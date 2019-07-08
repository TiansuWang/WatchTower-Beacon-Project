#include "SPI.h"
#include "FS.h"
#include "RTClib.h"

//Node definition
uint8_t node_ID = 1;
uint8_t num_of_node = 2;

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


long int lasttimeCheckGPS = 0;
long int lasttimeCheck_unicast = 0;
long int lasttimeReport = 0;
long int lasttimeCheckACK = 0;
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

//configureation definition
uint8_t config_ID = 0;
uint8_t config_dest = 0;
String instring;

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

  
  receive_915(0x00,0x00,0x00);
  receive_434(0x00,0x00,0x00);

  // used for testing indoor 
//  node_longtitude.coordinate = 86.053978;
//  node_latitude.coordinate = 38.001604;
unicast_destination = node_ID - 1;
}



void loop() {
/*
 *serial port:
 */
 if (Serial.available() > 0) {
    // read the incoming byte:
    incomingByte = Serial.read();
    if(char(incomingByte) == 'e'){
      Serial.println("send echo");
      uint8_t echo_message[3]  = {};
      // ECH 0x45 0x43 0x48
      echo_message[0] = 0x45;
      echo_message[1] = 0x43;
      echo_message[2] = 0x48;
      transmit_434(echo_message,3);
      receive_434(0x00,0x00,0x00);
      
    }else if(char(incomingByte) == ','){
      config_ID = instring.toInt();
      instring = "";
    }else if(char(incomingByte) == '.'){
      config_dest = instring.toInt();
      instring = "";
      if(config_ID<20 & config_dest <config_ID){
        Serial.println("send configuration:");
        Serial.print("ID:");Serial.println(config_ID);
        Serial.print("Destination:");Serial.println(config_dest);
        uint8_t configure_message[5] = {};
        //CON 0x43 0x4F 0x4E   RPL 0x52 0x50 0x4C
        configure_message[0] = 0x43;
        configure_message[1] = 0x4F;
        configure_message[2] = 0x4E;
        configure_message[3] = config_ID;
        configure_message[4] = config_dest;
        transmit_434(configure_message,5);
        receive_434(0x00,0x00,0x00);
      }else{
        Serial.println("invalid comfiguration");
      }
      
      
    }else if(incomingByte == 10){
      instring = "";
    }else{
      instring += char(incomingByte);
      //Serial.println(instring);
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
    receive_915(0xFF,0x00,0x00);
    reset_flag_915 = false;
    }

////check if the RF433 has errors
    if(reset_flag_434){
      RF_reset_434();
      init_434M();
      receive_434(0x00,0x00,0x00);
      reset_flag_434 = false;
    }


}
