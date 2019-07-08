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
const int Rx_int_pin = 15;
const int Tx_int_pin = 39;
const int busy_pin = 32;
const int chipSelectPin_915  = 22;
const int nreset_pin = 14;

const int Rx_int_pin_434 = 12;
const int Tx_int_pin_434 = 13;
const int busy_pin_434 = 27;
const int chipSelectPin_434 = 4;
const int nreset_pin_434 = 33;

bool incoming_data_flag_915 = false;
bool transmit_done_flag_915 = false;

bool incoming_data_flag_434 = false;
bool transmit_done_flag_434 = false;

bool reset_flag_915 = false;
bool reset_flag_434 = false;

bool ACK_flag = false;


long int lasttimeGPSUpdate = 0;
long int lasttimeCheckGPS = 0;
long int lasttimeCheck_unicast = 0;
long int lasttimeReport = 0;
long int lasttimeCheckACK = 0;
int reset_count = 0;

//incomming packet information
uint8_t packet_dest_ID;
uint8_t packet_source_ID;
String packet_type;
uint8_t * packet_data;

//configureation definition
uint8_t config_ID = 0;
uint8_t config_dest = 0;
String instring;

//test environment;
uint8_t incomingByte = 0;
uint8_t long_echo_dest = 0;
uint8_t reconfig_node_ID = 0;

//long echo definition 
uint8_t try_to_find = 0;
//test 
long int perform_send = 0;

void setup() {

  // connect at 115200 so we can read the GPS fast enough and echo without dropping chars
  Serial.begin(115200);
  
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
  //receive_434(0x00,0x00,0x00);

}



void loop() {

 
  if(reset_flag_915){
    RF_reset_915();
    init_915M();
    receive_915(0x00,0x00,0x00);
    reset_flag_915 = false;
  }
  
  

////check if there are incoming data to RF915; if messages are being relayed from another node to this one
    if(incoming_data_flag_915 == true){
      uint8_t receive_buffer[40];
      uint8_t len;
      extract_rx_data(receive_buffer,&len); 
      //validate_packet(receive_buffer,len);
      transmit(receive_buffer,len);
      incoming_data_flag_915 = false;
      receive_915(0x00,0x00,0x00);
    }

    //check status of GPS and both RF chips every 10 seconds
  if ((millis()-lasttimeCheckGPS) >=10000) {
    lasttimeCheckGPS=millis();
    
      uint8_t * ptr1 = report_status();
      if (*ptr1 ==0 |*ptr1 ==1| *(ptr1+1) ==3 |*(ptr1+1) ==4|*(ptr1+1) ==5){
        Serial.println("915 Mhz Problem occured.");
        RF_reset_915();
        init_915M();
      }else{
        //Serial.println(" 915 Mhz Good so far"); 
      }

    }

}
