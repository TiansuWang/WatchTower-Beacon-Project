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
#define chipSelectPin_915 A1
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
  receive_434(0x00,0x00,0x00);

}



void loop() {
/*
 *serial port:
 */
 if (Serial.available() > 0) {
    // read the incoming byte:
    incomingByte = Serial.read();
    
    if(char(incomingByte) == 'e'){
      while (Serial.available() > 0){
        incomingByte = Serial.read();

        if(incomingByte == 10){
          long_echo_dest = instring.toInt();
          
          if(long_echo_dest > 0 | instring.compareTo("0") == 0){
            try_to_find = long_echo_dest;
            Serial.print("send long echo to "); Serial.println(long_echo_dest);
            //Serial.println("");
            uint8_t long_echo_message[3]  = {};
            // LEC 0x4C 0x45 0x43
            long_echo_message[0] = 0x4C;
            long_echo_message[1] = 0x45;
            long_echo_message[2] = 0x43;
            long_echo_message[3] = long_echo_dest;
            transmit_434(long_echo_message,4);
            receive_434(0x00,0x00,0x00);
            instring = "";
            break;
            
          }else{
            Serial.println("send echo");
            uint8_t echo_message[3]  = {};
            // ECH 0x45 0x43 0x48
            echo_message[0] = 0x45;
            echo_message[1] = 0x43;
            echo_message[2] = 0x48;
            perform_send = millis();
            transmit_434(echo_message,3);
            receive_434(0x00,0x00,0x00);
            Serial.println(millis() - perform_send);
            break;
          }
        }
        instring += char(incomingByte);
      } 
    }else if(char(incomingByte) == 'r'){
      while (Serial.available() > 0){
        incomingByte = Serial.read();

        if(incomingByte == 10){
          reconfig_node_ID = instring.toInt();
          if(reconfig_node_ID > 0 | instring.compareTo("0") == 0){
            Serial.print("reconfigure "); Serial.println(reconfig_node_ID);
            uint8_t reconfig_message[4]  = {};
            // RCN 0x52 0x43 0x4E
            reconfig_message[0] = 0x52;
            reconfig_message[1] = 0x43;
            reconfig_message[2] = 0x4E;
            reconfig_message[3] = reconfig_node_ID;
            transmit_434(reconfig_message,4);
            receive_434(0x00,0x00,0x00);
            instring = "";
            break;
            
          }else{
            Serial.println("invalid reconfigure comand.");
            break;
          }
        }
        instring += char(incomingByte);
      }
    }else if(char(incomingByte) == ','){
      config_ID = instring.toInt();
      instring = "";
    }else if(char(incomingByte) == '.'){
      config_dest = instring.toInt();
      instring = "";
      if(config_dest==253|config_ID == 0 | (config_ID<30 & config_dest < config_ID)){
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
    }else if(char(incomingByte) == 'l'){ // 900 listen
      Serial.println("900 hz start listen");
      receive_915(0x00,0x00,0x00);
    }else if(char(incomingByte) == 'm'){ // 900 mute
      Serial.println("900 hz mute");
      wait();
      set_standby_rc();
    }else if(char(incomingByte) == 'c'){ 
      report_status_434_visible();
    }else if(char(incomingByte) == 'd'){ 
      Serial.println("send out DONE message");
      uint8_t done_message[3] = {};
      //DON 0x44 0x4F 0x4E
      done_message[0] = 0x44;
      done_message[1] = 0x4F;
      done_message[2] = 0x4E;
      transmit_434(done_message,3);
      receive_434(0x00,0x00,0x00);
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
      receive_434(0x00,0x00,0x00);
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
      uint8_t * ptr2 = report_status_434();
      if (*ptr2 ==0 |*ptr2 ==1| *(ptr2+1) ==3 |*(ptr2+1) ==4|*(ptr2+1) ==5){
        Serial.println("434 Mhz Problem occured.");
        report_status_434_visible();
        RF_reset_434();
        init_434M();
      }else{
        //Serial.println("434 Mhz Good so far"); 
      }
    }

}
