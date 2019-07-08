#include "SPI.h"
#include "FS.h"
#include "RTClib.h"

//Node definition
//uint8_t node_ID = 254;
//uint8_t unicast_destination = node_ID - 1;
//uint8_t unicast_backward_destination = node_ID + 1;
uint8_t node_ID = 1;
uint8_t unicast_destination = node_ID - 1;
uint8_t unicast_backward_destination = node_ID + 1;

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

DOUBLEUNION_t node_longitude;
DOUBLEUNION_t node_latitude;

uint8_t greenwich_hour;
uint8_t greenwich_minute;
uint8_t greenwich_seconds;

uint8_t rbt_pass_hour_start = 0;
uint8_t rbt_pass_minute_start = 0;
uint8_t rbt_pass_seconds_start = 0;

uint8_t rbt_pass_hour_end = 0;
uint8_t rbt_pass_minute_end = 0;
uint8_t rbt_pass_seconds_end = 0;

uint8_t rbt_pass_hour_MaxRssi = 0;
uint8_t rbt_pass_minute_MaxRssi = 0;
uint8_t rbt_pass_seconds_MaxRssi = 0;

int Rssi = -200;
int beacon_Rssi = -200;
int MaxRssi = -200;

//RF definition
const int Rx_int_pin = 39;
const int Tx_int_pin = 36;
const int busy_pin = 34;
const int chipSelectPin_915 = 21;
const int nreset_pin = 14;

const int Rx_int_pin_434 = 33;
const int Tx_int_pin_434 = 27;
const int busy_pin_434 = 15;
const int chipSelectPin_434 = 4;
const int nreset_pin_434 = 32;

bool incoming_data_flag_915 = false;
bool transmit_done_flag_915 = false;

bool incoming_data_flag_434 = false;
bool transmit_done_flag_434 = false;

bool reset_flag_915 = false;
bool reset_flag_434 = false;

bool config_done_flag = false;

bool wait_sending_ACK_on = false;
bool wait_passon = false;
bool wait_sending_LEB_on = false;

bool robot_passed_flag = false;

bool echo_timeout_on = false;
bool long_echo_timeout_on = false;
bool LEB_timeout_on = false;
bool second_long_echo_timeout_on = false;
bool succ_echo_flag = false;
bool signal_flag = false;

long int lasttimeGPSUpdate = 0;
long int lasttimeCheckGPS = 0;
long int lasttimeSendACK = 0;
long int lasttimeSendEcho = 0;
long int lasttimeSendLongEcho = 0;
long int lasttimeReceiveLongEcho = 0;
long int lasttimeSendLEB = 0;
long int lasttimeSignal = 0;
int reset_count = 0;
int unicast_count = 0;

//incomming packet information
uint8_t packet_dest_ID;
uint8_t packet_source_ID;
String packet_type;
uint8_t * packet_data;

//outcomming unicast message
uint8_t unicast_packet[255];
uint8_t unicast_len;

//LEC definition
uint8_t long_echo_dest = 0;
uint8_t long_echo_message_length = 8;
uint8_t long_echo_message[8] = {};

uint8_t long_echo_source = 0;
uint8_t hop_count = 0;
//LEB definition
uint8_t LEB_message[31] = {};
uint8_t LEB_message_length = 31;
uint8_t LEB_destination = 0;

//test environment;
uint8_t incomingByte = 0;

//OLED definition
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define OLED_RESET A0
Adafruit_SSD1306 display(OLED_RESET);

#define NUMFLAKES 10
#define XPOS 0
#define YPOS 1
#define DELTAY 2


#define LOGO16_GLCD_HEIGHT 16 
#define LOGO16_GLCD_WIDTH  16 

#if (SSD1306_LCDHEIGHT != 64)
#error("Height incorrect, please fix Adafruit_SSD1306.h!");
#endif

String last_434_error = "434 good so far";
String last_915_error = "915 good so far";

void setup() {

  // connect at 115200 so we can read the GPS fast enough and echo without dropping chars
  Serial.begin(115200);
  GPSSerial.begin(9600, SERIAL_8N1, 16, 17);
  display.begin(SSD1306_SWITCHCAPVCC, 0x3D);
  display.display();
  display.clearDisplay();
  
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
  display.setTextSize(1);
  display.setTextColor(WHITE);

  display.setCursor(0,0);
  display.print("434 Mhz reset ...");
  display.display();
  RF_reset_434();
  display.println("done");
  display.display();
  
  display.print("915 Mhz reset ...");
  display.display();
  RF_reset_915();
  display.println("done");
  display.display();
  
  display.print("init 434 Mhz  ...");
  display.display();
  init_434M();
  display.println("done");
  display.display();

  display.print("init 915 Mhz  ...");
  display.display();
  init_915M();
  display.println("done");
  display.display();

  display.print("set up GPS    ...");
  display.display();
  GPS_setup();
  display.println("done");
  display.display();
  
  display.print("434 RX,");
  display.display();
  receive_434(0x00,0x00,0x00);
  display.print("915 RX ...");
  display.display();
  receive_915(0x00,0x00,0x00);
  display.println("done");
  display.display();

  display.clearDisplay();
  display.setCursor(0,24);
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.println("Init done!");
  display.display();

}



void loop() {

 if(echo_timeout_on){
  if(millis() - lasttimeSendEcho >3000){
    uint8_t echo_message[4] = {};
    // ECB 1/0
    echo_message[0] = 0x45;
    echo_message[1] = 0x43;
    echo_message[2] = 0x42;
    echo_message[3] = succ_echo_flag;
    transmit_434(echo_message,4);
    succ_echo_flag = false;
    echo_timeout_on = false;
    receive_434(0x00,0x00,0x00);
  }
 }



 //count for 200ms give time to source back to RX
 if(wait_sending_ACK_on){
  if(millis() - lasttimeReceiveLongEcho > 200){
    send_ACK(packet_source_ID);
    lasttimeSendACK = millis();
    wait_sending_ACK_on = false;
    receive_915(0x00,0x00,0x00);
  }
 }

 //count for  another 200ms(totoal 400ms) give time to next hop back to RX
 if(!wait_sending_ACK_on & wait_passon){
  if(millis() - lasttimeSendACK > 200){
    wait_passon = false;
    transmit(long_echo_message,long_echo_message_length);
    lasttimeSendLongEcho = millis();
    long_echo_timeout_on = true;
    receive_915(0x00,0x00,0x00);
  }
 }

  if(!wait_sending_ACK_on & wait_sending_LEB_on){
    if(millis() - lasttimeSendACK > 200){
      wait_sending_LEB_on = false;
      transmit(LEB_message,LEB_message_length);
      lasttimeSendLEB = millis();
      //wait for ACK
      LEB_timeout_on = true;
      receive_915(0x00,0x00,0x00);
    }
  }

 if(long_echo_timeout_on){
  if(millis() - lasttimeSendLongEcho >2000){
    long_echo_timeout_on = false;
    transmit(long_echo_message,long_echo_message_length);
    lasttimeSendLongEcho = millis();
    //should code for another time out
    second_long_echo_timeout_on  = true;
    receive_915(0x00,0x00,0x00);
  }
 }
 
if(second_long_echo_timeout_on){
  if(millis() - lasttimeSendLongEcho >2000){
    second_long_echo_timeout_on = false;
    Serial.println("cannot reach the next hop, send back LEB message");
    //generate LEB message and send back
    generate_LEB_message();
    if(long_echo_source == node_ID){
          uint8_t self_generate_LEB_data_len = LEB_message_length-5;
          uint8_t self_generate_LEB_data[self_generate_LEB_data_len];
          for(int i = 0;i < LEB_message_length-5; i++){
            self_generate_LEB_data[i] = LEB_message[i+5];
          }
          report_LEB(self_generate_LEB_data,self_generate_LEB_data_len);
          receive_434(0x00,0x00,0x00);
    }else{
      transmit(LEB_message,LEB_message_length);
      lasttimeSendLEB = millis();
      //wait for ACK
      LEB_timeout_on = true;
      receive_915(0x00,0x00,0x00);
    }    
  }
} 

 if(LEB_timeout_on){
  if(millis() - lasttimeSendLEB >2000){
    LEB_timeout_on = false;
    transmit(LEB_message,LEB_message_length);
    lasttimeSendLEB = millis();
    receive_915(0x00,0x00,0x00);
    // no second time out
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
  
//update gps information and calibrate time as frequent as possible
    GPS_update();
  
////check status of GPS and both RF chips every 1 second
  if ((millis()-lasttimeCheckGPS) >=1000) {
    lasttimeCheckGPS=millis();
    
    //GPS_displayData();
    
      uint8_t * ptr1 = report_status();
      if (*ptr1 ==0 |*ptr1 ==1| *(ptr1+1) ==3 |*(ptr1+1) ==4|*(ptr1+1) ==5){
        Serial.println("915 Mhz Problem occured.");
        last_915_error = "915 error "+String(*ptr1)+" "+String(*(ptr1+1));
        //Serial.print(last_915_error);
        RF_reset_915();
        init_915M();
      }else{
        //Serial.println(" 915 Mhz Good so far"); 
      }
      uint8_t * ptr2 = report_status_434();
      if (*ptr2 ==0 |*ptr2 ==1| *(ptr2+1) ==3 |*(ptr2+1) ==4|*(ptr2+1) ==5){
        Serial.println("434 Mhz Problem occured.");
        last_434_error = "434 error "+String(*ptr2)+" "+String(*(ptr2+1));
        //Serial.print(last_915_error);
        RF_reset_434();
        init_434M();
      }else{
        //Serial.println("434 Mhz Good so far"); 
      }

      update_OLED();
    }
////check if there are incoming data to RF915; if messages are being relayed from another node to this one
    if(incoming_data_flag_915 == true){
      incoming_data_flag_915 = false;
        uint8_t * ptr1 = report_status();
        if(*(ptr1+1)==2){
          Serial.println("incomming packeage...");
          uint8_t receive_buffer[40];
          uint8_t len;
          extract_rx_data(receive_buffer,&len); 
          validate_packet(receive_buffer,len);
        }else{
          Serial.println("915 rx unsuccess...");
          last_915_error = "915 rx unsuccess.";
          update_OLED();
          clear_IRQ_915();
        }
        receive_915(0x00,0x00,0x00);
    }
////check if there are incoming data to RF433; if robot is here
    if(incoming_data_flag_434 == true){
      incoming_data_flag_434 = false;
      uint8_t * ptr1 = report_status_434();
      if(*(ptr1+1)==2){
        Serial.println("incomming packeage...");
        uint8_t receive_buffer[40];
        uint8_t len;
        extract_rx_data_434(receive_buffer,&len); 
        validate_packet_434(receive_buffer,len);
      }else{
        Serial.println("434 rx unsuccess...");
        last_434_error = "434 rx unsuccess.";
        update_OLED();
        //Serial.println("******:"+*(ptr1+1));
        //report_status_434_visible();
        clear_IRQ_434();
      }
      
      //delay(1000);// this line for signal strenth test
      receive_434(0x00,0x00,0x00);
    }

      if(signal_flag == true && (millis() - lasttimeSignal)>1000){
        signal_flag = false;
        update_OLED();
      }
}
