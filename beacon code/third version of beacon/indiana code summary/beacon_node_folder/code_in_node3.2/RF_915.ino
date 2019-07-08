//RF 
/*
 * ------------------------interrupt function-------------------
 */

void Receive_Int(){
  
  uint8_t * ptr1 = report_status();
  if(*(ptr1+1)==2){
    Serial.println("incomming packeage...");
    incoming_data_flag_915 = true;
    detachInterrupt(digitalPinToInterrupt(Rx_int_pin));
  }else{
    Serial.println("915 rx unsuccess...");
    last_915_error = "915 rx unsuccess.";
    update_OLED();
    receive_915(0x00,0x00,0x00);
  }
}

void Tx_done_Int(){
  Serial.println("Tx done...");
  transmit_done_flag_915 = true;
  detachInterrupt(digitalPinToInterrupt(Tx_int_pin));
}

/*-------------------------transport layer-----------------------
 * transport layer function
 * responsible for send and handle message 
 * varries form: computer node, sensor node.
 */
void validate_packet(uint8_t * receive_buffer,uint8_t len){
    for(int j = 0; j <len;j++){
      Serial.print(*(receive_buffer+j));Serial.print(" ");
    }
    Serial.println("");
    
    // check destination address
    packet_dest_ID = *(receive_buffer);
    Serial.print("packet_dest_ID:");Serial.println(packet_dest_ID);
    if(packet_dest_ID != node_ID & packet_dest_ID != 0xFF){
      //neither correct destination address nor broadcast packet
      Serial.println("Unicast not aim at me.");
      return;
    }
    packet_source_ID = *(receive_buffer+1);
    
    //check packet type
    char  type_char[3];
     for(int i = 0; i < 3 ; i++){
        type_char[i] = *(receive_buffer + 2 +i);
     }
    String packet_type = String(type_char);
    int packet_data_len = len-5;
    uint8_t packet_data[packet_data_len];
    for(int j = 0; j < packet_data_len ; j++){
        packet_data[j] = *(receive_buffer +  5 +j);
     }
    packet_type = packet_type.substring(0,3);
    Serial.print("packet_type:");Serial.println(packet_type);
    
/*
 * short echo
 */
    if(packet_type.compareTo("ECH") == 0){
      
      unicast_backward_destination = packet_source_ID;
      
      delay(100);
      uint8_t back_message[5] = {};
      back_message[0] = packet_source_ID;
      back_message[1] = node_ID;
      //BAC 0x42 0x41 0x43
      back_message[2] = 0x42;
      back_message[3] = 0x41;
      back_message[4] = 0x43;
      back_message[5] = GPS.fix;     
      transmit(back_message,6);
      update_OLED();
      
    }else if(packet_type.compareTo("BAC") == 0){
      succ_echo_flag = true;
      echo_timeout_on = false;
      uint8_t echo_message[4] = {};
      // ECB 1
      echo_message[0] = 0x45;
      echo_message[1] = 0x43;
      echo_message[2] = 0x42;
      echo_message[3] = succ_echo_flag;
      echo_message[4] = packet_data[0];
      transmit_434(echo_message,5);
      succ_echo_flag = false;
      receive_434(0x00,0x00,0x00);
      
 /*
  * long echo
  */
    }else if(packet_type.compareTo("LEC") == 0){
      long_echo_dest = packet_data[0];
      long_echo_source = packet_data[1];
      hop_count = packet_data[2] + 1;
      Serial.println("get LEC message");
      //send back ACK
      //count for 200ms give time to source back to RX
      lasttimeReceiveLongEcho = millis();
      wait_sending_ACK_on = true;
      if(long_echo_dest == node_ID){
        // generate LEB message 
        generate_LEB_message();
        wait_sending_LEB_on = true;

      }else if(packet_source_ID == unicast_destination | 
               packet_source_ID == unicast_backward_destination){
        //count for  another 200ms(totoal 400ms) give time to next hop back to RX
        // and then pass on backwards
        if(unicast_destination == 253 | unicast_backward_destination ==255){
          //means end of the link, need to report back
          //generate LEB
        generate_LEB_message();
        wait_sending_LEB_on = true;
  
        }else{
          wait_passon = true;
          //add one hop ,store the message for retransmition
          genarate_LEC_passon_message( long_echo_dest,
                                       long_echo_source,
                                       hop_count);
        } 
      }else{
        Serial.println("Error occurse when dealing with incoming LEC message.");
      }
    }else if(packet_type.compareTo("LEB") == 0){
      LEB_destination = packet_data[1];
      Serial.println("get LEB message");
      //send back ACK
      //count for 200ms give time to source back to 
      lasttimeReceiveLongEcho = millis();
      wait_sending_ACK_on = true;
      if(LEB_destination == node_ID){
        //if LEB is aim to itself
        report_LEB(packet_data,packet_data_len);
        receive_434(0x00,0x00,0x00);
      }else{
        //passon LEB
        genarate_LEB_passon_message(packet_data,packet_data_len);
        wait_sending_LEB_on = true;
      }
      
    }else if(packet_type.compareTo("ACK") == 0){
      long_echo_timeout_on = false;
      second_long_echo_timeout_on = false;
      LCB_timeout_on = false;
      Serial.println("unicast success!");
    }
}

 void send_ACK(uint8_t packet_source_ID){
  uint8_t ACK_packet[5] = {};
  ACK_packet[0] = packet_source_ID;
  ACK_packet[1] = node_ID;
  // ACK
  ACK_packet[2] = 0x41;
  ACK_packet[3] = 0x43;
  ACK_packet[4] = 0x4B;
  transmit(ACK_packet,5);
 }

 void genarate_LEC_passon_message(uint8_t find_node_ID,
                                  uint8_t long_echo_source,
                                  uint8_t hop_count){
  if(packet_source_ID == unicast_destination){
    long_echo_message[0] = unicast_backward_destination;
  }else{
    long_echo_message[0] = unicast_destination;
  }
  long_echo_message[1] = node_ID;
  // LEC 0x4C 0x45 0x43
  long_echo_message[2] = 0x4C;
  long_echo_message[3] = 0x45;
  long_echo_message[4] = 0x43;
  long_echo_message[5] = find_node_ID;
  long_echo_message[6] = long_echo_source;
  long_echo_message[7] = hop_count;
 }

void genarate_LEB_passon_message(uint8_t * packet_data, uint8_t packet_data_length){
  if(packet_source_ID == unicast_destination){
    LEB_message[0] = unicast_backward_destination;
  }else{
    LEB_message[0] = unicast_destination;
  }
  LEB_message[1] = node_ID;
  //LEB 0x4C 0x45 0x42
  LEB_message[2] = 0x4C; 
  LEB_message[3] = 0x45;
  LEB_message[4] = 0x42;
  for(int i = 0; i < packet_data_length; i++){
    LEB_message[5+i] = packet_data[i];
  }
  
}

void generate_LEB_message(){
  LEB_message[0] = packet_source_ID;
  LEB_message[1] = node_ID;
  //LEB 0x4C 0x45 0x42
  LEB_message[2] = 0x4C; 
  LEB_message[3] = 0x45;
  LEB_message[4] = 0x42;

  LEB_message[5] = node_ID;
  LEB_message[6] = long_echo_source;
  LEB_message[7] = hop_count;
  for(int i = 0; i<8 ; i++){
      LEB_message[8+i] = node_latitude.bytes[i];
      LEB_message[16+i] = node_longitude.bytes[i];
    }
  LEB_message[24] = robot_passed_flag;

  LEB_message[25] = rbt_pass_hour_MaxRssi;
  LEB_message[26] = rbt_pass_minute_MaxRssi;
  LEB_message[27] = rbt_pass_seconds_MaxRssi;

  long int start_totoal_second = rbt_pass_hour_start * 3600
                             +rbt_pass_minute_start * 60
                             +rbt_pass_seconds_start;
  long int end_totoal_second = rbt_pass_hour_end * 3600
                           +rbt_pass_minute_end * 60
                           +rbt_pass_seconds_end;
  if(end_totoal_second < start_totoal_second){
      end_totoal_second = end_totoal_second + 24 * 3600;
  }
  long int average_totoal_second = (end_totoal_second + start_totoal_second)/2;
  if(average_totoal_second > 24 * 3600){
      average_totoal_second = average_totoal_second - 24 * 3600;
  }
  LEB_message[28] = average_totoal_second / 3600;
  LEB_message[29] = (average_totoal_second - LEB_message[28] * 3600)/60;
  LEB_message[30] = average_totoal_second % 60;
}

void report_LEB(uint8_t * packet_data,uint8_t packet_data_length){
  uint8_t report_LEB_message[packet_data_length] = {};
//LEB 0x4C 0x45 0x42
  report_LEB_message[0] = 0x4C; 
  report_LEB_message[1] = 0x45;
  report_LEB_message[2] = 0x42;
  for(int i = 0; i < packet_data_length; i++){
    report_LEB_message[3+i] = *(packet_data+i);
  }
  transmit_434(report_LEB_message,packet_data_length + 3);
}
/*-------------------------physical layer-----------------------
 * physical layer function
 * responsible for initialization, status control. 
 */


void RF_reset_915(){
  digitalWrite(nreset_pin,LOW);
  delay(100);
  digitalWrite(nreset_pin,HIGH);
  wait();
  delay(500);
  for (int i=0;i<3;i++){
    set_standby_rc();
    wait();
    uint8_t * ptr = report_status();
    if (*ptr == 2 & *(ptr+1) ==1){
      Serial.println("******915 Mhz Reset done.Standing by.....******");
      return;
    }    
    delay(500);
  }
  Serial.println("Reset unsuccess.");
  while (1);
}

void init_915M(){
  set_Lora_Packagetype();
  set_dio3_as_tcxo_ctrl();
  set_frequency();
  image_calibration();
  set_pa_config();
  SetRxTxFallbackMode();
  set_boost_rx();
  set_buff_base_addr(0x64,0x00);
  set_modulation_parameter();
  set_packet_parameter(40);
  set_dio_irq_parameter();
  uint8_t * ptr = report_status();
    if (*ptr == 2 & *(ptr+1) ==1){
      Serial.println("******915 Mhz Initialization done.Standing by.....******");
    } else{
      Serial.println("******915 Mhz Initialization unsuccess.******");
      while (1);  
    }
}

void receive_915(uint8_t timeout1,uint8_t timeout2,uint8_t timeout3){
    //step 0: attach interupt
    attachInterrupt(digitalPinToInterrupt(Rx_int_pin), Receive_Int, CHANGE);
    wait();
    set_rx(timeout1,timeout2,timeout3);  
}

void extract_rx_data(uint8_t * receive_buffer,uint8_t * len){
  
  // step 1:get the buffer information
    wait();
    uint8_t * ptr2 = GetRxBufferStatus();
    uint8_t PayloadLengthRx = *ptr2;
    uint8_t RxStartBufferPointer = *(ptr2+1);
    
    // step 2:read data from buffer and print it out
    wait();
    uint8_t * buff = read_buffer(RxStartBufferPointer,PayloadLengthRx);
    Serial.println("done.");
    memcpy(receive_buffer, buff, PayloadLengthRx);
    *len = PayloadLengthRx;
    
    // step 3:print out Rssi
    wait();
    GetPacketStatus();
    
    // step 4:clear Irq
    wait();
    ClearIrqStatus();
    wait();
    uint8_t * ptr3 = get_irq_status();
    uint8_t ClearIrqCount = 0;
    if (*(ptr3+1) ==0){
        Serial.println("Successful reset IRQ.");
      }else{
        Serial.println("Unsuccessful reset IRQ.");
        do{
          wait();
          ClearIrqStatus();  
          ClearAllIrqStatus();
          ptr3 = get_irq_status();  
          delay(500);  
          ClearIrqCount++;                    
        }while(*(ptr3+1) != 0|ClearIrqCount>5);
        ClearIrqCount = 0;
        if(*(ptr3+1) != 0){
          reset_flag_915 = true;
        }
      }
}


void transmit(uint8_t * transmit_buffer,uint8_t len){

  //wait packet sent
  uint8_t * ptr1 = report_status();
  while (*ptr1 == 6){
    delay(200);
    ptr1 = report_status();
  }
  
  //step 0:attach interrupt
  attachInterrupt(digitalPinToInterrupt(Tx_int_pin), Tx_done_Int, CHANGE);

  //step 1: write transmit buffer
  write_buff(transmit_buffer,len);
  wait();
  set_packet_parameter(len);
  //step 2: transmit
  wait();
  set_tx();
  //step 3:wait until transimit done
  while(transmit_done_flag_915 == false){
      delay(10);
    }
  // step 5:clear Irq
    wait();
    ClearIrqStatus();
    wait();
    uint8_t ClearIrqCount = 0;
    uint8_t * ptr2 = get_irq_status();  
    if (*(ptr2+1) ==0){
        Serial.println("Successful reset IRQ.");
      }else{
        Serial.println("Unsuccessful reset IRQ.");
        last_915_error = "915 reset IRQ error.";
        update_OLED();
        do{
          wait();
          ClearIrqStatus();  
          ClearAllIrqStatus();
          ptr2 = get_irq_status();  
          delay(500);  
          ClearIrqCount++;                    
        }while(*(ptr2+1) != 0|ClearIrqCount>10);
        ClearIrqCount = 0;
        if(*(ptr2+1) != 0){
          reset_flag_915 = true;
        }
      }
   //step 6: clear flag
    transmit_done_flag_915 = false;
}

uint8_t * report_status(){
  static uint8_t return_val[2];
  uint8_t stat = check_status();
  // read Chip mode 
  uint8_t chip_mode;
  chip_mode = stat << 1;
  chip_mode = chip_mode >>5;
//  Serial.print("chip mode:");
//  switch (chip_mode){
//    case 0:
//      Serial.println("0x0 Unused");
//      break;
//    case 1:
//      Serial.println("0x1 RFU");
//      break;
//    case 2:
//      Serial.println("0x2 STBY_RC");
//      break;
//    case 3:
//      Serial.println("0x3 STBY_XOSC");
//      break;
//    case 4:
//      Serial.println("0x4 FS");
//      break;
//    case 5:
//      Serial.println("0x5 RX");
//      break;
//    case 6:
//      Serial.println("0x6 TX");
//  }
  //read command status
  uint8_t command_status;
  command_status = stat << 4;
  command_status = command_status >>5;
//  Serial.print("command status:");
//  switch (command_status){
//    case 0:
//      Serial.println("0x0 Reserved");
//      break;
//    case 1:
//      Serial.println("0x1 RFU");
//      break;
//    case 2:
//      Serial.println("0x2 Data available to host");
//      break;
//    case 3:
//      Serial.println("0x3 Command timeout");
//      break;
//    case 4:
//      Serial.println("0x4 command processing error");
//      break;
//    case 5:
//      Serial.println("0x5 Failure to execute command");
//      break;
//    case 6:
//      Serial.println("0x6 command TX done");
//  }
  return_val[0] = chip_mode;
  return_val[1] = command_status;
  return return_val;
}

/*---------------------------operation function-----------------------
 * following is basic operation function
 * they put the spi command into the radio chip
 */

void wait(){
  while(digitalRead(busy_pin)){
  delay(100);
  }
}

void set_Lora_Packagetype(){
  digitalWrite(chipSelectPin_915,LOW);
  uint8_t re1 = SPI.transfer(0x8A);
  uint8_t re2 = SPI.transfer(0x01);
  digitalWrite(chipSelectPin_915,HIGH);
//  PrintHex8(&re1,1);
//  PrintHex8(&re2,1);
  Serial.println("Finish set package type to lora");
}

void set_sleep(){
  digitalWrite(chipSelectPin_915,LOW);
  uint8_t re1 = SPI.transfer(0x84);
  uint8_t re2 = SPI.transfer(0x04);
  digitalWrite(chipSelectPin_915,HIGH);
//  PrintHex8(&re1,1);Serial.println("");
//  PrintHex8(&re2,1);Serial.println("");
  Serial.println("sleep....");
}

void set_standby_rc(){
  digitalWrite(chipSelectPin_915,LOW);
  uint8_t re1 = SPI.transfer(0x80);
  uint8_t re2 = SPI.transfer(0x00);
  digitalWrite(chipSelectPin_915,HIGH);
//  PrintHex8(&re1,1);Serial.println("");
//  PrintHex8(&re2,1);Serial.println("");
  Serial.println("standing by(RC)....");
}

void set_standby_xosc(){
  digitalWrite(chipSelectPin_915,LOW);
  uint8_t re1 = SPI.transfer(0x80);
  uint8_t re2 = SPI.transfer(0x01);
  digitalWrite(chipSelectPin_915,HIGH);
//  PrintHex8(&re1,1);Serial.println("");
//  PrintHex8(&re2,1);Serial.println("");
  Serial.println("standing by(XOSC)....");
}

uint8_t check_status(){
  digitalWrite(chipSelectPin_915,LOW);
  uint8_t stat = SPI.transfer(0xC0);
  digitalWrite(chipSelectPin_915,HIGH);
  //Serial.println("curent status:");
  //PrintHex8(&stat,1);Serial.println("");
  return stat;
}

void image_calibration(){
  digitalWrite(chipSelectPin_915,LOW);
  uint8_t re1 = SPI.transfer(0x98);
  uint8_t re2 = SPI.transfer(0xE1);
  uint8_t re3 = SPI.transfer(0xE9);
  digitalWrite(chipSelectPin_915,HIGH);
  Serial.println("image calibrating...");
  wait();
  Serial.println("Done.");
//  PrintHex8(&re1,1);Serial.println("");
//  PrintHex8(&re2,1);Serial.println("");
//  PrintHex8(&re3,1);Serial.println("");
}

void calibration_function(){
    digitalWrite(chipSelectPin_915,LOW);
  uint8_t re1 = SPI.transfer(0x89);
  uint8_t re2 = SPI.transfer(0x7F);
  digitalWrite(chipSelectPin_915,HIGH);
  Serial.println("calibrating all...");
  wait();
  Serial.println("Done.");
//  PrintHex8(&re1,1);Serial.println("");
//  PrintHex8(&re2,1);Serial.println("");
}

void set_frequency(){
  //SPI.transfer(buffer, size)
  digitalWrite(chipSelectPin_915,LOW);
  uint8_t re1 = SPI.transfer(0x86);
  uint8_t re2 = SPI.transfer(0x39);
  uint8_t re3 = SPI.transfer(0x30);
  uint8_t re4 = SPI.transfer(0x00);
  uint8_t re5 = SPI.transfer(0x00);
  digitalWrite(chipSelectPin_915,HIGH);
  Serial.println("setting RF frequency...");
  wait();
  Serial.println("Done.");
//  PrintHex8(&re1,1);Serial.println("");
//  PrintHex8(&re2,1);Serial.println("");
//  PrintHex8(&re3,1);Serial.println("");
//  PrintHex8(&re4,1);Serial.println("");
//  PrintHex8(&re5,1);Serial.println("");
}

void set_fs(){
  digitalWrite(chipSelectPin_915,LOW);
  uint8_t re1 = SPI.transfer(0xC1);
  digitalWrite(chipSelectPin_915,HIGH);
  Serial.println("setting frequency...");
  wait();
  Serial.println("Done. Frequency setting mode...");
  //PrintHex8(&re1,1);Serial.println("");
}

void set_pa_config(){
  digitalWrite(chipSelectPin_915,LOW);
  uint8_t re1 = SPI.transfer(0x95);
  uint8_t re2 = SPI.transfer(0x04);
  uint8_t re3 = SPI.transfer(0x07);
  uint8_t re4 = SPI.transfer(0x00);
  uint8_t re5 = SPI.transfer(0x01);
  digitalWrite(chipSelectPin_915,HIGH);
  Serial.println("setting pa config...");
  wait();
  Serial.println("Done.");
//  PrintHex8(&re1,1);Serial.println("");
//  PrintHex8(&re2,1);Serial.println("");
//  PrintHex8(&re3,1);Serial.println("");
//  PrintHex8(&re4,1);Serial.println("");
//  PrintHex8(&re5,1);Serial.println("");
}

void set_tx_params(){
  digitalWrite(chipSelectPin_915,LOW);
  uint8_t re1 = SPI.transfer(0x8E);
  uint8_t re2 = SPI.transfer(0x16);
  uint8_t re3 = SPI.transfer(0x07);
  digitalWrite(chipSelectPin_915,HIGH);
  Serial.println("setting Tx parameters...");
  wait();
  Serial.println("Done.");
  //PrintHex8(&re1,1);Serial.println("");
  //PrintHex8(&re2,1);Serial.println("");
  //PrintHex8(&re3,1);Serial.println("");
}

void set_buff_base_addr(uint8_t tx_addr,uint8_t rx_addr){
  digitalWrite(chipSelectPin_915,LOW);
  uint8_t re1 = SPI.transfer(0x8F);
  uint8_t re2 = SPI.transfer(tx_addr);
  uint8_t re3 = SPI.transfer(rx_addr);
  digitalWrite(chipSelectPin_915,HIGH);
  Serial.println("setting base address...");
  wait();
  Serial.println("Done.");
//  PrintHex8(&re1,1);Serial.println("");
//  PrintHex8(&re2,1);Serial.println("");
//  PrintHex8(&re3,1);Serial.println("");
}

void write_buff(uint8_t * transmit_buffer,uint8_t len){
  digitalWrite(chipSelectPin_915,LOW);
  Serial.println("writing buffer...");
  wait();
  uint8_t re1 = SPI.transfer(0x0E);
  uint8_t re2 = SPI.transfer(0x64);
//  PrintHex8(&re1,1);Serial.println("");
//  PrintHex8(&re2,1);Serial.println("");
  for (int i=0; i<len;i++){
    wait();
    uint8_t re3 = SPI.transfer(transmit_buffer[i]);
    //PrintHex8(&re3,1);Serial.println("");
  }
  digitalWrite(chipSelectPin_915,HIGH);
  wait();
  Serial.println("Done.");
}


void set_modulation_parameter(){
  digitalWrite(chipSelectPin_915,LOW);
  uint8_t re1 = SPI.transfer(0x8B);
  uint8_t re2 = SPI.transfer(0x0A);
  uint8_t re3 = SPI.transfer(0x05);
  uint8_t re4 = SPI.transfer(0x04);
  uint8_t re5 = SPI.transfer(0x00);
  digitalWrite(chipSelectPin_915,HIGH);
  Serial.println("setting modulation parameter...");
  wait();
  Serial.println("Done.");
//  PrintHex8(&re1,1);Serial.println("");
//  PrintHex8(&re2,1);Serial.println("");
//  PrintHex8(&re3,1);Serial.println("");
//  PrintHex8(&re4,1);Serial.println("");
//  PrintHex8(&re5,1);Serial.println("");
}

void set_packet_parameter(int len){
  digitalWrite(chipSelectPin_915,LOW);
  uint8_t re1 = SPI.transfer(0x8C);
  uint8_t re2 = SPI.transfer(0x00);
  uint8_t re3 = SPI.transfer(0x08);
  uint8_t re4 = SPI.transfer(0x00);
  uint8_t re5 = SPI.transfer(len);
  uint8_t re6 = SPI.transfer(0x01);
  uint8_t re7 = SPI.transfer(0x00);
  digitalWrite(chipSelectPin_915,HIGH);
  Serial.println("setting packet parameter...");
  wait();
  Serial.println("Done.");
//  PrintHex8(&re1,1);Serial.println("");
//  PrintHex8(&re2,1);Serial.println("");
//  PrintHex8(&re3,1);Serial.println("");
//  PrintHex8(&re4,1);Serial.println("");
//  PrintHex8(&re5,1);Serial.println("");
//  PrintHex8(&re6,1);Serial.println("");
//  PrintHex8(&re7,1);Serial.println("");
}

void set_dio_irq_parameter(){
  digitalWrite(chipSelectPin_915,LOW);
  uint8_t re1 = SPI.transfer(0x08);
  uint8_t re2 = SPI.transfer(0x00);
  uint8_t re3 = SPI.transfer(0x03);
  uint8_t re4 = SPI.transfer(0x00);
  uint8_t re5 = SPI.transfer(0x02);
  uint8_t re6 = SPI.transfer(0x00);
  uint8_t re7 = SPI.transfer(0x01);
  uint8_t re8 = SPI.transfer(0x00);
  uint8_t re9 = SPI.transfer(0x00);
  digitalWrite(chipSelectPin_915,HIGH);
  Serial.println("setting IRQ...");
  wait();
  Serial.println("Done.");
//  PrintHex8(&re1,1);Serial.println("");
//  PrintHex8(&re2,1);Serial.println("");
//  PrintHex8(&re3,1);Serial.println("");
//  PrintHex8(&re4,1);Serial.println("");
//  PrintHex8(&re5,1);Serial.println("");
//  PrintHex8(&re6,1);Serial.println("");
//  PrintHex8(&re7,1);Serial.println("");
//  PrintHex8(&re8,1);Serial.println("");
//  PrintHex8(&re9,1);Serial.println("");
}
void ClearIrqStatus(){
  digitalWrite(chipSelectPin_915,LOW);
  uint8_t re1 = SPI.transfer(0x02);
  uint8_t re2 = SPI.transfer(0x00);
  uint8_t re3 = SPI.transfer(0x03);
  digitalWrite(chipSelectPin_915,HIGH);
  Serial.println("clearing IRQ flags...");
  wait();
  Serial.println("done");
}
void ClearAllIrqStatus(){
  digitalWrite(chipSelectPin_915,LOW);
  uint8_t re1 = SPI.transfer(0x02);
  uint8_t re2 = SPI.transfer(0x7F);
  uint8_t re3 = SPI.transfer(0xFF);
  digitalWrite(chipSelectPin_915,HIGH);
  Serial.println("clearing IRQ flags...");
  wait();
  Serial.println("done");
}

uint8_t * get_irq_status(){
  static uint8_t irq_status[2];
  digitalWrite(chipSelectPin_915,LOW);
  uint8_t re1 = SPI.transfer(0x12);
  uint8_t re2 = SPI.transfer(0x00);
  uint8_t re3 = SPI.transfer(0x00);
  uint8_t re4 = SPI.transfer(0x00);
  digitalWrite(chipSelectPin_915,HIGH);
  Serial.println("getting IRQ status...");
  wait();
  Serial.println("Done.");
  irq_status[0] = re3;
  irq_status[1] = re4;
//  PrintHex8(&re1,1);Serial.println("");
//  PrintHex8(&re2,1);Serial.println("");
//  PrintHex8(&re3,1);Serial.println("");
//  PrintHex8(&re4,1);Serial.println("");
  return irq_status;
}

void set_tx(){
  digitalWrite(chipSelectPin_915,LOW);
  uint8_t re1 = SPI.transfer(0x83);
  uint8_t re2 = SPI.transfer(0x00);
  uint8_t re3 = SPI.transfer(0x00);
  uint8_t re4 = SPI.transfer(0x00);
  digitalWrite(chipSelectPin_915,HIGH);
  Serial.println("setting TX...");
  wait();
  Serial.println("Done.");
//  PrintHex8(&re1,1);Serial.println("");
//  PrintHex8(&re2,1);Serial.println("");
//  PrintHex8(&re3,1);Serial.println("");
//  PrintHex8(&re4,1);Serial.println("");
}

void set_rx(uint8_t timeout1,uint8_t timeout2,uint8_t timeout3){
  digitalWrite(chipSelectPin_915,LOW);
  uint8_t re1 = SPI.transfer(0x82);
  uint8_t re2 = SPI.transfer(timeout1);
  uint8_t re3 = SPI.transfer(timeout2);
  uint8_t re4 = SPI.transfer(timeout3);
  digitalWrite(chipSelectPin_915,HIGH);
  Serial.println("setting RX...");
  wait();
  Serial.println("Done.");
//  PrintHex8(&re1,1);Serial.println("");
//  PrintHex8(&re2,1);Serial.println("");
//  PrintHex8(&re3,1);Serial.println("");
//  PrintHex8(&re4,1);Serial.println("");
}

void set_dio3_as_tcxo_ctrl(){
  digitalWrite(chipSelectPin_915,LOW);
  uint8_t re1 = SPI.transfer(0x97);
  uint8_t re2 = SPI.transfer(0x06);
  uint8_t re3 = SPI.transfer(0x01);
  uint8_t re4 = SPI.transfer(0x01);
  uint8_t re5 = SPI.transfer(0x01);
  digitalWrite(chipSelectPin_915,HIGH);
  Serial.println("set_dio3_as_tcxo_ctrl...");
  wait();
  Serial.println("Done.");
//  PrintHex8(&re1,1);Serial.println("");
//  PrintHex8(&re2,1);Serial.println("");
//  PrintHex8(&re3,1);Serial.println("");
//  PrintHex8(&re4,1);Serial.println("");
//  PrintHex8(&re5,1);Serial.println("");
}

void get_device_errors(){
  digitalWrite(chipSelectPin_915,LOW);
  uint8_t re1 = SPI.transfer(0x17);
  uint8_t re2 = SPI.transfer(0x00);
  uint8_t re3 = SPI.transfer(0x00);
  uint8_t re4 = SPI.transfer(0x00);
  digitalWrite(chipSelectPin_915,HIGH);
  Serial.println("getting device error...");
  wait();
  Serial.println("Done.");
  PrintHex8(&re1,1);Serial.println("");
  PrintHex8(&re2,1);Serial.println("");
  PrintHex8(&re3,1);Serial.println("");
  PrintHex8(&re4,1);Serial.println("");
}

void clear_device_errors(){
  digitalWrite(chipSelectPin_915,LOW);
  uint8_t re1 = SPI.transfer(0x07);
  uint8_t re2 = SPI.transfer(0x00);
  digitalWrite(chipSelectPin_915,HIGH);
  Serial.println("clearing device error...");
  wait();
  Serial.println("Done.");
  //PrintHex8(&re1,1);Serial.println("");
  //PrintHex8(&re2,1);Serial.println("");
}

uint8_t * read_buffer(uint8_t offset,int count){
  Serial.println("Reading buffer:");
  static uint8_t receive_buffer[255];
  digitalWrite(chipSelectPin_915,LOW);
  uint8_t re1 = SPI.transfer(0x1E);
  uint8_t re2 = SPI.transfer(offset);
  uint8_t re3 = SPI.transfer(0x00);
  //PrintHex8(&re1,1);Serial.println("");
  //PrintHex8(&re2,1);Serial.println("");
  for (int i=0; i<count;i++){
    wait();
    uint8_t re4 = SPI.transfer(0x00);
    //PrintHex8(&re3,1);Serial.println("");
    receive_buffer[i] = re4;
  }
  digitalWrite(chipSelectPin_915,HIGH);
  return receive_buffer;
}

/*
 * return a pointer 
 * *ptr = PayloadLengthRx
 * *(ptr+1) = RxStartBufferPointer
 */
uint8_t * GetRxBufferStatus(){
  static uint8_t rx_buffer_status[2];
  digitalWrite(chipSelectPin_915,LOW);
  uint8_t re1 = SPI.transfer(0x13);
  uint8_t re2 = SPI.transfer(0x00);
  uint8_t re3 = SPI.transfer(0x00);
  uint8_t re4 = SPI.transfer(0x00);
  digitalWrite(chipSelectPin_915,HIGH);
//  PrintHex8(&re1,1);Serial.println("");
//  PrintHex8(&re2,1);Serial.println("");
//  PrintHex8(&re3,1);Serial.println("");
//  PrintHex8(&re4,1);Serial.println("");
  rx_buffer_status[0] = re3;
  rx_buffer_status[1] = re4;
  return rx_buffer_status;
}

void GetPacketStatus(){
  digitalWrite(chipSelectPin_915,LOW);
  uint8_t re1 = SPI.transfer(0x14);
  uint8_t re2 = SPI.transfer(0x00);
  uint8_t re3 = SPI.transfer(0x00);
  uint8_t re4 = SPI.transfer(0x00);
  digitalWrite(chipSelectPin_915,HIGH);
//  PrintHex8(&re1,1);Serial.println("");
//  PrintHex8(&re2,1);Serial.println("");
//  PrintHex8(&re3,1);Serial.println("");
//  PrintHex8(&re4,1);Serial.println("");
  Serial.print("SNR:");
  Serial.print(re3 / 4);
  Serial.println(" dB");
  Serial.print("Rssi:");
  Serial.print(-re4 / 2);
  Serial.println(" dBm");
}

void GetRssilnst(){
  digitalWrite(chipSelectPin_915,LOW);
  uint8_t re1 = SPI.transfer(0x15);
  uint8_t re2 = SPI.transfer(0x00);
  uint8_t re3 = SPI.transfer(0x00);
  digitalWrite(chipSelectPin_915,HIGH);
  PrintHex8(&re1,1);Serial.println("");
  PrintHex8(&re2,1);Serial.println("");
  PrintHex8(&re3,1);Serial.println("");
  Serial.print("Rssi:");
  Serial.println(-re3 / 2);
}

/*
 * FS         0x40 The radio goes into FS mode after Tx or Rx
 * STDBY_XOSC 0x30 The radio goes into STDBY_XOSC mode after Tx or Rx
 * STDBY_RC   0x20 The radio goes into STDBY_RC mode after Tx or Rx
 */
void SetRxTxFallbackMode(){
  digitalWrite(chipSelectPin_915,LOW);
  uint8_t re1 = SPI.transfer(0x93);
  uint8_t re2 = SPI.transfer(0x40);
  digitalWrite(chipSelectPin_915,HIGH);
  Serial.println("SetRxTxFallbackMode to FS");
  wait();
  Serial.println("done");
}

void ResetStats(){
  digitalWrite(chipSelectPin_915,LOW);
  for (int i=0;i<7;i++){
    uint8_t re = SPI.transfer(0x00);
  }
  digitalWrite(chipSelectPin_915,HIGH);
}

void SetRxDutyCycle(){
  digitalWrite(chipSelectPin_915,LOW);
  uint8_t re1 = SPI.transfer(0x94);
  uint8_t re2 = SPI.transfer(0x00);
  uint8_t re3 = SPI.transfer(0xFA);
  uint8_t re4 = SPI.transfer(0x00);
  uint8_t re5 = SPI.transfer(0x07);
  uint8_t re6 = SPI.transfer(0xD0);
  uint8_t re7 = SPI.transfer(0x00);
  digitalWrite(chipSelectPin_915,HIGH);
  Serial.println("Set Rx duty cycle to 1s/8s");
  wait();
  Serial.println("done");
}

void set_boost_rx(){
  digitalWrite(chipSelectPin_915,LOW);
  uint8_t re1 = SPI.transfer(0x0D);
  uint8_t re2 = SPI.transfer(0x08);
  uint8_t re3 = SPI.transfer(0xAC);
  uint8_t re4 = SPI.transfer(0x96);
  digitalWrite(chipSelectPin_915,HIGH);
//  PrintHex8(&re1,1);Serial.println("");
//  PrintHex8(&re2,1);Serial.println("");
//  PrintHex8(&re3,1);Serial.println("");
//  PrintHex8(&re4,1);Serial.println("");
  Serial.println("Set Rx boost");
  wait();
  Serial.println("done");
}

void PrintHex8(uint8_t *data, uint8_t length) // prints 8-bit data in hex with leading zeroes
{
       Serial.print("0x"); 
       for (int i=0; i<length; i++) { 
         if (data[i]<0x10) {Serial.print("0");} 
         Serial.print(data[i],HEX); 
         Serial.print(" "); 
       }
}
