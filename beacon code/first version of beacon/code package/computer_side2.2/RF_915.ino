//RF 
/*
 * ------------------------interrupt function-------------------
 */

void Receive_Int(){
  detachInterrupt(digitalPinToInterrupt(Rx_int_pin));
  uint8_t * ptr1 = report_status();
  if(*(ptr1+1)==2){
    //Serial.println("incomming packeage...");
    incoming_data_flag = true;
  }else{
    //Serial.println("rx timeout...");
    another_receive_flag = true;
  }
}

void Tx_done_Int(){
  //Serial.println("Tx done...");
  transmit_done_flag = true;
  detachInterrupt(digitalPinToInterrupt(Tx_int_pin));
}

/*-------------------------transport layer-----------------------
 * transport layer function
 * responsible for send and handle message 
 * varries form: computer node, sensor node.
 */
void validate_packet(uint8_t * receive_buffer,uint8_t len){
  //Serial.print("packet coming in:");
//    for(int j = 0; j <len;j++){
//      //Serial.print(*(receive_buffer+j));////Serial.print(" ");
//    }
    ////Serial.println("");
    // ID not assigned then return to rx
    if(node_ID < 0){
      return;
    }
    
    // check destination address
    packet_dest_ID = *(receive_buffer+message_base);
    //Serial.print("packet_dest_ID:");//Serial.println(packet_dest_ID);
    if(packet_dest_ID != node_ID){
      //not correct destination address
      //Serial.println("Unicast not aim at me.");
      return;
    }
    packet_source_ID = *(receive_buffer+message_base+1);
    //Serial.print("packet_source_ID:");//Serial.println(packet_source_ID);
    //check packet type
    char  type_char[3];
     for(int i = 0; i < 3 ; i++){
        type_char[i] = *(receive_buffer + message_base+ 2 +i);
     }
    packet_type = String(type_char);
    int packet_data_len = len-message_base-5;
    uint8_t packet_data[packet_data_len];
    for(int j = 0; j < packet_data_len ; j++){
        packet_data[j] = *(receive_buffer + message_base+ 5 +j);
     }
    packet_type = packet_type.substring(0,3);
    //Serial.print("type:");//Serial.println(packet_type);
    if(packet_type.compareTo("UNI") == 0){
      //it is a unicast message
      // send back ACK packet
      ACK_flag = true;
      parse_data(packet_data);
    }
    
 }

 void parse_data(uint8_t * packet_data){
  uint8_t packet_generator_ID = *packet_data;
  //Serial.print("get packet from:");
  //Serial.println(packet_generator_ID);
  Serial.print(packet_generator_ID);
  Serial.print(",");
  for(int i = 0;i<8;i++){
    latitude_data[packet_generator_ID].bytes[i] = *(packet_data+1+i); 
  }
  //Serial.print("latitude:"); 
  //Serial.println(latitude_data[packet_generator_ID].coordinate,6);
  Serial.print(latitude_data[packet_generator_ID].coordinate,6);
  Serial.print(",");
  for(int j = 0;j<8;j++){
    longtitude_data[packet_generator_ID].bytes[j] = *(packet_data+9+j);
  }
  //Serial.print("longtitude:");
  //Serial.println(longtitude_data[packet_generator_ID].coordinate,6);
  Serial.print(longtitude_data[packet_generator_ID].coordinate,6);
  Serial.print(",");
  robot_passed_flag[packet_generator_ID] = *(packet_data+17);
  //Serial.print("Passed?:");
  //Serial.println(robot_passed_flag[packet_generator_ID]);
  Serial.print(robot_passed_flag[packet_generator_ID]);
  Serial.print(",");
  robot_passed_hour[packet_generator_ID] = *(packet_data+18);
  robot_passed_minute[packet_generator_ID] = *(packet_data+19);
  robot_passed_seconds[packet_generator_ID] = *(packet_data+20);
  //Serial.print("time:");
  //Serial.print(robot_passed_hour[packet_generator_ID]);
  Serial.print(robot_passed_hour[packet_generator_ID]);
  Serial.print(",");
  //Serial.print(":");
  //Serial.print(robot_passed_minute[packet_generator_ID]);
  Serial.print(robot_passed_minute[packet_generator_ID]);
  Serial.print(",");
  //Serial.print(":");
  //Serial.println(robot_passed_seconds[packet_generator_ID]);
  Serial.print(robot_passed_seconds[packet_generator_ID]);
  Serial.println("");
  
 }

 void send_ACK(uint8_t packet_sour_ID){
  uint8_t ACK_packet[5] = {};
  ACK_packet[0] = packet_sour_ID;
  ACK_packet[1] = node_ID;
  ACK_packet[2] = 0x41;
  ACK_packet[3] = 0x43;
  ACK_packet[4] = 0x4B;
  transmit(ACK_packet,5);
 }

 void sendout_broadcast_packet(){
//  uint8_t unicast_packet[255];
//    if(message_base == 4){
//      unicast_packet[0] = 0xFF;
//      unicast_packet[1] = 0xFF;
//      unicast_packet[2] = 0x00;
//      unicast_packet[3] = 0x00;
//    }
//    //destination address
//    unicast_packet[message_base] = node_ID - 1;
//    //source ID
//    unicast_packet[1+message_base] = node_ID;
//    //type UNI
//    unicast_packet[2+message_base] = 0x55; //U
//    unicast_packet[3+message_base] = 0x4E; //N
//    unicast_packet[4+message_base] = 0x49; //I
//    //latitude
//    for(int i = 0; i<8 ; i++){
//      unicast_packet[5+i+message_base] = computer_latitude.bytes[i];
//    }
//    //longtitude
//    for(int i = 0; i<8 ; i++){
//      unicast_packet[13+i+message_base] = computer_latitude.bytes[i];
//    }
//    //robot passed flag
//    unicast_packet[21+message_base] =robot_passed_flag;
//    //time 
//    unicast_packet[22+message_base]  = rbt_pass_hour;
//    unicast_packet[23+message_base]  = rbt_pass_minute;
//    unicast_packet[24+message_base]  = rbt_pass_seconds;
//    //set length
//    unicast_len[unicast_pointer] = 25+message_base;
//    memcpy(unicast_packet_array[unicast_pointer],unicast_packet,unicast_len[unicast_pointer]);
//    unicast_pointer++;     
 }
/*-------------------------physical layer-----------------------
 * physical layer function
 * responsible for initialization, status control. 
 */


void RF_reset(){
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
      ////Serial.println("******915 Mhz Reset done.Standing by.....******");
      return;
    }    
    delay(500);
  }
  //Serial.println("Reset unsuccess.");
  while (1);
}

void init_915M(){
  set_Lora_Packagetype();
  set_dio3_as_tcxo_ctrl();
  set_frequency();
  image_calibration();
  set_pa_config();
  //SetRxTxFallbackMode();
  set_boost_rx();
  read_register();
  set_buff_base_addr(0x64,0x00);
  set_modulation_parameter();
  set_packet_parameter(40);
  set_dio_irq_parameter();
  uint8_t * ptr = report_status();
    if (*ptr == 2 & *(ptr+1) ==1){
      ////Serial.println("******915 Mhz Initialization done.Standing by.....******");
    } else{
      //////Serial.println("******915 Mhz Initialization unsuccess.******");
      while (1);  
    }
}

void receive(uint8_t timeout1,uint8_t timeout2,uint8_t timeout3){
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
    ////Serial.println("done.");
    memcpy(receive_buffer, buff, PayloadLengthRx);
    *len = PayloadLengthRx;
    
    // step 3:print out Rssi
    wait();
    GetPacketStatus();
    
    // step 4:clear Irq
    wait();
    ClearIrqStatus();
    wait();
    uint8_t ClearIrqCount = 0;
    uint8_t * ptr3 = get_irq_status();  
    if (*(ptr3+1) ==0){
        ////Serial.println("Successful reset IRQ.");
      }else{
        ////Serial.println("Unsuccessful reset IRQ.");
        do{
          wait();
          ClearIrqStatus();  
          ClearAllIrqStatus();
          ptr3 = get_irq_status();  
          delay(500);  
          ClearIrqCount++;                    
        }while(*(ptr3+1) != 0&ClearIrqCount>10);
        ClearIrqCount = 0;
        if(*(ptr3+1) != 0){
          reset_flag = true;
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
  //step 2: transmit
  wait();
  set_packet_parameter(len);
  wait();
  set_tx();
  //step 3:wait until transimit done
  while(transmit_done_flag == false){
      delay(10);
    }
  // step 5:clear Irq
    wait();
    ClearIrqStatus();
    wait();
    uint8_t * ptr2 = get_irq_status();
    uint8_t ClearIrqCount = 0;
    if (*(ptr2+1) ==0){
        ////Serial.println("Successful reset IRQ.");
      }else{
        ////Serial.println("Unsuccessful reset IRQ.");
        do{
          wait();
          ClearIrqStatus();  
          ClearAllIrqStatus();
          ptr2 = get_irq_status();  
          delay(500);  
          ClearIrqCount++;                    
        }while(*(ptr2+1) != 0 & ClearIrqCount>10);
        ClearIrqCount = 0;
        if(*(ptr2+1) != 0){
          reset_flag = true;
        }                      
      }
   //step 6: clear flag
    transmit_done_flag = false;
}

uint8_t * report_status(){
  static uint8_t return_val[2];
  uint8_t stat = check_status();
  // read Chip mode 
  uint8_t chip_mode;
  chip_mode = stat << 1;
  chip_mode = chip_mode >>5;
  ////Serial.print("chip mode:");
  switch (chip_mode){
    case 0:
      ////Serial.println("0x0 Unused");
      break;
    case 1:
      ////Serial.println("0x1 RFU");
      break;
    case 2:
      ////Serial.println("0x2 STBY_RC");
      break;
    case 3:
      ////Serial.println("0x3 STBY_XOSC");
      break;
    case 4:
      ////Serial.println("0x4 FS");
      break;
    case 5:
      ////Serial.println("0x5 RX");
      break;
    case 6:
      ////Serial.println("0x6 TX");
      break;
  }
  //read command status
  uint8_t command_status;
  command_status = stat << 4;
  command_status = command_status >>5;
  ////Serial.print("command status:");
  switch (command_status){
    case 0:
      ////Serial.println("0x0 Reserved");
      break;
    case 1:
      ////Serial.println("0x1 RFU");
      break;
    case 2:
      ////Serial.println("0x2 Data available to host");
      break;
    case 3:
      ////Serial.println("0x3 Command timeout");
      break;
    case 4:
      ////Serial.println("0x4 command processing error");
      break;
    case 5:
      ////Serial.println("0x5 Failure to execute command");
      break;
    case 6:
      ////Serial.println("0x6 command TX done");
      break;
      }

  return_val[0] = chip_mode;
  return_val[1] = command_status;
  return return_val;
  }

void report_status_visible(){
  uint8_t stat = check_status();
  // read Chip mode 
  uint8_t chip_mode;
  chip_mode = stat << 1;
  chip_mode = chip_mode >>5;
  Serial.print("chip mode:");
  switch (chip_mode){
    case 0:
      Serial.println("0x0 Unused");
      break;
    case 1:
      Serial.println("0x1 RFU");
      break;
    case 2:
      Serial.println("0x2 STBY_RC");
      break;
    case 3:
      Serial.println("0x3 STBY_XOSC");
      break;
    case 4:
      Serial.println("0x4 FS");
      break;
    case 5:
      Serial.println("0x5 RX");
      break;
    case 6:
      Serial.println("0x6 TX");
      break;
    }
  //read command status
  uint8_t command_status;
  command_status = stat << 4;
  command_status = command_status >>5;
  Serial.print("command status:");
  switch (command_status){
    case 0:
      Serial.println("0x0 Reserved");
      break;
    case 1:
      Serial.println("0x1 RFU");
      break;
    case 2:
      Serial.println("0x2 Data available to host");
      break;
    case 3:
      Serial.println("0x3 Command timeout");
      break;
    case 4:
      Serial.println("0x4 command processing error");
      break;
    case 5:
      Serial.println("0x5 Failure to execute command");
      break;
    case 6:
      Serial.println("0x6 command TX done");
      break;
  }
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
  digitalWrite(chipSelectPin,LOW);
  uint8_t re1 = SPI.transfer(0x8A);
  uint8_t re2 = SPI.transfer(0x01);
  digitalWrite(chipSelectPin,HIGH);
//  PrintHex8(&re1,1);
//  PrintHex8(&re2,1);
  ////Serial.println("Finish set package type to lora");
}

void set_sleep(){
  digitalWrite(chipSelectPin,LOW);
  uint8_t re1 = SPI.transfer(0x84);
  uint8_t re2 = SPI.transfer(0x04);
  digitalWrite(chipSelectPin,HIGH);
//  PrintHex8(&re1,1);////Serial.println("");
//  PrintHex8(&re2,1);////Serial.println("");
  ////Serial.println("sleep....");
}

void set_standby_rc(){
  digitalWrite(chipSelectPin,LOW);
  uint8_t re1 = SPI.transfer(0x80);
  uint8_t re2 = SPI.transfer(0x00);
  digitalWrite(chipSelectPin,HIGH);
//  PrintHex8(&re1,1);////Serial.println("");
//  PrintHex8(&re2,1);////Serial.println("");
  ////Serial.println("standing by(RC)....");
}

void set_standby_xosc(){
  digitalWrite(chipSelectPin,LOW);
  uint8_t re1 = SPI.transfer(0x80);
  uint8_t re2 = SPI.transfer(0x01);
  digitalWrite(chipSelectPin,HIGH);
//  PrintHex8(&re1,1);////Serial.println("");
//  PrintHex8(&re2,1);////Serial.println("");
  ////Serial.println("standing by(XOSC)....");
}

uint8_t check_status(){
  digitalWrite(chipSelectPin,LOW);
  uint8_t stat = SPI.transfer(0xC0);
  digitalWrite(chipSelectPin,HIGH);
  //////Serial.println("curent status:");
  //PrintHex8(&stat,1);////Serial.println("");
  return stat;
}

void image_calibration(){
  digitalWrite(chipSelectPin,LOW);
  uint8_t re1 = SPI.transfer(0x98);
  uint8_t re2 = SPI.transfer(0xE1);
  uint8_t re3 = SPI.transfer(0xE9);
  digitalWrite(chipSelectPin,HIGH);
  ////Serial.println("image calibrating...");
  wait();
  ////Serial.println("Done.");
//  PrintHex8(&re1,1);////Serial.println("");
//  PrintHex8(&re2,1);////Serial.println("");
//  PrintHex8(&re3,1);////Serial.println("");
}

void calibration_function(){
    digitalWrite(chipSelectPin,LOW);
  uint8_t re1 = SPI.transfer(0x89);
  uint8_t re2 = SPI.transfer(0x7F);
  digitalWrite(chipSelectPin,HIGH);
  ////Serial.println("calibrating all...");
  wait();
  ////Serial.println("Done.");
//  PrintHex8(&re1,1);////Serial.println("");
//  PrintHex8(&re2,1);////Serial.println("");
}

void set_frequency(){
  //SPI.transfer(buffer, size)
  digitalWrite(chipSelectPin,LOW);
  //915
  uint8_t re1 = SPI.transfer(0x86);
  uint8_t re2 = SPI.transfer(0x39);
  uint8_t re3 = SPI.transfer(0x30);
  uint8_t re4 = SPI.transfer(0x00);
  uint8_t re5 = SPI.transfer(0x00);
  //433
//  uint8_t re2 = SPI.transfer(0x1B);
//  uint8_t re3 = SPI.transfer(0x20);
//  uint8_t re4 = SPI.transfer(0x00);
//  uint8_t re5 = SPI.transfer(0x00);
  digitalWrite(chipSelectPin,HIGH);
  ////Serial.println("setting RF frequency...");
  wait();
  ////Serial.println("Done.");
//  PrintHex8(&re1,1);////Serial.println("");
//  PrintHex8(&re2,1);////Serial.println("");
//  PrintHex8(&re3,1);////Serial.println("");
//  PrintHex8(&re4,1);////Serial.println("");
//  PrintHex8(&re5,1);////Serial.println("");
}

void set_fs(){
  digitalWrite(chipSelectPin,LOW);
  uint8_t re1 = SPI.transfer(0xC1);
  digitalWrite(chipSelectPin,HIGH);
  ////Serial.println("setting frequency...");
  wait();
  ////Serial.println("Done. Frequency setting mode...");
  //PrintHex8(&re1,1);////Serial.println("");
}

void set_pa_config(){
  digitalWrite(chipSelectPin,LOW);
  uint8_t re1 = SPI.transfer(0x95);
  uint8_t re2 = SPI.transfer(0x04);
  uint8_t re3 = SPI.transfer(0x07);
  uint8_t re4 = SPI.transfer(0x00);
  uint8_t re5 = SPI.transfer(0x01);
  digitalWrite(chipSelectPin,HIGH);
  ////Serial.println("setting pa config...");
  wait();
  ////Serial.println("Done.");
//  PrintHex8(&re1,1);////Serial.println("");
//  PrintHex8(&re2,1);////Serial.println("");
//  PrintHex8(&re3,1);////Serial.println("");
//  PrintHex8(&re4,1);////Serial.println("");
//  PrintHex8(&re5,1);////Serial.println("");
}

void set_tx_params(){
  digitalWrite(chipSelectPin,LOW);
  uint8_t re1 = SPI.transfer(0x8E);
  uint8_t re2 = SPI.transfer(0x16);
  uint8_t re3 = SPI.transfer(0x07);
  digitalWrite(chipSelectPin,HIGH);
  ////Serial.println("setting Tx parameters...");
  wait();
  ////Serial.println("Done.");
  //PrintHex8(&re1,1);////Serial.println("");
  //PrintHex8(&re2,1);////Serial.println("");
  //PrintHex8(&re3,1);////Serial.println("");
}

void set_buff_base_addr(uint8_t tx_addr,uint8_t rx_addr){
  digitalWrite(chipSelectPin,LOW);
  uint8_t re1 = SPI.transfer(0x8F);
  uint8_t re2 = SPI.transfer(tx_addr);
  uint8_t re3 = SPI.transfer(rx_addr);
  digitalWrite(chipSelectPin,HIGH);
  ////Serial.println("setting base address...");
  wait();
  ////Serial.println("Done.");
//  PrintHex8(&re1,1);////Serial.println("");
//  PrintHex8(&re2,1);////Serial.println("");
//  PrintHex8(&re3,1);////Serial.println("");
}

void write_buff(uint8_t * transmit_buffer,uint8_t len){
  digitalWrite(chipSelectPin,LOW);
  ////Serial.println("writing buffer...");
  wait();
  uint8_t re1 = SPI.transfer(0x0E);
  uint8_t re2 = SPI.transfer(0x64);
//  PrintHex8(&re1,1);////Serial.println("");
//  PrintHex8(&re2,1);////Serial.println("");
  for (int i=0; i<len;i++){
    wait();
    uint8_t re3 = SPI.transfer(transmit_buffer[i]);
    //PrintHex8(&re3,1);////Serial.println("");
  }
  digitalWrite(chipSelectPin,HIGH);
  wait();
  ////Serial.println("Done.");
}


void set_modulation_parameter(){
  digitalWrite(chipSelectPin,LOW);
  uint8_t re1 = SPI.transfer(0x8B);
  uint8_t re2 = SPI.transfer(0x0A);
  uint8_t re3 = SPI.transfer(0x05);
  uint8_t re4 = SPI.transfer(0x04);
  uint8_t re5 = SPI.transfer(0x00);
  digitalWrite(chipSelectPin,HIGH);
  ////Serial.println("setting modulation parameter...");
  wait();
  ////Serial.println("Done.");
//  PrintHex8(&re1,1);////Serial.println("");
//  PrintHex8(&re2,1);////Serial.println("");
//  PrintHex8(&re3,1);////Serial.println("");
//  PrintHex8(&re4,1);////Serial.println("");
//  PrintHex8(&re5,1);////Serial.println("");
}

void set_packet_parameter(int len){
  digitalWrite(chipSelectPin,LOW);
  uint8_t re1 = SPI.transfer(0x8C);
  uint8_t re2 = SPI.transfer(0x00);
  uint8_t re3 = SPI.transfer(0x08);
  uint8_t re4 = SPI.transfer(0x00);
  uint8_t re5 = SPI.transfer(len);
  uint8_t re6 = SPI.transfer(0x01);
  uint8_t re7 = SPI.transfer(0x00);
  digitalWrite(chipSelectPin,HIGH);
  ////Serial.println("setting modulation parameter...");
  wait();
  ////Serial.println("Done.");
//  PrintHex8(&re1,1);////Serial.println("");
//  PrintHex8(&re2,1);////Serial.println("");
//  PrintHex8(&re3,1);////Serial.println("");
//  PrintHex8(&re4,1);////Serial.println("");
//  PrintHex8(&re5,1);////Serial.println("");
//  PrintHex8(&re6,1);////Serial.println("");
//  PrintHex8(&re7,1);////Serial.println("");
}

void set_dio_irq_parameter(){
  digitalWrite(chipSelectPin,LOW);
  uint8_t re1 = SPI.transfer(0x08);
  uint8_t re2 = SPI.transfer(0x00);
  uint8_t re3 = SPI.transfer(0x03);
  uint8_t re4 = SPI.transfer(0x00);
  uint8_t re5 = SPI.transfer(0x02);
  uint8_t re6 = SPI.transfer(0x00);
  uint8_t re7 = SPI.transfer(0x01);
  uint8_t re8 = SPI.transfer(0x00);
  uint8_t re9 = SPI.transfer(0x00);
  digitalWrite(chipSelectPin,HIGH);
  ////Serial.println("setting IRQ...");
  wait();
  ////Serial.println("Done.");
//  PrintHex8(&re1,1);////Serial.println("");
//  PrintHex8(&re2,1);////Serial.println("");
//  PrintHex8(&re3,1);////Serial.println("");
//  PrintHex8(&re4,1);////Serial.println("");
//  PrintHex8(&re5,1);////Serial.println("");
//  PrintHex8(&re6,1);////Serial.println("");
//  PrintHex8(&re7,1);////Serial.println("");
//  PrintHex8(&re8,1);////Serial.println("");
//  PrintHex8(&re9,1);////Serial.println("");
}
void ClearIrqStatus(){
  digitalWrite(chipSelectPin,LOW);
  uint8_t re1 = SPI.transfer(0x02);
  uint8_t re2 = SPI.transfer(0x00);
  uint8_t re3 = SPI.transfer(0x03);
  digitalWrite(chipSelectPin,HIGH);
  ////Serial.println("clearing IRQ flags...");
  wait();
  ////Serial.println("done");
}

void ClearAllIrqStatus(){
  digitalWrite(chipSelectPin,LOW);
  uint8_t re1 = SPI.transfer(0x02);
  uint8_t re2 = SPI.transfer(0x7F);
  uint8_t re3 = SPI.transfer(0xFF);
  digitalWrite(chipSelectPin,HIGH);
  ////Serial.println("clearing IRQ flags...");
  wait();
  ////Serial.println("done");
}

uint8_t * get_irq_status(){
  static uint8_t irq_status[2];
  digitalWrite(chipSelectPin,LOW);
  uint8_t re1 = SPI.transfer(0x12);
  uint8_t re2 = SPI.transfer(0x00);
  uint8_t re3 = SPI.transfer(0x00);
  uint8_t re4 = SPI.transfer(0x00);
  digitalWrite(chipSelectPin,HIGH);
  ////Serial.println("getting IRQ status...");
  wait();
  ////Serial.println("Done.");
  irq_status[0] = re3;
  irq_status[1] = re4;
//  PrintHex8(&re1,1);////Serial.println("");
//  PrintHex8(&re2,1);////Serial.println("");
//  PrintHex8(&re3,1);////Serial.println("");
//  PrintHex8(&re4,1);////Serial.println("");
  return irq_status;
}

void set_tx(){
  digitalWrite(chipSelectPin,LOW);
  uint8_t re1 = SPI.transfer(0x83);
  uint8_t re2 = SPI.transfer(0x00);
  uint8_t re3 = SPI.transfer(0x00);
  uint8_t re4 = SPI.transfer(0x00);
  digitalWrite(chipSelectPin,HIGH);
  ////Serial.println("setting TX...");
  wait();
  ////Serial.println("Done.");
//  PrintHex8(&re1,1);////Serial.println("");
//  PrintHex8(&re2,1);////Serial.println("");
//  PrintHex8(&re3,1);////Serial.println("");
//  PrintHex8(&re4,1);////Serial.println("");
}

void set_rx(uint8_t timeout1,uint8_t timeout2,uint8_t timeout3){
  digitalWrite(chipSelectPin,LOW);
  uint8_t re1 = SPI.transfer(0x82);
  uint8_t re2 = SPI.transfer(timeout1);
  uint8_t re3 = SPI.transfer(timeout2);
  uint8_t re4 = SPI.transfer(timeout3);
  digitalWrite(chipSelectPin,HIGH);
  ////Serial.println("setting RX...");
  wait();
  ////Serial.println("Done.");
//  PrintHex8(&re1,1);////Serial.println("");
//  PrintHex8(&re2,1);////Serial.println("");
//  PrintHex8(&re3,1);////Serial.println("");
//  PrintHex8(&re4,1);////Serial.println("");
}

void set_dio3_as_tcxo_ctrl(){
  digitalWrite(chipSelectPin,LOW);
  uint8_t re1 = SPI.transfer(0x97);
  uint8_t re2 = SPI.transfer(0x06);
  uint8_t re3 = SPI.transfer(0x01);
  uint8_t re4 = SPI.transfer(0x01);
  uint8_t re5 = SPI.transfer(0x01);
  digitalWrite(chipSelectPin,HIGH);
  ////Serial.println("set_dio3_as_tcxo_ctrl...");
  wait();
  ////Serial.println("Done.");
//  PrintHex8(&re1,1);////Serial.println("");
//  PrintHex8(&re2,1);////Serial.println("");
//  PrintHex8(&re3,1);////Serial.println("");
//  PrintHex8(&re4,1);////Serial.println("");
//  PrintHex8(&re5,1);////Serial.println("");
}

void get_device_errors(){
  digitalWrite(chipSelectPin,LOW);
  uint8_t re1 = SPI.transfer(0x17);
  uint8_t re2 = SPI.transfer(0x00);
  uint8_t re3 = SPI.transfer(0x00);
  uint8_t re4 = SPI.transfer(0x00);
  digitalWrite(chipSelectPin,HIGH);
  ////Serial.println("getting device error...");
  wait();
  ////Serial.println("Done.");
  PrintHex8(&re1,1);////Serial.println("");
  PrintHex8(&re2,1);////Serial.println("");
  PrintHex8(&re3,1);////Serial.println("");
  PrintHex8(&re4,1);////Serial.println("");
}

void clear_device_errors(){
  digitalWrite(chipSelectPin,LOW);
  uint8_t re1 = SPI.transfer(0x07);
  uint8_t re2 = SPI.transfer(0x00);
  digitalWrite(chipSelectPin,HIGH);
  ////Serial.println("clearing device error...");
  wait();
  ////Serial.println("Done.");
  //PrintHex8(&re1,1);////Serial.println("");
  //PrintHex8(&re2,1);////Serial.println("");
}

uint8_t * read_buffer(uint8_t offset,int count){
  ////Serial.println("Reading buffer:");
  static uint8_t receive_buffer[255];
  digitalWrite(chipSelectPin,LOW);
  uint8_t re1 = SPI.transfer(0x1E);
  uint8_t re2 = SPI.transfer(offset);
  uint8_t re3 = SPI.transfer(0x00);
  //PrintHex8(&re1,1);////Serial.println("");
  //PrintHex8(&re2,1);////Serial.println("");
  for (int i=0; i<count;i++){
    wait();
    uint8_t re4 = SPI.transfer(0x00);
    //PrintHex8(&re3,1);////Serial.println("");
    receive_buffer[i] = re4;
  }
  digitalWrite(chipSelectPin,HIGH);
  return receive_buffer;
}

/*
 * return a pointer 
 * *ptr = PayloadLengthRx
 * *(ptr+1) = RxStartBufferPointer
 */
uint8_t * GetRxBufferStatus(){
  static uint8_t rx_buffer_status[2];
  digitalWrite(chipSelectPin,LOW);
  uint8_t re1 = SPI.transfer(0x13);
  uint8_t re2 = SPI.transfer(0x00);
  uint8_t re3 = SPI.transfer(0x00);
  uint8_t re4 = SPI.transfer(0x00);
  digitalWrite(chipSelectPin,HIGH);
//  PrintHex8(&re1,1);////Serial.println("");
//  PrintHex8(&re2,1);////Serial.println("");
//  PrintHex8(&re3,1);////Serial.println("");
//  PrintHex8(&re4,1);////Serial.println("");
  rx_buffer_status[0] = re3;
  rx_buffer_status[1] = re4;
  return rx_buffer_status;
}

void GetPacketStatus(){
  digitalWrite(chipSelectPin,LOW);
  uint8_t re1 = SPI.transfer(0x14);
  uint8_t re2 = SPI.transfer(0x00);
  uint8_t re3 = SPI.transfer(0x00);
  uint8_t re4 = SPI.transfer(0x00);
  uint8_t re5 = SPI.transfer(0x00);
  digitalWrite(chipSelectPin,HIGH);
//  PrintHex8(&re1,1);////Serial.println("");
//  PrintHex8(&re2,1);////Serial.println("");
//  PrintHex8(&re3,1);////Serial.println("");
//  PrintHex8(&re4,1);////Serial.println("");
////Serial.print("packet Rssi:");
  ////Serial.print(-re3 / 2);
  ////Serial.println(" dBm");
  ////Serial.print("SNR:");
  ////Serial.print(re4 / 4);
  ////Serial.println(" dB");
  ////Serial.print("signal Rssi:");
  ////Serial.print(-re5 / 2);
  ////Serial.println(" dBm");
}

void GetRssilnst(){
  digitalWrite(chipSelectPin,LOW);
  uint8_t re1 = SPI.transfer(0x15);
  uint8_t re2 = SPI.transfer(0x00);
  uint8_t re3 = SPI.transfer(0x00);
  digitalWrite(chipSelectPin,HIGH);
  //PrintHex8(&re1,1);////Serial.println("");
  //PrintHex8(&re2,1);////Serial.println("");
  //PrintHex8(&re3,1);////Serial.println("");
  ////Serial.print("Rssi:");
  ////Serial.println(-re3 / 2);
}

/*
 * FS         0x40 The radio goes into FS mode after Tx or Rx
 * STDBY_XOSC 0x30 The radio goes into STDBY_XOSC mode after Tx or Rx
 * STDBY_RC   0x20 The radio goes into STDBY_RC mode after Tx or Rx
 */
void SetRxTxFallbackMode(){
  digitalWrite(chipSelectPin,LOW);
  uint8_t re1 = SPI.transfer(0x93);
  uint8_t re2 = SPI.transfer(0x40);
  digitalWrite(chipSelectPin,HIGH);
  ////Serial.println("SetRxTxFallbackMode to FS");
  wait();
  ////Serial.println("done");
}

void ResetStats(){
  digitalWrite(chipSelectPin,LOW);
  for (int i=0;i<7;i++){
    uint8_t re = SPI.transfer(0x00);
  }
  digitalWrite(chipSelectPin,HIGH);
}

void SetRxDutyCycle(){
  digitalWrite(chipSelectPin,LOW);
  uint8_t re1 = SPI.transfer(0x94);
  uint8_t re2 = SPI.transfer(0x00);
  uint8_t re3 = SPI.transfer(0xFA);
  uint8_t re4 = SPI.transfer(0x00);
  uint8_t re5 = SPI.transfer(0x07);
  uint8_t re6 = SPI.transfer(0xD0);
  uint8_t re7 = SPI.transfer(0x00);
  digitalWrite(chipSelectPin,HIGH);
  ////Serial.println("Set Rx duty cycle to 1s/8s");
  wait();
  ////Serial.println("done");
}

void set_boost_rx(){
  digitalWrite(chipSelectPin,LOW);
  uint8_t re1 = SPI.transfer(0x0D);
  uint8_t re2 = SPI.transfer(0x08);
  uint8_t re3 = SPI.transfer(0xAC);
  uint8_t re4 = SPI.transfer(0x96);
  digitalWrite(chipSelectPin,HIGH);
//  PrintHex8(&re1,1);////Serial.println("");
//  PrintHex8(&re2,1);////Serial.println("");
//  PrintHex8(&re3,1);////Serial.println("");
//  PrintHex8(&re4,1);////Serial.println("");
  ////Serial.println("Set Rx boost");
  wait();
  ////Serial.println("done");
}

void read_register(){
  digitalWrite(chipSelectPin,LOW);
  uint8_t re1 = SPI.transfer(0x1D);
  uint8_t re2 = SPI.transfer(0x08);
  uint8_t re3 = SPI.transfer(0xAC);
  uint8_t re4 = SPI.transfer(0x00);
  uint8_t re5 = SPI.transfer(0x00);
  digitalWrite(chipSelectPin,HIGH);
  //PrintHex8(&re5,1);////Serial.println("");
  wait();
  ////Serial.println("done");
}
void PrintHex8(uint8_t *data, uint8_t length) // prints 8-bit data in hex with leading zeroes
{
       ////Serial.print("0x"); 
       for (int i=0; i<length; i++) { 
         if (data[i]<0x10) {Serial.print("0");} 
         Serial.print(data[i],HEX); 
         Serial.print(" "); 
       }
}
