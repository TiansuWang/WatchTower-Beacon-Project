//RF 
/*
 * ------------------------interrupt function-------------------
 */

void Receive_Int_434(){
    detachInterrupt(digitalPinToInterrupt(Rx_int_pin_434));
    incoming_data_flag_434 = true;
}

void Tx_done_Int_434(){
  //Serial.println("Tx done...");
  transmit_done_flag_434 = true;
  detachInterrupt(digitalPinToInterrupt(Tx_int_pin_434));
}

/*-------------------------transport layer-----------------------
 * transport layer function
 * responsible for send and handle message 
 * varries form: computer node, sensor node.
 */
void validate_packet_434(uint8_t * receive_buffer,uint8_t len){
//    for(int j = 0; j <len;j++){
//      Serial.println(*(receive_buffer+j));
//    }
    char  receive_char[len];
     for(int i = 0; i < len ; i++){
        receive_char[i] = *(receive_buffer+i);
     }
    String receive_string = String(receive_char);
    
    //String packet_header = receive_string.substring(0,5);
    String config_reply = receive_string.substring(0,3);
//    if(packet_header.compareTo("WTRCO") == 0){
//      Serial.println("robot passed!!!!!");
//      robot_passed_flag = true;
//      rbt_pass_hour = greenwich_hour;
//      rbt_pass_minute = greenwich_minute;
//      rbt_pass_seconds = greenwich_seconds;
//    }else 
    if(config_reply.compareTo("RPL") == 0){
      Serial.println("get configure reply:");
      Serial.print("ID:");Serial.println(*(receive_buffer+3));
      Serial.print("DEST:");Serial.println(*(receive_buffer+4));
     
    }else if(config_reply.compareTo("ECB") == 0){
      if(*(receive_buffer+3) == 1){
        Serial.println("echo success.");
        Serial.print("GPS fix:");Serial.println(*(receive_buffer+4));
      }else{
        Serial.println("echo failed.");
      }
    }else if(config_reply.compareTo("LEB") == 0){
        int packet_data_len = len-3;
        uint8_t packet_data[packet_data_len];
        for(int j = 0; j < packet_data_len ; j++){
            packet_data[j] = *(receive_buffer + 3 +j);
         }
        parse_data(packet_data);
    }else if(config_reply.compareTo("RCB") == 0){
      Serial.print("Unlock node ");
      Serial.print(*(receive_buffer+3));
      Serial.println(" to reconfigure");
    }else{
      //Serial.println("false alarm.");
      receive_434(0xFF,0x00,0x00);
    } 
 }
void parse_data(uint8_t * packet_data){
    uint8_t report_from = *(packet_data+1);
    Serial.println("************************************");
    Serial.print("get report from:");Serial.println(report_from);
    uint8_t reach_to = *packet_data;
    uint8_t hop = *(packet_data+2);
    if(reach_to != try_to_find){
      Serial.print("only made to:");Serial.println(reach_to);
      Serial.print("hop:");Serial.println(hop);
    }else{
      Serial.print("find node ");Serial.println(reach_to);
      Serial.print("hop:");Serial.println(hop);
    }
    DOUBLEUNION_t latitude_data;
    DOUBLEUNION_t longtitude_data;
    for(int i = 0;i<8;i++){
      latitude_data.bytes[i] = *(packet_data+3+i); 
      longtitude_data.bytes[i] = *(packet_data+11+i);
    }
  Serial.print("latitude:"); 
  Serial.println(latitude_data.coordinate,6);
  
  Serial.print("longtitude:");
  Serial.println(longtitude_data.coordinate,6);

  uint8_t robot_passed_flag= *(packet_data+19);
  Serial.print("Passed?:");
  Serial.println(robot_passed_flag);

  uint8_t robot_passed_hour_max = *(packet_data+20);
  uint8_t robot_passed_minute_max = *(packet_data+21);
  uint8_t robot_passed_seconds_max = *(packet_data+22);
  
  uint8_t robot_passed_hour_mid = *(packet_data+23);
  uint8_t robot_passed_minute_mid = *(packet_data+24);
  uint8_t robot_passed_seconds_mid = *(packet_data+25);
  
  Serial.print("max algorithm time:");
  Serial.print(robot_passed_hour_max);
  Serial.print(":");
  Serial.print(robot_passed_minute_max);
  Serial.print(":");
  Serial.println(robot_passed_seconds_max);
  
  Serial.print("mid algorithm time:");
  Serial.print(robot_passed_hour_mid);
  Serial.print(":");
  Serial.print(robot_passed_minute_mid);
  Serial.print(":");
  Serial.println(robot_passed_seconds_mid);

}
/*-------------------------physical layer-----------------------
 * physical layer function
 * responsible for initialization, status control. 
 */


void RF_reset_434(){
  digitalWrite(nreset_pin_434,LOW);
  delay(100);
  digitalWrite(nreset_pin_434,HIGH);
  wait_434();
  for (int i=0;i<3;i++){
    set_standby_rc_434();
    wait_434();
    uint8_t * ptr = report_status_434();
    if (*ptr == 2 & *(ptr+1) ==1){
      Serial.println("******434 Mhz Reset done.Standing by.....******");
      return;
    }    
    delay(500);
  }
  Serial.println("Reset unsuccess.");
  while (1);
}

void init_434M(){
  set_Lora_Packagetype_434();
  set_dio3_as_tcxo_ctrl_434();
  set_frequency_434();
  image_calibration_434();
  set_pa_config_434();
  SetRxTxFallbackMode_434();
  set_boost_rx_434();
  set_buff_base_addr_434(0x64,0x00);
  set_modulation_parameter_434();
  set_packet_parameter_434(10);
  set_dio_irq_parameter_434();
  uint8_t * ptr = report_status_434();
    if (*ptr == 2 & *(ptr+1) ==1){
      Serial.println("******434 Mhz Initialization done.Standing by.....******");
    } else{
      Serial.println("******434 Mhz Initialization unsuccess.******");
      while (1);  
    }
}

void receive_434(uint8_t timeout1,uint8_t timeout2,uint8_t timeout3){
    //step 0: attach interupt
    attachInterrupt(digitalPinToInterrupt(Rx_int_pin_434), Receive_Int_434, CHANGE);
    wait_434();
    set_rx_434(timeout1,timeout2,timeout3);  
}

void extract_rx_data_434(uint8_t * receive_buffer,uint8_t * len){
  
  // step 1:get the buffer information
    wait_434();
    uint8_t * ptr2 = GetRxBufferStatus_434();
    uint8_t PayloadLengthRx = *ptr2;
    uint8_t RxStartBufferPointer = *(ptr2+1);
    
    // step 2:read data from buffer and print it out
    wait_434();
    uint8_t * buff = read_buffer_434(RxStartBufferPointer,PayloadLengthRx);
    //Serial.println("done.");
    memcpy(receive_buffer, buff, PayloadLengthRx);
    *len = PayloadLengthRx;
    
    // step 3:print out Rssi
    wait_434();
    GetPacketStatus_434();
    
    // step 4:clear Irq
    wait_434();
    ClearIrqStatus_434();
    wait_434();
    uint8_t * ptr3 = get_irq_status_434();
    uint8_t ClearIrqCount = 0;
      if (*(ptr3+1) ==0){
        //Serial.println("Successful reset IRQ.");
      }else{
        Serial.println("Unsuccessful reset IRQ.");
        do{
          wait();
          ClearIrqStatus_434();  
          ClearAllIrqStatus_434();
          ptr3 = get_irq_status_434();  
          delay(500);  
          ClearIrqCount++;                    
        }while(*(ptr3+1) != 0|ClearIrqCount>10);
        ClearIrqCount = 0;
        if(*(ptr3+1) != 0){
          reset_flag_434 = true;
        }
      }
}


void transmit_434(uint8_t * transmit_buffer,uint8_t len){

  //wait packet sent
  uint8_t * ptr1 = report_status_434();
  while (*ptr1 == 6){
    delay(200);
    ptr1 = report_status_434();
  }
  
  //step 0:attach interrupt
  attachInterrupt(digitalPinToInterrupt(Tx_int_pin_434), Tx_done_Int_434, CHANGE);

  //step 1: write transmit buffer
  write_buff_434(transmit_buffer,len);
  wait_434();
  set_packet_parameter_434(len);
  //step 2: transmit
  wait_434();
  set_tx_434();
  //step 3:wait until transimit done
  //delay(5000);
  while(transmit_done_flag_434 == false){
      delay(10);
    }
  // step 5:clear Irq
    wait_434();
    ClearIrqStatus_434();
    wait_434();
    uint8_t * ptr3 = get_irq_status_434();
    uint8_t ClearIrqCount = 0;
      if (*(ptr3+1) ==0){
        //Serial.println("Successful reset IRQ.");
      }else{
        Serial.println("Unsuccessful reset IRQ.");
        do{
          wait_434();
          ClearIrqStatus_434();  
          ClearAllIrqStatus_434();
          ptr3 = get_irq_status_434();  
          delay(500);  
          ClearIrqCount++;                    
        }while(*(ptr3+1) != 0|ClearIrqCount>10);
        ClearIrqCount = 0;
        if(*(ptr3+1) != 0){
          reset_flag_434 = true;
        }
      }
   //step 6: clear flag
    transmit_done_flag_434 = false;
}

uint8_t * report_status_434(){
  static uint8_t return_val[2];
  uint8_t stat = check_status_434();
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

void report_status_434_visible(){
  static uint8_t return_val[2];
  uint8_t stat = check_status_434();
  // read Chip mode 
  uint8_t chip_mode;
  chip_mode = stat << 1;
  chip_mode = chip_mode >>5;
  Serial.print("434 chip mode:");
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
  }
  //read command status
  uint8_t command_status;
  command_status = stat << 4;
  command_status = command_status >>5;
  Serial.print("434 command status:");
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
  }
}
/*---------------------------operation function-----------------------
 * following is basic operation function
 * they put the spi command into the radio chip
 */

void wait_434(){
  while(digitalRead(busy_pin_434)){
  delay(100);
  }
}

void set_Lora_Packagetype_434(){
  digitalWrite(chipSelectPin_434,LOW);
  uint8_t re1 = SPI.transfer(0x8A);
  uint8_t re2 = SPI.transfer(0x01);
  digitalWrite(chipSelectPin_434,HIGH);
//  PrintHex8(&re1,1);
//  PrintHex8(&re2,1);
  Serial.println("Finish set package type to lora");
}

void set_sleep_434(){
  digitalWrite(chipSelectPin_434,LOW);
  uint8_t re1 = SPI.transfer(0x84);
  uint8_t re2 = SPI.transfer(0x04);
  digitalWrite(chipSelectPin_434,HIGH);
//  PrintHex8(&re1,1);Serial.println("");
//  PrintHex8(&re2,1);Serial.println("");
  Serial.println("sleep....");
}

void set_standby_rc_434(){
  digitalWrite(chipSelectPin_434,LOW);
  uint8_t re1 = SPI.transfer(0x80);
  uint8_t re2 = SPI.transfer(0x00);
  digitalWrite(chipSelectPin_434,HIGH);
//  PrintHex8(&re1,1);Serial.println("");
//  PrintHex8(&re2,1);Serial.println("");
  Serial.println("standing by(RC)....");
}

void set_standby_xosc_434(){
  digitalWrite(chipSelectPin_434,LOW);
  uint8_t re1 = SPI.transfer(0x80);
  uint8_t re2 = SPI.transfer(0x01);
  digitalWrite(chipSelectPin_434,HIGH);
//  PrintHex8(&re1,1);Serial.println("");
//  PrintHex8(&re2,1);Serial.println("");
  Serial.println("standing by(XOSC)....");
}

uint8_t check_status_434(){
  digitalWrite(chipSelectPin_434,LOW);
  uint8_t stat = SPI.transfer(0xC0);
  digitalWrite(chipSelectPin_434,HIGH);
  //Serial.println("curent status:");
  //PrintHex8(&stat,1);Serial.println("");
  return stat;
}

void image_calibration_434(){
  digitalWrite(chipSelectPin_434,LOW);
  uint8_t re1 = SPI.transfer(0x98);
  uint8_t re2 = SPI.transfer(0x6B);
  uint8_t re3 = SPI.transfer(0x6F);
  digitalWrite(chipSelectPin_434,HIGH);
  Serial.println("image calibrating...");
  wait_434();
  Serial.println("Done.");
//  PrintHex8(&re1,1);Serial.println("");
//  PrintHex8(&re2,1);Serial.println("");
//  PrintHex8(&re3,1);Serial.println("");
}

void calibration_function_434(){
    digitalWrite(chipSelectPin_434,LOW);
  uint8_t re1 = SPI.transfer(0x89);
  uint8_t re2 = SPI.transfer(0x7F);
  digitalWrite(chipSelectPin_434,HIGH);
  Serial.println("calibrating all...");
  wait_434();
  Serial.println("Done.");
//  PrintHex8(&re1,1);Serial.println("");
//  PrintHex8(&re2,1);Serial.println("");
}

void set_frequency_434(){
  //SPI.transfer(buffer, size)
  digitalWrite(chipSelectPin_434,LOW);
  uint8_t re1 = SPI.transfer(0x86);
  uint8_t re2 = SPI.transfer(0x1B);
  uint8_t re3 = SPI.transfer(0x20);
  uint8_t re4 = SPI.transfer(0x00);
  uint8_t re5 = SPI.transfer(0x00);
  digitalWrite(chipSelectPin_434,HIGH);
  Serial.println("setting RF frequency...");
  wait_434();
  Serial.println("Done.");
//  PrintHex8(&re1,1);Serial.println("");
//  PrintHex8(&re2,1);Serial.println("");
//  PrintHex8(&re3,1);Serial.println("");
//  PrintHex8(&re4,1);Serial.println("");
//  PrintHex8(&re5,1);Serial.println("");
}

void set_fs_434(){
  digitalWrite(chipSelectPin_434,LOW);
  uint8_t re1 = SPI.transfer(0xC1);
  digitalWrite(chipSelectPin_434,HIGH);
  Serial.println("setting frequency...");
  wait_434();
  Serial.println("Done. Frequency setting mode...");
  //PrintHex8(&re1,1);Serial.println("");
}

void set_pa_config_434(){
  digitalWrite(chipSelectPin_434,LOW);
  uint8_t re1 = SPI.transfer(0x95);
  uint8_t re2 = SPI.transfer(0x04);
  uint8_t re3 = SPI.transfer(0x07);
  uint8_t re4 = SPI.transfer(0x00);
  uint8_t re5 = SPI.transfer(0x01);
  digitalWrite(chipSelectPin_434,HIGH);
  Serial.println("setting pa config...");
  wait_434();
  Serial.println("Done.");
//  PrintHex8(&re1,1);Serial.println("");
//  PrintHex8(&re2,1);Serial.println("");
//  PrintHex8(&re3,1);Serial.println("");
//  PrintHex8(&re4,1);Serial.println("");
//  PrintHex8(&re5,1);Serial.println("");
}

void set_tx_params_434(){
  digitalWrite(chipSelectPin_434,LOW);
  uint8_t re1 = SPI.transfer(0x8E);
  uint8_t re2 = SPI.transfer(0x16);
  uint8_t re3 = SPI.transfer(0x07);
  digitalWrite(chipSelectPin_434,HIGH);
  Serial.println("setting Tx parameters...");
  wait_434();
  Serial.println("Done.");
  //PrintHex8(&re1,1);Serial.println("");
  //PrintHex8(&re2,1);Serial.println("");
  //PrintHex8(&re3,1);Serial.println("");
}

void set_buff_base_addr_434(uint8_t tx_addr,uint8_t rx_addr){
  digitalWrite(chipSelectPin_434,LOW);
  uint8_t re1 = SPI.transfer(0x8F);
  uint8_t re2 = SPI.transfer(tx_addr);
  uint8_t re3 = SPI.transfer(rx_addr);
  digitalWrite(chipSelectPin_434,HIGH);
  Serial.println("setting base address...");
  wait_434();
  Serial.println("Done.");
//  PrintHex8(&re1,1);Serial.println("");
//  PrintHex8(&re2,1);Serial.println("");
//  PrintHex8(&re3,1);Serial.println("");
}

void write_buff_434(uint8_t * transmit_buffer,uint8_t len){
  digitalWrite(chipSelectPin_434,LOW);
  //Serial.println("writing buffer...");
  wait_434();
  uint8_t re1 = SPI.transfer(0x0E);
  uint8_t re2 = SPI.transfer(0x64);
//  PrintHex8(&re1,1);Serial.println("");
//  PrintHex8(&re2,1);Serial.println("");
  for (int i=0; i<len;i++){
    wait_434();
    uint8_t re3 = SPI.transfer(transmit_buffer[i]);
    //PrintHex8(&re3,1);Serial.println("");
  }
  digitalWrite(chipSelectPin_434,HIGH);
  wait_434();
  //Serial.println("Done.");
}


void set_modulation_parameter_434(){
  digitalWrite(chipSelectPin_434,LOW);
  uint8_t re1 = SPI.transfer(0x8B);
  uint8_t re2 = SPI.transfer(0x0A);
  uint8_t re3 = SPI.transfer(0x05);
  uint8_t re4 = SPI.transfer(0x04);
  uint8_t re5 = SPI.transfer(0x00);
  digitalWrite(chipSelectPin_434,HIGH);
  Serial.println("setting modulation parameter...");
  wait_434();
  Serial.println("Done.");
//  PrintHex8(&re1,1);Serial.println("");
//  PrintHex8(&re2,1);Serial.println("");
//  PrintHex8(&re3,1);Serial.println("");
//  PrintHex8(&re4,1);Serial.println("");
//  PrintHex8(&re5,1);Serial.println("");
}

void set_packet_parameter_434(int len){
  digitalWrite(chipSelectPin_434,LOW);
  uint8_t re1 = SPI.transfer(0x8C);
  uint8_t re2 = SPI.transfer(0x00);
  uint8_t re3 = SPI.transfer(0x08);
  uint8_t re4 = SPI.transfer(0x00);
  uint8_t re5 = SPI.transfer(len);
  uint8_t re6 = SPI.transfer(0x01);
  uint8_t re7 = SPI.transfer(0x00);
  digitalWrite(chipSelectPin_434,HIGH);
  //Serial.println("setting packet parameter...");
  wait_434();
  //Serial.println("Done.");
//  PrintHex8(&re1,1);Serial.println("");
//  PrintHex8(&re2,1);Serial.println("");
//  PrintHex8(&re3,1);Serial.println("");
//  PrintHex8(&re4,1);Serial.println("");
//  PrintHex8(&re5,1);Serial.println("");
//  PrintHex8(&re6,1);Serial.println("");
//  PrintHex8(&re7,1);Serial.println("");
}

void set_dio_irq_parameter_434(){
  digitalWrite(chipSelectPin_434,LOW);
  uint8_t re1 = SPI.transfer(0x08);
  uint8_t re2 = SPI.transfer(0x00);
  uint8_t re3 = SPI.transfer(0x03);
  uint8_t re4 = SPI.transfer(0x00);
  uint8_t re5 = SPI.transfer(0x02);
  uint8_t re6 = SPI.transfer(0x00);
  uint8_t re7 = SPI.transfer(0x01);
  uint8_t re8 = SPI.transfer(0x00);
  uint8_t re9 = SPI.transfer(0x00);
  digitalWrite(chipSelectPin_434,HIGH);
  Serial.println("setting IRQ...");
  wait_434();
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
void ClearIrqStatus_434(){
  digitalWrite(chipSelectPin_434,LOW);
  uint8_t re1 = SPI.transfer(0x02);
  uint8_t re2 = SPI.transfer(0x00);
  uint8_t re3 = SPI.transfer(0x03);
  digitalWrite(chipSelectPin_434,HIGH);
  //Serial.println("clearing IRQ flags...");
  wait_434();
  //Serial.println("done");
}

void ClearAllIrqStatus_434(){
  digitalWrite(chipSelectPin_434,LOW);
  uint8_t re1 = SPI.transfer(0x02);
  uint8_t re2 = SPI.transfer(0x7F);
  uint8_t re3 = SPI.transfer(0xFF);
  digitalWrite(chipSelectPin_434,HIGH);
  //Serial.println("clearing IRQ flags...");
  wait_434();
  //Serial.println("done");
}

uint8_t * get_irq_status_434(){
  static uint8_t irq_status[2];
  digitalWrite(chipSelectPin_434,LOW);
  uint8_t re1 = SPI.transfer(0x12);
  uint8_t re2 = SPI.transfer(0x00);
  uint8_t re3 = SPI.transfer(0x00);
  uint8_t re4 = SPI.transfer(0x00);
  digitalWrite(chipSelectPin_434,HIGH);
  //Serial.println("getting IRQ status...");
  wait_434();
  //Serial.println("Done.");
  irq_status[0] = re3;
  irq_status[1] = re4;
//  PrintHex8(&re1,1);Serial.println("");
//  PrintHex8(&re2,1);Serial.println("");
//  PrintHex8(&re3,1);Serial.println("");
//  PrintHex8(&re4,1);Serial.println("");
  return irq_status;
}

void set_tx_434(){
  digitalWrite(chipSelectPin_434,LOW);
  uint8_t re1 = SPI.transfer(0x83);
  uint8_t re2 = SPI.transfer(0x00);
  uint8_t re3 = SPI.transfer(0x00);
  uint8_t re4 = SPI.transfer(0x00);
  digitalWrite(chipSelectPin_434,HIGH);
  //Serial.println("setting TX...");
  wait_434();
  //Serial.println("Done.");
//  PrintHex8(&re1,1);Serial.println("");
//  PrintHex8(&re2,1);Serial.println("");
//  PrintHex8(&re3,1);Serial.println("");
//  PrintHex8(&re4,1);Serial.println("");
}

void set_rx_434(uint8_t timeout1,uint8_t timeout2,uint8_t timeout3){
  digitalWrite(chipSelectPin_434,LOW);
  uint8_t re1 = SPI.transfer(0x82);
  uint8_t re2 = SPI.transfer(timeout1);
  uint8_t re3 = SPI.transfer(timeout2);
  uint8_t re4 = SPI.transfer(timeout3);
  digitalWrite(chipSelectPin_434,HIGH);
  //Serial.println("setting RX...");
  wait_434();
  //Serial.println("Done.");
//  PrintHex8(&re1,1);Serial.println("");
//  PrintHex8(&re2,1);Serial.println("");
//  PrintHex8(&re3,1);Serial.println("");
//  PrintHex8(&re4,1);Serial.println("");
}

void set_dio3_as_tcxo_ctrl_434(){
  digitalWrite(chipSelectPin_434,LOW);
  uint8_t re1 = SPI.transfer(0x97);
  uint8_t re2 = SPI.transfer(0x06);
  uint8_t re3 = SPI.transfer(0x01);
  uint8_t re4 = SPI.transfer(0x01);
  uint8_t re5 = SPI.transfer(0x01);
  digitalWrite(chipSelectPin_434,HIGH);
  Serial.println("set_dio3_as_tcxo_ctrl_434...");
  wait_434();
  Serial.println("Done.");
//  PrintHex8(&re1,1);Serial.println("");
//  PrintHex8(&re2,1);Serial.println("");
//  PrintHex8(&re3,1);Serial.println("");
//  PrintHex8(&re4,1);Serial.println("");
//  PrintHex8(&re5,1);Serial.println("");
}

void get_device_errors_434(){
  digitalWrite(chipSelectPin_434,LOW);
  uint8_t re1 = SPI.transfer(0x17);
  uint8_t re2 = SPI.transfer(0x00);
  uint8_t re3 = SPI.transfer(0x00);
  uint8_t re4 = SPI.transfer(0x00);
  digitalWrite(chipSelectPin_434,HIGH);
  Serial.println("getting device error...");
  wait_434();
  Serial.println("Done.");
  PrintHex8(&re1,1);Serial.println("");
  PrintHex8(&re2,1);Serial.println("");
  PrintHex8(&re3,1);Serial.println("");
  PrintHex8(&re4,1);Serial.println("");
}

void clear_device_errors_434(){
  digitalWrite(chipSelectPin_434,LOW);
  uint8_t re1 = SPI.transfer(0x07);
  uint8_t re2 = SPI.transfer(0x00);
  digitalWrite(chipSelectPin_434,HIGH);
  Serial.println("clearing device error...");
  wait_434();
  Serial.println("Done.");
  //PrintHex8(&re1,1);Serial.println("");
  //PrintHex8(&re2,1);Serial.println("");
}

uint8_t * read_buffer_434(uint8_t offset,int count){
  //Serial.println("Reading buffer:");
  static uint8_t receive_buffer[255];
  digitalWrite(chipSelectPin_434,LOW);
  uint8_t re1 = SPI.transfer(0x1E);
  uint8_t re2 = SPI.transfer(offset);
  uint8_t re3 = SPI.transfer(0x00);
  //PrintHex8(&re1,1);Serial.println("");
  //PrintHex8(&re2,1);Serial.println("");
  for (int i=0; i<count;i++){
    wait_434();
    uint8_t re4 = SPI.transfer(0x00);
    //PrintHex8(&re3,1);Serial.println("");
    receive_buffer[i] = re4;
  }
  digitalWrite(chipSelectPin_434,HIGH);
  return receive_buffer;
}

/*
 * return a pointer 
 * *ptr = PayloadLengthRx
 * *(ptr+1) = RxStartBufferPointer
 */
uint8_t * GetRxBufferStatus_434(){
  static uint8_t rx_buffer_status[2];
  digitalWrite(chipSelectPin_434,LOW);
  uint8_t re1 = SPI.transfer(0x13);
  uint8_t re2 = SPI.transfer(0x00);
  uint8_t re3 = SPI.transfer(0x00);
  uint8_t re4 = SPI.transfer(0x00);
  digitalWrite(chipSelectPin_434,HIGH);
//  PrintHex8(&re1,1);Serial.println("");
//  PrintHex8(&re2,1);Serial.println("");
//  PrintHex8(&re3,1);Serial.println("");
//  PrintHex8(&re4,1);Serial.println("");
  rx_buffer_status[0] = re3;
  rx_buffer_status[1] = re4;
  return rx_buffer_status;
}

void GetPacketStatus_434(){
  digitalWrite(chipSelectPin_434,LOW);
  uint8_t re1 = SPI.transfer(0x14);
  uint8_t re2 = SPI.transfer(0x00);
  uint8_t re3 = SPI.transfer(0x00);
  uint8_t re4 = SPI.transfer(0x00);
  digitalWrite(chipSelectPin_434,HIGH);
//  PrintHex8(&re1,1);Serial.println("");
//  PrintHex8(&re2,1);Serial.println("");
//  PrintHex8(&re3,1);Serial.println("");
//  PrintHex8(&re4,1);Serial.println("");
//  Serial.print("SNR:");
//  Serial.print(re3 / 4);
//  Serial.println(" dB");
//  Serial.print("Rssi:");
//  Serial.print(-re4 / 2);
//  Serial.println(" dBm");
}

void GetRssilnst_434(){
  digitalWrite(chipSelectPin_434,LOW);
  uint8_t re1 = SPI.transfer(0x15);
  uint8_t re2 = SPI.transfer(0x00);
  uint8_t re3 = SPI.transfer(0x00);
  digitalWrite(chipSelectPin_434,HIGH);
//  PrintHex8(&re1,1);Serial.println("");
//  PrintHex8(&re2,1);Serial.println("");
//  PrintHex8(&re3,1);Serial.println("");
//  Serial.print("Rssi:");
//  Serial.println(-re3 / 2);
}

/*
 * FS         0x40 The radio goes into FS mode after Tx or Rx
 * STDBY_XOSC 0x30 The radio goes into STDBY_XOSC mode after Tx or Rx
 * STDBY_RC   0x20 The radio goes into STDBY_RC mode after Tx or Rx
 */
void SetRxTxFallbackMode_434(){
  digitalWrite(chipSelectPin_434,LOW);
  uint8_t re1 = SPI.transfer(0x93);
  uint8_t re2 = SPI.transfer(0x40);
  digitalWrite(chipSelectPin_434,HIGH);
  Serial.println("SetRxTxFallbackMode to FS");
  wait_434();
  Serial.println("done");
}

void ResetStats_434(){
  digitalWrite(chipSelectPin_434,LOW);
  for (int i=0;i<7;i++){
    uint8_t re = SPI.transfer(0x00);
  }
  digitalWrite(chipSelectPin_434,HIGH);
}

void set_boost_rx_434(){
  digitalWrite(chipSelectPin_434,LOW);
  uint8_t re1 = SPI.transfer(0x0D);
  uint8_t re2 = SPI.transfer(0x08);
  uint8_t re3 = SPI.transfer(0xAC);
  uint8_t re4 = SPI.transfer(0x96);
  digitalWrite(chipSelectPin_434,HIGH);
//  PrintHex8(&re1,1);Serial.println("");
//  PrintHex8(&re2,1);Serial.println("");
//  PrintHex8(&re3,1);Serial.println("");
//  PrintHex8(&re4,1);Serial.println("");
  Serial.println("Set Rx boost");
  wait_434();
  Serial.println("done");
}

void SetRxDutyCycle_434(){
  digitalWrite(chipSelectPin_434,LOW);
  uint8_t re1 = SPI.transfer(0x94);
  uint8_t re2 = SPI.transfer(0x00);
  uint8_t re3 = SPI.transfer(0xFA);
  uint8_t re4 = SPI.transfer(0x00);
  uint8_t re5 = SPI.transfer(0x07);
  uint8_t re6 = SPI.transfer(0xD0);
  uint8_t re7 = SPI.transfer(0x00);
  digitalWrite(chipSelectPin_434,HIGH);
  Serial.println("Set Rx duty cycle to 1s/8s");
  wait_434();
  Serial.println("done");
}
