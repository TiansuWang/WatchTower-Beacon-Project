// OLED display 

void update_OLED(){
  display.clearDisplay();
  
  display.setTextSize(1);
  display.setTextColor(WHITE);
  
  display.setCursor(0,0);
  display.println(last_434_error);
  display.println(last_915_error);

  if(GPS.fix){
      display.print("Time:");
      uint8_t Boston_hour = 0;
      if(greenwich_hour<4){
        Boston_hour = greenwich_hour + 24 - 5;
      }else{
        Boston_hour = greenwich_hour - 5;
      }
      display.print(Boston_hour); display.print(':');
      display.print(greenwich_minute); display.print(':');
      display.print(greenwich_seconds); display.println('.');

      display.print(GPS.latitude/100, 3); display.print(GPS.lat);
      display.print(", ");
      display.print(GPS.longitude/100, 3); display.println(GPS.lon);
  }

  if(node_ID != 254){
    display.setTextColor(WHITE);
    display.setCursor(48,40);
    display.print("(");display.print(node_ID);display.print(")");
  }

  if(unicast_destination != 253){
    display.setTextColor(WHITE);
    display.setCursor(16,40);
    display.print(unicast_destination);
    display.print("<-");
  }

  if(!config_done_flag){
    display.setTextColor(WHITE);
    display.setCursor(0,40);
    display.print("UL");
  }

  if(unicast_backward_destination != 255){
    display.setCursor(80,40);
    display.print("<-");
    display.print(unicast_backward_destination);
  }

  if(robot_passed_flag){

//    long int start_totoal_second = rbt_pass_hour_start * 3600
//                             +rbt_pass_minute_start * 60
//                             +rbt_pass_seconds_start;
//    long int end_totoal_second = rbt_pass_hour_end * 3600
//                           +rbt_pass_minute_end * 60
//                           +rbt_pass_seconds_end;
//    if(end_totoal_second < start_totoal_second){
//      end_totoal_second = end_totoal_second + 24 * 3600;
//    }
//    long int average_totoal_second = (end_totoal_second + start_totoal_second)/2;
//    if(average_totoal_second > 24 * 3600){
//      average_totoal_second = average_totoal_second - 24 * 3600;
//    }
//    uint8_t M1_hour = average_totoal_second / 3600;
//    uint8_t M1_minute = (average_totoal_second - M1_hour * 3600)/60;
//    uint8_t M1_seconds = average_totoal_second % 60;
    uint8_t M1_hour = rbt_pass_hour_end;
    uint8_t M1_minute = rbt_pass_minute_end;
    uint8_t M1_seconds = rbt_pass_seconds_end;
    
    display.setCursor(0,48);
    display.print("Rssi: ");
    //display.print("M1:");
    if(signal_flag){
      display.setTextColor(BLACK, WHITE);
      display.print(Rssi);display.println("dB");
    }else{
      display.setTextColor(WHITE);
      display.println();
    }
    display.print(M1_hour); display.print(':');
    display.print(M1_minute); display.print(':');
    display.print(M1_seconds); 
    display.setTextColor(WHITE);
    display.print(" ");
    display.print(rbt_pass_hour_MaxRssi); display.print(':');
    display.print(rbt_pass_minute_MaxRssi); display.print(':');
    display.print(rbt_pass_seconds_MaxRssi); 
  }

  
  display.display();
}
