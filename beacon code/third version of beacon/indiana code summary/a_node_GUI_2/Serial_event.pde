void serialEvent(Serial p) {
  //update the text area
  inString = p.readString();
  textArea_content = textArea_content +hour()+":"+minute()+":"+second()+"->"+ inString;
  //read configuration reply
  log_writer.print(hour()+":"+minute()+":"+second()+"->"+ inString);
  log_writer.flush();
  check_configure_reply(inString);
  check_short_echo_reply(inString);
  check_long_echo(inString);
}

void check_configure_reply(String inString){
  //print(inString);
  //println(get_configure_reply);
  //println(inString.length());
  if(inString.length()==22){
    if(inString.substring(0,19).compareTo("get configure reply") == 0){
      get_configure_reply = true;
    } 
  }

  if(get_configure_reply){
    if(inString.substring(0,3).compareTo("ID:") == 0){
      ID_Dest = ID_Dest + inString.substring(3,inString.length()-2);
    }else if(inString.substring(0,5).compareTo("DEST:") == 0){
      ID_Dest = ID_Dest + inString.substring(5,inString.length()-2);
      //println(ID_Dest);
      //println(ID_Dest.length());
      //println(node_ID_string + destination_string);
      //println((node_ID_string + destination_string).length());
      println(ID_Dest.compareTo(node_ID_string + destination_string));
      if(ID_Dest.compareTo(node_ID_string + destination_string) == 0){
         cp5.get(Toggle.class,"toggle"+tab_num).setState(true);
         //cp5.get(Textfield.class,"destination"+tab_num).setLock(true);
         //cp5.get(Textfield.class,"node_ID"+tab_num).setLock(true);
         //cp5.get(Button.class,"configure"+tab_num).setLock(true);
         //cp5.getTab("tab"+tab_num).setValue(1);
         cp5.getTab("tab"+tab_num).setCaptionLabel(node_ID_string)
                                  .setId(Integer.parseInt(node_ID_string));
         get_configure_reply = false;
         
      }
    }
  }  
}

void check_short_echo_reply(String inString){
  if(inString.length()==15){
    if(inString.substring(0,13).compareTo("echo success.") == 0){
      cp5.get(Button.class,"check"+tab_num).setVisible(true);
    } 
  }
  if(inString.length()==11){
    if(inString.substring(0,9).compareTo("GPS fix:1") == 0){
      destination_string = cp5.get(Textfield.class,"destination"+tab_num).getText();
      try {
        int destination  = Integer.parseInt(destination_string);
        for(int i = tab_num-1; i >= 0; i--){
            Tab tb= cp5.getTab("tab"+i);
            if(tb.getId()==-1){
              tb.remove();
              continue;
            }
            println(tb.getId());
            if(tb.getId() == destination){
              cp5.get(Button.class,"gps"+i).setVisible(true);
              break;
            }
        }
      }
      catch( Exception e ) {
        println("error:you have to do configure first.");
      }
    } 
  }
}

void check_long_echo(String inString){
  if(inString.length()==38){
    if(inString.substring(0,36).compareTo("************************************")==0){
      data_array.add("fetched data:\n");
      data_coming = true;  
      data_count++;
      data_pointer = data_count;
    }
  }
  
  if(data_coming){
    data_array.set(data_pointer,data_array.get(data_pointer) + inString);
  }
  
  if(inString.length()>25 && inString.length()<29){
    if(inString.substring(0,19).compareTo("mid algorithm time:")==0){
      data_coming = false;
      data_writer.println(hour()+":"+minute()+":"+second()+"->"+data_array.get(data_pointer));
      data_writer.flush();
    }
  }
  //print(inString);
  //print(inString.length());
  //println(data_coming);
  
}
