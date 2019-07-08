void controlEvent(ControlEvent theControlEvent) {
  String name = theControlEvent.getName();
  int ID; 
  String group;
  String tab_num_string;
  if(name.charAt(name.length()-2)>47 && name.charAt(name.length()-2)<58){
    group = name.substring(0,name.length()-1);
    tab_num_string = name.substring(name.length()-1,name.length());
  }else{
    group = name.substring(0,name.length()-1);
    tab_num_string = name.substring(name.length()-1,name.length());
  }
  
  if (theControlEvent.isTab()|name.compareTo("reconnect") == 0) {
    ID = theControlEvent.getId();
  }else{
    ID = cp5.getTab("tab"+tab_num_string).getId();
  }
  println(name);
  println(ID);
  
  try {
      tab_num = Integer.parseInt(tab_num_string);
    }
  catch( Exception e ) {
      //println("Not a proper input");
    }

  if (theControlEvent.isTab()) {
    //println("got an event from tab : "+name+" with id "+ID);
    if(ID == 255){
      create_new_tab();
      Data_demonstrsation.setVisible(false);
    } else{
      Data_demonstrsation.setVisible(true);
    }
  }else if(group.compareTo("Unpower node") == 0){ 
    println("Unpower node pressed in tab:" + tab_num);
    cp5.getTab("tab"+ tab_num).remove();
    tab_num = -1;
    
  }else if(group.compareTo("configure") == 0){
    println("config button in tab:" + tab_num);
    destination_string = cp5.get(Textfield.class,"destination"+tab_num).getText();
    node_ID_string = cp5.get(Textfield.class,"node_ID"+tab_num).getText();
    if(node_ID_string.compareTo("") == 0 ||destination_string.compareTo("") == 0){
      return;
    }
    int destination  = Integer.parseInt(destination_string);
    int node_ID  = Integer.parseInt(node_ID_string);
    // check for validate input
    if(destination >= 0 && destination <= 254 && node_ID >= 0 && node_ID <= 255 && node_ID != destination) {
      println("send out configuration: " + node_ID + "," + destination + ".");
      String configure_command = node_ID + "," + destination + ".\n";
      //println(configure_command);
      myPort.write(configure_command);
      ID_Dest = "";
    }else{
      println("not a valid input.");
      return;
    }
    
  }else if(group.compareTo("toggle") == 0){
    if(cp5.get(Toggle.class,"toggle"+tab_num).getState()){
      fix_configure_area(tab_num);
    }else{
      unfix_configure_area(tab_num);
    } 
  }else if(group.compareTo("short_echo") == 0){
    println("perform short echo");
    String short_echo_command = "e\n";
    myPort.write(short_echo_command);
    cp5.get(Button.class,"check"+tab_num).setVisible(false);
  }else if(group.compareTo("Done") == 0){
    println("lock all the node");
    String lock_all_command = "d\n";
    myPort.write(lock_all_command);
    
  }else if(group.compareTo("unlock") == 0){
    if(ID != -1){
      println("unlock node "+ ID);
      String unlock_command = "r"+ID+"\n";
      myPort.write(unlock_command);
    }else{
      println("need to configure first");
    }
  }else if(group.compareTo("long_echo") == 0){
    println("long echo button in tab:" + tab_num);
    long_echo_dest_string = cp5.get(Textfield.class,"long_echo_destination"+tab_num).getText();
    if(long_echo_dest_string.compareTo("") == 0){
      return;
    }
      int long_echo_dest  = Integer.parseInt(long_echo_dest_string);
      // check for validate input
      if(long_echo_dest >= 0 && long_echo_dest <= 254) {
        println("long echo to find: " + long_echo_dest + ".");
        String long_echo_command = "e" + long_echo_dest + "\n";
        //println(configure_command);
        myPort.write(long_echo_command);
      }else{
        println("not a valid input.");
        return;
      }   
    
  }else if(group.compareTo("roll_left") == 0){
    if(data_pointer>0){
      data_pointer--;
    }
  }else if(group.compareTo("roll_right") == 0){
    if(data_pointer < data_array.size()-1){
      data_pointer++;
    }
  }else if(group.compareTo("reconnec") == 0){
    printArray(Serial.list());
    println(Serial.list()[1]);
    println(Serial.list().length);
    for(int i = 0; i < Serial.list().length ; i++){
      if(Serial.list()[i].compareTo("/dev/cu.SLAB_USBtoUART") == 0){
        myPort = new Serial(this, Serial.list()[i], 115200);
        break;
      }
    }
    if(myPort == null){
      println("No proper serial port found");
      //while(true);
    }
    myPort.bufferUntil(10);
  }
}
