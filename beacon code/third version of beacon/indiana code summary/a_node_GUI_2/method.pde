void create_new_tab(){
  //add new tab
      tab_count ++;
      cp5.addTab("tab"+tab_count);
      
     cp5.getTab("tab"+tab_count).getCaptionLabel().setFont(createFont("arial", 20, true));
      cp5.getTab("tab"+tab_count)
         .activateEvent(true)
         .setLabel("?")
         .setValue(0)
         .setHeight(20)
         .setId(254)
     ;
   //add Unpower button  
     cp5.addButton("Unpower node"+tab_count)
     .setBroadcast(false)
     .setPosition(300,360)
     .setSize(70,30)
     .setValue(tab_count)
     .setLabel("Unpower node")
     .setBroadcast(true)
     .getCaptionLabel().align(CENTER,CENTER)
     ;
     cp5.getController("Unpower node"+tab_count).moveTo("tab"+tab_count);
     
   //add done button  
     cp5.addButton("Done"+tab_count)
     .setBroadcast(false)
     .setPosition(220,360)
     .setSize(70,30)
     .setValue(tab_count)
     .setLabel("lock all")
     .setBroadcast(true)
     .getCaptionLabel().align(CENTER,CENTER)
     ;
     cp5.getController("Done"+tab_count).moveTo("tab"+tab_count);
     
     //add unlock button  
     cp5.addButton("unlock"+tab_count)
     .setBroadcast(false)
     .setPosition(140,360)
     .setSize(70,30)
     .setValue(tab_count)
     .setLabel("unlock")
     .setBroadcast(true)
     .getCaptionLabel().align(CENTER,CENTER)
     ;
     cp5.getController("unlock"+tab_count).moveTo("tab"+tab_count);
     
   /*
    *configure area
    */
    //text feild
     cp5.addTextfield("destination"+tab_count)
     .setPosition(40,30)
     .setSize(40,20)
     .setValue(tab_count)
     .setFont(createFont("arial",20))
     .setAutoClear(false)
     .getCaptionLabel().align(ControlP5.LEFT_OUTSIDE, ControlP5.CENTER)
                       .setFont(createFont("arial",10))
                       .setText("Dest:")
                       .setColor(0)
                       .getStyle().setPaddingLeft(-10);
     
     cp5.getController("destination"+tab_count).moveTo("tab"+tab_count);
     
     cp5.addTextfield("node_ID"+tab_count)
     .setPosition(120,30)
     .setSize(40,20)
     .setValue(tab_count)
     .setFont(createFont("arial",20))
     .setAutoClear(false)
     .getCaptionLabel().align(ControlP5.LEFT_OUTSIDE, ControlP5.CENTER)
                       .setFont(createFont("arial",10))
                       .setText("ID:")
                       .setColor(0)
                       .getStyle().setPaddingLeft(-10);
     
     cp5.getController("node_ID"+tab_count).moveTo("tab"+tab_count);
     
     //add configure button
     cp5.addButton("configure"+tab_count)
     .setBroadcast(false)
     .setPosition(200,30)
     .setSize(40,20)
     .setValue(tab_count)
     .setLabel("config")
     .setBroadcast(true)
     .getCaptionLabel().align(CENTER,CENTER)
     ;
     cp5.getController("configure"+tab_count).moveTo("tab"+tab_count);
     
     //add a toggle button
     cp5.addToggle("toggle"+tab_count)
     .setBroadcast(false)
     .setPosition(280,30)
     .setSize(60,20)
     .setValue(tab_count)
     .setMode(ControlP5.SWITCH)
     .moveTo("tab"+tab_count)
     .setState(false)
     .setBroadcast(true)
     .getCaptionLabel().align(ControlP5.CENTER, ControlP5.CENTER)
                       .setFont(createFont("arial",10))
                       .setText("fixed")
                       .setColor(0)
                       .getStyle().setPaddingLeft(-15)
     ;
     
     /*
      * short echo area
      */
     //add short echo button
     cp5.addButton("short_echo"+tab_count)
     .setBroadcast(false)
     .setPosition(40,80)
     .setSize(60,20)
     .setValue(tab_count)
     .setLabel("short echo")
     .setBroadcast(true)
     .getCaptionLabel().align(CENTER,CENTER)
     ;
     cp5.getController("short_echo"+tab_count).moveTo("tab"+tab_count);
     
     // add short echo check button(image)
     PImage[] imgs = {check_img,check_img,check_img};
     cp5.addButton("check"+tab_count)
     .setBroadcast(false)
     .setPosition(140,80)
     .updateSize()
     .setImages(imgs)
     .moveTo("tab"+tab_count)
     .setVisible(false)
     .lock()
     ;
     
     //add short echo button
     cp5.addButton("gps"+tab_count)
     .setBroadcast(false)
     .setPosition(200,80)
     .setSize(40,20)
     .setColorBackground(color(0,150,225))
     .setValue(tab_count)
     .setLabel("GPS")
     .moveTo("tab"+tab_count)
     .setVisible(false)
     .lock()
     .getCaptionLabel().align(CENTER,CENTER)
                       .setColor(0)
     ;
  /*
    * long echo part
    */
    // long echo destination text field
     cp5.addTextfield("long_echo_destination"+tab_count)
     .setPosition(40,130)
     .setSize(40,20)
     .setValue(tab_count)
     .setFont(createFont("arial",20))
     .moveTo("tab"+tab_count)
     .setAutoClear(false)
     .getCaptionLabel().align(ControlP5.LEFT_OUTSIDE, ControlP5.CENTER)
                       .setFont(createFont("arial",10))
                       .setText("Find:")
                       .setColor(0)
                       .getStyle().setPaddingLeft(-10);
     

    
     //add long_echo button
     cp5.addButton("long_echo"+tab_count)
     .setBroadcast(false)
     .setPosition(120,130)
     .setSize(50,20)
     .setLabel("long echo")
     .moveTo("tab"+tab_count)
     .setBroadcast(true)
     .getCaptionLabel().align(CENTER,CENTER)
     ;
    //add roll left button
     cp5.addButton("roll_left"+tab_count)
     .setBroadcast(false)
     .setPosition(300,160)
     .setSize(20,20)
     .setLabel("<")
     .moveTo("tab"+tab_count)
     .setBroadcast(true)
     .getCaptionLabel().align(CENTER,CENTER)
     ;
     //add roll right button
     cp5.addButton("roll_right"+tab_count)
     .setBroadcast(false)
     .setPosition(320,160)
     .setSize(20,20)
     .setLabel(">")
     .moveTo("tab"+tab_count)
     .setBroadcast(true)
     .getCaptionLabel().align(CENTER,CENTER)
     ;
}

void fix_configure_area(int tab_num){
    cp5.get(Textfield.class,"destination"+tab_num).setLock(true);
    cp5.get(Textfield.class,"node_ID"+tab_num).setLock(true);
    cp5.get(Button.class,"configure"+tab_num).lock().setPosition(700,0).setVisible(false);
    cp5.getTab("tab"+tab_num).setValue(1);
}

void unfix_configure_area(int tab_num){
    cp5.get(Textfield.class,"destination"+tab_num).setLock(false);
    cp5.get(Textfield.class,"node_ID"+tab_num).setLock(false);
    cp5.get(Button.class,"configure"+tab_num).unlock().setPosition(200,30).setVisible(true);;
    cp5.getTab("tab"+tab_num).setValue(0);
}
