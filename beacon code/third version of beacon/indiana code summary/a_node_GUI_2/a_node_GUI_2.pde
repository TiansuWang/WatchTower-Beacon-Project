import controlP5.*;
import processing.serial.*;

ControlP5 cp5;
Serial myPort; 
Textarea myTextarea;
Textarea Data_demonstrsation;

int tab_count = -1;
int tab_num = -1;
int last_operation_tab = 0;
String textArea_content = "Receive serial reply here:\n";
//String fetched_data = "Fetched data:";
String inString = "";
String destination_string;
String node_ID_string;
String long_echo_dest_string;
String ID_Dest = "";
boolean get_configure_reply = false;
boolean data_coming = false;
PImage check_img;
ArrayList<String> data_array = new ArrayList<String>();
int data_pointer = 0;
int data_count = 0;

String data_filename = "";
String log_filename = "";
PrintWriter data_writer;
PrintWriter log_writer;

void setup() {
  data_filename = "beacon data/"+ year() +"." + month() +"."  + day() + "_" + hour() +":"+ minute() +":"+ second()+".txt";
  log_filename =  "log/" + year() +"."  + month() +"."  + day() + "_" + hour() +":"+ minute() +":"+ second()+".txt";
  data_writer=createWriter(data_filename);
  log_writer =createWriter(log_filename);
  
  check_img = loadImage("check.jpg");
  check_img.resize(20,20);
  size(700,400);
  noStroke();
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
    while(true);
  }
  myPort.bufferUntil(10);
  
  cp5 = new ControlP5(this);
  create_new_tab(); 
  Tab tab = cp5.getTab("default")
               .activateEvent(true)
               .setId(255)
               .setHeight(20);
     tab.getCaptionLabel().setFont(createFont("arial", 20, true));
     tab.setLabel("+");
  
  myTextarea = cp5.addTextarea("txt")
                  .setPosition(380,20)
                  .setSize(300,360)
                  .setFont(createFont("arial",12))
                  .setLineHeight(14)
                  .setColor(color(255))
                  .setColorBackground(color(255,100))
                  .setColorForeground(color(255,100))
                  .scroll(1)
                  .moveTo("global")
                  ;
   myTextarea.setText(textArea_content);

   Data_demonstrsation = cp5.addTextarea("data_demonstration")
                        .setPosition(20,180)
                        .setSize(350,170)
                        .setFont(createFont("arial",12))
                        .setLineHeight(14)
                        .setColor(color(0))
                        .setColorBackground(color(255))
                        .setColorForeground(color(255))
                        .scroll(1)
                        .setVisible(false)
                        .moveTo("global")
                        ;
   data_array.add("fetched data:\n");                    
   Data_demonstrsation.setText(data_array.get(data_pointer));
   
   //reconnect button
   cp5.addButton("reconnect")
     .setBroadcast(false)
     .setPosition(100,200)
     .setSize(70,30)
     .setValue(tab_count)
     .setLabel("reconnect")
     .setBroadcast(true)
     .getCaptionLabel().align(CENTER,CENTER)
     ;
     
   
}

void draw() {
  background(120);
  //println(tab_num);
  int node_ID = -1;
  if(tab_num >= 0){
    String node_ID_string = cp5.get(Textfield.class,"node_ID"+tab_num).getText();
  
    try {
        node_ID = Integer.parseInt( node_ID_string );
      }
    catch( Exception e ) {
        //println("Not a proper input");
      }
      if(node_ID == 0){
        cp5.get(Textfield.class,"destination"+tab_num).setText("253").setLock(true);
      }else{
        if(int(cp5.getTab("tab"+tab_num).getValue()) != 1){
          cp5.get(Textfield.class,"destination"+tab_num).setLock(false);
        }
      }
  }
  
      
   myTextarea.setText(textArea_content);
   Data_demonstrsation.setText(data_array.get(data_pointer));
}


void keyPressed() {

}

//public void reconnect(int theValue) {
//  printArray(Serial.list());
//  println(Serial.list()[1]);
//  println(Serial.list().length);
//  for(int i = 0; i < Serial.list().length ; i++){
//    if(Serial.list()[i].compareTo("/dev/cu.SLAB_USBtoUART") == 0){
//      myPort = new Serial(this, Serial.list()[i], 115200);
//      break;
//    }
//  }
//  if(myPort == null){
//    println("No proper serial port found");
//    //while(true);
//  }
//  myPort.bufferUntil(10);
//}
