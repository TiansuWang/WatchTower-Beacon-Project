// gps code
void GPS_setup(){
  while (!Serial);  // uncomment to have the sketch wait until Serial is ready
     
  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(9600);
  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // uncomment this line to turn on only the "minimum recommended" data
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
  // the parser doesn't care about other sentences at this time
  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate
  // For the parsing code to work nicely and have time to sort thru the data, and
  // print it out we don't suggest using anything higher than 1 Hz
     
  // Request updates on antenna status, comment out to keep quiet
  GPS.sendCommand(PGCMD_ANTENNA);
  delay(1000);

}

void GPS_update(){
  char c = GPS.read();
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences!
    // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
    //Serial.println(GPS.lastNMEA()); // this also sets the newNMEAreceived() flag to false
    if (!GPS.parse(GPS.lastNMEA())) {// this also sets the newNMEAreceived() flag to false
      return; // we can fail to parse a sentence in which case we should just wait for another
    }else{
      greenwich_hour = GPS.hour;
      greenwich_minute = GPS.minute;
      greenwich_seconds = GPS.seconds;
      if (GPS.fix) {
        node_latitude.coordinate = GPS.latitude/100.0;
        node_longtitude.coordinate = GPS.longitude/100.0;
      }
    }  
}
}

void GPS_displayData(){
  //if (Serial.available()){
    Serial.print("\nTime: ");
      Serial.print(GPS.hour); Serial.print(':');
      Serial.print(GPS.minute); Serial.print(':');
      Serial.print(GPS.seconds); Serial.print('.');
      
      Serial.print("Fix: "); Serial.print((int)GPS.fix);
      if (GPS.fix) {
        Serial.print("Location: ");
        Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
        Serial.print(", ");
        Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);
      }
  //}
}
