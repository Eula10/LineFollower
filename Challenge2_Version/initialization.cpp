void initSystem(){  
  lineSensors.initFiveSensors();
  
  Serial.begin(115200); 
  Serial1.begin(38400);

  uint16_t batteryLevel = readBatteryMillivolts();
  Serial1.print("Battery Level: ");  
  Serial1.println(batteryLevel);
  
  calibrateSensors();
  
  Serial1.println("Initialization completed");
  

}
