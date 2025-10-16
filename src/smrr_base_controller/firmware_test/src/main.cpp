String inputString = "";
boolean stringComplete = false;

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ; // Wait for serial port to connect
  }
  
  // Send a startup message
  Serial.println("STM32 Ready!");
}

void loop() {
  // Read incoming commands
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    inputString += inChar;
    
    if (inChar == '\n') {
      stringComplete = true;
    }
  }
  
  // Process complete command
  if (stringComplete) {
    // Parse received command (format: "vel1,vel2,0,0\n")
    float vel1 = 0.0, vel2 = 0.0;
    int dummy1 = 0, dummy2 = 0;
    
    // Parse the command
    sscanf(inputString.c_str(), "%f,%f,%d,%d", &vel1, &vel2, &dummy1, &dummy2);
    
    // TODO: Send vel1 and vel2 to your motor controllers here
    // For now, just echo what we received
    
    // Send feedback - echo velocities in expected format
    char buf[50];
    char prefix1 = (vel1 >= 0) ? 'p' : 'n';
    char prefix2 = (vel2 >= 0) ? 'p' : 'n';
    sprintf(buf, "r%c%.2f,l%c%.2f", prefix1, fabs(vel1), prefix2, fabs(vel2));
    Serial.println(buf);
    
    // Clear for next command
    inputString = "";
    stringComplete = false;
  }
}
