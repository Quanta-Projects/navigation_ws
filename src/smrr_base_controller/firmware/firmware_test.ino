/*
 * Simple Test Firmware for BaseController
 * 
 * This firmware reads velocity commands from the serial port
 * and echoes them back in the format: right{value}.left{value}
 * 
 * Expected input format: {right_vel},{left_vel},0,0\n
 * Example: 6.17,6.17,0,0
 * 
 * Output format: right{value}.left{value}
 * Example: right6.17.left6.17
 */

// Command parsing variables
String inputBuffer = "";
float rightVel = 0.0;
float leftVel = 0.0;
bool commandReady = false;

void setup() {
  // Initialize serial communication at 115200 baud
  Serial.begin(115200);
  
  // Wait for serial port to connect
  while (!Serial) {
    ; // wait for serial port to connect
  }
  
  // Reserve space for the input buffer
  inputBuffer.reserve(50);
  
  // Send a startup message
  delay(100);
  Serial.println("0.00,0.00");
}

void loop() {
  // Read incoming serial data
  while (Serial.available() > 0) {
    char inChar = (char)Serial.read();
    
    // Check for newline (end of command)
    if (inChar == '\n') {
      commandReady = true;
    } else {
      // Add character to buffer
      inputBuffer += inChar;
    }
  }
  
  // Process complete command
  if (commandReady) {
    parseAndEcho();
    
    // Clear buffer and flag for next command
    inputBuffer = "";
    commandReady = false;
  }
  
  // Small delay to prevent overwhelming the serial port
  delay(10);
}

void parseAndEcho() {
  // Parse the input buffer
  // Expected format: {right_vel},{left_vel},0,0
  
  int firstComma = inputBuffer.indexOf(',');
  int secondComma = inputBuffer.indexOf(',', firstComma + 1);
  
  if (firstComma > 0 && secondComma > firstComma) {
    // Extract right wheel velocity
    String rightStr = inputBuffer.substring(0, firstComma);
    rightVel = rightStr.toFloat();
    
    // Extract left wheel velocity
    String leftStr = inputBuffer.substring(firstComma + 1, secondComma);
    leftVel = leftStr.toFloat();
    
    // Echo back in the format: right{value}.left{value}
    Serial.print("right");
    Serial.print(rightVel, 2);  // 2 decimal places
    Serial.print(".left");
    Serial.println(leftVel, 2);  // 2 decimal places
  } else {
    // If parsing fails, send default encoder feedback
    Serial.println("0.00,0.00");
  }
}
