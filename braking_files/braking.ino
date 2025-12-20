#include <SPI.h>


// Define the maximum size of our serial message
// (e.g., "<-100,-100>\n" is 12 chars, so 32 is plenty)
const byte SERIAL_BUFFER_SIZE = 32;
char serialBuffer[SERIAL_BUFFER_SIZE]; // The buffer to store incoming data
int bufferIndex = 0;                   // Current position in the buffer
bool newDataAvailable = false;

const int volt_output_pin = 6;
long brake_val = 0; 
long brake2_val = 0;    // Not currently used
unsigned long lastPacketTime = 0;
const unsigned long SERIAL_TIMEOUT_MS = 500;


void setup() {

  // put your setup code here, to run once:
  Serial.begin(115200);

    while (!Serial) {
      delay(50); // wait
    }
    Serial.println("Arduino is ready.");

    delay(1000);
    pinMode(volt_output_pin, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:

  get_command();
  send_brake_cmd( brake_val); 

}

void send_brake_cmd(long brake_val){
  brake_val = constrain(brake_val, 0, 255);

  if (millis() - lastPacketTime > SERIAL_TIMEOUT_MS) {
    analogWrite(volt_output_pin, 0);
  }
  else{
    analogWrite(volt_output_pin, brake_val);
  }
    
  // analogWrite(volt_output_pin, brake_val);
}

void get_command() {
  // --- Step 1: Check for new serial data ---
  checkSerial();
  if (newDataAvailable) {
    char* startMarker = strchr(serialBuffer, '<');
    char* endMarker = strchr(serialBuffer, '>');

    if (startMarker != NULL && endMarker != NULL) {
      
      char* comma = strchr(serialBuffer, ',');

      if (comma != NULL) {
        
        brake_val = strtol(startMarker + 1, NULL, 10);
        brake2_val = strtol(comma + 1, NULL, 10);

      } else {
        Serial.println("Error: Packet received, but no comma found.");
      }
    } else {
      Serial.println("Error: Received malformed packet.");
    }
    clearBuffer();
    newDataAvailable = false;
  }
}

void checkSerial() {
  // Serial.println("Checking serial....");
  while (Serial.available() > 0 && !newDataAvailable) {
    char c = Serial.read();

    // If we hit the newline, the message is complete
    if (c == '\n' || c == '\r') {
      if (bufferIndex > 0) { // Make sure we got at least one char
        serialBuffer[bufferIndex] = '\0'; // Null-terminate the string
        newDataAvailable = true;

        //  ****** This is to make sure that post partial disconnt motor doesnt keep spining  ******
        brake_val = 0;  
      }
    }
    // If we get the start marker, reset the buffer
    // This helps re-sync if we get a partial message
    else if (c == '<') {
      clearBuffer();
      serialBuffer[bufferIndex] = c;
      bufferIndex++;
      newDataAvailable = false;

      //  ****** This is to make sure that post partial disconnt motor doesnt keep spining  ******
        brake_val = 0;  
    }
    // Otherwise, add the character to our buffer
    // as long as we don't overflow
    else if (bufferIndex < SERIAL_BUFFER_SIZE - 1) {
      serialBuffer[bufferIndex] = c;
      bufferIndex++;
    }
  }
}

void clearBuffer() {
  memset(serialBuffer, 0, SERIAL_BUFFER_SIZE);
  bufferIndex = 0;
  // Serial.flush();
}

