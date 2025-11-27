#include <mcp_can.h>
#include <SPI.h>

const int SPI_CS_PIN = 9;   
MCP_CAN CAN0(SPI_CS_PIN);

// Define the maximum size of our serial message
// (e.g., "<-100,-100>\n" is 12 chars, so 32 is plenty)
const byte SERIAL_BUFFER_SIZE = 32;
char serialBuffer[SERIAL_BUFFER_SIZE]; // The buffer to store incoming data
int bufferIndex = 0;                   // Current position in the buffer
bool newDataAvailable = false;
long vel_x = 0;
long vel_y = 0;

void setup() {
    Serial.begin(9600);

    while (!Serial) {
      delay(50); // wait
    }
    Serial.println("Arduino is ready.");

    delay(1000);

    while (CAN0.begin(MCP_ANY, CAN_1000KBPS, MCP_8MHZ) != CAN_OK) {
        Serial.println("CAN Init Failed");
        
        delay(200);
    }
    Serial.println("CAN Init OK!");

    // 2. Set to Normal Mode
    CAN0.setMode(MCP_NORMAL); 
}

void loop() {

    get_command();
    send_rpm_command(104, vel_x); 

    // --- PART B: READ FEEDBACK ---
    // Check if the motor is sending us status data (Temp, Error Codes, etc.)
    if (CAN0.checkReceive() == CAN_MSGAVAIL) {
        long unsigned int rxId;
        unsigned char len = 0;
        unsigned char rxBuf[8];

        CAN0.readMsgBuf(&rxId, &len, rxBuf);
        // Serial.println("Got message");

        // Print the ID we received
        Serial.print("RX ID: 0x");
        Serial.print(rxId, HEX);
        
        // Manual Page 44: Feedback Data Structure
        // Data[0-1]: Position
        // Data[2-3]: Speed
        // Data[4-5]: Current
        // Data[6]: Temp
        // Data[7]: Error Code
        
        if (len >= 8) {
            int8_t pos = (rxBuf[0] << 8) | rxBuf[1] ;
            int8_t Speed = (rxBuf[2] << 8) | rxBuf[3] ;
            int8_t curr = (rxBuf[4] << 8) | rxBuf[5] ;
            int8_t temp = rxBuf[6];
            int8_t error = rxBuf[7];

            Serial.print("Pos: "); 
            Serial.print(pos * 0.1); 
            
            // Speed (Scale: 10 ERPM) [cite: 992]
            Serial.print(" rad | Spd: "); 
            Serial.print(Speed * 10); 
            
            // Current (Scale: 0.01 A) [cite: 993]
            Serial.print(" ERPM | Cur: "); 
            Serial.print(curr * 0.01); 
            
            // Temperature & Error
            Serial.print(" A | T: "); 
            Serial.print(temp); 
            Serial.print("C | Err: "); 
            Serial.println(error); // println here ends the line
            
            // Error Code Reference (Page 45):
            // 1: Over Temp
            // 2: Over Current
            // 3: Over Voltage
            // 4: Under Voltage
            // 7: Motor Stall
        } else {
            Serial.println(" | Received Packet (Len < 8)");
        }
    }
    
    delay(10);  // 20 Hz
}

void send_rpm_command(uint8_t controller_id, long rpm) {
    byte buffer[4];
    buffer[0] = (rpm >> 24) & 0xFF;
    buffer[1] = (rpm >> 16) & 0xFF;
    buffer[2] = (rpm >> 8)  & 0xFF;
    buffer[3] = rpm & 0xFF;
    
    // ID = 3 (RPM Mode) | Motor ID
    uint32_t can_id = controller_id | ((uint32_t)0x03 << 8);
    
    byte send_ok =CAN0.sendMsgBuf(can_id, 1, 4, buffer);

    if (send_ok != CAN_OK) Serial.println("its gonee... ");
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
        
        vel_x = strtol(startMarker + 1, NULL, 10);
        vel_y = strtol(comma + 1, NULL, 10);

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
      }
    }
    // If we get the start marker, reset the buffer
    // This helps re-sync if we get a partial message
    else if (c == '<') {
      clearBuffer();
      serialBuffer[bufferIndex] = c;
      bufferIndex++;
      newDataAvailable = false;
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