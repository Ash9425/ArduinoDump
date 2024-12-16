#include <Wire.h>

// I2C Pins
#define SDA 21
#define SCL 22

// Interrupt Pins
#define INTA 16
#define INTB 17
#define ERR 5

// Shift Register Pins
#define SR_SHnLD 32
#define SR_CLK   33
#define SR_DATA  35

// Warning Disable Pins
#define WARN 18

// MCP23017 Addresses
#define EXP_OUT_0x21 0x21
#define EXP_OUT_0x22 0x22
#define EXP_IN_0x23  0x23
// MCP23017_OUT Reset Pin
#define EXP_OUT_RESET 25
// MCP23017 Registers
#define IODIRA  0x00  // I/O Direction Register for GPIOA
#define IODIRB  0x01  // I/O Direction Register for GPIOB
#define INTFA   0x0E  // Interrupt Flag Register for GPIOA
#define INTFB   0x0F  // Interrupt Flag Register for GPIOB
#define INTCAPA 0x10  // Interrupt Captured Value Register for GPIOA
#define INTCAPB 0x11  // Interrupt Captured Value Register for GPIOB
#define GPIOA   0x12  // GPIOA Data Register
#define GPIOB   0x13  // GPIOB Data Register
#define OLATA   0x14  // Output Latch Register for GPIOA
#define OLATB   0x15  // Output Latch Register for GPIOB

uint8_t bitWidth = 11;           // Number of lines on the program card
uint16_t programCode[11];        // Program code array
uint8_t numberOfPhases = 0;      // Stores how many phases the programcard stores
uint16_t activePhases = 0x0000;  // Currently active phases (after conflict check)
uint8_t activePhaseIndex = 0x00; // Index of currently active phase in program
uint16_t activeSensors = 0;      // Stores which sensors are active and have to be checked for timers (bit 0-7 = Port A 0-7; bit 8-15 = Port B 0-7)
uint32_t sensorTimers[16];       // Stores millis when the sensor was triggered, element 0 for activeSensors[bit 0]
uint32_t sensorLimits[16] = {    // Stores the time limit when a sensor will trigger a phase change in millis, lower is higher priority
  10000, 5000,       // Lane 1 Sensor Port A bit 0 1
  10000, 5000,       // Lane 2 Sensor Port A bit 2 3
  10000, 5000,       // Lane 3 Sensor Port A bit 4 5
  10000, 0, 5000,    // Lane 4 Sensor Port A bit 6 7 Port B bit 0
  13000, 9000,       // Lane 5 Sensor Port B bit 1 2
  13000, 9000,       // Lane 6 Sensor Port B bit 3 4
  13000, 9000, 0     // Lane 7 Sensor Port B bit 5 6 7
};
uint32_t phaseTimer = 0;         // Stores millis when the phase was started
uint32_t phaseLimitsMin[8] = {   // Stores the minimum time the phase has to be active for in millis
  5000, 5000,       // Lane 1, Lane 2
  5000, 5000,       // Lane 3, Lane 4
  4000, 3000,       // Lane 5, Lane 6
  4000, 3000,       // Lane 7, Lane 8 non-existent
};

uint32_t phaseLimitsMax[8] = {   // Stores the maximum time the phase can be active for in millis
  13000, 13000,      // Lane 1, Lane 2
  13000, 13000,      // Lane 3, Lane 4
  10000, 7000,       // Lane 5, Lane 6
  10000, 7000,       // Lane 7, Lane 8 non-existent
};

uint32_t lastChangeTime = 0;
uint32_t changeDelay = 0;

uint8_t lampLocLine[8][6] = {    // [L1-8] [address, register, red, yellow, green, warning]
  {0x21, 0x12, 0x01, 0x02, 0x04, 0x00},
  {0x21, 0x12, 0x08, 0x10, 0x20, 0x40},
  {0x21, 0x13, 0x02, 0x04, 0x08, 0x00},
  {0x21, 0x13, 0x10, 0x20, 0x40, 0x80},
  {0x22, 0x12, 0x04, 0x08, 0x10, 0x00},
  {0x22, 0x12, 0x20, 0x40, 0x80, 0x00},
  {0x22, 0x13, 0x04, 0x08, 0x10, 0x20},
  {0x00, 0x00, 0x00, 0x00, 0x00, 0x00}};

uint8_t lampLocPed[4][5] = {     // [P1-4] [address, register, red, register, green]
  {0x21, 0x12, 0x80, 0x13, 0x01},
  {0x22, 0x12, 0x01, 0x12, 0x02},
  {0x22, 0x13, 0x01, 0x13, 0x02},
  {0x22, 0x13, 0x40, 0x13, 0x80}};

uint16_t storedProgram[] = { // Backup programCode
  0b110101000011,
  0b110100000011,
  0b001101000011,
  0b111101000011,
  0b000011001100,
  0b101111101101,
  0b000001101100,
  0b000000011111,
  0b1101, 0b111, 0b11};

uint16_t Lines[12] = {0x0800, 0x0400, 0x0200, 0x0100, 0x0080, 0x0040, 0x0020, 0x0010, 0x0008, 0x0004, 0x0002, 0x0001}; // not used
bool changePhase = false;
bool changeInProgress = false;
uint8_t lampState = 0;
uint16_t switchToRed = 0;
uint16_t switchToGreen = 0;

volatile bool interruptFlagA = false;
volatile bool interruptFlagB = false;
volatile bool interruptFlagErr = false;
bool errorActive = false;

volatile bool debug = false; // Varialbe to toggle debug Serial.prints

void IRAM_ATTR handleInterruptA() {
  interruptFlagA = true;     // Set a flag to handle in the main loop
}

void IRAM_ATTR handleInterruptB() {
  interruptFlagB = true;     // Set a flag to handle in the main loop
}

void IRAM_ATTR handleInterruptErr() {
  interruptFlagErr = true;   // Set a flag to handle in the main loop
}


void setup() {
  Serial.begin(115200);
  Wire.begin(SDA, SCL);

  //Configure Shift Register pins
  pinMode(SR_SHnLD, OUTPUT);
  pinMode(SR_CLK, OUTPUT);
  pinMode(SR_DATA, INPUT);
  // Set initial states
  digitalWrite(SR_SHnLD, HIGH);
  digitalWrite(SR_CLK, LOW);

  // Load program card data into program array
  readProgramCard(programCode);
  // Fill half matrix to full
  completeProgramMatrix(programCode);
  if (debug) {
    for (int i = 0; i < 12; i++) {
      programCode[i] = storedProgram[i];
    }
    numberOfPhases = 6;
    Serial.println("Program loaded from EEPROM");
  }
  printProgram(programCode);

  pinMode(EXP_OUT_RESET, OUTPUT);
  digitalWrite(EXP_OUT_RESET, HIGH);
  configureMCP23017ForInterrupt();
  configureMCP23017AsOutput(EXP_OUT_0x21);
  configureMCP23017AsOutput(EXP_OUT_0x22);

  // Configure ESP32 pins for interrupt handling
  pinMode(INTA, INPUT);
  pinMode(INTB, INPUT);
  pinMode(ERR, INPUT);

  pinMode(WARN, OUTPUT);
  digitalWrite(WARN, HIGH);

  attachInterrupt(digitalPinToInterrupt(INTA), handleInterruptA, RISING);
  attachInterrupt(digitalPinToInterrupt(INTB), handleInterruptB, RISING);
  attachInterrupt(digitalPinToInterrupt(ERR), handleInterruptErr, RISING);

  // Serial.println("MCP23017 configured for interrupts with active-high pins.");
  Serial.println("Version 0.15");

  setFirstPhase();
}


void loop() {
  if (interruptFlagErr) {
    Serial.println("Error interrupt detected");
    interruptFlagErr = false;
    handleErrorInterrupt();
  }

  if (interruptFlagA) {
    interruptFlagA = false;
    handleInterrupt("A");
  }

  if (interruptFlagB) {
    interruptFlagB = false;
    handleInterrupt("B");
  }

  if (changePhase) {
    phaseChange(switchToRed, switchToGreen);
  }

  if (!changeInProgress) {
    phaseHandler();
  }
  delay(10);
}

//DONE
void readProgramCard(uint16_t results[11]) {
  digitalWrite(SR_SHnLD, LOW);   // Activate parallel load
  delayMicroseconds(2);          // Allow time for loading
  digitalWrite(SR_SHnLD, HIGH);  // Disable parallel load
  // Read 11 variables
  for (int i = 0; i < 11; i++) {
    results[i] = 0;              // Initialize the current program line
    // Set the bit for 1-1, 2-2...
    results[i] <<= 1; 
    results[i] |= 1;
    // Read bits for the current variable
    for (int j = 0; j < bitWidth; j++) {
      results[i] <<= 1;          // Shift left to make room for the new bit
      if (digitalRead(SR_DATA)) {
        results[i] |= 1;         // Set the least significant bit if DATA is HIGH
      }
      // Generate a clock pulse to shift data
      digitalWrite(SR_CLK, HIGH);
      delayMicroseconds(6);
      digitalWrite(SR_CLK, LOW);
      delayMicroseconds(3);
    }
    bitWidth--;
  }
}

//DONE
void completeProgramMatrix(uint16_t programCode[]) { // 
  for (int j = 4; j < 12; j++) {                           // Iterate over bits from 4 to 11
    uint16_t workRegister = 0;
    for (uint8_t i = 0; i < 8; i++) {                      // Iterate over the first 8 elements
      workRegister <<= 1;
      workRegister |= bitRead(programCode[i], j);          // Add the extracted bit to workRegister
    }
    workRegister = workRegister << 4;                      // Shifting workRegister to allign it
    programCode[11-j] = programCode[11-j] | workRegister;  // Merge horizontal and vertical lines
  }
  numberOfPhases = (programCode[8] & 0x07) + 1;            // Get the number of phases
}

//DONE
void printProgram(uint16_t programCode[]) {
  for (int i = 0; i < 12; i++) {  // Iterate over array
    Serial.println(programCode[i], BIN);
  }
}

//DONE
void handleInterrupt(const String&portName) { // Stores millis() in sensorTimers[] until
    uint8_t intfReg = 0x00;
    uint8_t intcapReg = 0x00;
    uint8_t offset = 0;
  if (portName == "A") {   // Setup for PortA
    intfReg = INTFA;       // Assign interrup flag registry address
    intcapReg = INTCAPA;   // Assign interrup capture registry address
  }
  else {                   // Setup for PortB
    intfReg = INTFB;       // Assign interrup flag registry address
    intcapReg = INTCAPB;   // Assign interrup capture registry address
    offset = 8;            // Offset for merging in activeSensors register
  }
  uint16_t intf = readRegister(EXP_IN_0x23, intfReg);    // Read interrupt flag register
  uint8_t intcap = readRegister(EXP_IN_0x23, intcapReg); // Read interrupt capture register
  intf = intf << offset;                                 // Offests the register value by 0 for A and by 8 for B
  activeSensors = activeSensors ^ intf;                  // Flips the bits where interrupt was triggered
  Serial.println(activeSensors, BIN);
  for (int i = 0; i < 16; i++) {                         // Set and reset sensorTimer[i]
    if (bitRead(intf, i)) {
      sensorTimers[i] = millis();
    }
    if (!bitRead(activeSensors, i)) {
      sensorTimers[i] = 0;
    }
  }

  if (debug) { // Print interrupt messages
    for (uint8_t i = 0; i < 16; i++) { // Print millis() stored in sensorTimers[i]
      Serial.print("Sens bit: ");
      Serial.print(i);
      Serial.print(" - ");
      Serial.println(sensorTimers[i]);
    }
    Serial.print("Interrupt on register ");
    Serial.println(portName);
    Serial.print("INTCAP: ");
    Serial.println(intcap, BIN);
    Serial.print("INTF: ");
    Serial.println(intf, BIN);
  }
}

//DONE
void handleErrorInterrupt() { // Set all Lines to yellow blink by driving WARN LOW
  writeRegister(EXP_OUT_0x21, IODIRA, 0xFF);
  writeRegister(EXP_OUT_0x21, IODIRB, 0xFF);
  writeRegister(EXP_OUT_0x22, IODIRA, 0xFF);
  writeRegister(EXP_OUT_0x22, IODIRB, 0xFF);
  digitalWrite(WARN, LOW);
  errorActive = true;
  if (debug) {Serial.println("Error Interrupt");};
}

//DONE
uint8_t sensorTimersCheck() { // Compare the sensorTimers + sensorLimits with the current time, returns Index, 8 if no sensor is tripped (priority for Lines with 2 sensors over the limit)
  uint8_t result = 0;
  uint16_t temp = 0;
  uint8_t group = 0;
  for (uint8_t i = 0; i < 16; i++) { // Read and check all active sensor timers against limit
    if (bitRead(activeSensors, i)) {
      if (millis() > (sensorTimers[i] + sensorLimits[i])) {
        bitSet(temp, i);
      }
    }
  }
  temp = remapSensors(temp);  // Remove non existent sensors
  if (temp != 0) { // Execute only if any sensorTimer has been tripped
    for (uint8_t i = 0; i < 8; i++) {  // Modify uint16_t into uint8_t with priority to return triggered Lines
      group = (temp >> (i * 2)) & 0b11;
      if (group == 0b11) { // If group is 11, set corresponding output bit to 1
        bitSet(result, i);
      } 
    }
    if (result == 0) { // Execute only if no sensorTimer group has been tripped
      for (uint8_t i = 0; i < 8; i++) {  // Modify uint16_t into uint8_t with priority to return triggered Lines
        group = (temp >> (i * 2)) & 0b11;
        if (group == 0b01 || group == 0b10) { // If group is 01 or 10, set corresponding output bit to 1
          bitSet(result, i);
        } 
      }
    }
    for (uint8_t i = 7; i >= 0; i--) { // Leaves only the most priority lane in the register to avoid conflicts (higher bit is higher priority)
      if ((i != activePhaseIndex) & bitRead(result, i)) { // Ignores bit if that is equals the activePhaseIndex
        result = i;
      }
    }
  }
  else {
    result = 8;
  }
  return result;
}

//DONE
uint16_t remapSensors(uint16_t sensors) { // Remap sensor signals for missing inputs on Port A and B bit7
  uint16_t remappedSensors = 0x7F & sensors;
  sensors >> 1;
  remappedSensors = remappedSensors ^ (0xFF80 & sensors);
  return remappedSensors;
}

//DONE
bool phaseTimeout(uint8_t index) { // Check if current phase has reached timeout limit
  bool timeout = false;
  if (millis() > (phaseTimer + phaseLimitsMax[index])) {
    timeout = true;
  }
  return timeout;
}

//DONE
bool phaseMinimumReached(uint8_t index) { // Check if current phase has reached its miminum active time
  bool minimumReached = false;
  if (millis() > (phaseTimer + phaseLimitsMin[index])) {
    minimumReached = true;
  }
  return minimumReached;
}

//#CHECK
void phaseHandler() { // Handle the switching between phases based on sensordata or timeout currentPhase - phaseA, nextPhase - phaseB
  changePhase = false;
  int8_t offset = 0;
  uint8_t phaseB = 0xFF;

  // DONE Sensor based trigger (with minimum timer check)
  if (phaseMinimumReached(activePhaseIndex)) { // Only allow sensor based phase switching once phaseMinimum is elapsed
    phaseB = sensorTimersCheck();
    if (phaseB < 8) { // Value higher than 7 is invalid, no sensors has been tripped
      changePhase = true;
      if (debug) {
        Serial.print("phaseHandler : Sensors triggered switch to phase ");
        Serial.println(phaseB);
      }
    }
  }

  // DONE Timer based trigger
  if (!changePhase & phaseTimeout(activePhaseIndex)) { // Increment activePhaseIndex until +1 would be greater than numberOfPhases, reset to 0
    if (activePhaseIndex < numberOfPhases) { // phaseB is next phase in order (based on number of phases, last is looping back to first);
      phaseB = activePhaseIndex + 1;
    }
    else {
      phaseB = 0;
    }
    if (debug) {
        Serial.print("phaseHandler : Timers triggered switch to phase ");
        Serial.println(phaseB);
        Serial.print("             : activePhaseindex ");
        Serial.print(activePhaseIndex);        
        Serial.print(" : numberOfPhases ");
        Serial.println(numberOfPhases);
    }
    changePhase = true;
  }

  if (changePhase) {
    if (activePhaseIndex/2 == phaseB/2) { // Same phase group, no need to switch
      if (debug) {Serial.println("phaseHandler : No phase change needed");}
      changePhase = false;
    }
    else if (bitRead(phaseB, 0)) {  // If LSL is 1 we need to generate next activePhases (AND operation) from LSL - 0, else the opposite, with its phase partner
      offset = -1;
      if (debug) {Serial.println("phaseHandler : offset = -1");}
    }
    else {
      offset = 1;
      if (phaseB + offset > numberOfPhases) {
        offset = 0;
      }
      if (debug) {Serial.println("phaseHandler : offset = 1");}
    }
    activePhaseIndex = phaseB;
    if (changePhase) {  // Check to skip phase generation when staying in the same 2 bit group
      uint16_t activePhasesB = programCode[phaseB] & programCode [phaseB + offset];
      uint16_t mask = activePhases & activePhasesB;
      switchToRed = mask ^ activePhases;
      switchToGreen = mask ^ activePhasesB;
      if (activePhaseIndex < 4 && (activePhasesB & 0x00F0) != 0) {
        switchToRed = switchToRed | (activePhases & 0x000F); // Add active pedestrian phases to switchToRed
      }
      else if (activePhaseIndex >= 4 && (activePhasesB & 0x0F00) != 0){
        switchToRed = switchToRed | (activePhases & 0x000F); // Add active pedestrian phases to switchToRed
      }
      activePhases = activePhasesB; // Store activating phases
      if (debug) {
        Serial.print("phaseHandler : New active phases ");
        Serial.println(activePhases, BIN);
      }
    }
  }
}

//#DONE
void phaseChange(uint16_t turnToRed, uint16_t turnToGreen) {  // Set lines to red/green
  if (millis() - lastChangeTime < changeDelay) {              // Ensures that the change is timed, get out of the call unless time is elapsed
    return;
  }
  if (debug) {
      Serial.print("changePhase called with : switchToRed   0b");
      Serial.println(switchToRed, BIN);
      Serial.print("                        : switchToGreen 0b");
      Serial.println(switchToGreen, BIN);
  }
  uint8_t byteIndex = 0;
  changeInProgress = true;
  switch (lampState) {          // use this to decide what bit to set on the lamp?
          case 0:               // Switch all required phases from GREEN, to YELLOW
              for (uint8_t i = 11; i > 4; i--) {
                if (bitRead(turnToRed, i)) {
                  if (debug) {
                    Serial.print("case 0 : bitRead 11-4 : turnToRed ");
                    Serial.println(i);
                  }
                  // turn off green lights with setBit()
                  setBit(false, true, byteIndex, 4);
                  if (lampLocLine[byteIndex][5] != 0x00) { // turn off warning lights with setBit() if any
                    setBit(false, true, byteIndex, 5);
                    if (debug) {
                      Serial.print("case 0 : bitRead 11-4 : turn off warning ");
                      Serial.println(i);
                    }
                  }
                  // turn on yellow lights with setBit()
                  setBit(true, true, byteIndex, 3);
                }
                byteIndex++;
              }
              byteIndex = 0;
              for (int8_t i = 3; i > -1; i--) { // Handle pedestrian phase
                if (bitRead(turnToRed, i)) {
                  // turn off pedestrian green lights with setBit()
                  setBit(false, false, byteIndex, 4);
                  // turn on pedestrian red lights with setBit()
                  setBit(true, false, byteIndex, 2);
                }
                byteIndex++;
              }
              byteIndex = 0;
              lampState++;
              lastChangeTime = millis();
              changeDelay = 4000; // 4 second delay from yellow to red
              if (debug) {Serial.println("phaseChange : case 0 #DONE");}
              break;
          case 1:               // Switch all required phases from YELLOW to RED
              for (uint8_t i = 11; i > 4; i--) {
                if (bitRead(turnToRed, i)) {
                  if (debug) {
                    Serial.print("case 1 : bitRead 11-4 : turnToRed ");
                    Serial.println(i);
                  }
                  // turn off yellow lights with setBit()
                  setBit(false, true, byteIndex, 3);
                  // turn on red lights with setBit()
                  setBit(true, true, byteIndex, 2);
                }
                byteIndex++;
              }
              byteIndex = 0;
              lampState++;
              lastChangeTime = millis();
              changeDelay = 2000; // 2 second delay from red to next phase
              if (debug) {Serial.println("phaseChange : case 1 #DONE");}
              break;
          case 2:               // Switch all required phases to RED AND YELLOW
              for (uint8_t i = 11; i > 4; i--) {
                if (bitRead(turnToGreen, i)) {
                  if (debug) {
                    Serial.print("case 2 : bitRead 11-4 : turnToGreen ");
                    Serial.println(i);
                  }
                  // turn on yellow lights with setBit()
                  setBit(true, true, byteIndex, 3);
                }
                byteIndex++;
              }
              byteIndex = 0;
              for (int8_t i = 3; i > -1; i--) { // Handle pedestrian phase
                if (bitRead(turnToGreen, i)) {
                  // turn off pedestrian red lights with setBit()
                  setBit(false, false, byteIndex, 2);
                  // turn on pedestrian green lights with setBit()
                  setBit(true, false, byteIndex, 4);
                }
                byteIndex++;
              }
              byteIndex = 0;
              lampState++;
              lastChangeTime = millis();
              changeDelay = 2000; // 2 second delay from red and yellow to green
              if (debug) {Serial.println("phaseChange : case 2 #DONE");}
              break;
          case 3:               // Switch all required phases to GREEN
              for (uint8_t i = 11; i > 4; i--) {
                if (bitRead(turnToGreen, i)) {
                  if (debug) {
                    Serial.print("case 3 : bitRead 11-4 : turnToGreen ");
                    Serial.println(i);
                  }
                  // turn off red lights with setBit()
                  setBit(false, true, byteIndex, 2);
                  // turn off yellow lights with setBit()
                  setBit(false, true, byteIndex, 3);
                  // turn on green lights with setBit()
                  setBit(true, true, byteIndex, 4);
                  if (lampLocLine[byteIndex][5] != 0x00) { // turn on warning lights with setBit() if any
                    setBit(true, true, byteIndex, 5);
                    if (debug) {
                      Serial.print("case 3 : bitRead 11-4 : turn off warning ");
                      Serial.println(i);
                    }
                  }
                }
                byteIndex++;
              }
              byteIndex = 0;
              sensorTimers[activePhaseIndex] = millis();  // Reset sensor timer for active phase
              phaseTimer = millis();                      // Update phase timer
              changePhase = false;
              changeInProgress = false;
              lampState = 0;
              changeDelay = 0;
              if (debug) {Serial.println("phaseChange : case 3 #DONE");}
              break;
  }
}

//DONE
void setFirstPhase() { // Set all red then switch to Phase1

  for (uint8_t i = 0; i < 8; i++) {                // Set all lampLocLine reds to active
    setBit(true, true, i, 2);
  }
  for (uint8_t i = 0; i < 4; i++) {                // Set all lampLocPed reds to active
    setBit(true, false, i, 2);
  }
  if (digitalRead(ERR)) {interruptFlagErr = true;}
  if (debug) {
    Serial.println("setFirstPhase : initial RED phases are SET");
  }
  switchToGreen = programCode[0] & programCode[1]; // Generate first phase
  activePhases = switchToGreen;                    // Store active phases for phaseHandler()
  if (debug) {
    Serial.print("setFirstPhase : switchToGreen ");
    Serial.println(switchToGreen, BIN);
  }
  changeDelay = 5000;                              // Set changeDelay to 5 seconds to delay first phase setup
  changePhase = true;                              // Set changePhase to true to trigger phaseChange()
  changeInProgress = true;                         // Set changeInProgress to true to block phaseHandler() from interrupting
  lampState = 2;                                   // Set starting point in phaseChange() switch case
}

//DONE
void setBit(bool set, bool lamp, uint8_t index, uint8_t element) { // 1 - set / 0 - reset; 1 - lampLocLine[] / 0 - lampLocPed[]; index in lampLocX; element in indexed array
  uint8_t currentOutput = 0;
  uint8_t bitLocation = 0;
  if (lamp) {
    currentOutput = readRegister(lampLocLine[index][0], lampLocLine[index][1]);
    bitLocation = log2(lampLocLine[index][element]);
  }
  else {
    currentOutput = readRegister(lampLocPed[index][0], lampLocPed[index][(element - 1)]);
    bitLocation = log2(lampLocPed[index][element]);
  }
  if (set) {
    bitSet(currentOutput, bitLocation);
  }
  else {
    bitClear(currentOutput, bitLocation);
  }
  if (lamp) {
    writeRegister(lampLocLine[index][0], lampLocLine[index][1], currentOutput);
  }
  else {
    writeRegister(lampLocPed[index][0], lampLocPed[index][(element - 1)], currentOutput);
  }
}

//DONE
void configureMCP23017ForInterrupt() {    // Configure the input MCP23017 for interrupt detection
  // Enable interrupt-on-change for all pins
  writeRegister(EXP_IN_0x23, 0x04, 0xFF); // GPINTENA: Enable interrupts on Bank A
  writeRegister(EXP_IN_0x23, 0x05, 0xFF); // GPINTENB: Enable interrupts on Bank B
  // Reset previous interrupt flags by GPIO read
  readRegister(EXP_IN_0x23, GPIOA);
  readRegister(EXP_IN_0x23, GPIOB);
  // Configure IOCON register for active-low interrupt pins
  writeRegister(EXP_IN_0x23, 0x0A, 0x02); // IOCON: INT active-high, not open-drain
}

//DONE
void configureMCP23017AsOutput(uint8_t address) { // Configure the MCP23017 as an output
  digitalWrite(EXP_OUT_RESET, LOW);
  delay(10);
  digitalWrite(EXP_OUT_RESET, HIGH);
  writeRegister(address, IODIRA, 0x00);   // Configure IO direction as output on Bank A
  writeRegister(address, IODIRB, 0x00);   // Configure IO direction as output on Bank B
}

//DONE
void writeRegister(uint8_t address, uint8_t reg, uint8_t value) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

//DONE
uint8_t readRegister(uint8_t address, uint8_t reg) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom(address, 1);
  return Wire.available() ? Wire.read() : 0;
}