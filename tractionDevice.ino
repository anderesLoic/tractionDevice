#include <SPI.h>
#include "mcp2515_can.h"
#include <HX711_ADC.h>

// ===============================
// ======== Pin Definitions =======
// ===============================
const int SPI_CS_PIN = 9;                // Chip Select pin for CAN module
const int homingSwitchPin = 4;           // Homing switch connected to pin 4
const int maxLevelLimitSwitchPin = 3;    // Limit switch after the max level connected to pin 3
const int HX711_DOUT_PIN = 6;            // HX711 Data Out pin
const int HX711_SCK_PIN = 7;             // HX711 Clock pin
const int ledPin = 13;                   // Built-in LED for status indication

// ===============================
// ======== CAN Settings =========
// ===============================
mcp2515_can CAN_BUS(SPI_CS_PIN);        // Create a CAN object
#define CAN_SPEED CAN_1000KBPS          // CAN bus speed

// ===============================
// ======== CANopen Settings ======
// ===============================
#define MOTOR_NODE_ID 1                  // Changed NODE_ID to MOTOR_NODE_ID to avoid conflicts
#define RESPONSE_TIMEOUT 1000            // Timeout for SDO responses in milliseconds
#define POSITION_THRESHOLD 10            // Threshold for considering target position reached

// ===============================
// ======== State Machine =========
// ===============================
enum ArduinoState {
    STATE_HOMING,                // 0
    STATE_STANDBY,               // 1
    STATE_SETUP,                 // 2
    STATE_CUSTOM_POSITION,       // 3
    STATE_TENSIONING_MOVE,       // 4
    STATE_TENSIONING,            // 5
    STATE_TRACTION,              // 6
    STATE_STOP_MEASURE,          // 7
    STATE_RETURN_TO_CUSTOM,      // 8
    STATE_PAUSE,                 // 9
    STATE_ADJUSTING,              // 10
    STATE_ALERT //ALERT switch touched
};

ArduinoState currentState = STATE_STANDBY;
ArduinoState previousState = STATE_SETUP;



// ===============================
// ======== Command Handling =====
// ===============================
String command;
String param;

// ===============================
// ======== Debounce Settings =====
// ===============================
const unsigned long debounceDelay = 50; // milliseconds
unsigned long lastDebounceTime = 0;
bool lastLimitSwitchState = HIGH;

// ===============================
// ======== Balance Class =========
// ===============================
class Balance {
private:
    HX711_ADC LoadCell;
    float currentWeight;
    float patientWeight;

public:
    Balance(uint8_t doutPin, uint8_t sckPin)
        : LoadCell(doutPin, sckPin), currentWeight(0.0) {}

    void initialize() {
        LoadCell.begin();
        long stabilizingtime = 2000; // Stabilizing time after power-up
        LoadCell.start(stabilizingtime, false);
        //LoadCell.setCalFactor(22295.50); // Set the calibration factor     previous
        //LoadCell.setCalFactor(21742.67); // Set the calibration factor     previous
        LoadCell.setCalFactor(17792.45);
        Serial.println("LoadCell initialized");
    }

    void measureWeight() {
        if (LoadCell.update()) {  // 'update()' returns true when new data is available
            currentWeight = LoadCell.getData() -468.52 + 1.13; //- 371.21-1.80 -9.00;
        }
    }

    double getWeight() {
        return currentWeight;
    }

    void simulateWeight(double weight) {
        currentWeight = weight;
    }

};

// ===============================
// ======== Motor Class ===========
/*
 * Motor class handles all motor-related operations including homing,
 * moving to positions, and continuous movement. It communicates with
 * the motor controller via CANopen SDOs.
 */
class Motor {
public:
    // State machine for motor operations
    enum State {IDLE, HOMING, MOVING_TO_POSITION, VELOCITY_MOVING, STOPPING};
    State currentState;

    // Constructor
    Motor(uint8_t nodeID, mcp2515_can& canBus, int homingSwitchPin);

    // Initialization
    void init();

    // Update motor state (non-blocking)
    void update();

    // Motor control methods
    void startHoming();
    void moveToPosition(int32_t position);
    void startVelocityMove(int32_t speed);
    void stopMotor();
    void haltMotor();
    int32_t getCurrentPosition();

    // Configure motor parameters
    void configureMotorParameters();

    // Set and get home position
    void setHomePosition(int32_t position);
    int32_t getHomePosition() const;

    // Method to get absolute position
    int32_t getAbsolutePosition();

    // Public flag to indicate if a velocity move is active
    bool isVelocityMoveFlag;

private:
    // Member variables
    uint8_t nodeID;
    mcp2515_can& CAN;
    int32_t lastPosition;
    int32_t homePosition;
    bool motorEnabled;
    int32_t maxRPM;
    int32_t stepSize;
    int32_t maxPosition;
    int32_t operatingSpeed;
    int homingSwitchPin;

    // Home offset
    int32_t homeOffset = 0;

    // Helper functions
    bool enableMotor();
    bool setMaxSpeedPositionMode(int32_t maxRPM);
    bool setPosition(int32_t position);
    bool setMode(uint8_t mode);
    bool setControlWord(uint16_t controlWord);
    bool readStatusWord(uint16_t& statusWord);
    bool readPosition(int32_t& position);
    void setHomePositionInternal();

    // SDO communication
    bool SDOwrite(uint16_t index, uint8_t subindex, int32_t data, uint8_t dataLength);
    bool SDOread(uint16_t index, uint8_t subindex, int32_t* data, uint8_t* dataLength);

    // Timing variables for non-blocking operations
    uint32_t lastUpdateTime;
};

// Implement the Motor constructor
Motor::Motor(uint8_t nodeID, mcp2515_can& canBus, int homingSwitchPin)
    : nodeID(nodeID), CAN(canBus), homingSwitchPin(homingSwitchPin) {
    homePosition = 0;
    lastPosition = 0;
    motorEnabled = false;
    currentState = IDLE;
    maxRPM = 100;          // Max RPM for position mode remains at 100 RPM
    stepSize = 1000;       // Step Size
    operatingSpeed = 100;  // Operating Speed for Continuous Movement
    lastUpdateTime = millis();
    isVelocityMoveFlag = false;
}

// Configure motor parameters as per user specifications
void Motor::configureMotorParameters() {
    // Set Max Speed to 100
    if (SDOwrite(0x6081, 0x00, 100, 4)) {
        Serial.println("Max Speed set to 100.");
    }

      // Set Profile Velocity (0x6081:00) to 500
  if (SDOwrite(0x6081, 0x00, 500, 4)) {
      Serial.println("Profile Velocity set to 500.");
  }

  // Set Profile Acceleration (0x6083:00) to 50
  if (SDOwrite(0x6083, 0x00, 50, 4)) {
      Serial.println("Profile Acceleration set to 50.");
  }


    // Set Max Acceleration (0x60C5:00) to 5000
    if (SDOwrite(0x60C5, 0x00, 5000, 4)) {
        Serial.println("Max Acceleration set to 5000.");
    }

    // Set Profile Deceleration (0x6084:00) to 500
    if (SDOwrite(0x6084, 0x00, 500, 4)) {
        Serial.println("Profile Deceleration set to 500.");
    }

    // Set Max Deceleration (0x60C6:00) to 5000
    if (SDOwrite(0x60C6, 0x00, 5000, 4)) {
        Serial.println("Max Deceleration set to 5000.");
    }

    // Set Velocity Window (0x60C7:00) to 30
    if (SDOwrite(0x60C7, 0x00, 30, 4)) {
        Serial.println("Velocity Window set to 30.");
    }
}

// Initialize motor settings
void Motor::init() {
    pinMode(homingSwitchPin, INPUT_PULLUP);
    setMaxSpeedPositionMode(maxRPM);

    // Configure additional motor parameters
    configureMotorParameters();
}

// Update motor state based on current operations
void Motor::update() {
    switch (currentState) {
        case IDLE:
            // Do nothing
            break;

        case HOMING:
            // Non-blocking homing
            if (digitalRead(homingSwitchPin) == LOW) { // Active LOW
                haltMotor();
                setHomePositionInternal();
                Serial.println(F("Homing completed."));
                currentState = IDLE; // Transition to IDLE after homing
                isVelocityMoveFlag = false;
            }
            break;

        case MOVING_TO_POSITION: {
            int32_t currentPosition;
            if (readPosition(currentPosition)) {
                if (abs(currentPosition - lastPosition) < POSITION_THRESHOLD) {
                    currentState = IDLE;
                    Serial.println(F("Position reached."));
                    isVelocityMoveFlag = false;
                }
            }
            break;
        }

        case VELOCITY_MOVING:
            // Update position periodically
            if (millis() - lastUpdateTime >= 100) { // Update every 100ms
                int32_t currentPosition;
                if (readPosition(currentPosition)) {
                    lastPosition = currentPosition;
                }
                lastUpdateTime = millis();
            }
            break;

        case STOPPING:
            haltMotor();
            currentState = IDLE;
            Serial.println(F("Motor stopped."));
            isVelocityMoveFlag = false;
            break;
    }
}

// Start the homing procedure
void Motor::startHoming() {
    if (currentState != HOMING) {
        Serial.println(F("Starting homing procedure..."));
        currentState = HOMING;
        setMode(3); // Profile Velocity Mode
        enableMotor();

        int32_t homingSpeed = -100; // Negative speed towards homing switch (set to 100 RPM)
        Serial.println(F("Homing speed is "));
        Serial.println(F(homingSpeed));
        SDOwrite(0x60FF, 0x00, homingSpeed, 4);
        setControlWord(0x0F); // Enable operation
        isVelocityMoveFlag = true;
    } else {
        Serial.println(F("Already homing."));
    }
}

// Move motor to a specific position (absolute position)
void Motor::moveToPosition(int32_t position) {
    Serial.print(F("Moving to position: "));
    Serial.println(position);

    lastPosition = position; // Store the absolute target position
    setMode(1); // Profile Position Mode
    enableMotor();
    setMaxSpeedPositionMode(maxRPM);
    setPosition(position);
    setControlWord(0x1F); // Start movement
    currentState = MOVING_TO_POSITION;
    isVelocityMoveFlag = true;
}

// Start continuous velocity movement
void Motor::startVelocityMove(int32_t speed) {
    Serial.print(F("Starting velocity move at speed: "));
    Serial.println(speed);

    setMode(3); // Profile Velocity Mode
    enableMotor();
    SDOwrite(0x60FF, 0x00, speed, 4);
    setControlWord(0x0F); // Enable operation
    currentState = VELOCITY_MOVING;
    isVelocityMoveFlag = true;
}

// Stop the motor by transitioning to STOPPING state
void Motor::stopMotor() {
    Serial.println(F("Stopping motor."));
    currentState = STOPPING;
}

// Immediately halt the motor
void Motor::haltMotor() {
    Serial.println(F("Halting motor..."));
    setControlWord(0x010F); // Halt command
    isVelocityMoveFlag = false;
}

// Retrieve the current motor position (absolute position)
int32_t Motor::getCurrentPosition() {
    int32_t position = 0;
    if (readPosition(position)) {
        return position;
    }
    return 0;
}

// Set the home position based on current motor position
void Motor::setHomePosition(int32_t position) {
    homePosition = position;
    homeOffset = homePosition; // Set homeOffset to current position
    Serial.print(F("Home position set to: "));
    Serial.println(homePosition);
}

// Get the home position
int32_t Motor::getHomePosition() const {
    return homeOffset;
}

// Get the absolute motor position
int32_t Motor::getAbsolutePosition() {
    return getCurrentPosition();
}

// Enable the motor by transitioning through states
bool Motor::enableMotor() {
    if (!SDOwrite(0x6040, 0x00, 0x06, 2)) {
        Serial.println(F("Failed to request state 'Ready to switch on'"));
        return false;
    }

    delay(10); // Small delay for state transition

    if (!SDOwrite(0x6040, 0x00, 0x07, 2)) {
        Serial.println(F("Failed to request state 'Switched on'"));
        return false;
    }

    delay(10);

    if (!SDOwrite(0x6040, 0x00, 0x0F, 2)) {
        Serial.println(F("Failed to request state 'Operation enabled'"));
        return false;
    }

    motorEnabled = true;
    return true;
}

// Set the maximum speed in position mode
bool Motor::setMaxSpeedPositionMode(int32_t maxRPM) {
    if (!SDOwrite(0x6081, 0x00, maxRPM, 4)) {
        Serial.println(F("Failed to set maximum speed in position mode."));
        return false;
    } else {
        Serial.print(F("Max speed set to "));
        Serial.print(maxRPM);
        Serial.println(F(" RPM for position mode."));
        return true;
    }
}

// Set the target position
bool Motor::setPosition(int32_t position) {
    if (!SDOwrite(0x607A, 0x00, position, 4)) {
        Serial.println(F("Failed to set target position."));
        return false;
    } else {
        return true;
    }
}

// Set the operation mode
bool Motor::setMode(uint8_t mode) {
    if (!SDOwrite(0x6060, 0x00, mode, 1)) {
        Serial.println(F("Failed to set mode."));
        return false;
    } else {
        return true;
    }
}

// Set the control word to manage motor states
bool Motor::setControlWord(uint16_t controlWord) {
    if (!SDOwrite(0x6040, 0x00, controlWord, 2)) {
        Serial.println(F("Failed to set control word."));
        return false;
    } else {
        return true;
    }
}

// Read the status word from the motor
bool Motor::readStatusWord(uint16_t& statusWord) {
    int32_t data;
    uint8_t dataLength;
    if (SDOread(0x6041, 0x00, &data, &dataLength)) {
        statusWord = data & 0xFFFF;
        return true;
    } else {
        return false;
    }
}

// Read the current position from the motor
bool Motor::readPosition(int32_t& position) {
    int32_t data;
    uint8_t dataLength;
    if (SDOread(0x6064, 0x00, &data, &dataLength)) {
        position = data;
        return true;
    } else {
        return false;
    }
}

// Write data to the motor via SDO
bool Motor::SDOwrite(uint16_t index, uint8_t subindex, int32_t data, uint8_t dataLength) {
    uint8_t sdoData[8];
    uint8_t commandByte;

    switch (dataLength) {
        case 1: commandByte = 0x2F; break;
        case 2: commandByte = 0x2B; break;
        case 4: commandByte = 0x23; break;
        default: Serial.println(F("Unsupported data length")); return false;
    }

    sdoData[0] = commandByte;
    sdoData[1] = index & 0xFF;
    sdoData[2] = index >> 8;
    sdoData[3] = subindex;

    union { int32_t sint; uint8_t bytes[4]; } dataUnion;
    dataUnion.sint = data;
    for (uint8_t i = 0; i < 4; i++) {
        sdoData[4 + i] = (i < dataLength) ? dataUnion.bytes[i] : 0x00;
    }

    uint32_t canID = 0x600 + nodeID;
    if (CAN.sendMsgBuf(canID, 0, 8, sdoData) == CAN_OK) {
        uint32_t timeout = millis() + RESPONSE_TIMEOUT;
        while (millis() < timeout) {
            if (CAN.checkReceive() == CAN_MSGAVAIL) {
                unsigned char len;
                unsigned char rxBuf[8];
                CAN.readMsgBuf(&len, rxBuf);
                long unsigned int rxId = CAN.getCanId();
                if (rxId == (0x580 + nodeID)) {
                    if (rxBuf[0] == 0x60 && rxBuf[1] == sdoData[1] && rxBuf[2] == sdoData[2] && rxBuf[3] == sdoData[3]) {
                        return true;
                    }
                    if (rxBuf[0] == 0x80) {
                        uint32_t abortCode = ((uint32_t)rxBuf[7] << 24) | ((uint32_t)rxBuf[6] << 16) | ((uint32_t)rxBuf[5] << 8) | rxBuf[4];
                        Serial.print(F("SDO abort code: 0x")); Serial.println(abortCode, HEX);
                        return false;
                    }
                }
            }
        }
        Serial.println(F("SDO write timeout"));
        return false;
    } else {
        Serial.println(F("CAN send error"));
        return false;
    }
}

// Read data from the motor via SDO
bool Motor::SDOread(uint16_t index, uint8_t subindex, int32_t* data, uint8_t* dataLength) {
    uint8_t sdoData[8] = {0x40, (uint8_t)(index & 0xFF), (uint8_t)(index >> 8), subindex, 0, 0, 0, 0};
    uint32_t canID = 0x600 + nodeID;

    if (CAN.sendMsgBuf(canID, 0, 8, sdoData) == CAN_OK) {
        uint32_t timeout = millis() + RESPONSE_TIMEOUT;
        while (millis() < timeout) {
            if (CAN.checkReceive() == CAN_MSGAVAIL) {
                unsigned char len;
                unsigned char rxBuf[8];
                CAN.readMsgBuf(&len, rxBuf);
                long unsigned int rxId = CAN.getCanId();

                if (rxId == (0x580 + nodeID) && (rxBuf[0] & 0x43) == 0x43 &&
                    rxBuf[1] == sdoData[1] && rxBuf[2] == sdoData[2] && rxBuf[3] == sdoData[3]) {

                    *dataLength = 4 - ((rxBuf[0] >> 2) & 0x03);
                    union { int32_t sint; uint8_t bytes[4]; } dataUnion;
                    dataUnion.sint = 0;
                    for (uint8_t i = 0; i < *dataLength; i++) {
                        dataUnion.bytes[i] = rxBuf[4 + i];
                    }
                    *data = dataUnion.sint;
                    return true;
                }
            }
        }
        Serial.println(F("SDO read timeout"));
        return false;
    } else {
        Serial.println(F("CAN send error"));
        return false;
    }
}

// Internal method to set home position
void Motor::setHomePositionInternal() {
    int32_t currentPosition;
    if (readPosition(currentPosition)) {
        setHomePosition(currentPosition);
    } else {
        Serial.println(F("Failed to read position during homing."));
    }
}

// ===============================
// ======== ScissorLift Class =====
// ===============================

/*
 * ScissorLift class manages the scissor lift operations, translating relative positions
 * to absolute motor positions based on the home offset established during homing.
 */
class ScissorLift {
private:
    Motor& motor;            // Motor object to handle motor operations
    Balance balance;         // Balance object to handle weight measurements
    bool pullingToWeight = false;  // Flag to indicate if the motor is pulling until a target weight
    double targetWeight = 0.0;     // Target weight to reach
    double initialWeight = 0.0;    // The initial weight of the patient    
    bool initialWeightSet = false; // Flag to check if initial weight is set        
    float percentageDecrease = 0.0; // Percentage decrease to achieve
    bool isHoming = false;
    bool homingCompleted = false;

    bool currentMaxLimitSwitchState = false;
    bool currentMinLimitSwitchState = false;
    bool limitSwitchOverrideActive = false;

public:
    int32_t minPosition = 2000;    // Minimum position set to +2000
    int32_t maxPosition = 200000;   // Maximum position set to +10000
    int32_t operatingSpeed = 30;   // Set operating speed to 50 RPM for TENSIONING and START

    bool isVelocityMove = false;    // Now public

    ScissorLift(Motor& motorObj, uint8_t doutPin, uint8_t sckPin)
        : motor(motorObj), balance(doutPin, sckPin) {}


    bool getCurrentMaxLimitSwitchState()
    {
      return currentMaxLimitSwitchState;
    }

    bool getCurrentMinLimitSwitchState()
    {
      return currentMinLimitSwitchState;
    }


    void initialize() {
        balance.initialize();  // Initialize the balance (HX711)
        motor.init();
    }

    // Perform homing
    void performHoming() {
        if (!isHoming) {
            motor.startHoming();
            isHoming = true;
             homingCompleted = false;
            Serial.println(F("Homing initiated."));
        }
    }

    // Check if homing is completed
    bool isHomed() const {
       return homingCompleted;
    }

    // Send motor to target position
    void sendMotorTo(int32_t position) {
        if (!isHomed() || currentState == STATE_ALERT) {
            Serial.println(F("Error: Homing not completed. Cannot execute position command."));
            return;
        }

        // Calculate the absolute motor position
        int32_t absolutePosition = position + motor.getHomePosition();

        // Ensure absolutePosition is within motor limits
        if (absolutePosition < minPosition + motor.getHomePosition()) {
            absolutePosition = minPosition + motor.getHomePosition();
            Serial.println("Adjusted position to minimum limit.");
        }
        if (absolutePosition > maxPosition + motor.getHomePosition()) {
            absolutePosition = maxPosition + motor.getHomePosition();
            Serial.println("Adjusted position to maximum limit.");
        }

        motor.moveToPosition(absolutePosition);
        pullingToWeight = false;  // Disable weight pulling mode
        isVelocityMove = false;
    }

    // Check if motor has arrived at the target position using encoder feedback
    bool motorHasArrived() const {
        return motor.currentState == Motor::IDLE;  // Check if motor is in IDLE state
    }

    // Start pulling until a percentage decrease in weight is achieved
    void pullUntilPercentageDecrease(float percentage) {
        if (!isHomed() || currentState == STATE_ALERT) {
            Serial.println(F("Error: Homing not completed. Cannot execute TENSIONING command."));
            return;
        }

        percentageDecrease = percentage;

        // Set initial weight only if it hasn't been set
          if (initialWeightSet == false) {
            initialWeight = balance.getWeight();
            Serial.println(F("we are in initial weight state."));
            initialWeightSet = true;  // Mark initial weight as set
            Serial.println(F("initial weight state is true"));
           }
        //initialWeight = balance.getWeight();
        targetWeight = initialWeight * (1.0 - percentageDecrease / 100.0);
        Serial.print("target weight is set to ");
        Serial.print(targetWeight);
        pullingToWeight = true;
        isVelocityMove = true;
        motor.startVelocityMove(-operatingSpeed); // Start moving in negative direction at 50 RPM
        Serial.print("Pulling until weight decreases by ");
        Serial.print(percentage);
        Serial.println("%");
    }

   bool resetInitialWeight() {
    initialWeightSet = false;
    return false;  // Allows initial weight to be re-measured
    Serial.print("WeightSet ");
    Serial.print(initialWeightSet);
    }

    // Check if the target weight is reached and stop the motor if so
    bool hasReachedWeight() {
        if (pullingToWeight) {
          Serial.println("getWeight: ");

          Serial.println("TargetWeight");
          Serial.println(targetWeight);
            if (balance.getWeight() <= targetWeight) {
                Serial.println("we are checking if getweight<target weight");
                motor.stopMotor();  // Stop the motor when target weight is reached
                pullingToWeight = false;
                isVelocityMove = false;
                Serial.println("Target weight achieved. Motor stopped.");
                return true;
            }
        }
        return false;
    }

    double getWeight() const {
        return balance.getWeight();
    }

    int32_t getPos() const {
        if (!isHomed()) {
            return 0;
        }
        return motor.getAbsolutePosition() - motor.getHomePosition();
    }

    // Update motor position and check balance weight continuously
    void update() {
        balance.measureWeight();
        motor.update(); // Update motor state

        if (isHoming && motor.currentState == Motor::IDLE) {
            isHoming = false;
            homingCompleted = true; // Mark homing as complete
            Serial.println("Homing completed. System is ready for operations.");
            sendMotorTo(minPosition);
        }

        //ALERT SWITCH DETECTION
        // Read the current state of the max level limit switch
        currentMaxLimitSwitchState = !digitalRead(maxLevelLimitSwitchPin);
        // Read the current state of the min level (homing) limit switch
        currentMinLimitSwitchState = !digitalRead(homingSwitchPin);
        // If not in homing
        //if ((currentState != STATE_HOMING) && (currentMaxLimitSwitchState ==1 || currentMinLimitSwitchState == 1) && (currentState != STATE_ALERT) && (currentState != STATE_ADJUSTING && previousState != STATE_ALERT)) 
        if ((currentState != STATE_HOMING) && (currentMaxLimitSwitchState ==1 || currentMinLimitSwitchState == 1) && (currentState != STATE_ALERT)) 
        {
            stopMotor();
            previousState = currentState;
            currentState = STATE_ALERT;
        }

        if(currentMaxLimitSwitchState ==0 && currentMinLimitSwitchState == 0 && currentState == STATE_ALERT)
        {
            currentState = previousState;
            previousState = STATE_ALERT;
        }

        // Check if target weight is achieved
        if (pullingToWeight && hasReachedWeight()) {
            // Transition handled in hasReachedWeight()
        }

        // Update isVelocityMove based on motor state
        if (motor.currentState == Motor::VELOCITY_MOVING) {
            isVelocityMove = true;
        } else {
            isVelocityMove = false;
        }
    }

    // Stop the motor
    void stopMotor() {
        motor.stopMotor();
        pullingToWeight = false;
        isVelocityMove = false;
        Serial.println("Motor has been stopped manually.");
    }

    // Check if motor is close to boundaries and stop if necessary
    void checkBoundaries() {
        if (!isHomed()) return;

        int32_t position = getPos();
        if (isVelocityMove) {
            if (position < minPosition) {
                sendMotorTo(minPosition);
                Serial.println("Motor reached minimum position and stopped.");
            }
            if (position > maxPosition) {
                sendMotorTo(maxPosition);
                Serial.println("Motor reached maximum position and stopped.");
            }
        }
    }

    // Start continuous movement
    void startContinuousMove(int32_t speed) {        
        motor.startVelocityMove(speed);
        isVelocityMove = true;
    }
};

// ===============================
// ======== Instantiation =========
// ===============================
// Instantiate Motor and ScissorLift objects
Motor motorInstance(MOTOR_NODE_ID, CAN_BUS, homingSwitchPin);
ScissorLift liftInstance(motorInstance, HX711_DOUT_PIN, HX711_SCK_PIN);

// ===============================
// ========= Setup Function ======
// ===============================
void setup() {
    Serial.begin(115200);

    Serial.println(F("Initializing CAN Bus..."));
    while (CAN_BUS.begin(CAN_SPEED) != CAN_OK) {
        Serial.println(F("CAN init fail, retry..."));
        delay(100);
    }
    Serial.println(F("CAN init ok!"));

    // Initialize limit switch pin
    pinMode(maxLevelLimitSwitchPin, INPUT_PULLUP);

    // Initialize built-in LED
    pinMode(ledPin, OUTPUT);
    digitalWrite(ledPin, LOW); // LED off initially

    // Initialize scissor lift system
    liftInstance.initialize();

    // User Instructions
    Serial.println("Available Commands (after homing):");
    Serial.println("SETUP");
    Serial.println("TENSIONING:x");
    Serial.println("START:x");
    Serial.println("MIN");
    Serial.println("MAX");
    Serial.println("UP");
    Serial.println("DOWN");
    Serial.println("UP_STEP");
    Serial.println("DOWN_STEP");
    Serial.println("DONE"); // Replaced 'HIGH' with 'DONE'
    Serial.println("STOP");
    Serial.println("-----------------------------");
    Serial.println("Enter commands followed by Enter.");
}

// ===============================
// ========= Loop Function =======
// ===============================
void loop() {
    // ===============================
    // ==== Limit Switch Monitoring ==
    // ===============================
    bool currentLimitSwitchState = digitalRead(maxLevelLimitSwitchPin);

    // Debounce the limit switch
    if (currentLimitSwitchState != lastLimitSwitchState) {
        lastDebounceTime = millis();
    }

    if ((millis() - lastDebounceTime) > debounceDelay) {
        if (currentLimitSwitchState == LOW) { // Active LOW
            Serial.println("Max level limit switch triggered! Stopping motor.");
            liftInstance.stopMotor();
            currentState = STATE_STANDBY; // Transition to STANDBY state
        }
    }

    lastLimitSwitchState = currentLimitSwitchState;

    // ===============================
    // ====== Serial Command Input ===
    // ===============================
    if (Serial.available() > 0) {
        String input = Serial.readStringUntil('\n');  // Read command and parameter
        input.trim();         // Remove any leading/trailing whitespace
        int separator = input.indexOf(':');
        if (separator != -1) {
            command = input.substring(0, separator);
            param = input.substring(separator + 1);
        } else {
            command = input;
            param = "";
        }
        command.toUpperCase(); // Convert command to uppercase for consistency
    } else {
        command = "";
    }

    // ===============================
    // ======= Command Processing =====
    // ===============================
    if (command.length() > 0) { // Process only if there's a command
        if (command == "HOMING") {
            // Allow homing to be re-initiated at any time
            if(currentState != STATE_ALERT)
            {
              liftInstance.performHoming();
              currentState = STATE_HOMING;
              command = "";
            }
        }
        else if (command == "SETUP") {
            if (liftInstance.isHomed() && currentState != STATE_ALERT) {
                liftInstance.sendMotorTo(liftInstance.maxPosition);
                liftInstance.resetInitialWeight();
                currentState = STATE_SETUP;
            } else {
                Serial.println(F("Error: Homing not completed or switch is active. Cannot execute SETUP command."));
            }
            command = "";
        }
        else if (command.startsWith("TENSIONING")) {
            if (liftInstance.isHomed() && currentState != STATE_ALERT) {
                float percentage = param.toFloat();
                if (percentage > 0 && percentage < 100) {
                    liftInstance.pullUntilPercentageDecrease(percentage);
                    currentState = STATE_TENSIONING_MOVE;
                } else {
                    Serial.println("Invalid percentage for TENSIONING. Must be between 0 and 100.");
                }
            } else {
                Serial.println(F("Error: Homing not completed or switch is active. Cannot execute TENSIONING command."));
            }
            command = "";
        }
        else if (command.startsWith("START")) {
            if (liftInstance.isHomed() && currentState != STATE_ALERT) {
                float percentage = param.toFloat();
                Serial.println(F("Start is ok"));

                if (percentage > 0 && percentage < 100) {
                    liftInstance.pullUntilPercentageDecrease(percentage);
                    Serial.println(F("we have percentage"));
                    currentState = STATE_TRACTION;
                } else {
                    Serial.println("Invalid percentage for START. Must be between 0 and 100.");
                }
            } else {
                Serial.println(F("Error: Homing not completed or switch is active. Cannot execute START command."));
            }
            command = "";
        }
        else if (command == "MIN") {
            if (liftInstance.isHomed() && currentState != STATE_ALERT) {
                liftInstance.sendMotorTo(liftInstance.minPosition);
                currentState = STATE_SETUP;
            } else {
                Serial.println(F("Error: Homing not completed or switch is active. Cannot execute MIN command."));
            }
            command = "";
        }
        else if (command == "MID") {
            if (liftInstance.isHomed() && currentState != STATE_ALERT) {
                liftInstance.sendMotorTo((liftInstance.minPosition + liftInstance.maxPosition)/2);
                currentState = STATE_SETUP;
            } else {
                Serial.println(F("Error: Homing not completed or switch active. Cannot execute MIN command."));
            }
            command = "";
        }
        else if (command == "MAX") {
            if (liftInstance.isHomed() && currentState != STATE_ALERT) {
                liftInstance.sendMotorTo(liftInstance.maxPosition);
                currentState = STATE_SETUP;
            } else {
                Serial.println(F("Error: Homing not completed or switch active. Cannot execute MAX command."));
            }
            command = "";
        }
        else if (command == "UP") {
                if(liftInstance.getCurrentMaxLimitSwitchState() == 0)
                {
                  previousState = currentState;
                  currentState = STATE_ADJUSTING;
                  liftInstance.startContinuousMove(liftInstance.operatingSpeed); // Negative direction
                  command = "";
                }
        }
        else if (command == "DOWN") {
                if(liftInstance.getCurrentMinLimitSwitchState() == 0)
                {
                  previousState = currentState;
                  currentState = STATE_ADJUSTING;
                  liftInstance.startContinuousMove(-liftInstance.operatingSpeed); // Negative direction
                  command = "";
                }
        }
        else if (command == "DONE") { // 'DONE'
            if (currentState == STATE_TENSIONING || currentState == STATE_TRACTION) {
                liftInstance.sendMotorTo(liftInstance.maxPosition); // Return to setup position
                liftInstance.resetInitialWeight();  //InitialWeight could be any weight
                currentState = STATE_RETURN_TO_CUSTOM;
            } else {
                Serial.println("Error: DONE command only valid after TENSIONING or START.");
            }
            command = "";
        }
        else if (command == "STOP") {
            // Allow STOP command at any time
            liftInstance.stopMotor();
            // Check if currently in STATE_ADJUSTING to revert to previousState
            if (currentState == STATE_ADJUSTING) {
                currentState = previousState;
                Serial.println(F("Returned to previous state after STOP."));
            } else {
                currentState = STATE_STANDBY;
            }
            command = "";
        }
        else {
            Serial.println(F("Unknown Command."));
            command = "";
        }
    }

    // ===============================
    // ======= State Machine =========
    // ===============================
    switch (currentState) {
        case STATE_HOMING:
            // During homing, no additional actions required
            break;

        case STATE_STANDBY:
            // Do nothing or wait for commands
            break;

        case STATE_SETUP:
            // Wait for motor to arrive at position
            if (liftInstance.motorHasArrived()) {
                currentState = STATE_CUSTOM_POSITION;
                Serial.println("Setup completed. Awaiting further commands.");
            }
            break;

        case STATE_CUSTOM_POSITION:
            // Awaiting user commands for custom positioning
            break;

        case STATE_TENSIONING_MOVE:
            if (liftInstance.hasReachedWeight()) {
                currentState = STATE_TENSIONING;
                Serial.println("Tensioning completed. Awaiting further commands.");
            }
            break;

        case STATE_TENSIONING:
            // Awaiting further commands after tensioning
            break;

        case STATE_TRACTION:
            if (liftInstance.hasReachedWeight()) {
                currentState = STATE_STOP_MEASURE;
            }
            break;

        case STATE_STOP_MEASURE:
            // Awaiting DONE command to finalize measurement
            break;

        case STATE_RETURN_TO_CUSTOM:
            if (liftInstance.motorHasArrived()) {
                currentState = STATE_CUSTOM_POSITION;
                Serial.println("Returned to custom position.");
            }
            break;

        case STATE_ADJUSTING:
            // Monitor if the motor has finished the adjustment
            if (!liftInstance.isVelocityMove) {
                currentState = previousState;
            }
            break;

        case STATE_PAUSE:
            // Handle pause state if needed
            break;

        default:
            break;
    }

    // ===============================
    // ====== State Change Logging ===
    // ===============================
    static ArduinoState lastLoggedState = STATE_HOMING;
    if (currentState != lastLoggedState) {
        Serial.print("STATE:");
        Serial.println(currentState);
        lastLoggedState = currentState;
    }

    // ===============================
    // ====== System Updates =========
    // ===============================
    liftInstance.update();           // Update motor and balance status
    liftInstance.checkBoundaries();  // Check boundaries and stop if necessary

    // ===============================
    // ====== Serial Feedback ========
    // ===============================
    static unsigned long previousMillis = 0;
    unsigned long currentMillis = millis();
    const long interval = 1000; // 1000 milliseconds
    if (currentMillis - previousMillis >= interval) {
        previousMillis = currentMillis;

        Serial.print("W:");
        Serial.print(liftInstance.getWeight(), 2); // Send weight with 2 decimal places
        Serial.print(" "); // Space as delimiter

        Serial.print("M:");
        Serial.print(liftInstance.getPos()); // Send relative motor position
        Serial.println(); // End of line to separate messages



    }
}
