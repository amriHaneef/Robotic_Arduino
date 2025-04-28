#include <HX711_ADC.h>
#include <Keypad.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// Sensor setup
const int LOADCELL_DATA_PIN = A1;
const int LOADCELL_CLOCK_PIN = A2;
HX711_ADC weightSensor(LOADCELL_DATA_PIN, LOADCELL_CLOCK_PIN);
const float SENSOR_CALIBRATION = 21.71;

// Actuator configuration
const int MOTOR_PIN1 = 9;
const int MOTOR_PIN2 = 10;
const int MOTOR_PWM_PIN = 11;
const int MAX_MOTOR_SPEED = 255;
bool isMotorOn = false;
bool fineControlMode = false;

// User interface setup
const byte KEYPAD_ROWS = 4;
const byte KEYPAD_COLS = 3;
char keypadButtons[KEYPAD_ROWS][KEYPAD_COLS] = {
  {'1','2','3'},
  {'4','5','6'},
  {'7','8','9'},
  {'*','0','#'}
};
byte rowConnections[KEYPAD_ROWS] = {2, 3, 4, 5};
byte colConnections[KEYPAD_COLS] = {6, 7, 8};
Keypad userKeypad = Keypad(makeKeymap(keypadButtons), rowConnections, colConnections, KEYPAD_ROWS, KEYPAD_COLS);

// Display setup
LiquidCrystal_I2C displayScreen(0x27, 16, 2);

// Program states
enum SystemMode { WEIGHT_INPUT, ZEROING, DISPENSING, FINISHED };
SystemMode currentMode = WEIGHT_INPUT;

// Measurement variables
float desiredWeight = 0.0;
float measuredWeight = 0.0;
String userInput = "";

// Timing control
unsigned long previousUpdate = 0;
unsigned long motorStartTime = 0;
unsigned long completionTime = 0;
const int SCREEN_UPDATE_DELAY = 500;
const int SENSOR_READ_DELAY = 200;

void logMessage(String text) {
  Serial.print("[LOG] ");
  Serial.println(text);
}

void showCurrentMode() {
  Serial.print("Current mode: ");
  switch(currentMode) {
    case WEIGHT_INPUT: Serial.println("WEIGHT_INPUT"); break;
    case ZEROING: Serial.println("ZEROING"); break;
    case DISPENSING: Serial.println("DISPENSING"); break;
    case FINISHED: Serial.println("FINISHED"); break;
  }
}

void setup() {  // Restored original function name
  Serial.begin(9600);
  logMessage("Starting system setup");
  
  // Configure weight sensor
  weightSensor.begin();
  weightSensor.start(2000, true);
  if(weightSensor.getTareTimeoutFlag() || weightSensor.getSignalTimeoutFlag()) {
    logMessage("Error: Sensor initialization problem");
    showOnDisplay("Sensor Fault!");
    while(1);
  }
  weightSensor.setCalFactor(SENSOR_CALIBRATION);
  logMessage("Weight sensor ready");

  // Set up display
  displayScreen.init();
  displayScreen.backlight();
  showOnDisplay("Input Weight (g):");
  logMessage("Display activated");

  // Prepare motor control
  pinMode(MOTOR_PIN1, OUTPUT);
  pinMode(MOTOR_PIN2, OUTPUT);
  analogWrite(MOTOR_PWM_PIN, MAX_MOTOR_SPEED);
  turnMotorOff();
  logMessage("Motor system prepared");

  logMessage("Setup finished");
  showCurrentMode();
}

void loop() {  // Restored original function name
  switch(currentMode) {
    case WEIGHT_INPUT:
      processUserInput();
      break;
    case ZEROING:
      performTaring();
      break;
    case DISPENSING:
      controlDispensing();
      break;
    case FINISHED:
      showCompletion();
      break;
  }
}

void processUserInput() {
  char pressedKey = userKeypad.getKey();
  
  if (pressedKey) {
    logMessage("Pressed key: " + String(pressedKey));
    
    if (pressedKey == '#') {
      if (userInput.length() > 0) {
        desiredWeight = userInput.toFloat();
        logMessage("Target weight: " + String(desiredWeight) + " grams");
        if(desiredWeight > 0) {
          currentMode = ZEROING;
          showOnDisplay("Calibrating...");
          logMessage("Changing to ZEROING mode");
          showCurrentMode();
        }
      }
    }
    else if (pressedKey == '*') {
      userInput = "";
      showOnDisplay("Input Weight (g):");
      displayScreen.setCursor(0, 1);
      displayScreen.print("                ");
      logMessage("Cleared user input");
    }
    else if ((isdigit(pressedKey) || pressedKey == '.') && userInput.length() < 6) {
      userInput += pressedKey;
      refreshDisplay();
      logMessage("User entry: " + userInput);
    }
  }
}

void performTaring() {
  static bool isTaring = false;
  static unsigned long tareBeginTime = millis();
  
  if (!isTaring) {
    logMessage("Beginning taring process...");
    weightSensor.tare();
    isTaring = true;
    tareBeginTime = millis();
    return;
  }

  // Check for timeout
  if(millis() - tareBeginTime > 10000) {
    logMessage("ERROR: Taring took too long!");
    if(weightSensor.getTareTimeoutFlag()) logMessage("Tare timeout occurred");
    if(weightSensor.getSignalTimeoutFlag()) logMessage("Signal timeout occurred");
    restartSystem();
    return;
  }
  
  if(weightSensor.update()) {
    if (weightSensor.getTareStatus()) {
      logMessage("Taring successful");
      isTaring = false;
      currentMode = DISPENSING;
      turnMotorOn();
    } else {
      logMessage("Taring ongoing...");
    }
  } else {
    logMessage("Waiting for taring update...");
  }
}

void controlDispensing() {
  static unsigned long lastMeasurement = 0;
  static int motorPulseWidth = 100;

  // Get current weight
  if (millis() - lastMeasurement >= SENSOR_READ_DELAY) {
    if (weightSensor.update()) {
      measuredWeight = abs(weightSensor.getData());
      Serial.print("Measured: ");
      Serial.print(measuredWeight);
      Serial.print(" / Needed: ");
      Serial.println(desiredWeight);
    }
    lastMeasurement = millis();
  }

  // Refresh display
  if (millis() - previousUpdate >= SCREEN_UPDATE_DELAY) {
    displayScreen.setCursor(0, 1);
    displayScreen.print(measuredWeight, 2);
    displayScreen.print("g/");
    displayScreen.print(desiredWeight, 2);
    displayScreen.print("g   ");
    previousUpdate = millis();
  }

  if(desiredWeight <= 600) {
    fineControlMode = true;
  }

  if(measuredWeight >= desiredWeight - 200 && fineControlMode) {
    preciseDispensing(20);
  } else if(fineControlMode)  {
    preciseDispensing(80);
  }
  
  // Manage motor operation
  if (measuredWeight >= desiredWeight) {
    logMessage("Reached target weight");
    turnMotorOff();
    currentMode = FINISHED;
    showOnDisplay("Completed!");
    completionTime = millis();
    logMessage("Switching to FINISHED mode");
    showCurrentMode();
  }
  else if (measuredWeight > desiredWeight - 500 && !fineControlMode) {
    fineControlMode = true;
    logMessage("Starting fine control");
  }
}

void preciseDispensing(long onTime) {
  static unsigned long lastMotorChange = 0;
  static bool isMotorActive = false;
  const unsigned long offTime = 200;

  unsigned long now = millis();
  unsigned long timeSinceChange = now - lastMotorChange;

  if (isMotorActive) {
    if (timeSinceChange >= onTime) {
      digitalWrite(MOTOR_PIN1, LOW);
      isMotorActive = false;
      lastMotorChange = now;
      logMessage("Motor deactivated (fine control)");
    }
  } else {
    if (timeSinceChange >= offTime) {
      digitalWrite(MOTOR_PIN1, HIGH);
      digitalWrite(MOTOR_PIN2, LOW);
      isMotorActive = true;
      lastMotorChange = now;
      logMessage("Motor activated (fine control)");
    }
  }
}

void showCompletion() {
  if (millis() - completionTime >= 5000) {
    logMessage("Preparing for restart");
    restartSystem();
  }
}

void turnMotorOn() {
  if (!fineControlMode) {
    digitalWrite(MOTOR_PIN1, HIGH);
    digitalWrite(MOTOR_PIN2, LOW);
    logMessage("Motor activated (full power)");
  }
  isMotorOn = true;
}

void turnMotorOff() {
  digitalWrite(MOTOR_PIN1, LOW);
  digitalWrite(MOTOR_PIN2, LOW);
  isMotorOn = false;
  logMessage("Motor stopped");
}

void restartSystem() {
  desiredWeight = 0.0;
  measuredWeight = 0.0;
  userInput = "";
  fineControlMode = false;
  turnMotorOff();
  currentMode = WEIGHT_INPUT;
  showOnDisplay("Input Weight (g):");
  logMessage("System reset done");
  showCurrentMode();
}

void showOnDisplay(const char* message) {
  displayScreen.clear();
  displayScreen.setCursor(0, 0);
  displayScreen.print(message);
  logMessage("Display shows: " + String(message));
}

void refreshDisplay() {
  displayScreen.setCursor(0, 1);
  displayScreen.print("                ");
  displayScreen.setCursor(0, 1);
  displayScreen.print(userInput);
}