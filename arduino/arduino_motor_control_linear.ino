// Define pins for the stepper motor (used for steering)
const int directionPin = 2; // Pin for controlling the direction of the steering motor
const int stepPin = 3;      // Pin for controlling the steps of the steering motor

// Define pins for the linear actuator (used for acceleration control)
int RPWM = 5;               // Right PWM pin for controlling speed in reverse
int LPWM = 6;               // Left PWM pin for controlling speed in forward direction
int L_EN = 7;               // Left enable pin for the linear actuator
int R_EN = 8;               // Right enable pin for the linear actuator

// Constants for steering control
const int maxSteps = 75000; // Maximum steps for full turn (e.g., full right or full left)
int currentSteps = 0;       // Track current step count for steering

const int numReadings = 10;

int readings[numReadings];  // the readings from the analog input
int readIndex = 0;          // the index of the current reading
int total = 0;              // the running total
int average = 0;            // the average

void setup() {
    // Set stepper motor pins for steering as outputs
    pinMode(directionPin, OUTPUT);
    pinMode(stepPin, OUTPUT);
    pinMode(A1, INPUT);
    
    // Set linear actuator control pins as outputs
    for (int i = 5; i < 9; i++) {
        pinMode(i, OUTPUT);
        digitalWrite(i, LOW);  // Initialize all linear actuator pins to LOW (off)
    }
    
    
    Serial.begin(9600); // Begin serial communication at 9600 baud rate

    for (int thisReading = 0; thisReading < numReadings; thisReading++) {
        readings[thisReading] = 0;
    }
}

void loop() {
    // Check if there is serial data available
    delay(10);
    if (Serial.available()) {
        // Read the entire line as a string (format: "linearVel angularVel")
        String input = Serial.readStringUntil('\n');
        
        // Find the space in the input and split the string into two parts
        float linearVel;
        float angularVel;
        int spaceIndex = input.indexOf(' ');
        int feedback = smooth_feedback();
        Serial.println(String(feedback));
        
        if (spaceIndex != -1) {
            // Extract and convert the linear velocity
            linearVel = input.substring(0, spaceIndex).toFloat();
            
            // Extract and convert the angular velocity
            angularVel = input.substring(spaceIndex + 1).toFloat();

            //Serial.println("Linear Vel: " + String(linearVel) + ", Angular Vel: " + String(angularVel) + "\n");
            controlAcceleration(linearVel, feedback);
        }
    } else {
        // If no command, stop both motors
        digitalWrite(L_EN, LOW);
        digitalWrite(R_EN, LOW);
    }
}

// Function to smooth feedback
int smooth_feedback()
{
  // subtract the last reading:
  total = total - readings[readIndex];
  // read from the sensor:
  readings[readIndex] = analogRead(A1);
  // add the reading to the total:
  total = total + readings[readIndex];
  // advance to the next position in the array:
  readIndex = readIndex + 1;

  // if we're at the end of the array...
  if (readIndex >= numReadings) {
    // ...wrap around to the beginning:
    readIndex = 0;
  }

  // calculate the average:
  average = total / numReadings;
  return average;
}

// Added debug macros to print information
#define DEBUG

// Updated controlAcceleration function with verbose output
void controlAcceleration(float linearVel, int feedback) {
    const int deadband = 5;  // Tolerance range to ignore small fluctuations
    int mapping_value = 0;

    if (linearVel >= 0.0) {
        mapping_value = map(linearVel, 0.0, 5.0, 312, 600);  // Adjust for forward motion
    } else {
        mapping_value = map(linearVel, -5.0, 0.0, 120, 312);  // Adjust for reverse motion
    }

    #ifdef DEBUG
    Serial.print("Feedback: ");
    Serial.print(feedback);
    Serial.print(", Mapped Value: ");
    Serial.print(mapping_value);
    Serial.print(", Deadband: ");
    Serial.println(deadband);
    #endif

    if (abs(feedback - mapping_value) > deadband) {
        if (feedback > mapping_value) {
            #ifdef DEBUG
            Serial.println("Moving motor: Decreasing feedback");
            #endif
            digitalWrite(L_EN, HIGH);
            digitalWrite(R_EN, HIGH);
            analogWrite(RPWM, 128);  // Adjust speed as needed
            analogWrite(LPWM, 0);
        } else {
            #ifdef DEBUG
            Serial.println("Moving motor: Increasing feedback");
            #endif
            digitalWrite(L_EN, HIGH);
            digitalWrite(R_EN, HIGH);
            analogWrite(RPWM, 0);
            analogWrite(LPWM, 128);
        }
    } else {
        #ifdef DEBUG
        Serial.println("Within deadband: Stopping motor");
        #endif
        digitalWrite(L_EN, LOW);
        digitalWrite(R_EN, LOW);
    }
}
