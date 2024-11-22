const int directionPin = 2; // Pin for direction control
const int stepPin = 3;      // Pin for step control

int RPWM=5;
int LPWM=6;
int L_EN=7;
int R_EN=8;

const int maxSteps = 75000;   // Max steps for turning right
int currentSteps = 0;       // Track current steps taken

void setup() {
    pinMode(directionPin, OUTPUT);
    pinMode(stepPin, OUTPUT);
    for (int i = 5; i < 9; i++)
    {
      pinMode(i, OUTPUT);
      digitalWrite(i, LOW);
    }
    Serial.begin(9600);
}

void loop() {
    Serial.println(currentSteps);
    if (Serial.available()) {
        char command = Serial.read();
        // Serial.println(command);
        if (command == 'd') {
            turnRight();
        } else if (command == 'a') {
            turnLeft();
        } else if (command == 'w'){
          digitalWrite(L_EN, HIGH);
          digitalWrite(R_EN, HIGH);
          analogWrite(RPWM, 0);
          analogWrite(LPWM, 255);
          delay(35);
        } else if (command == 's'){
          digitalWrite(L_EN, HIGH);
          digitalWrite(R_EN, HIGH);
          analogWrite(RPWM, 255);
          analogWrite(LPWM, 0);
          delay(35);
        } else if (command == 'm'){
          digitalWrite(L_EN, LOW);
          digitalWrite(R_EN, LOW);
        }
    } else {
        straightenWheel();
        digitalWrite(L_EN, LOW);
        digitalWrite(R_EN, LOW);

    }
}



void turnRight() {
    if (currentSteps < maxSteps) {
        digitalWrite(directionPin, HIGH); // Set direction to right
        for (int i = 0; i < 250; i++) {stepMotor();}
        currentSteps += 250;
    }
}

void turnLeft() {
    if (currentSteps > -maxSteps) {
        digitalWrite(directionPin, LOW); // Set direction to left
        for (int i = 0; i < 250; i++) {stepMotor();}
        currentSteps -= 250;
    }
}

void straightenWheel() {
    if (currentSteps > 0) {
        digitalWrite(directionPin, LOW); // Set direction to left
        for (int i = 0; i < 50; i++) {stepMotor();}
        currentSteps -= 50;
    } else if (currentSteps < 0) {
        digitalWrite(directionPin, HIGH); // Set direction to right
        for (int i = 0; i < 50; i++) {stepMotor();}
        currentSteps += 50;
    }
}

void stepMotor() {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(100);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(100);
}
