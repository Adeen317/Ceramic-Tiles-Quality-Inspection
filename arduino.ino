int Lswitch = 1; 
int led = 13; 
int flag = 0;
const int newLed = 12;
const int motorPin1 = 5; // Motor control pin 1
const int motorPin2 = 6; // Motor control pin 2
const int enablePin = 9; // Motor enable pin
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 50;  // the debounce time; increase if the output flickers
int lastSwitchState = HIGH;
int switchState;

void setup() {
  Serial.begin(9600); 
  pinMode(Lswitch, INPUT_PULLUP); // Use internal pull-up resistor
  pinMode(led, OUTPUT);
  pinMode(newLed, OUTPUT);
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(enablePin, OUTPUT);
  digitalWrite(newLed, LOW);
  digitalWrite(motorPin1, LOW);
  digitalWrite(motorPin2, LOW);
  digitalWrite(enablePin, LOW); // Disable motor initially
}

void loop() {
  int reading = digitalRead(Lswitch);

  if (reading != lastSwitchState) {
    // reset the debouncing timer
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    // whatever the reading is at, it's been there for longer than the debounce
    // delay, so take it as the actual current state:
    switchState = reading;

    // Only toggle the LED if the new switch state is HIGH
    if (switchState == LOW && flag == 0) {
      delay(2000);
      digitalWrite(led, HIGH);
      //Serial.println("tile is detected"); 
      //flag = 1;
      delay(2000); 
      Serial.println("tile is detected");
      flag = 1;
      digitalWrite(led, LOW);
    }

    if (switchState == HIGH && flag == 1) {
      Serial.println("Tile is moving forward"); 
      flag = 0;
      delay(20); 
    }
  }

  lastSwitchState = reading;

  if (Serial.available() > 0) {
    String receivedMessage = Serial.readStringUntil('\n');
    receivedMessage.trim(); // Remove any extra whitespace or newline characters
    if (receivedMessage == "image captured") {
      Serial.println("image is taken"); 
      // Perform any actions upon receiving acknowledgment from Python
    }
    if (receivedMessage == "tile is defected") {
      delay(2400);
      digitalWrite(led, HIGH);
      
      // Enable motor and run bidirectionally
      digitalWrite(enablePin, HIGH);
      
      // Run motor in one direction for 10 seconds
      digitalWrite(motorPin1, HIGH);
      digitalWrite(motorPin2, LOW);
      delay(18000);

      // Run motor in the opposite direction for 10 seconds
      digitalWrite(motorPin1, LOW);
      digitalWrite(motorPin2, HIGH);
      delay(18000);
      
       digitalWrite(led, LOW);
      // Disable motor
      digitalWrite(motorPin1, LOW);
      digitalWrite(motorPin2, LOW);
      digitalWrite(enablePin, LOW);
    }
  }
}