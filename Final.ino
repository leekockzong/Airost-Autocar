#include <SoftwareSerial.h>
#include <Servo.h> // Include the Servo library

// Define HC-05 TX and RX pins for software serial
SoftwareSerial Bluetooth(0, 1); // RX, TX (connect to pins 0 and 1)

// Motor and sensor pins
#define in1 9
#define in2 8
#define in3 7
#define in4 6
#define enA 10
#define enB 11
#define RIGHT_SENSOR_PIN A0  // Right sensor pin
#define LEFT_SENSOR_PIN A1   // Left sensor pin

int M1_Speed = 200;
int M2_Speed = 200;

// Servo pin and object
int servoPin = 3; 
Servo myServo; // Create a Servo object

// Bluetooth command variable
char command;

// Sensor control flag
bool bluetoothMode = false;
bool isAt85 = false; // Tracks the current position of the servo
// Servo position flag


// Function prototypes
void forward();
void backward();
void left();
void right();
void Stop();
void enableSensors();
void disableSensors();
void toggleServo(); // Servo toggle function

void setup() {
  // Initialize motor pins as outputs and stop the motors
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);

  // Ensure motors are stopped at startup
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  analogWrite(enA, 0);
  analogWrite(enB, 0);

  // Initialize IR sensor pins as inputs
  pinMode(RIGHT_SENSOR_PIN, INPUT);
  pinMode(LEFT_SENSOR_PIN, INPUT);

  // Attach and initialize the servo motor
  myServo.attach(servoPin);
  myServo.write(0); // Set initial position to 0 degrees

  // Initialize Bluetooth communication
  Bluetooth.begin(9600); // Match the baud rate of your HC-05
  Serial.begin(9600);    // Optional: for debugging

  Serial.println("System Ready. Waiting for commands...");
}

void loop() {
  // Check for Bluetooth commands
  if (Bluetooth.available()) {
    command = Bluetooth.read(); // Read the command
    Serial.print("Command received: ");
    Serial.println(command);

    if (command == 'T' || command == 't') { // Toggle command
      if (isAt85) {
        myServo.write(0); // Rotate back to 0 degrees
        Serial.println("Servo rotated to 0 degrees");
        isAt85 = false;  // Update position state
      } else {
        myServo.write(85); // Rotate to 85 degrees
        Serial.println("Servo rotated to 85 degrees");
        isAt85 = true;   // Update position state
      }
    } else {
      Serial.println("Invalid command. Send 'T' to toggle servo position.");
    }
  
    // Process commands
    switch (command) {
      case 'F': 
        forward(); 
        bluetoothMode = true;
        disableSensors();
        break;
      case 'B': 
        backward(); 
        bluetoothMode = true;
        disableSensors();
        break;
      case 'L': 
        left(); 
        bluetoothMode = true;
        disableSensors();
        break;
      case 'R': 
        right(); 
        bluetoothMode = true;
        disableSensors();
        break;
      case 'S': 
        Stop(); 
        bluetoothMode = true;
        disableSensors();
        break;
      case 'A': // Resume auto line-following mode
        bluetoothMode = false;
        enableSensors();
        break;
      default:
        Serial.println("Invalid Command");
        Stop();
        break;
    }
  }

  // If not in Bluetooth mode, run line-following logic
  if (!bluetoothMode) {
    // Read IR sensor values (HIGH = no line detected, LOW = line detected)
    int rightSensor = digitalRead(RIGHT_SENSOR_PIN);
    int leftSensor = digitalRead(LEFT_SENSOR_PIN);

    // Print sensor readings for debugging
    Serial.print("Left Sensor: ");
    Serial.print(leftSensor);
    Serial.print("  Right Sensor: ");
    Serial.println(rightSensor);

    // Line-following logic
    if (leftSensor == LOW && rightSensor == LOW) {
      forward(); // Both sensors detect the line
      Serial.println("Moving forward");
    } else if (leftSensor == HIGH && rightSensor == LOW) {
      left(); // Left sensor detects the line, turn left
      Serial.println("Turning left");
    } else if (rightSensor == HIGH && leftSensor == LOW) {
      right(); // Right sensor detects the line, turn right
      Serial.println("Turning right");
    } else if (leftSensor == HIGH && rightSensor == HIGH) {
      Stop(); // Stop if no line is detected
      Serial.println("Stopping, no line detected");
    }
  }

  delay(50); // Small delay for stability
}

 //Enable IR sensors by setting pins back to INPUT mode
}
void enableSensors() {
  pinMode(RIGHT_SENSOR_PIN, INPUT);
  pinMode(LEFT_SENSOR_PIN, INPUT);
  Serial.println("Sensors enabled");
}

// Disable IR sensors by setting pins to OUTPUT and driving them LOW
void disableSensors() {
  pinMode(RIGHT_SENSOR_PIN, OUTPUT);
  pinMode(LEFT_SENSOR_PIN, OUTPUT);
  digitalWrite(RIGHT_SENSOR_PIN, LOW);
  digitalWrite(LEFT_SENSOR_PIN, LOW);
  Serial.println("Sensors disabled");
}

// Motor control functions
void forward() {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  analogWrite(enA, 255);
  analogWrite(enB, 255);
}

void backward() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  analogWrite(enA, 255);
  analogWrite(enB, 255);
}

void left() {
  digitalWrite(in1, HIGH); // Left motor forward
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH); // Right motor forward
  digitalWrite(in4, LOW);
  analogWrite(enA, 0); // Slow down the left motor
  analogWrite(enB, 255);     // Right motor at full speed
}

void right() {
  digitalWrite(in1, HIGH); // Left motor forward
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH); // Right motor forward
  digitalWrite(in4, LOW);
  analogWrite(enA, 255);     // Left motor at full speed
  analogWrite(enB, 0); // Slow down the right motor
}

void Stop() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}

// Toggle servo position
void toggleServo() {
  if (isAt85) {
    myServo.write(0); // Rotate to 0 degrees
    Serial.println("Servo rotated to 0 degrees");
    isAt85 = false;
  } else {
    myServo.write(85); // Rotate to 85 degrees
    Serial.println("Servo rotated to 85 degrees");
    isAt85 = true;
  }
}
