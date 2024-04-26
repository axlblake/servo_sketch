#include <Servo.h>


const int mode_btn_pin = 7;  

const int led_red = 8; // pin for red signal
const int led_green = 9;



// variables will change:
int mode_btn_state = 0;

int state = 0;

// Define the number of servo motors
const int numServos = 5;
const int numButtons = 5;


// Define the pins to which the servo motors are connected
int servoPins[numServos] = {A0, A1, A2, A3, A4};
int buttonPins[numButtons] = {2, 3, 4, 5, 6};

// Create servo objects
Servo servos[numServos];

// Define the current angle for each servo
int servoAngles[numServos] = {0, 0, 0, 0, 0};

// Define the increment step for each servo
int angleIncrement = 20;

void setup() {
  // Attach servo objects to their respective pins
  for (int i = 0; i < numServos; i++) {
    servos[i].attach(servoPins[i]);
  }

  for (int n = 0; n < numButtons; n++) {
    pinMode(buttonPins[n], INPUT);
  }

  // Initialize serial communication
  Serial.begin(9600);
  pinMode(mode_btn_pin, INPUT);

  pinMode(led_red, OUTPUT);
  pinMode(led_green, OUTPUT);
  Serial.println("| Unit started |");
}

void led_to_red() {
  analogWrite(led_green, 0);
  analogWrite(led_red, 255);
}

void led_to_green() {
  analogWrite(led_red, 0);
  analogWrite(led_green, 255);
}

void led_to_none() {
  analogWrite(led_red, 0);
  analogWrite(led_green, 0);
}

void button_click(int num, int state) {
  if (state == HIGH) {
    led_to_green();
    Serial.println("| btn clicked! |");
    Serial.println(num);
    move_servo(num);
  }
}

void move_servo(int servo_num) {
  servoAngles[servo_num] += angleIncrement;
   if (servoAngles[servo_num] > 180) {
      servoAngles[servo_num] = 180;
      angleIncrement *= -1;  // Reverse the direction of increment
      Serial.print("Servo ");
      Serial.print(servo_num);
      Serial.println(": Reached maximum angle, reversing direction.");
    } else if (servoAngles[servo_num] < 0) {
      servoAngles[servo_num] = 0;
      angleIncrement *= -1;  // Reverse the direction of increment
      Serial.print("Servo ");
      Serial.print(servo_num);
      Serial.println(": Reached minimum angle, reversing direction.");
    } else {
      Serial.print("Servo ");
      Serial.print(servo_num);
      Serial.print(": Angle set to ");
      Serial.println(servoAngles[servo_num]);
    }

    // Set the angle for the servo
    servos[servo_num].write(servoAngles[servo_num]);
}

void loop() {
  led_to_none();
  mode_btn_state = digitalRead(mode_btn_pin);
    
  if (mode_btn_state == HIGH) {
    led_to_red();
    for (int i = 0; i < numServos; i++) {
    move_servo(i);
    }
  } else {
    mode_btn_state = 0;
  }

  for (int i = 0; i < numButtons; i++) {
    int btn_state = digitalRead(buttonPins[i]);
    button_click(i, btn_state);
    btn_state = 0;
  }
}
