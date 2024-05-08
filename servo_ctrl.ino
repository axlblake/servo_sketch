#include <Servo.h>


const int mode_btn_pin = 7;

const int led_red = 8;  // pin for red signal
const int led_green = 9;



// variables will change:
int mode_btn_state = 0;

// Define the number of servo motors
const int numServos = 5;
const int numButtons = 5;

const int MAX_ANGLE = 160;
const int MIN_ANGLE = 20;

// Define the pins to which the servo motors are connected
int servoPins[numServos] = { A0, A1, A2, A3, A4 };
int buttonPins[numButtons] = { 2, 3, 4, 5, 6 };

// Create servo objects
Servo servos[numServos];

// Define the current angle for each servo
int servoAngles[numServos] = { 90, 90, 90, 90, 90 };

// Define the increment step for each servo
int angleIncrement = 1;

int reverse_mode = 0;

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
    Serial.println(reverse_mode);

    Serial.println(servoAngles[num]);
    move_servo(num);
  } else {
    led_to_none();
  }
}

void move_servo(int servo_num) {
  if (servoAngles[servo_num] >= MAX_ANGLE) {
    servoAngles[servo_num] = MAX_ANGLE;
  }

  if (servoAngles[servo_num] <= MIN_ANGLE) {
    servoAngles[servo_num] = MIN_ANGLE;
  }

  if (reverse_mode == 1) {
    servoAngles[servo_num] -= angleIncrement;
  } else {
    servoAngles[servo_num] += angleIncrement;
  }

  servos[servo_num].write(servoAngles[servo_num]);
}

void reverse_mode_btn_state() {
  mode_btn_state = digitalRead(mode_btn_pin);
  if (mode_btn_state == HIGH) {
    led_to_none();
    delay(100);
    if (reverse_mode == 1) {
      reverse_mode = 0;
      Serial.println("| Reverse OFF 0! |");
    } else {
      led_to_red();
      reverse_mode = 1;
      Serial.println("| Reverse ON 1! |");
    }
    delay(100);
    // for (int i = 0; i < numServos; i++) {
    // move_servo(i);
    // }
  }
}

void loop() {
  reverse_mode_btn_state();

  for (int i = 0; i < numButtons; i++) {
    int btn_state = digitalRead(buttonPins[i]);
    button_click(i, btn_state);
    btn_state = 0;
  }
}
