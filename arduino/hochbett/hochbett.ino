#include <SoftwareSerial.h>
#include "TimerOne.h"

#define BT_RX_PIN 7
#define BT_TX_PIN 8

#define MOTOR_ENABLE_PIN 4
#define MOTOR_DIRECTION_PIN 3

#define BLINK_PIN 13

#define TIMER_INTERVAL_US (10 * 1000)

#define MOVE_STOP 0
#define MOVE_UP 1
#define MOVE_DOWN 2

#define MOVE_ENABLE_DELAY_THRESHOLD_COUNT 15
#define RECEIVE_TIMOUT_THRESHOLD_COUNT 25

SoftwareSerial btSerial(BT_RX_PIN, BT_TX_PIN);
volatile uint8_t moveDirection;
volatile uint8_t motorEnableCounter;
volatile uint8_t receiveTimeotCounter;
volatile uint8_t blinkOnThreshold;
volatile uint8_t blinkOffThreshold;
volatile uint8_t blinkState;

inline void blinkShort(){
  blink(5,100);
}

inline void blinkConnected(){
  blink(10,20);
}

inline void blink(uint8_t on, uint8_t off) {
  blinkOnThreshold = on;
  blinkOffThreshold = off;
}

void setup() {
  Serial.begin(115200);
  btSerial.begin(115200);

  Serial.println("Hochbett v1.0");

  pinMode(MOTOR_ENABLE_PIN, OUTPUT);
  pinMode(MOTOR_DIRECTION_PIN, OUTPUT);

  moveDirection = MOVE_STOP;
  motorEnableCounter = 0;
  receiveTimeotCounter = RECEIVE_TIMOUT_THRESHOLD_COUNT;

  blinkState = 0;
  blink(5, 100);

  Timer1.initialize(TIMER_INTERVAL_US);
  Timer1.attachInterrupt(timer10ms);
}

void timer10ms() {
  // Blink LED
  ++blinkState;
  if (blinkState <= blinkOnThreshold) {
    digitalWrite(BLINK_PIN, HIGH);
  } else if (blinkState <= blinkOffThreshold) {
    digitalWrite(BLINK_PIN, LOW);
  } else {
    blinkState = 0;
  }

  if (++receiveTimeotCounter >= RECEIVE_TIMOUT_THRESHOLD_COUNT) {
    receiveTimeotCounter = RECEIVE_TIMOUT_THRESHOLD_COUNT; // prevent overflow
    moveDirection = MOVE_STOP;
    motorEnableCounter = 0;
    blinkShort();
  }else{
    blinkConnected();
  }

  if (moveDirection == MOVE_STOP) {
    // MOVE_STOP -> disable all
    digitalWrite(MOTOR_ENABLE_PIN, LOW);
    digitalWrite(MOTOR_DIRECTION_PIN, LOW);

  } else {
    ++motorEnableCounter;
    if (motorEnableCounter >= MOVE_ENABLE_DELAY_THRESHOLD_COUNT) {
      motorEnableCounter = MOVE_ENABLE_DELAY_THRESHOLD_COUNT; // prevent overflow
      digitalWrite(MOTOR_ENABLE_PIN, HIGH);      
    } else {
      digitalWrite(MOTOR_ENABLE_PIN, LOW);      
    }
    digitalWrite(MOTOR_DIRECTION_PIN, (moveDirection == MOVE_UP) ? HIGH : LOW);
  }
}

inline void setMotorMove(boolean motorOn) {
  digitalWrite(MOTOR_ENABLE_PIN, motorOn ? HIGH : LOW);
}

inline void setMotorDirection(uint8_t motorDirection) {
  digitalWrite(MOTOR_DIRECTION_PIN, (motorDirection == MOVE_UP) ? HIGH : LOW);
}

void loop() {
  if (btSerial.available()) {
    char c = btSerial.read();
    //charReceived = true;

    uint8_t oldDirection = moveDirection;

    if (c == 'w') {
      // STOP/IDLE
      receiveTimeotCounter = 0;
      if (moveDirection != MOVE_STOP) {
        moveDirection = MOVE_STOP;
      }

    } else if (c == 'u') {
      // UP
      receiveTimeotCounter = 0;
      if (moveDirection != MOVE_UP) {
        moveDirection = MOVE_UP;
      }

    } else if (c == 'd') {
      // DOWN
      receiveTimeotCounter = 0;
      if (c != MOVE_DOWN) {
        moveDirection = MOVE_DOWN;
      }
    }

    if (moveDirection == MOVE_STOP || moveDirection != oldDirection) {
      motorEnableCounter = 0;
    }
  }
}
