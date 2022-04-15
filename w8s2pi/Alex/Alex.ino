// BAREMETAL FILES

#include <math.h>
#include <serialize.h>
#include <stdarg.h>

#include "constants.h"
#include "packet.h"

typedef enum {
  STOP = 0,
  FORWARD = 1,
  BACKWARD = 2,
  LEFT = 3,
  RIGHT = 4
} TDirection;

volatile TDirection dir = STOP;

/*
 * Alex's configuration constants
 */

// Number of ticks per revolution from the
// wheel encoder.
// 867 Right 5 times
// 845 Left 5 times
#define COUNTS_PER_REV 171

// Wheel circumference in cm.
// We will use this to calculate forward/backward distance traveled
// by taking revs * WHEEL_CIRC

#define WHEEL_CIRC 21

// Motor control pins. You need to adjust these till
// Alex moves in the correct direction
#define LR 5  // Left reverse pin
#define LF 6  // Left forward pin
#define RF 10 // Right forward pin
#define RR 11 // Right reverse pin

#define ALEX_LENGTH 18.5
#define ALEX_BREADTH 12

#define trigFront 7
#define echoFront 8
#define trigSide 12
#define echoSide 13

#define TIMEOUT 30000
#define WAITING_TIME 1000

#define SENSOR_M1 28.476
#define SENSOR_M2 28.249

#define RC 1
#define LC 1

#define UDRIEMASK 0b00100000

float alexDiagonal = 0.0;
float alexCirc = 0.0;

/*
 *    Alex's State Variables
 */

// Store the ticks from Alex's left and
// right encoders.
volatile unsigned long leftForwardTicks;
volatile unsigned long rightForwardTicks;
volatile unsigned long leftReverseTicks;
volatile unsigned long rightReverseTicks;

volatile unsigned long leftForwardTicksTurns;
volatile unsigned long leftReverseTicksTurns;
volatile unsigned long rightForwardTicksTurns;
volatile unsigned long rightReverseTicksTurns;

// Store the revolutions on Alex's left
// and right wheels
volatile unsigned long leftRevs;
volatile unsigned long rightRevs;

// Forward and backward distance traveled
volatile unsigned long forwardDist;
volatile unsigned long reverseDist;

unsigned long deltaDist;
unsigned long newDist;
unsigned long deltaTicks;
unsigned long targetTicks;

//Timers
volatile long _timerTicks;

char dataRecv, dataSend;

//Motor Interrupts
volatile int move_lf;
volatile int move_rf;
volatile int move_lr;
volatile int move_rr;
volatile long speed_l;
volatile long speed_r;

/*
 *
 * Alex Communication Routines.
 *
 */

TResult readPacket(TPacket *packet) {
  // Reads in data from the serial port and
  // deserializes it. Returns deserialized
  // data in "packet".

  char buffer[PACKET_SIZE];
  int len;

  len = readSerial(buffer);

  if (len == 0)
    return PACKET_INCOMPLETE;
  else
    return deserialize(buffer, len, packet);
}

long US_distance(char dir, int echo, float constant) {
  if (dir == "F") { // Trig Pin 7 = PD7
    // PD7 & PB0
    PORTD &= 0b01111111;
    //digitalWrite(trig, LOW);
    c_delay(2);
    // delayMicroseconds(2);
    PORTD |= 0b10000000;
    //digitalWrite(trig, HIGH);
    // delayMicroseconds(10);
    c_delay(10);
    PORTD &= 0b01111111;
    //digitalWrite(trig, LOW);
    long duration = pulseIn(echo, HIGH, TIMEOUT);
    long distance = (duration / 2.0) / constant;
    return distance;
  }

  if (dir == "S") { // Trig Pin 12 = PB4
    // PB4 & PB5
    PORTB &= 0b11101111; 
    //digitalWrite(trig,LOW);
    // delayMicroseconds(2);
    c_delay(2);
    PORTB |= 0b00010000;
    //digitalWrite(trig,HIGH);
    // delayMicroseconds(10);
    c_delay(10);
    PORTB &= 0b11101111;
    //digitalWrite(trig,LOW);
    long duration = pulseIn(echo, HIGH, TIMEOUT);
    long distance = (duration / 2.0) / constant;
    return distance;
  }
}

void sendStatus() {
  // Implement code to send back a packet containing key
  // information like leftTicks, rightTicks, leftRevs, rightRevs
  // forwardDist and reverseDist
  // Use the params array to store this information, and set the
  // packetType and command files accordingly, then use sendResponse
  // to send out the packet. See sendMessage on how to use sendResponse.
  //
  long distance_front = US_distance("F", echoFront, SENSOR_M1);
  long distance_side = US_distance("S", echoSide, SENSOR_M2);

  TPacket statusPacket;
  statusPacket.packetType = PACKET_TYPE_RESPONSE;
  statusPacket.command = RESP_STATUS;
  statusPacket.params[0] = leftForwardTicks;
  statusPacket.params[1] = rightForwardTicks;
  statusPacket.params[2] = leftReverseTicks;
  statusPacket.params[3] = rightReverseTicks;
  statusPacket.params[4] = leftForwardTicksTurns;
  statusPacket.params[5] = rightForwardTicksTurns;
  statusPacket.params[6] = leftReverseTicksTurns;
  statusPacket.params[7] = rightReverseTicksTurns;
  statusPacket.params[8] = forwardDist;
  statusPacket.params[9] = reverseDist;
  statusPacket.params[10] = distance_front;
  statusPacket.params[11] = distance_side;
  sendResponse(&statusPacket);
}

void sendMessage(const char *message) {
  // Sends text messages back to the Pi. Useful
  // for debugging.

  TPacket messagePacket;
  messagePacket.packetType = PACKET_TYPE_MESSAGE;
  strncpy(messagePacket.data, message, MAX_STR_LEN);
  sendResponse(&messagePacket);
}

void dbprint(char *format, ...) {
  va_list args;
  char buffer[128];

  va_start(args, format);
  vsprintf(buffer, format, args);
  sendMessage(buffer);
}

void sendBadPacket() {
  // Tell the Pi that it sent us a packet with a bad
  // magic number.

  TPacket badPacket;
  badPacket.packetType = PACKET_TYPE_ERROR;
  badPacket.command = RESP_BAD_PACKET;
  sendResponse(&badPacket);
}

void sendBadChecksum() {
  // Tell the Pi that it sent us a packet with a bad
  // checksum.

  TPacket badChecksum;
  badChecksum.packetType = PACKET_TYPE_ERROR;
  badChecksum.command = RESP_BAD_CHECKSUM;
  sendResponse(&badChecksum);
}

void sendBadCommand() {
  // Tell the Pi that we don't understand its
  // command sent to us.

  TPacket badCommand;
  badCommand.packetType = PACKET_TYPE_ERROR;
  badCommand.command = RESP_BAD_COMMAND;
  sendResponse(&badCommand);
}

void sendBadResponse() {
  TPacket badResponse;
  badResponse.packetType = PACKET_TYPE_ERROR;
  badResponse.command = RESP_BAD_RESPONSE;
  sendResponse(&badResponse);
}

void sendOK() {
  TPacket okPacket;
  okPacket.packetType = PACKET_TYPE_RESPONSE;
  okPacket.command = RESP_OK;
  sendResponse(&okPacket);
}

void sendResponse(TPacket *packet) {
  // Takes a packet, serializes it then sends it out
  // over the serial port.
  char buffer[PACKET_SIZE];
  int len;

  len = serialize(buffer, packet, sizeof(TPacket));
  writeSerial(buffer, len);
}

/*
 * Setup and start codes for external interrupts and
 * pullup resistors.
 *
 */
// Enable pull up resistors on pins 2 and 3
void enablePullups() {
  // Use bare-metal to enable the pull-up resistors on pins
  // 2 and 3. These are pins PD2 and PD3 respectively.
  // We set bits 2 and 3 in DDRD to 0 to make them inputs.
  DDRD &= 0b11110011;
  PORTD |= 0b00001100;
}

// Functions to be called by INT0 and INT1 ISRs.
void leftISR() {
  if (dir == FORWARD) {
    leftForwardTicks++;
    forwardDist =
        (unsigned long)((float)leftForwardTicks / COUNTS_PER_REV * WHEEL_CIRC);
  } else if (dir == BACKWARD) {
    leftReverseTicks++;
    reverseDist =
        (unsigned long)((float)leftReverseTicks / COUNTS_PER_REV * WHEEL_CIRC);
  } else if (dir == LEFT) {
    leftReverseTicksTurns++;
  } else if (dir == RIGHT) {
    leftForwardTicksTurns++;
  }
}

void rightISR() {
  if (dir == FORWARD) {
    rightForwardTicks++;
  } else if (dir == BACKWARD) {
    rightReverseTicks++;
  } else if (dir == LEFT) {
    rightForwardTicksTurns++;
  } else if (dir == RIGHT) {
    rightReverseTicksTurns++;
  }
}

// Set up the external interrupt pins INT0 and INT1
// for falling edge triggered. Use bare-metal.
void setupEINT() {
  EICRA = 0b00001010;
  EIMSK = 0b00000011;
}

// Implement the external interrupt ISRs below.
// INT0 ISR should call leftISR while INT1 ISR
// should call rightISR.

ISR(INT0_vect) { 
  leftISR(); 
}

ISR(INT1_vect) {
  rightISR(); 
}

// Delay timer (1s)
ISR(TIMER1_COMPA_vect)
{
  _timerTicks++;
}

/*
ISR(USART_RX_vect) { 
  dataRecv = UDR0;
}

ISR(USART_UDRE_vect) {
  UDR0 = dataSend;
  UCSR0B &= ~UDRIEMASK;
}
*/

void setupTimer() {
  // Set timer 1 to produce 1s (1000000us) ticks 
  // But do not start the timer here.
  TIMSK1 |= 0b10;
  TCNT1 = 0;
  OCR1A = 62499;
}

void c_delay (int dur) {
  TCCR1B = 0b00000000;
  cli();
  TCCR1A = 0b00000000; 
  TIMSK1 = 0b10;
  OCR1A = 15; // 1us
  TCNT1 = 0;
  _timerTicks = 0; // Reset counter
  TCCR1B = 0b00001001;
  sei();

  while (_timerTicks < dur);
  TCNT1 = 0;
  TCCR1A = 0b00100001; // Enable OC1B
  TIMSK1 |= 0b100;  
  TCCR1B = 0b00000011; 
  OCR1B = 0;
}

// Implement INT0 and INT1 ISRs above.

/*
 * Setup and start codes for serial communications
 *
 */
// Set up the serial connection. For now we are using
// Arduino Wiring, you will replace this later
// with bare-metal code.
void setupSerial() {
  // To replace later with bare-metal.
  Serial.begin(9600);
  // baud rate = ( 16MHz/(16*9600) ) - 1 = 103.1666s
  //UBRR0L = 103;
  //UBRR0H = 0;
  // Setting to asynchronous USART
  //UCSR0C = 00000110;
  //UCSR0A = 0; // to disable double-speed mode and multiprocessor mode
}

// Start the serial connection. For now we are using
// Arduino wiring and this function is empty. We will
// replace this later with bare-metal code.

void startSerial() {
  // Empty for now. To be replaced with bare-metal code
  // later on.
  //UCSR0B = 0b10011000;
}

// Read the serial port. Returns the read character in
// ch if available. Also returns TRUE if ch is valid.
// This will be replaced later with bare-metal code.

int readSerial(char *buffer) {

  int count = 0;

  while (Serial.available()) { // change
    buffer[count++] = Serial.read();
  }

  // Disable UDRE interrupt
  //UCSR0B &= 0b11011111;

  return count;
}

// Write to the serial port. Replaced later with
// bare-metal code

void writeSerial(const char *buffer, int len) {
  Serial.write(buffer, len);
  // not sure what to do here, combine all the items in the array buffer given
  // the length, to form a string? Enable UDRE interrupt
  //UCSR0B |= 0b00100000;
}

/*
 * Alex's motor drivers.
 *
 */

// Set up Alex's motors
void setupMotors() {
  /* Our motor set up is:
   *    A1IN - Pin 5, PD5, OC0B
   *    A2IN - Pin 6, PD6, OC0A
   *    B1IN - Pin 10, PB2, OC1B
   *    B2In - pIN 11, PB3, OC2A
   */
    
  DDRD |= 0b01100000; // Pin 5 and 6
  DDRB |= 0b00001100; // Pin 10 and 11
  
  TCNT0 = 0;
  TCCR0A = 0b10100001; // Enable OC0A and OC0B
  TIMSK0 |= 0b110; // OCIEA = 1 OCIEB = 1
 
  // Use mode 1 instead. so we dont have to convert to 10 bit resolution
  TCNT1 = 0;
  TCCR1A = 0b00100001; // Enable OC1B
  TIMSK1 |= 0b100;
   
  TCNT2 = 0;
  TCCR2A = 0b10000001; // Enable OC2A
  TIMSK2 |= 0b010;

  OCR0A = 0;
  OCR0B = 0;
  OCR1B = 0; 
  OCR2A = 0;
}

// Start the PWM for Alex's motors.
// We will implement this later. For now it is
// blank.
void startMotors() {
  TCCR0B = 0b00000011; 
  TCCR1B = 0b00000011; 
  TCCR2B = 0b00000011; 
  
  OCR0A = 0; //lf; // LF
  OCR0B = 0; //lr; // LR
  OCR1B = 0; //rf * (65535/255); // RF
  OCR2A = 0; //rr; // RR
}

// Convert percentages to PWM values
int pwmVal(float speed) {
  if (speed < 0.0)
    speed = 0;

  if (speed > 100.0)
    speed = 100.0;

  return (int)((speed / 100.0) * 255.0);
}

// Move Alex forward "dist" cm at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// move forward at half speed.
// Specifying a distance of 0 means Alex will
// continue moving forward indefinitely.
void forward(float dist, float speed) {
  dir = FORWARD;
  int val = pwmVal(speed);

  if (dist > 0)
    deltaDist = dist;
  else
    deltaDist = 9999999;
  newDist = forwardDist + deltaDist;

  speed_l = val * LC;
  speed_r = val * RC;
  OCR0A = speed_l;
  OCR1B = speed_r;
  OCR0B = 0;
  OCR2A = 0;
}

// Reverse Alex "dist" cm at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// reverse at half speed.
// Specifying a distance of 0 means Alex will
// continue reversing indefinitely.
void reverse(float dist, float speed) {
  dir = BACKWARD;
  int val = pwmVal(speed);

  if (dist > 0)
    deltaDist = dist;
  else
    deltaDist = 9999999;
  newDist = reverseDist + deltaDist;

  OCR2A = val * RC;
  OCR0B = val * LC;
  OCR0A = 0;
  OCR1B = 0;
}

// Turn Alex left "ang" degrees at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// turn left at half speed.
// Specifying an angle of 0 degrees will cause Alex to
// turn left indefinitely.
unsigned long computeDeltaTicks(float ang) {
  unsigned long ticks =
      (unsigned long)((ang * alexCirc * COUNTS_PER_REV) / (360.0 * WHEEL_CIRC));

  return ticks;
}

void left(float ang, float speed) {
  if (ang == 0)
    deltaTicks = 99999999;
  else
    deltaTicks = computeDeltaTicks(ang);
  targetTicks = leftReverseTicksTurns + deltaTicks * 45.0 / 106.6;
  dir = LEFT;
  int val = pwmVal(speed);

  OCR0B = val;
  OCR1B = val;
  OCR2A = 0;
  OCR0A = 0;
}

// Turn Alex right "ang" degrees at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// turn left at half speed.
// Specifying an angle of 0 degrees will cause Alex to
// turn right indefinitely.
void right(float ang, float speed) {
  if (ang == 0)
    deltaTicks = 99999999;
  else
    deltaTicks = computeDeltaTicks(ang);
  targetTicks = rightReverseTicksTurns + deltaTicks / 2.0;
  dir = RIGHT;
  int val = pwmVal(speed);

  OCR0A = val;
  OCR2A = val;
  OCR0B = 0;
  OCR1B = 0;
}

// Stop Alex. To replace with bare-metal code later.
void stop() {
  OCR0A = 0; 
  OCR0B = 0; 
  OCR1B = 0; 
  OCR2A = 0; 
}

/*
 * Alex's setup and run codes
 *
 */

// Clears all our counters
void clearCounters() {
  leftForwardTicks = 0;
  leftReverseTicks = 0;
  rightForwardTicks = 0;
  rightReverseTicks = 0;
  leftForwardTicksTurns = 0;
  leftReverseTicksTurns = 0;
  rightForwardTicksTurns = 0;
  rightReverseTicksTurns = 0;
  leftRevs = 0;
  rightRevs = 0;
  forwardDist = 0;
  reverseDist = 0;
}

// Clears one particular counter
void clearOneCounter(int which) { clearCounters(); }
// Intialize Vincet's internal states

void initializeState() { clearCounters(); }

void handleCommand(TPacket *command) {
  switch (command->command) {
  // For movement commands, param[0] = distance, param[1] = speed.
  case COMMAND_FORWARD:
    sendOK();
    forward((float)command->params[0], (float)command->params[1]);
    break;

  case COMMAND_REVERSE:
    sendOK();
    reverse((float)command->params[0], (float)command->params[1]);
    break;

  case COMMAND_TURN_LEFT:
    sendOK();
    left((float)command->params[0], (float)command->params[1]);
    break;

  case COMMAND_TURN_RIGHT:
    sendOK();
    right((float)command->params[0], (float)command->params[1]);
    break;

  case COMMAND_STOP:
    sendOK();
    stop();
    break;
  case COMMAND_GET_STATS:
    sendStatus();
    break;
  case COMMAND_CLEAR_STATS:
    clearOneCounter(command->params[0]);
    sendOK();
    break;
  default:
    sendBadCommand();
  }
}

void waitForHello() {
  int exit = 0;

  while (!exit) {
    TPacket hello;
    TResult result;

    do {
      result = readPacket(&hello);
    } while (result == PACKET_INCOMPLETE);

    if (result == PACKET_OK) {
      if (hello.packetType == PACKET_TYPE_HELLO) {

        sendOK();
        exit = 1;
      } else
        sendBadResponse();
    } else if (result == PACKET_BAD) {
      sendBadPacket();
    } else if (result == PACKET_CHECKSUM_BAD)
      sendBadChecksum();
  } // !exit
}

void setupultrasonic() {

  DDRB |= 0b00010000;
  DDRB &= 0b11011110;
  DDRD |= 0b10000000;
  
  // pinMode(trigFront, OUTPUT); // Pin 7 = PD7
  // pinMode(echoFront, INPUT); // Pin 8 = PB0
  // pinMode(trigSide, OUTPUT); // Pin 12 = PB4
  // pinMode(echoSide, INPUT); // Pin 13 = PB5
}

void setup() {
  // put your setup code here, to run once:
  alexDiagonal =
      sqrt((ALEX_LENGTH * ALEX_LENGTH) + (ALEX_BREADTH * ALEX_BREADTH));
  alexCirc = PI * alexDiagonal;
  cli();
  setupEINT();
  setupSerial();
  startSerial();
  setupMotors();
  startMotors();
  enablePullups();
  initializeState();
  setupultrasonic();
  sei();
}

void handlePacket(TPacket *packet) {
  switch (packet->packetType) {
  case PACKET_TYPE_COMMAND:
    handleCommand(packet);
    break;

  case PACKET_TYPE_RESPONSE:
    break;

  case PACKET_TYPE_ERROR:
    break;

  case PACKET_TYPE_MESSAGE:
    break;

  case PACKET_TYPE_HELLO:
    break;
  }
}
bool run = 1;

void loop() {
  // put your main code here, to run repeatedly:
  TPacket recvPacket; // This holds commands from the Pi

  TResult result = readPacket(&recvPacket);

  if (result == PACKET_OK)
    handlePacket(&recvPacket);
  else if (result == PACKET_BAD) {
    sendBadPacket();
  } else if (result == PACKET_CHECKSUM_BAD) {
    sendBadChecksum();
  }
  if (deltaDist > 0) {
    if (dir == FORWARD) {
      if (forwardDist >= newDist) {
        deltaDist = 0;
        newDist = 0;
        stop();
      }
    } else if (dir == BACKWARD) {
      if (reverseDist >= newDist) {
        deltaDist = 0;
        newDist = 0;
        stop();
      }
    } else if (dir == STOP) {
      deltaDist = 0;
      newDist = 0;
      stop();
    }
  }
  if (deltaTicks > 0) {
    if (dir == LEFT) {
      if (leftReverseTicksTurns >= targetTicks) {
        deltaTicks = 0;
        targetTicks = 0;
        stop();
      }
    } else if (dir == RIGHT) {
      if (rightReverseTicksTurns >= targetTicks) {
        deltaTicks = 0;
        targetTicks = 0;
        stop();
      }
    } else if (dir == STOP) {
      deltaTicks = 0;
      targetTicks = 0;
      stop();
    }
  }
}
