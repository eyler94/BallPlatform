#include <EnableInterrupt.h>
#include <ESC.h>
#include <Servo.h>
// #include <Wire.h>

#include <util/crc16.h>

#include "MPU6050.h"

// #define M_PI 3.14159

// encoders
#define E1APIN 4
#define E1BPIN 5
#define E2APIN 6
#define E2BPIN 7
#define E3APIN 8
#define E3BPIN 9

// ESCs
#define MOTOR1PIN 10
#define MOTOR2PIN 11

// reset button
#define RESETPIN 3

// serial communication
#define OUT_BUFFER_SIZE 64
#define OUT_START_BYTE 0xA5
#define OUT_PAYLOAD_LENGTH 36
#define OUT_MESSAGE_LENGTH 38

#define IN_START_BYTE 0xA5
#define IN_PAYLOAD_LENGTH 8
#define IN_MESSAGE_LENGTH 10

#define CRC_LENGTH 1
#define CRC_INITIAL_VALUE 0x00

enum ParseState {
  PARSE_STATE_IDLE,
  PARSE_STATE_GOT_START_BYTE,
  PARSE_STATE_GOT_PAYLOAD
};

//=============================================================================
// global variables
//=============================================================================

// encoders
volatile int16_t angle_counter1 = 0;
volatile int16_t angle_counter2 = 0;
volatile int16_t angle_counter3 = 0;

// IMU
int16_t ax, ay, az, temp, gx, gy, gz;
int16_t gx_offset, gy_offset, gz_offset;
int16_t gyro_fs, accel_fs;
float gyro_scale, accel_scale;

// motors
ESC Motor1;
ESC Motor2;
volatile float motor_1_speed;
volatile float motor_2_speed;
volatile long time_of_last_command;

// serial
uint8_t out_buf[OUT_BUFFER_SIZE];

ParseState parse_state;
uint8_t in_payload_buf[IN_PAYLOAD_LENGTH];
int in_payload_index;
uint8_t in_crc_value;

// timing
unsigned long previous_millis = 0;
const long interval = 10;

//=============================================================================
// Encoder ISRs
//=============================================================================

// Encoder 1
void E1AISR() {
  // if A went from high to low
  if(digitalRead(E1APIN) == LOW){
    if(digitalRead(E1BPIN) == HIGH){
      angle_counter1++;
    }else{
      angle_counter1--;
    }
  }else{ // A went from Low to high
    if(digitalRead(E1BPIN) == LOW){
      angle_counter1++;
    }else{
      angle_counter1--;
    }
  }
}

void E1BISR() {
  if(digitalRead(E1BPIN) == LOW){
    if(digitalRead(E1APIN) == LOW){
      angle_counter1++;
    }else{
      angle_counter1--;
    }
  }else{
    if(digitalRead(E1APIN) == HIGH){
      angle_counter1++;
    }else{
      angle_counter1--;
    }
  }
}

// Encoder 2
void E2AISR() {
  // if A went from high to low
  if(digitalRead(E2APIN) == LOW){
    if(digitalRead(E2BPIN) == HIGH){
      angle_counter2++;
    }else{
      angle_counter2--;
    }
  }else{ // A went from Low to high
    if(digitalRead(E2BPIN) == LOW){
      angle_counter2++;
    }else{
      angle_counter2--;
    }
  }
}

void E2BISR() {
  if(digitalRead(E2BPIN) == LOW){
    if(digitalRead(E2APIN) == LOW){
      angle_counter2++;
    }else{
      angle_counter2--;
    }
  }else{
    if(digitalRead(E2APIN) == HIGH){
      angle_counter2++;
    }else{
      angle_counter2--;
    }
  }
}

// Encoder 3
void E3AISR() {
  // if A went from high to low
  if(digitalRead(E3APIN) == LOW){
    if(digitalRead(E3BPIN) == HIGH){
      angle_counter3++;
    }else{
      angle_counter3--;
    }
  }else{ // A went from Low to high
    if(digitalRead(E3BPIN) == LOW){
      angle_counter3++;
    }else{
      angle_counter3--;
    }
  }
}

void E3BISR() {
  if(digitalRead(E3BPIN) == LOW){
    if(digitalRead(E3APIN) == LOW){
      angle_counter3++;
    }else{
      angle_counter3--;
    }
  }else{
    if(digitalRead(E3APIN) == HIGH){
      angle_counter3++;
    }else{
      angle_counter3--;
    }
  }
}

//=============================================================================
// serial
//=============================================================================
void handle_in_msg(float v1, float v2);
void unpack_in_payload(uint8_t buf[IN_PAYLOAD_LENGTH], float *v1, float *v2);
bool parse_in_byte(uint8_t c);
size_t pack_out_msg(uint8_t buf[OUT_MESSAGE_LENGTH],
  float roll, float pitch, float yaw,
  float accel_x, float accel_y, float accel_z,
  float gyro_x, float gyro_y, float gyro_z);

//=============================================================================
// initialize
//=============================================================================
void setup() {
  // Set up serial communication
  parse_state = PARSE_STATE_IDLE;
  Serial.begin(115200);

  // Connect Encoder Interrupts
  pinMode(E1APIN, INPUT_PULLUP);
  pinMode(E1BPIN, INPUT_PULLUP);
  enableInterrupt(E1APIN, E1AISR, CHANGE);
  enableInterrupt(E1BPIN, E1BISR, CHANGE);

  pinMode(E2APIN, INPUT_PULLUP);
  pinMode(E2BPIN, INPUT_PULLUP);
  enableInterrupt(E2APIN, E2AISR, CHANGE);
  enableInterrupt(E2BPIN, E2BISR, CHANGE);

  pinMode(E3APIN, INPUT_PULLUP);
  pinMode(E3BPIN, INPUT_PULLUP);
  enableInterrupt(E3APIN, E3AISR, CHANGE);
  enableInterrupt(E3BPIN, E3BISR, CHANGE);

  // Setup MPU6050
  // Wire.begin();
  //
  // Wire.beginTransmission(MPU_ADDR);
  // Wire.write(MPU_PWR_MGMT_1);  // PWR_MGMT_1 register
  // Wire.write(0);     // set to zero (wakes up the MPU-6050)
  // Wire.endTransmission(true);
  //
  // Wire.beginTransmission(MPU_ADDR);
  // Wire.write(MPU_GYRO_CONFIG);
  // Wire.write(MPU_GYRO_FS_500);
  // Wire.endTransmission(true);
  //
  // Wire.beginTransmission(MPU_ADDR);
  // Wire.write(MPU_ACCEL_CONFIG);
  // Wire.write(MPU_ACCEL_FS_4);
  // Wire.endTransmission(true);

  // gyro_fs = 500;
  // accel_fs = 4;
  // gyro_scale = 1.0 / 32767 * gyro_fs * M_PI / 180.0;
  // accel_scale = 1.0 / 32767 * accel_fs * 9.80665;
  //
  // gx_offset = 0;
  // gy_offset = 0;
  // gz_offset = 0;

  ax = ay = az = 0;
  gx = gy = gz = 0;
  temp = 0;

  // Configure ESCs
  Motor1.attach(MOTOR1PIN);
  Motor2.attach(MOTOR2PIN);

  motor_1_speed = 0.0;
  motor_2_speed = 0.0;
}

//=============================================================================
// loop
//=============================================================================
void loop() {

  unsigned long current_millis = millis();

  // turn off motors if no recent command messages
  if(current_millis - time_of_last_command > 100){
    motor_1_speed = 0.0;
    motor_2_speed = 0.0;
  }

  // check for reset button press
  if(digitalRead(RESETPIN) == HIGH)
  {
    angle_counter1 = 0;
    angle_counter2 = 0;
    angle_counter3 = 0;

    int32_t gx_sum = 0;
    int32_t gy_sum = 0;
    int32_t gz_sum = 0;

    int8_t num_samples = 20;
    // for (uint8_t i = 0; i < num_samples; i++)
    // {
    //   Wire.beginTransmission(MPU_ADDR);
    //   Wire.write(MPU_GYRO_XOUT_H);
    //   Wire.endTransmission(false);
    //   Wire.requestFrom(MPU_ADDR,6,true);
    //   gx_sum += Wire.read()<<8|Wire.read();
    //   gy_sum += Wire.read()<<8|Wire.read();
    //   gz_sum += Wire.read()<<8|Wire.read();
    //   delay(5);
    // }
    //
    // gx_offset = gx_sum / num_samples;
    // gy_offset = gy_sum / num_samples;
    // gz_offset = gz_sum / num_samples;
  }

  // send sensor measurements
  if (current_millis >= interval + previous_millis)
  {
    previous_millis = current_millis;

    // Encoders have 2500 Cycles per revolution, but we are doing quadrature, which
    // means we have 10,000 counts per revolotion, or 5,000 per half-revolution
    float angle1 = angle_counter1*M_PI/(5000.0);
    float angle2 = -1.0*angle_counter2*M_PI/(5000.0);  // reversed
    float angle3 = -1.0*angle_counter3*M_PI/(5000.0);  // reversed

    // Wire.beginTransmission(MPU_ADDR);
    // Wire.write(MPU_ACCEL_XOUT_H);  // starting with register 0x3B (ACCEL_XOUT_H)
    // Wire.endTransmission(false);
    // Wire.requestFrom(MPU_ADDR,14,true);  // request a total of 14 registers
    // ax=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
    // ay=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
    // az=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
    // temp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
    // gx=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
    // gy=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
    // gz=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

    size_t out_len = pack_out_msg(out_buf,
      angle1, angle2, angle3,
      accel_scale*ax, -accel_scale*ay, -accel_scale*az,
      gyro_scale*(gx - gx_offset), -gyro_scale*(gy - gy_offset), -gyro_scale*(gz - gz_offset));
    Serial.write(out_buf, out_len);
  }

  if (motor_1_speed < 0)
  {
    Motor1.write(0);
    Motor2.write(0);
  }
  else
  {
    Motor1.write(motor_1_speed);
    Motor2.write(motor_2_speed);
  }
}

//==================================================================
// handle received serial data
//==================================================================
void serialEvent()
{
  while (Serial.available())
  {
    uint8_t in_byte = (uint8_t) Serial.read();
    if (parse_in_byte(in_byte))
    {
      float v1, v2;
      unpack_in_payload(in_payload_buf, &v1, &v2);
      handle_in_msg(v1, v2);
    }
  }
}

//==================================================================
// handle received message
//==================================================================
void handle_in_msg(float v1, float v2)
{
  time_of_last_command = millis();
  motor_1_speed = v1;
  motor_2_speed = v2;
}

//==================================================================
// unpack incoming message payload
//==================================================================
void unpack_in_payload(uint8_t buf[], float *v1, float *v2)
{
  memcpy(v1, buf,     4);
  memcpy(v2, buf + 4, 4);
}

//==================================================================
// handle an incoming byte
//==================================================================
bool parse_in_byte(uint8_t c)
{
  bool got_message = false;
  switch (parse_state)
  {
  case PARSE_STATE_IDLE:
    if (c == IN_START_BYTE)
    {
      in_crc_value = CRC_INITIAL_VALUE;
      in_crc_value = _crc8_ccitt_update(in_crc_value, c);

      in_payload_index = 0;
      parse_state = PARSE_STATE_GOT_START_BYTE;
    }
    break;

  case PARSE_STATE_GOT_START_BYTE:
    in_crc_value = _crc8_ccitt_update(in_crc_value, c);
    in_payload_buf[in_payload_index++] = c;
    if (in_payload_index == IN_PAYLOAD_LENGTH)
    {
      parse_state = PARSE_STATE_GOT_PAYLOAD;
    }
    break;

  case PARSE_STATE_GOT_PAYLOAD:
    if (c == in_crc_value)
    {
      got_message = true;
    }
    parse_state = PARSE_STATE_IDLE;
    break;
  }

  return got_message;
}

//==================================================================
// pack up outgoing message
//==================================================================
size_t pack_out_msg(uint8_t buf[OUT_MESSAGE_LENGTH],
  float roll, float pitch, float yaw,
  float accel_x, float accel_y, float accel_z,
  float gyro_x, float gyro_y, float gyro_z)
{
  buf[0] = OUT_START_BYTE;
  memcpy(buf + 1,  &roll,    4);
  memcpy(buf + 5,  &pitch,   4);
  memcpy(buf + 9,  &yaw,     4);
  memcpy(buf + 13, &accel_x, 4);
  memcpy(buf + 17, &accel_y, 4);
  memcpy(buf + 21, &accel_z, 4);
  memcpy(buf + 25, &gyro_x,  4);
  memcpy(buf + 29, &gyro_y,  4);
  memcpy(buf + 33, &gyro_z,  4);

  uint8_t crc_value = CRC_INITIAL_VALUE;
  for (int i = 0; i < OUT_MESSAGE_LENGTH - CRC_LENGTH; i++)
  {
    crc_value = _crc8_ccitt_update(crc_value, buf[i]);
  }
  buf[OUT_MESSAGE_LENGTH - 1] = crc_value;

  return OUT_MESSAGE_LENGTH;
}
