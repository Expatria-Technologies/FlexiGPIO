#include <Arduino.h>
#include <Wire.h>
#include <SerialTransfer.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/i2c.h"
#include "hardware/irq.h"
#include "hardware/timer.h"
#include "hardware/gpio.h"
#include "hardware/flash.h"
#include "pico/time.h"
#include "i2c_flexigpio.h"

// RPI Pico

//--------------------------------------------------------------------+
// setup() & loop()
//--------------------------------------------------------------------+
void setup() {
  //Serial.begin(115200);
  //sleep_ms(5);
  //Serial.println("FlexiGPIO I2C Expander 1.0");

  init_i2c_responder();
}

void loop() {
  //Serial.flush();
  i2c_task();
}

