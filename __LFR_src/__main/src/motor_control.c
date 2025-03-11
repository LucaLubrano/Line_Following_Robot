#include <stdio.h>
#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include "pin_config.h"
#include "macros.h"

void motor_speed_set(float duty_cycle);

void motor_speed_set(float duty_cycle){
  OCR0A = duty_cycle * 255; // compare register 
}