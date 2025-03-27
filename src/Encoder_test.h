#pragma once

#include "esp_log.h"
#include "driver/pcnt.h"

// Define the GPIO pins for the quadrature encoder
#define PCNT_H_LIM_VAL      INT16_MAX
#define PCNT_L_LIM_VAL      INT16_MIN

// Define the GPIO pins for the quadrature encoder
#define ROTARY_ENCODER_A_GPIO 34
#define ROTARY_ENCODER_B_GPIO 36

// Define the PCNT unit and channel
#define PCNT_UNIT_NUM        PCNT_UNIT_0
#define PCNT_CHANNEL_NUM     PCNT_CHANNEL_0

void testEncoder();