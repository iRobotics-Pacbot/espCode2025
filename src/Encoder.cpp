#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "Encoder_test.h"

void testEncoder(void)
{
    // Configuration for PCNT unit
    pcnt_config_t pcnt_config = {
        .pulse_gpio_num = ROTARY_ENCODER_A_GPIO,   // GPIO for A channel
        .ctrl_gpio_num = ROTARY_ENCODER_B_GPIO,    // GPIO for B channel
        .lctrl_mode = PCNT_MODE_KEEP,              // Rising A on HIGH B = CW step
        .hctrl_mode = PCNT_MODE_REVERSE,           // Rising A on LOW B = CCW step
        .pos_mode = PCNT_COUNT_INC,                // Count on rising edge of A
        .neg_mode = PCNT_COUNT_DIS,                // Discard falling edge of A
        .counter_h_lim = PCNT_H_LIM_VAL,
        .counter_l_lim = PCNT_L_LIM_VAL,
        .unit = PCNT_UNIT_NUM,
        .channel = PCNT_CHANNEL_NUM,
    };

    // Initialize PCNT unit
    pcnt_unit_config(&pcnt_config);

    // Enable filter to reduce noise
    pcnt_set_filter_value(PCNT_UNIT_NUM, 250);
    pcnt_filter_enable(PCNT_UNIT_NUM);

    // Clear and resume counter
    pcnt_counter_pause(PCNT_UNIT_NUM);
    pcnt_counter_clear(PCNT_UNIT_NUM);
    pcnt_counter_resume(PCNT_UNIT_NUM);

    int16_t count;
    while (1) {
        // Read the current count
        pcnt_get_counter_value(PCNT_UNIT_NUM, &count);
        ESP_LOGI("encoder", "Count: %d", count);
        
        //printf("Count: %d", count); 

        // Delay and yield
        vTaskDelay(200 / portTICK_PERIOD_MS);
    }
}