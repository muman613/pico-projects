#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/pwm.h"
#include "pico/cyw43_arch.h"
#include "pico/stdlib.h"

#define BUZZER_PIN 18 // GPIO pin number for button

// Function declaration for the ISR handler
void gpio_callback(uint gpio, uint32_t events);
void prvInitHardware();

uint slice_num = 0;
uint chan18 = 0;

uint32_t pwm_set_freq_duty(uint slice_num, uint chan, uint32_t f, int d) {
    uint32_t clock = 125000000;
    uint32_t divider16 = clock / f / 4096 + (clock % (f * 4096) != 0);
    if (divider16 / 16 == 0)
        divider16 = 16;
    uint32_t wrap = clock * 16 / divider16 / f - 1;
    pwm_set_clkdiv_int_frac(slice_num, divider16 / 16, divider16 & 0xF);
    pwm_set_wrap(slice_num, wrap);
    pwm_set_chan_level(slice_num, chan, wrap * d / 100);
    return wrap;
}
uint32_t pwm_get_wrap(uint slice_num) {
    valid_params_if(PWM, slice_num >= 0 && slice_num < NUM_PWM_SLICES);
    return pwm_hw->slice[slice_num].top;
}

void pwm_set_duty(uint slice_num, uint chan, int d) { pwm_set_chan_level(slice_num, chan, pwm_get_wrap(slice_num) * d / 100); }

int main() {
    // Initialize I/O
    stdio_init_all();
    prvInitHardware();

    uint wrap = pwm_set_freq_duty(slice_num, chan18, 300, 50);

    pwm_set_enabled(slice_num, true);
    while (true) {
        pwm_set_duty(slice_num, chan18, 25);
        sleep_ms(500);
        pwm_set_duty(slice_num, chan18, 50);
        sleep_ms(500);
    }

    while (true) {
        // Main loop can be used to perform other tasks
        tight_loop_contents();
    }

    return 0;
}

void prvInitHardware() {
    cyw43_arch_init();
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);

    // Initialize BUZZER_PIN as input
    gpio_init(BUZZER_PIN);
    gpio_set_function(BUZZER_PIN, GPIO_FUNC_PWM);

    slice_num = pwm_gpio_to_slice_num(BUZZER_PIN);
    chan18 = pwm_gpio_to_channel(BUZZER_PIN);
}
