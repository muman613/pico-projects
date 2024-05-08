#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/pwm.h"
#include "pico/cyw43_arch.h"
#include "pico/stdlib.h"
#include "pwm-utils.h"

#define BUZZER_PIN 18 // GPIO pin number for button

// Function declaration for the ISR handler
void gpio_callback(uint gpio, uint32_t events);
void prvInitHardware();

uint slice_num = 0;
uint chan18 = 0;

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
