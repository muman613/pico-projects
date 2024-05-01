#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "pico/cyw43_arch.h"
#include "pico/stdlib.h"

#define BUTTON_PIN 20 // GPIO pin number for button

// Function declaration for the ISR handler
void gpio_callback(uint gpio, uint32_t events);

int main() {
  // Initialize I/O
  stdio_init_all();
  cyw43_arch_init();
  cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);

  // Initialize BUTTON_PIN as input
  gpio_init(BUTTON_PIN);
  gpio_set_dir(BUTTON_PIN, GPIO_IN);
  gpio_pull_up(BUTTON_PIN); // Enable internal pull-up

  // Set up the interrupt handler for the specified pin
  gpio_set_irq_enabled_with_callback(BUTTON_PIN, GPIO_IRQ_EDGE_FALL, true,
                                     &gpio_callback);

  while (true) {
    // Main loop can be used to perform other tasks
    tight_loop_contents();
  }

  return 0;
}

// ISR for handling button press
void gpio_callback(uint gpio, uint32_t events) {
  if (gpio == BUTTON_PIN) {
    // Toggle the LED state on CYW43 chip
    static bool led_state = false;
    led_state = !led_state;
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, led_state);
  }
}