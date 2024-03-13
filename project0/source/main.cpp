/*
 * FreeRTOS V202107.00
 * Copyright (C) 2020 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * http://www.FreeRTOS.org
 * http://aws.amazon.com/freertos
 *
 * 1 tab == 4 spaces!
 */

/* Scheduler include files. */
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

/* Library includes. */
#include "hardware/i2c.h"
#include "pico/stdlib.h"
#include <stdio.h>

#define RED_LED_PIN 2

TaskHandle_t blink_task_handle = NULL;
TaskHandle_t blink_gpio_task_handle = NULL;
TaskHandle_t i2c_bus_task_handle = NULL;

volatile QueueHandle_t queue = NULL;
const TickType_t ms_delay = 500 / portTICK_PERIOD_MS;

/*
 * Configure the hardware as necessary to run this demo.
 */
static void prvSetupHardware(void);

void led_blink_task(void *unused) {
  // Store the Pico LED state
  uint8_t pico_led_state = 0;

  // Configure the Pico's on-board LED
  // gpio_init(PICO_DEFAULT_LED_PIN);
  // gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

  while (true) {
    // Turn Pico LED on an add the LED state
    // to the FreeRTOS xQUEUE
    // log_debug("PICO LED FLASH");
    pico_led_state = 1;
    gpio_put(PICO_DEFAULT_LED_PIN, pico_led_state);
    xQueueSendToBack(queue, &pico_led_state, 0);
    vTaskDelay(ms_delay);

    // Turn Pico LED off an add the LED state
    // to the FreeRTOS xQUEUE
    pico_led_state = 0;
    gpio_put(PICO_DEFAULT_LED_PIN, pico_led_state);
    xQueueSendToBack(queue, &pico_led_state, 0);
    vTaskDelay(ms_delay);
  }
}

void led_task_gpio(void *unused_arg) {

  // This variable will take a copy of the value
  // added to the FreeRTOS xQueue
  uint8_t passed_value_buffer = 0;

  while (true) {
    // Check for an item in the FreeRTOS xQueue
    if (xQueueReceive(queue, &passed_value_buffer, portMAX_DELAY) == pdPASS) {
      // Received a value so flash the GPIO LED accordingly
      // (NOT the sent value)
      // if (passed_value_buffer)
      //   log_debug("GPIO LED FLASH");
      gpio_put(RED_LED_PIN, passed_value_buffer == 1 ? 0 : 1);

      printf("OK\n");
    }
  }
}

// I2C reserves some addresses for special purposes. We exclude these from the
// scan. These are any addresses of the form 000 0xxx or 111 1xxx
bool reserved_addr(uint8_t addr) {
  return (addr & 0x78) == 0 || (addr & 0x78) == 0x78;
}

void i2c_bus_scan(void *unused_arg) {
  while (true) {
    printf("\nI2C Bus Scan\n");
    printf("   0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F\n");

    for (int addr = 0; addr < (1 << 7); ++addr) {
      if (addr % 16 == 0) {
        printf("%02x ", addr);
      }

      // Perform a 1-byte dummy read from the probe address. If a slave
      // acknowledges this address, the function returns the number of bytes
      // transferred. If the address byte is ignored, the function returns
      // -1.

      // Skip over any reserved addresses.
      int ret;
      uint8_t rxdata;
      if (reserved_addr(addr))
        ret = PICO_ERROR_GENERIC;
      else
        ret = i2c_read_blocking(i2c_default, addr, &rxdata, 1, false);

      printf(ret < 0 ? "." : "@");
      printf(addr % 16 == 15 ? "\n" : "  ");
    }

    vTaskDelay(2000);
  }
}

int main(void) {
  /* Configure the hardware ready to run the demo. */
  prvSetupHardware();

  // Set up the event queue
  queue = xQueueCreate(4, sizeof(uint8_t));

  // TODO create tasks

  auto blink_status = xTaskCreate(led_blink_task, "blink task", 128, NULL, 1,
                                  &blink_task_handle);

  auto blink_gpio_status = xTaskCreate(led_task_gpio, "gpio task", 128, NULL, 1,
                                       &blink_gpio_task_handle);

  auto i2c_scan_status = xTaskCreate(i2c_bus_scan, "i2c bus scan", 256, NULL, 1,
                                     &i2c_bus_task_handle);

  /* Start the tasks and timer running. */
  vTaskStartScheduler();

  for (;;)
    ;

  return 0;
}

/*-----------------------------------------------------------*/

static void prvSetupHardware(void) {
  stdio_init_all();
  gpio_init(PICO_DEFAULT_LED_PIN);
  gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
  gpio_put(PICO_DEFAULT_LED_PIN, 0);

  // Configure the GPIO LED
  gpio_init(RED_LED_PIN);
  gpio_set_dir(RED_LED_PIN, GPIO_OUT);
  gpio_put(RED_LED_PIN, 0);

  i2c_init(i2c0, 100000);
  gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
  gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
  gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
  gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);

  // uart_init(uart0, 115200);
  // gpio_set_function(PICO_DEFAULT_UART_RX_PIN, GPIO_FUNC_UART);
  // gpio_set_function(PICO_DEFAULT_UART_TX_PIN, GPIO_FUNC_UART);
}
/*-----------------------------------------------------------*/

void vApplicationMallocFailedHook(void) {
  /* Called if a call to pvPortMalloc() fails because there is insufficient
  free memory available in the FreeRTOS heap.  pvPortMalloc() is called
  internally by FreeRTOS API functions that create tasks, queues, software
  timers, and semaphores.  The size of the FreeRTOS heap is set by the
  configTOTAL_HEAP_SIZE configuration constant in FreeRTOSConfig.h. */

  /* Force an assert. */
  configASSERT((volatile void *)NULL);
}
/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook(TaskHandle_t pxTask, char *pcTaskName) {
  (void)pcTaskName;
  (void)pxTask;

  /* Run time stack overflow checking is performed if
  configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
  function is called if a stack overflow is detected. */

  /* Force an assert. */
  configASSERT((volatile void *)NULL);
}
/*-----------------------------------------------------------*/

void vApplicationIdleHook(void) {
  volatile size_t xFreeHeapSpace;

  /* This is just a trivial example of an idle hook.  It is called on each
  cycle of the idle task.  It must *NOT* attempt to block.  In this case the
  idle task just queries the amount of FreeRTOS heap that remains.  See the
  memory management section on the http://www.FreeRTOS.org web site for memory
  management options.  If there is a lot of heap memory free then the
  configTOTAL_HEAP_SIZE value in FreeRTOSConfig.h can be reduced to free up
  RAM. */
  xFreeHeapSpace = xPortGetFreeHeapSize();

  /* Remove compiler warning about xFreeHeapSpace being set but never used. */
  (void)xFreeHeapSpace;
}
