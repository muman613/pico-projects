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
#include "queue.h"
#include "semphr.h"
#include "task.h"


/* Library includes. */
#include "hardware/i2c.h"
#include "pico/cyw43_arch.h"
#include "pico/stdlib.h"

#include <stdio.h>

#define BUTTON_PIN 20 // GPIO pin number for button

TaskHandle_t blink_task_handle = NULL, button_task_handle = NULL;

// volatile QueueHandle_t queue = NULL;
const TickType_t ms_delay = 500 / portTICK_PERIOD_MS;

// Function declaration for the ISR handler
void gpio_callback(uint gpio, uint32_t events);

void picoWLEDState(bool state) {
  cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, state);
}

/*
 * Configure the hardware as necessary to run this demo.
 */
static void prvSetupHardware(void);

void led_blink_task(void *unused) {
  // Store the Pico LED state
  uint8_t pico_led_state = false;

  while (true) {
    picoWLEDState(pico_led_state);
    pico_led_state = !pico_led_state;
    vTaskDelay(ms_delay);
  }
}

void button_task(void *arg) {
  while (true) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // Wait for notification
    // gpio_put(LED_PIN, !gpio_get(LED_PIN));   // Toggle LED
    printf("Button pressed\n");
    vTaskDelay(250 / portTICK_PERIOD_MS); // Debounce delay
  }
}

int main(void) {

  /* Configure the hardware ready to run the demo. */
  prvSetupHardware();

  // TODO create tasks

  if (xTaskCreate(led_blink_task, "blink task", 128, NULL, 1,
                  &blink_task_handle) != pdPASS) {
    printf("Unable to create task!\n");
    for (;;)
      ;
  }

  if (xTaskCreate(button_task, "blink task", 128, NULL, 1,
                  &button_task_handle) != pdPASS) {
    printf("Unable to create task!\n");
    for (;;)
      ;
  }

  /* Start the tasks and timer running. */
  vTaskStartScheduler();

  for (;;)
    ;

  return 0;
}

/*-----------------------------------------------------------*/

static void prvSetupHardware(void) {
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

// ISR for handling button press
void gpio_callback(uint gpio, uint32_t events) {
  if (gpio == BUTTON_PIN) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(button_task_handle, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  }
}
