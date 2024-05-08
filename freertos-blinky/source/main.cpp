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
#include "timers.h"

/* Library includes. */
#include "hardware/i2c.h"
#include "pico/cyw43_arch.h"
#include "pico/stdlib.h"

#include <stdio.h>

#include "pwm-utils.h"

#define WIFI_TASK 1
#define SSID "softwaremagic"
#define PASSWORD "realmagic"

#define BUTTON_PIN 20 // GPIO pin number for button
#define BUZZER_PIN 18

TaskHandle_t blink_task_handle = NULL, button_task_handle = NULL, wifi_task_handle = NULL;
QueueHandle_t event_queue = NULL;
TimerHandle_t event_timer = NULL;

struct event {
    uint8_t eventCode;
    uint8_t parameter1; // X
    uint8_t parameter2; // Y
};

#define EVENT_BUTTONDOWN 1
#define EVENT_BUTTONUP 2
#define EVENT_TIMER 3

// volatile QueueHandle_t queue = NULL;
TickType_t ms_delay = 500 / portTICK_PERIOD_MS;

// Function declaration for the ISR handler
void gpio_callback(uint gpio, uint32_t events);
void event_timer_callback(TimerHandle_t xTimer);

void picoWLEDState(bool state) { cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, state); }

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
        static TickType_t buttonDownTicks = 0;
        static TickType_t buttonUpTicks = 0;

        event ev;
        if (xQueueReceive(event_queue, &ev, portMAX_DELAY) == pdPASS) {
            if (ev.eventCode == EVENT_BUTTONDOWN) {
                buttonDownTicks = xTaskGetTickCount();
                printf("Button down\n");
                ms_delay = 100 / portTICK_PERIOD_MS;
            } else if (ev.eventCode == EVENT_BUTTONUP) {
                buttonUpTicks = xTaskGetTickCount();
                printf("Button up : ticks = %f\n", (double)(buttonUpTicks - buttonDownTicks) / 1000);
                ms_delay = 500 / portTICK_PERIOD_MS;
            } else if (ev.eventCode == EVENT_TIMER) {
                // printf("*");
                // fflush(stdout);
            }
        }
        // vTaskDelay(250 / portTICK_PERIOD_MS); // Debounce delay
    }
}

#ifdef WIFI_TASK
void wifi_task(void *arg) {
    vTaskDelay(pdMS_TO_TICKS(5000));
    printf("Connecting...\n");
    if (cyw43_arch_wifi_connect_async(SSID, PASSWORD, CYW43_AUTH_WPA2_AES_PSK) != 0) {
        printf("Connection failed, retrying...\n");
    }

    int status = 0;

    while (true) {
        // Check WiFi status and connect if not connected

        while (status != CYW43_LINK_UP) {
            status = cyw43_tcpip_link_status(&cyw43_state, CYW43_ITF_STA);
            printf("%d", status);
            fflush(stdout);
            // Delay for a period before rechecking connection status
            if (status != CYW43_LINK_UP)
                vTaskDelay(pdMS_TO_TICKS(100));
        }
        printf("Connected!\n");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
#endif

int main(void) {

    /* Configure the hardware ready to run the demo. */
    prvSetupHardware();

    // TODO create tasks

    event_queue = xQueueCreate(4, sizeof(event));
    if (event_queue == NULL) {
        printf("Unable to create queue\n");
        for (;;)
            ;
    }

    if (xTaskCreate(led_blink_task, "blink task", 128, NULL, 1, &blink_task_handle) != pdPASS) {
        printf("Unable to create task!\n");
        for (;;)
            ;
    }

    if (xTaskCreate(button_task, "button task", 256, NULL, 1, &button_task_handle) != pdPASS) {
        printf("Unable to create button task!\n");
        for (;;)
            ;
    }

#ifdef WIFI_TASK
    if (xTaskCreate(wifi_task, "wifi task", 512, NULL, 1, &wifi_task_handle) != pdPASS) {
        printf("Unable to create wifi task!\n");
        for (;;)
            ;
    }
#endif

    if ((event_timer = xTimerCreate("event timer", 10, pdTRUE, (void *)0, event_timer_callback)) == NULL) {
        printf("Unable to create timer!\n");
        for (;;)
            ;
    }

    xTimerStart(event_timer, 0);

    // for (int i = 0; i < 400; i++) {
    //     gpio_put(BUZZER_PIN, true);
    //     sleep_ms(2);
    //     gpio_put(BUZZER_PIN, false);
    //     sleep_ms(2);
    // }

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

    cyw43_arch_enable_sta_mode();
    cyw43_wifi_set_up(&cyw43_state, CYW43_ITF_STA, true, CYW43_COUNTRY_USA);

    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);

    // Initialize BUTTON_PIN as input
    gpio_init(BUTTON_PIN);
    gpio_set_dir(BUTTON_PIN, GPIO_IN);
    gpio_pull_up(BUTTON_PIN); // Enable internal pull-up

    gpio_init(BUZZER_PIN);
    gpio_set_dir(BUZZER_PIN, GPIO_OUT);

    // Set up the interrupt handler for the specified pin
    gpio_set_irq_enabled_with_callback(BUTTON_PIN, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, true, &gpio_callback);
}

/*-----------------------------------------------------------*/

extern "C" void vApplicationMallocFailedHook(void) {
    /* Called if a call to pvPortMalloc() fails because there is insufficient
    free memory available in the FreeRTOS heap.  pvPortMalloc() is called
    internally by FreeRTOS API functions that create tasks, queues, software
    timers, and semaphores.  The size of the FreeRTOS heap is set by the
    configTOTAL_HEAP_SIZE configuration constant in FreeRTOSConfig.h. */

    /* Force an assert. */
    configASSERT((volatile void *)NULL);
}
/*-----------------------------------------------------------*/

extern "C" void vApplicationStackOverflowHook(TaskHandle_t pxTask, char *pcTaskName) {
    (void)pcTaskName;
    (void)pxTask;

    /* Run time stack overflow checking is performed if
    configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
    function is called if a stack overflow is detected. */

    while (true) {
        picoWLEDState(false);
        sleep_ms(2000);
        picoWLEDState(true);
        sleep_ms(2000);
    }
    /* Force an assert. */
    configASSERT((volatile void *)NULL);
}
/*-----------------------------------------------------------*/

extern "C" void vApplicationIdleHook(void) {
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
    static bool firstPress = true;

    if (firstPress) {
        firstPress = false;
        return;
    }

    if (gpio == BUTTON_PIN) {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        // vTaskNotifyGiveFromISR(button_task_handle, &xHigherPriorityTaskWoken);
        // portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        event ev;
        if (events & GPIO_IRQ_EDGE_FALL) {
            ev = {.eventCode = EVENT_BUTTONDOWN, .parameter1 = 0x00, .parameter2 = 0x00};
        } else if (events & GPIO_IRQ_EDGE_RISE) {
            ev = {.eventCode = EVENT_BUTTONUP, .parameter1 = 0x00, .parameter2 = 0x00};
        }

        xQueueSendToBackFromISR(event_queue, &ev, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

void event_timer_callback(TimerHandle_t xTimer) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    event ev = {.eventCode = EVENT_TIMER, .parameter1 = 0, .parameter2 = 0};
    xQueueSendToBackFromISR(event_queue, &ev, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
