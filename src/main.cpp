#include <includes.h>

RegisterOut outRegister(REGISTER_OUT_DATA, REGISTER_OUT_CLK, REGISTER_OUT_LATCH);
RegisterIn  inRegister(REGISTER_IN_DATA, REGISTER_IN_CLK, REGISTER_IN_LATCH, REGISTER_IN_LOAD);
ADCManager  adcManager;
Debug       debug;
bool        sserial_timeoutFlag = true;

#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include <stdio.h>
#include <string.h>

static const char *TAG = "uart_events";

#define EX_UART_NUM     UART_NUM_0
#define PATTERN_CHR_NUM (3) /*!< Set the number of consecutive and identical characters received by receiver which defines a UART pattern*/

#define BUF_SIZE    (1024)
#define RD_BUF_SIZE (BUF_SIZE)
static QueueHandle_t uart0_queue;

void setup() {
    pinMode(I2C_OCS2_SDA, INPUT);
    pinMode(I2C_OCS2_SCL, INPUT);
    // Serial for debugging - too much output interferes with the smart serial communication
    // since it needs to be very fast and reliable. So, use debug output wisely.
    Serial.begin(115200);

    // Start serial instance for smart serial communication
    // Baudrate is 2.5MBit/s
    Serial1.begin(2500000, SERIAL_8N1, SSERIAL_RXD, SSERIAL_TXD);

    debug.init();
    inRegister.init();
    outRegister.init();
    adcManager.init();

    delay(2000);

    sserial_init();

    // Space for testing

    // /* Configure parameters of an UART driver,
    //  * communication pins and install the driver */
    // uart_config_t uart_config = {
    //     .baud_rate  = 115200,
    //     .data_bits  = UART_DATA_8_BITS,
    //     .parity     = UART_PARITY_DISABLE,
    //     .stop_bits  = UART_STOP_BITS_1,
    //     .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
    //     .source_clk = UART_SCLK_APB,
    // };
    // // Install UART driver, and get the queue.
    // uart_driver_install(EX_UART_NUM, BUF_SIZE * 2, BUF_SIZE * 2, 20, &uart0_queue, 0);
    // uart_param_config(EX_UART_NUM, &uart_config);
}

void loop() {
    // print counters in one line
    // Serial.printf("Task 1: %d, Task 2: %d\n", task1Counter, task2Counter);
    // delay(1000);

    // Print the value of counter every 1000ms
    // Serial.printf("Counter: %d\n", timerCounter);
    // delay(1000);
}
