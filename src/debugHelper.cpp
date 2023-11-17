#include <Arduino.h>
#include <debugHelper.h>

QueueHandle_t _printQueue;
QueueHandle_t _printQueueDirekt;
TaskHandle_t  debugSerialTaskHandle;

void addPrintToQueue(const char *format, ...) {
    char    buf[256];   // Puffer für die formatierte Zeichenkette
    va_list args;
    va_start(args, format);
    vsnprintf(buf, sizeof(buf), format, args);
    va_end(args);

    // Free space in queue if it is full
    while (uxQueueSpacesAvailable(_printQueue) == 0) {
        xQueueReceive(_printQueue, NULL, 0);   // remove oldest element
    }

    // Add element to queue
    xQueueSend(_printQueue, &buf, 0);
}

void addPrint(const char *format, ...) {
    char    buf[256];   // Puffer für die formatierte Zeichenkette
    va_list args;
    va_start(args, format);
    vsnprintf(buf, sizeof(buf), format, args);
    va_end(args);

    // Free space in queue if it is full
    while (uxQueueSpacesAvailable(_printQueueDirekt) == 0) {
        xQueueReceive(_printQueueDirekt, NULL, 0);   // remove oldest element
    }

    // Add element to queue
    xQueueSend(_printQueueDirekt, &buf, 0);
}

void printQueue() {
    char element[256];
    while (uxQueueMessagesWaiting(_printQueue) > 0) {
        xQueueReceive(_printQueue, &element, 0);
        Serial.println(element);
    }
}

void debugSerialTask(void *pvParameters) {
    // auto *ioControl = (IOCONTROL *)pvParameters;
    char element[256];
    for (;;) {
        // Check if queue has elements and process
        if (uxQueueMessagesWaiting(_printQueueDirekt) > 0) {
            xQueueReceive(_printQueueDirekt, &element, 0);
            Serial.println(element);
            vTaskDelay(1);
        }
        vTaskDelay(1);
    }
}

void debugHelperSetup() {
    _printQueue       = xQueueCreate(100, sizeof(char[256]));
    _printQueueDirekt = xQueueCreate(100, sizeof(char[256]));

    // Create a task for the debug serial communication
    xTaskCreatePinnedToCore(debugSerialTask,        /* Task function. */
                            "debugHelperQueueTask", /* name of task. */
                            3000,                   /* Stack size of task */
                            NULL,                   /* parameter of the task */
                            1,                      /* priority of the task */
                            &debugSerialTaskHandle, /* Task handle to keep track of created task */
                            1);
}
