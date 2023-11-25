#include <includes.h>

Debug::Debug() {}

void Debug::addPrint(const char *format, ...) {
    char    buf[256];   // Puffer für die formatierte Zeichenkette
    va_list args;
    va_start(args, format);
    vsnprintf(buf, sizeof(buf), format, args);
    va_end(args);

    if (xSemaphoreTake(_queueSemaphore, (TickType_t) 0) == pdTRUE) {
        char tempBuf[256];
        while (uxQueueSpacesAvailable(_printQueue) == 0) {
            xQueueReceive(_printQueue, &tempBuf, 0);   // remove oldest element safely
        }
        xQueueSend(_printQueue, &buf, 0);
        xSemaphoreGive(_queueSemaphore);
    }
}

void Debug::print(const char *format, ...) {
    char    buf[256];   // Puffer für die formatierte Zeichenkette
    va_list args;
    va_start(args, format);
    vsnprintf(buf, sizeof(buf), format, args);
    va_end(args);

    if (xSemaphoreTake(_directQueueSemaphore, (TickType_t) 0) == pdTRUE) {
        char tempBuf[256];
        while (uxQueueSpacesAvailable(_printQueueDirekt) == 0) {
            xQueueReceive(_printQueueDirekt, &tempBuf, 0);
        }
        xQueueSend(_printQueueDirekt, &buf, 0);
        xSemaphoreGive(_directQueueSemaphore);
    }
}

void Debug::printQueue() {
    char element[256];
    while (uxQueueMessagesWaiting(_printQueue) > 0) {
        xQueueReceive(_printQueue, &element, 0);
        Serial.println(element);
    }
}

void Debug::debugSerialTask(void *pvParameters) {
    Debug *debug = static_cast<Debug *>(pvParameters);
    char   element[256];
    for (;;) {
        if (xSemaphoreTake(debug->_directQueueSemaphore, portMAX_DELAY)) {
            if (uxQueueMessagesWaiting(debug->_printQueueDirekt) > 0) {
                xQueueReceive(debug->_printQueueDirekt, &element, 0);
                Serial.println(element);
            }
            xSemaphoreGive(debug->_directQueueSemaphore);
        }
        vTaskDelay(1);
    }
}

void Debug::init() {
    _printQueue       = xQueueCreate(100, sizeof(char[256]));
    _printQueueDirekt = xQueueCreate(100, sizeof(char[256]));

    _queueSemaphore       = xSemaphoreCreateMutex();   // Initialisieren des Semaphors für _printQueue
    _directQueueSemaphore = xSemaphoreCreateMutex();   // Initialisieren des Semaphors für _printQueueDirekt

    // Create a task for the debug serial communication
    xTaskCreatePinnedToCore(debugSerialTask,        /* Task function. */
                            "debugHelperQueueTask", /* name of task. */
                            3000,                   /* Stack size of task */
                            this,                   /* parameter of the task */
                            1,                      /* priority of the task */
                            NULL,                   /* Task handle to keep track of created task */
                            DEFAULT_CPU);
}
