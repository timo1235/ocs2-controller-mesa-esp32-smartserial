#include <includes.h>

Debug::Debug() : _queue(NULL) {}

void Debug::print(const char *format, ...) {
    if (!_queue) return;

    char    buf[DEBUG_MSG_SIZE];
    va_list args;
    va_start(args, format);
    vsnprintf(buf, sizeof(buf), format, args);
    va_end(args);

    // Non-blocking send, drops message if queue is full
    xQueueSend(_queue, buf, 0);
}

void Debug::addPrint(const char *format, ...) {
    if (!_queue) return;

    char    buf[DEBUG_MSG_SIZE];
    va_list args;
    va_start(args, format);
    vsnprintf(buf, sizeof(buf), format, args);
    va_end(args);

    xQueueSend(_queue, buf, 0);
}

void Debug::debugSerialTask(void *pvParameters) {
    Debug *self = static_cast<Debug *>(pvParameters);
    char   buf[DEBUG_MSG_SIZE];
    for (;;) {
        if (xQueueReceive(self->_queue, buf, portMAX_DELAY) == pdTRUE) {
            Serial.println(buf);
        }
    }
}

void Debug::init() {
    _queue = xQueueCreate(DEBUG_QUEUE_LEN, DEBUG_MSG_SIZE);
    xTaskCreatePinnedToCore(debugSerialTask, "DebugTask", 3000, this, 1, NULL, DEFAULT_CPU);
}
