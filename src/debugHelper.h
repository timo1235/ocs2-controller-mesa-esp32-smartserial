#pragma once

#include <Arduino.h>

#define DEBUG_MSG_SIZE  128
#define DEBUG_QUEUE_LEN 20

class Debug {
  public:
    Debug();
    void init();

    // Thread-safe: adds formatted message to print queue (non-blocking, drops if full)
    void print(const char *format, ...);

    // Alias for backwards compatibility
    void addPrint(const char *format, ...);

  private:
    static void   debugSerialTask(void *pvParameters);
    QueueHandle_t _queue;
};
