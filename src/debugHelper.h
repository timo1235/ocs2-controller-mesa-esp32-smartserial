#pragma once

class Debug {
  public:
    Debug();
    /**
     * @brief Adds a formatted string to the print queue.
     *
     * This function takes a formatted string and any additional arguments and adds it to the print queue.
     * The string is formatted using the same syntax as printf.
     *
     * @param format The format string.
     * @param ... Additional arguments to be formatted.
     */
    void addPrint(const char *format, ...);

    /**
     * @brief Prints all strings in the print queue.
     *
     * This function prints all strings in the print queue and clears the queue.
     */
    void printQueue();

    /**
     * @brief Prints a formatted string immediately.
     *
     * This function takes a formatted string and any additional arguments and prints it immediately.
     * The string is formatted using the same syntax as printf.
     *
     * @param format The format string.
     * @param ... Additional arguments to be formatted.
     */
    void print(const char *format, ...);

    void init();

  private:
    static void       debugSerialTask(void *pvParameters);
    QueueHandle_t     _printQueue;
    QueueHandle_t     _printQueueDirekt;
    SemaphoreHandle_t _queueSemaphore;         // Semaphore für _printQueue
    SemaphoreHandle_t _directQueueSemaphore;   // Semaphore für _printQueueDirekt
};
