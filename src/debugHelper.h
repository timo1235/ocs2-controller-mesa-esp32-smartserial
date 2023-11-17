#pragma once

/**
 * @brief Adds a formatted string to the print queue.
 *
 * This function takes a formatted string and any additional arguments and adds it to the print queue.
 * The string is formatted using the same syntax as printf.
 *
 * @param format The format string.
 * @param ... Additional arguments to be formatted.
 */
void addPrintToQueue(const char *format, ...);

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
void addPrint(const char *format, ...);
