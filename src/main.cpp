#include <includes.h>

RegisterOut outRegister(REGISTER_OUT_DATA, REGISTER_OUT_CLK, REGISTER_OUT_LATCH);
RegisterIn  inRegister(REGISTER_IN_DATA, REGISTER_IN_CLK, REGISTER_IN_LATCH, REGISTER_IN_LOAD);
ADCManager  adcManager;
Debug       debug;
bool        sserial_timeoutFlag = true;

void taskSetupDemo();

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
    // outRegister.init();
    // adcManager.init();

    // outRegister.set(RegisterOut::Output::ena, true);

    // taskSetupDemo();
    delay(5000);

    sserial_init();
}

uint32_t lastTask1Message = 0;
uint32_t task1Counter     = 0;
void     task0(void *pvParameters) {
    for (;;) {

        task1Counter++;
        if (millis() - lastTask1Message > 1000) {
            Serial.printf("Task 1: %d, CPU-Core: %d\n", task1Counter, xPortGetCoreID());
            lastTask1Message = millis();
            task1Counter     = 0;
        }
        // yield();
        // Serial.println("Task 1");
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
}

uint32_t lastTask2Message = 0;
uint32_t task2Counter     = 0;
void     task1(void *pvParameters) {
    for (;;) {
        task2Counter++;
        if (millis() - lastTask2Message > 1000) {
            // print the task counter and how many times per second it was called
            Serial.printf("Task 2: %d, CPU-Core: %d\n", task2Counter, xPortGetCoreID());

            lastTask2Message = millis();
            task2Counter     = 0;
        }
        // ohne alles: 759380 ausführungen pro sekunde
        // yield();   //-> nur yield: 203450 ausführungen pro sekunde
        // Serial.println("Task 2");
        // vTaskDelay(1 / portTICK_PERIOD_MS);   // 1000 ausführungen pro sekunde
    }
}

// Beide tasks gleiche prio: jeweils 250000 ausführungen pro sekunde

void taskSetupDemo() {
    xTaskCreatePinnedToCore(task0,   /* Task function. */
                            "Task0", /* name of task. */
                            10000,   /* Stack size of task */
                            NULL,    /* parameter of the task */
                            1,       /* priority of the task */
                            NULL,    /* Task handle to keep track of created task */
                            0);

    xTaskCreatePinnedToCore(task1,   /* Task function. */
                            "Task1", /* name of task. */
                            10000,   /* Stack size of task */
                            NULL,    /* parameter of the task */
                            20,      /* priority of the task */
                            NULL,    /* Task handle to keep track of created task */
                            1);
}

void loop() {
    // print counters in one line
    // Serial.printf("Task 1: %d, Task 2: %d\n", task1Counter, task2Counter);
    // delay(1000);
}
