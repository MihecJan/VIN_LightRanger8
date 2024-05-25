#include <Arduino.h>
#include <Wire.h>
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <vl53lx_class.h>

#define DEV_I2C Wire

#define SHUTDOWN_PIN 17
#define INTERRUPT_PIN 16

VL53LX sensor_vl53lx_sat(&DEV_I2C, SHUTDOWN_PIN);

volatile int interruptCount = 0;

// ISR - Interrupt service routine
void measure() { interruptCount = 1; }

void setup() {

    pinMode(INTERRUPT_PIN, INPUT_PULLUP);

    // FALLING -> for when the pin goes from high to low.
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), measure, FALLING);

    // Initialize serial for output.
    Serial.begin(115200);
    Serial.println("Starting...");

    // Initialize I2C bus.
    DEV_I2C.begin();

    // Configure VL53LX satellite component.
    sensor_vl53lx_sat.begin();

    // Switch off VL53LX satellite component.
    sensor_vl53lx_sat.VL53LX_Off();

    // Initialize VL53LX satellite component.
    sensor_vl53lx_sat.InitSensor(0x12);

    // Start Measurements
    sensor_vl53lx_sat.VL53LX_StartMeasurement();
}

void loop() {
    VL53LX_MultiRangingData_t MultiRangingData;
    VL53LX_MultiRangingData_t *pMultiRangingData = &MultiRangingData;
    uint8_t NewDataReady = 0;
    int no_of_object_found = 0, j;
    char report[64];
    int status;

    if (interruptCount) {

        /*
            on success: status = VL53LX_Error_NONE
            else:       status = "Other error code"

            NewDataReady:
                0 = data not ready,
                1 = data ready
        */
        status =
            sensor_vl53lx_sat.VL53LX_GetMeasurementDataReady(&NewDataReady);

        if ((!status) && (NewDataReady != 0)) {
            // Data is ready...

            // pMultiRangingData: Pointer to the data structure to fill up.
            status =
                sensor_vl53lx_sat.VL53LX_GetMultiRangingData(pMultiRangingData);

            no_of_object_found = pMultiRangingData->NumberOfObjectsFound;

            // Loop through found objects..
            for (j = 0; j < no_of_object_found; j++) {

                int distInMiliMeters =
                    pMultiRangingData->RangeData[j].RangeMilliMeter;


                snprintf(report, sizeof(report), "%d ", distInMiliMeters);
                Serial.print(report);
            }

            Serial.println();

            status =
                sensor_vl53lx_sat.VL53LX_ClearInterruptAndStartMeasurement();
        }
    }
}