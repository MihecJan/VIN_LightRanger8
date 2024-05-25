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

// Components.
VL53LX sensor_vl53lx_sat(&DEV_I2C, SHUTDOWN_PIN);

void setup() {

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

    // Keep polling until data is ready
    do {
        /*
            on success: status = VL53LX_Error_NONE
            else:       status = "Other error code"

            NewDataReady:
                0 = data not ready,
                1 = data ready
        */
        status =
            sensor_vl53lx_sat.VL53LX_GetMeasurementDataReady(&NewDataReady);

    } while (!NewDataReady);

    if ((!status) && (NewDataReady != 0)) {
        // Data is ready...

        // pMultiRangingData: Pointer to the data structure to fill up.
        status =
            sensor_vl53lx_sat.VL53LX_GetMultiRangingData(pMultiRangingData);

        no_of_object_found = pMultiRangingData->NumberOfObjectsFound;

        snprintf(report, sizeof(report), "VL53L3CX: #Objs=%1d ",
                 no_of_object_found);
        Serial.print(report);

        // Loop through found objects..
        for (j = 0; j < no_of_object_found; j++) {

            int distInMiliMeters =
                pMultiRangingData->RangeData[j].RangeMilliMeter;

            snprintf(report, sizeof(report), "\n\tD%d=%dmm", (j + 1),
                     distInMiliMeters);
            Serial.print(report);
        }

        Serial.println();
        Serial.println();

        status = sensor_vl53lx_sat.VL53LX_ClearInterruptAndStartMeasurement();
    }
}
