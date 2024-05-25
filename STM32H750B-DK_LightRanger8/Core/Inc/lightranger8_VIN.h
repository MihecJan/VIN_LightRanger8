// Descriptions are a guess!


#define LIGHTRANGER8_SOFT_RESET                                         0x0000

// If it holds a certain value then a newly measured value is ready to be read.
#define LIGHTRANGER8_GPIO_TIO_HV_STATUS                                 0x0031

// Controls the period between consecutive measurements
#define LIGHTRANGER8_SYSTEM_INTERMEASUREMENT_PERIOD                     0x006C

// Write a certain value into this register to clear the interrupt.
#define LIGHTRANGER8_SYSTEM_INTERRUPT_CLEAR                             0x0086

// Disable or enable ranging mode
#define LIGHTRANGER8_SYSTEM_MODE_START                                  0x0087

// Holds the measured distance
#define LIGHTRANGER8_RESULT_FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0      0x0096

// Holds a value related to the calibration of the oscillator
#define LIGHTRANGER8_RESULT_OSC_CALIBRATE_VAL                           0x00DE

// Holds model ID of the sensor
#define LIGHTRANGER8_IDENTIFICATION_MODEL_ID                            0x010F

// These registers configure the LightRanger8 sensor's distance measurement settings (SHORT, MEDIUM, LONG).
#define LIGHTRANGER8_RANGE_CONFIG_VCSEL_PERIOD_A                        0x0060
#define LIGHTRANGER8_RANGE_CONFIG_VCSEL_PERIOD_B						0x0063
#define LIGHTRANGER8_RANGE_CONFIG_VALID_PHASE_HIGH                      0x0069
#define LIGHTRANGER8_SD_CONFIG_WOI_SD0                                  0x0078
#define LIGHTRANGER8_SD_CONFIG_WOI_SD1                                  0x0079
#define LIGHTRANGER8_SD_CONFIG_INITIAL_PHASE_SD0                        0x007A
#define LIGHTRANGER8_SD_CONFIG_INITIAL_PHASE_SD1                        0x007B

// Calibration
// Likely a configuration register where a range offset value in millimeters is stored.
// Offset is probably used by the sensor's algorithm to correct for variations
// Probably just adds the offset to the measured distance
#define LIGHTRANGER8_ALGO_PART_TO_PART_RANGE_OFFSET_MM                  0x001E

// Maybe for accuracy across different parts of its range
// and errors that occur at different distances.
// NOT USED in our program, we only reset them on the start.
#define LIGHTRANGER8_MM_CONFIG_INNER_OFFSET_MM                          0x0020
#define LIGHTRANGER8_MM_CONFIG_OUTER_OFFSET_MM                          0x0022

// Definicije kopirane iz knjižnice, lightranger8.h
/**
 * @brief Checks for communication with the LightRanger8 Click.
 *
 * This function continuously attempts to establish communication with the LightRanger8 Click
 * via I2C until an acknowledgment (ACK) is received. It transmits a status messages
 * indicating the current state of the communication process.
 */
void checkForLightRangerCommunication ( void );

/**
 * @brief Checks the model ID of the LightRanger8 Click to ensure it matches the expected ID.
 *
 * This function reads the model ID from the LightRanger8 Click and compares it with the expected ID.
 */
void checkModelID ( void );

/**
 * @brief LightRanger 8 software reset function.
 *
 * This function performs a software reset of the VL53Lx ranging sensor.
 */
void lightranger8_software_reset ( void );

/**
 * @brief LightRanger 8 set distance mode function.
 *
 * This function sets the distance mode of the VL53Lx
 * ranging sensor with three possible modes.
 */
void lightranger8_set_distance_mode ( uint8_t );

/**
 * @brief LightRanger 8 start measurement function.
 *
 * This function enables the range measuring with the adjusted
 * intermeasurement period.
 */
void lightranger8_start_measurement( uint32_t );

/**
 * @brief LightRanger 8 stop measurement function.
 *
 * This function stops the measurement of the VL53Lx ranging sensor.
 */
void lightranger8_stop_measurement ( void );

/**
 * @brief LightRanger 8 data ready function.
 *
 * This function returns the information if new data is ready
 * for acquiring the distance measured.
 */
uint8_t lightranger8_new_data_ready ( void );

/**
 * @brief LightRanger 8 get distance function.
 *
 * This function returns the corrected measured distance
 * from the VL53Lx ranging sensor described in milimeters.
 */
uint16_t lightranger8_get_distance ( void );

/**
 * @brief LightRanger 8 calibrate offset function.
 *
 * This function calibrates the offset which affects the final
 * corrected measurement from the VL53Lx ranging sensor. The averaging
 * is concluded 50 times before the actual measurement sets in place.
 * The target distance value is in milimeters. Period should be set at
 * the same value as inteded measurement later on.
 */
void lightranger8_calibrate_offset ( uint16_t, uint32_t );

/**
 * @brief LightRanger 8 get int pin state function.
 *
 * This function checks the interrupt pin state.
 */
uint8_t lightranger8_get_interrupt_state ( void );



