/*
 * MMA8451Q DRIVER
 * 2023 Henry Franks
 * 
 * Be excellent to each other...
 */

typedef struct {
    int16_t x, y, z;
    bool error;
} devMMA8451Q_accel_reading_t;

extern int angle_buffer[10];
extern size_t wt_idx;
extern bool angle_buffer_full;

WarpStatus
devMMA8451Q_init(
    const uint8_t i2cAddress, uint16_t operatingVoltageMillivolts);

devMMA8451Q_accel_reading_t
devMMA8451Q_getAccel(void);

/* TODO: offset correction */
