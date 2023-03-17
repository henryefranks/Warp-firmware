/*
 * MMA8451Q DRIVER
 * 2023 Henry Franks
 * 
 * Be excellent to each other...
 */

typedef struct {
    int16_t x, y, z;
    bool error; /* add error flag to readings to simplify logic */
} devMMA8451Q_accel_reading_t;

extern int angle_buffer[10];   /* 10-entry (10s) circular buffer             */
extern size_t wt_idx;          /* current write pointer of circular buffer   */
extern bool angle_buffer_full; /* flag to detect 10s of valid data           */

WarpStatus
devMMA8451Q_init(
    const uint8_t i2cAddress, uint16_t operatingVoltageMillivolts);

devMMA8451Q_accel_reading_t
devMMA8451Q_getAccel(void);

void
devMMA8451Q_updateBuffer(void);
