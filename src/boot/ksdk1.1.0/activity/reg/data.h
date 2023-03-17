/* REGISTER TYPE DEFINITIONS */
/* OUT_X_XSB --- 0x01-0x06   */
/* XYZ_DATA_CFG --- 0x0E     */

#include <stdint.h>


typedef uint8_t devMMA8451Q_data_void_t;

typedef enum {
    data_flag_disabled = 0b0,
    data_flag_enabled  = 0b1,
} devMMA8451Q_data_flag_t;

typedef union {
    /* BROKEN IMPLEMENTATION: DO NOT USE */
    /* reading .data doesn't sign extend properly */
    struct {
        int16_t      :  2;
        int16_t data : 14;
    } __attribute__((packed));
    uint8_t lsb, msb;
} devMMA8451Q_data_t;

typedef enum {
    fsr_2g       = 0b00,
    fsr_4g       = 0b01,
    fsr_8g       = 0b10,
    fsr_reserved = 0b11,
} devMMA8451Q_xyz_data_cfg_fs_t;

typedef union {
    struct {
        devMMA8451Q_xyz_data_cfg_fs_t fs      : 2;
        devMMA8451Q_data_void_t               : 2;
        devMMA8451Q_data_flag_t       hpf_out : 1;
        devMMA8451Q_data_void_t               : 3;
    } __attribute__((packed));
    uint8_t raw_byte;
} devMMA8451Q_xyz_data_cfg_t;
