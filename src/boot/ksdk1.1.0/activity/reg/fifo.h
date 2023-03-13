/* REGISTER TYPE DEFINITIONS */
/* F_SETUP --- 0x09          */

#include <stdint.h>

typedef enum {
    fifo_disabled   = 0b00,
    circular_buffer = 0b01,
    stop            = 0b10,
    trigger         = 0b11,
} devMMA8451Q_f_setup_f_mode_t;

typedef union {
    struct {
        uint8_t
            f_wmrk : 6,
            f_mode : 2;
    } __attribute__((packed));
    uint8_t raw_byte;
} devMMA8451Q_f_setup_t;
