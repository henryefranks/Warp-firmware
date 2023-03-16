/* REGISTER TYPE DEFINITIONS */
/* F_SETUP --- 0x09          */

#include <stdint.h>


typedef enum {
    f_flag_no_events_detected = 0b0,
    f_flag_event_detected     = 0b1,
} devMMA8451Q_f_flag_t;

typedef union {
    struct {
        uint8_t                 f_cnt       : 6;
        devMMA8451Q_f_flag_t f_wmrk_flag : 1;
        devMMA8451Q_f_flag_t f_ovf       : 1;
    } __attribute__((packed));
    uint8_t raw_byte;
} devMMA8451Q_f_status_t;


typedef enum {
    fifo_disabled        = 0b00,
    fifo_circular_buffer = 0b01,
    fifo_stop            = 0b10,
    fifo_trigger         = 0b11,
} devMMA8451Q_f_setup_f_mode_t;

typedef union {
    struct {
        uint8_t
            f_wmrk : 6,
            f_mode : 2;
    } __attribute__((packed));
    uint8_t raw_byte;
} devMMA8451Q_f_setup_t;
