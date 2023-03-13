/* REGISTER TYPE DEFINITIONS */
/* CTRL_REGx --- 0x2A-0x2E   */

#include <stdint.h>


typedef uint8_t devMMA8451Q_cfg_void_t;

typedef enum {
    cfg_flag_disabled = 0b0,
    cfg_flag_enabled  = 0b1,
} devMMA8451Q_cfg_flag_t;

typedef enum {
    fs_mode_standby = 0b0,
    fs_mode_active  = 0b1,
} devMMA8451Q_cfg_active_t;

typedef enum {
    fr_mode_normal     = 0b0,
    fr_mode_fast_read  = 0b1,
} devMMA8451Q_cfg_f_read_t;

typedef enum {
    lnoise_mode_normal         = 0b0,
    lnoise_mode_reduced_noise  = 0b1,
} devMMA8451Q_cfg_lnoise_t;

typedef enum {
    /* output data rate */
    odr_800hz  = 0x0,       /* ODR = 800 Hz  / period = 1.25 ms */
    odr_400hz  = 0x1,       /* ODR = 400 Hz  / period = 2.5 ms  */
    odr_200hz  = 0x2,       /* ODR = 200 Hz  / period = 5 ms    */
    odr_100hz  = 0x3,       /* ODR = 100 Hz  / period = 10 ms   */
    odr_50hz   = 0x4,       /* ODR = 50 Hz   / period = 20 ms   */
    odr_12_5hz = 0x5,       /* ODR = 12.5 Hz / period = 80 ms   */
    odr_6_25hz = 0x6,       /* ODR = 6.25 Hz / period = 160 ms  */
    odr_1_56hz = 0x7,       /* ODR = 1.56 Hz / period = 640 ms  */
} devMMA8451Q_cfg_dr_t;

typedef enum {
    /* auto-wake sample frequency when device in sleep mode */
    aslp_rate_50hz   = 0b00,
    alsp_rate_12_5hz = 0b01,
    aslp_rate_6_25hz = 0b10,
    aslp_rate_1_56hz = 0b11,
} devMMA8451Q_cfg_aslp_rate_t;

typedef enum {
    /* oversampling modes (see datasheet table 67 for more info) */
    mods_normal              = 0b00,
    mods_low_power_low_noise = 0b01,
    mods_high_resolution     = 0b10,
    mods_low_power           = 0b11,
} devMMA8451Q_cfg_mods_t;

typedef enum {
    auto_sleep_disable = 0b0,
    auto_sleep_enable  = 0b1,
} devMMA8451Q_cfg_slpe_t;

typedef enum {
    reset_disable = 0b0,
    reset_enable  = 0b1,
} devMMA8451Q_cfg_rst_t;

typedef enum {
    self_test_disable = 0b0,
    self_test_enable  = 0b1,
} devMMA8451Q_cfg_st_t;

typedef enum {
    int_push_pull  = 0b0,
    int_open_drain = 0b1,
} devMMA8451Q_cfg_pp_od_t;

typedef enum {
    int_active_low  = 0b0,
    int_active_high = 0b1,
} devMMA8451Q_cfg_ipol_t;

typedef enum {
    fifo_gate_bypass = 0b0,
    fifo_gate_block  = 0b1,
} devMMA8451Q_cfg_fifo_gate_t;

typedef union {
    struct {
        /* CTRL_REG1 */
        devMMA8451Q_cfg_active_t    active    : 1;
        devMMA8451Q_cfg_f_read_t    f_read    : 1;
        devMMA8451Q_cfg_lnoise_t    lnoise    : 1;
        devMMA8451Q_cfg_dr_t        dr        : 3;
        devMMA8451Q_cfg_aslp_rate_t aslp_rate : 2;

        /* CTRL_REG2 */
        devMMA8451Q_cfg_mods_t mods  : 2;
        devMMA8451Q_cfg_slpe_t slpe  : 1;
        devMMA8451Q_cfg_mods_t smods : 2;
        devMMA8451Q_cfg_void_t       : 1;
        devMMA8451Q_cfg_rst_t  rst   : 1;
        devMMA8451Q_cfg_st_t   st    : 1;

        /* CTRL_REG3 */
        devMMA8451Q_cfg_pp_od_t     pp_od       : 1;
        devMMA8451Q_cfg_ipol_t      ipol        : 1;
        devMMA8451Q_cfg_void_t                  : 1;
        devMMA8451Q_cfg_flag_t      wake_ff_mt  : 1;
        devMMA8451Q_cfg_flag_t      wake_pulse  : 1;
        devMMA8451Q_cfg_flag_t      wake_lndprt : 1;
        devMMA8451Q_cfg_flag_t      wake_trans  : 1;
        devMMA8451Q_cfg_fifo_gate_t fifo_gate   : 1;

        /* CTRL_REG4 */
        devMMA8451Q_cfg_flag_t int_en_drdy   : 1;
        devMMA8451Q_cfg_void_t               : 1;
        devMMA8451Q_cfg_flag_t int_en_ff_mt  : 1;
        devMMA8451Q_cfg_flag_t int_en_pulse  : 1;
        devMMA8451Q_cfg_flag_t int_en_lndprt : 1;
        devMMA8451Q_cfg_flag_t int_en_trans  : 1;
        devMMA8451Q_cfg_flag_t int_en_fifo   : 1;
        devMMA8451Q_cfg_flag_t int_en_aslp   : 1;

        /* CTRL_REG5 */
        devMMA8451Q_cfg_flag_t int_cfg_drdy   : 1;
        devMMA8451Q_cfg_void_t                : 1;
        devMMA8451Q_cfg_flag_t int_cfg_ff_mt  : 1;
        devMMA8451Q_cfg_flag_t int_cfg_pulse  : 1;
        devMMA8451Q_cfg_flag_t int_cfg_lndprt : 1;
        devMMA8451Q_cfg_flag_t int_cfg_trans  : 1;
        devMMA8451Q_cfg_flag_t int_cfg_fifo   : 1;
        devMMA8451Q_cfg_flag_t int_cfg_aslp   : 1;

    } __attribute__((packed));
    uint8_t raw_bytes[5];
} devMMA8451Q_ctrl_t;
