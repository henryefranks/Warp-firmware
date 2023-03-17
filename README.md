# Wrist Raising/Lowering Classifier - Built on Warp
## Henry Franks (hef36) | Pembroke College

---

This is a heavily-edited version of the Warp firmware, replacing much of
the original codebase (including the menu) to reduce program size.

See
[the original repo](https://github.com/[physical-computation]/Warp-firmware)
for the original codebase this is built on.

### **Changes (In Brief)**
- **`boot.c` has been heavily edited**
- **New files are in `src/boot/ksdk1.1/activity`**

## Aims

This firmware classifies the movement of the FRDM-KL03 development board
as either wrist raising or lowering. For this to work, the board must be
strapped to a user's wrist with the USB port pointing towards the hand. By
taking the arctangent of the measured accelerations, the board estimates the
current angle, and classifies 10-second datasets as either raising or lowering.

This functionality mimics the sort of algorithms used by smartwatches or
other wearable devices, which modulate the display (the largest source of
power dissipation) based on whether the user is checking their watch or not.

The 2-page report for this coursework outlines the method and statistics behind
the classification, but in short, the device uses MAP classification with two
carefully-constructed symmetric references, which greatly simpify the maths
(vital for running it on a device with no FPU).

## Implementation and Repository Structure

The classification logic can be found in `boot.c`, which has been stripped
of most other functionality to boost legibility. The rest of the code is
found in the `src/boot/ksdk1.1/activity` folder, including:
- Driver files for the MMA8451Q (`devMMA8451Q.c`, `devMMA8451Q.h`)
- Register structure and enumeration definitions (in the `reg` folder)
- Maths helper functions (`simple_math.h`)

The accelerometer driver implements a 10-entry circular buffer, `angle_buffer`,
which is declared `extern` in the header for `boot.c` to read. This, along with
`wt_idx`, the write pointer, allow the main loop to eaily sample the last
10-seconds of readings.

The accelerometer is configured to read at 50Hz, and store these readings
in the device's internal FIFO. `devMMA8451Q_updateBuffer` polls this buffer
and updates `angle_buffer` with every 50th reading to give a consistent
buffer of 1Hz angle data. This hands off much of the timing-constrained work
to the accelerometer, so the requirements on the KL03 are pretty lax.

The algorithm in boot.c runs in 13ms, meaning higher data rates
could be processed. The 1Hz data used, however, is sufficient for an
approximate estimate of the classification and uncertainty. This algorithm
uses `exp` from `<math.h>` along with basic floating-point arithmetic, which
is slow on the M0+, but the low complexity of the algorithm allows this to
run in good time regardless.

The maths functions in `simple_math.h` are really just a wrapper
around `atan` from `<math.h>`, although an early implementation of a
high-speed but low-accuracy approximate arctan function based on integer
arithmetic (as the KL03 has no FPU) can also be found there. The compiler's
generated floating point code is found to be sufficient for the operations
being performed here. The main point of the `arctan` function, therefore, is
to correctly place readings within the +/-180 degree range.

## CSV Mode

By changing flags in `config.h`, the device can be configured to run in
CSV mode, where it waits for user calibration before dumping a CSV of
5000 readings to the terminal. This is useful for uncertainty characterisation.

## Output Format

The firmware prints, every second, the current estimates for each class over
the previous 10 seconds, as well as computed angles and probabilities for
each data point.

## To Build

1. follow instructions in the original repo to configure paths correctly.
2. run `source setup.sh` to set environment variables
3. run `make` (this runs `make clean frdmkl03 load-warp`) to build and run

---

Original Code by Phillip Stanley-Marbell. See `LICENSE.txt` for more info


