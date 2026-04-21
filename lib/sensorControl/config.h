#ifndef __CONFIG__
#define __CONFIG__

// ─── Debug flags ──────────────────────────────────────────────────────────────
//
// DEBUG_SMA: when defined, the Arduino performs the CircularMovingAverage and
//   prints the smoothed theta at 20 Hz over serial.  Useful for scope/plotter
//   debugging without a Teensy in the loop.
//
// When NOT defined (production), the Arduino sends raw theta at full rate
//   (~240 Hz) and the Teensy is responsible for smoothing.  The higher baud
//   rate is required — 9600 cannot carry 240 frames/sec of ASCII floats.
//
// Uncomment the line below to enable on-Arduino SMA (debug mode):
// #define DEBUG_SMA

// ─── Serial baud rates ────────────────────────────────────────────────────────
#ifdef DEBUG_SMA
    // 20 Hz output — 9600 is fine for debugging
    #define ARDUINO_BAUD 9600
#else
    // ~240 Hz output — need headroom above 240 * 8 bytes = 1920 bytes/sec
    // 115200 gives ~11520 bytes/sec; plenty of margin
    #define ARDUINO_BAUD 115200
#endif

// Teensy reads from hardware serial at the same rate the Arduino transmits
#define TEENSY_BAUD ARDUINO_BAUD

// ─── Smoothing ────────────────────────────────────────────────────────────────
// CircularMovingAverage window size.
// Debug (Arduino): 80 samples at ~240 Hz ≈ 333 ms of history.
// Production (Teensy): same window, processed at the same rate.
#define SMA_WINDOW_SIZE 80

#endif // __CONFIG__