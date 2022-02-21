#include <driver/adc.h>
#include <driver/i2s.h>

// why is this needed? the include has the classic ifdef __cplusplus code
extern "C" {
#include <soc/syscon_struct.h>
}

// sample rate for each channel in samples per second
// 64k seems to soak the cpu and prevent the comms to the copro from running
// #define PER_CHANNEL_SAMPLE_RATE (32 * 1024) // works ok
#define PER_CHANNEL_SAMPLE_RATE (10000)

// // how many samples to process at a time (across all channels)
// // #define GROUP_SAMPLE_LENGTH 64
// #define GROUP_SAMPLE_LENGTH 64

// // the corresponding aggrate rate across four channels
// #define TOTAL_SAMPLE_RATE (PER_CHANNEL_SAMPLE_RATE * 1)

// This is a single entry in the pattern table.
// The table itself consists of 4 words each containing 4 entries
typedef struct {
    union {
        struct {
            uint8_t atten:     2;   /*!< ADC sampling voltage attenuation configuration.
                                         0: input voltage * 1;
                                         1: input voltage * 1/1.34;
                                         2: input voltage * 1/2;
                                         3: input voltage * 1/3.6. */
            uint8_t bit_width: 2;   /*!< ADC resolution.
                                         0: 9 bit;
                                         1: 10 bit;
                                         2: 11 bit;
                                         3: 12 bit. */
            uint8_t channel:   4;   /*!< ADC channel index. */
        };
        uint8_t val;
    };
} adc_hal_digi_pattern_table_t;

