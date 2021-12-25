#include <stdio.h>
#include <algorithm>
#include "pico/stdlib.h"
#include "hardware/dma.h"
#include "hardware/pio.h"
#include "hardware/interp.h"
#include "hardware/clocks.h"
#include "audio_i2s_16.pio.h"
#include "audio_i2s_24.pio.h"
#include "audio_i2s_32.pio.h"

constexpr uint PIN_BCK = 0;
constexpr uint PIN_LRCK = 1;
constexpr uint PIN_DIN = 2;
constexpr uint PIN_SCK = 3;
constexpr uint PIN_XSMT = 4;

const PIO& audio_pio = pio0;
const uint audio_sample_rate = 96000;
const uint audio_quantize_bits = 24;

uint bits()
{
    return 0;
}
template<typename T, typename ...Args>
uint bits(T value, Args... indices)
{
    return (1<<value) | bits(indices...);
}
template<typename ...Args>
uint bits(Args... indices)
{
    return bits(indices...);
}

static void update_pio_frequency(uint32_t sample_freq, uint sm) {
    uint32_t system_clock_frequency = clock_get_hz(clk_sys);
    assert(system_clock_frequency < 0x40000000);
    uint32_t divider = 0;
    switch(audio_quantize_bits)
    {
        case 24:
            divider = system_clock_frequency*8 / (sample_freq*3);
            break;
        case 32:
            divider = system_clock_frequency*2 / sample_freq;
            break;
        default:
            divider = system_clock_frequency*4 / sample_freq; // (clock<<fraction_bits(8)) / (sample_freq*quantized_bits(16)*channels(2)*clocks_per_output(2))
    }
    assert(divider < 0x1000000);
    pio_sm_set_clkdiv_int_frac(audio_pio, sm, divider >> 8u, divider & 0xffu);
    printf("pio %u Hz\n", (int)(system_clock_frequency / (divider/256.f)));
}

int16_t fpsin(int16_t i)
{
    /* Convert (signed) input to a value between 0 and 8192. (8192 is pi/2, which is the region of the curve fit). */
    /* ------------------------------------------------------------------- */
    i <<= 1;
    uint8_t c = i<0; //set carry for output pos/neg

    if(i == (i|0x4000)) // flip input value to corresponding value in range [0..8192)
        i = (1<<15) - i;
    i = (i & 0x7FFF) >> 1;
    /* ------------------------------------------------------------------- */

    /* The following section implements the formula:
     = y * 2^-n * ( A1 - 2^(q-p)* y * 2^-n * y * 2^-n * [B1 - 2^-r * y * 2^-n * C1 * y]) * 2^(a-q)
    Where the constants are defined as follows:
    */
    enum {A1=3370945099UL, B1=2746362156UL, C1=292421UL};
    enum {n=13, p=32, q=31, r=3, a=12};

    uint32_t y = (C1*((uint32_t)i))>>n;
    y = B1 - (((uint32_t)i*y)>>r);
    y = (uint32_t)i * (y>>n);
    y = (uint32_t)i * (y>>n);
    y = A1 - (y>>(p-q));
    y = (uint32_t)i * (y>>n);
    y = (y+(1UL<<(q-a-1)))>>(q-a); // Rounding

    return c ? -y : y;
}

int main()
{
    stdio_init_all();

    busy_wait_ms(1000);

    const uint pinMask = bits(PIN_XSMT, PIN_SCK, PIN_BCK, PIN_DIN, PIN_LRCK);
    gpio_init_mask(pinMask);
    gpio_set_dir_out_masked(pinMask);
    gpio_clr_mask(pinMask);

    gpio_set_function(PIN_DIN, GPIO_FUNC_PIO0);
    gpio_set_function(PIN_BCK, GPIO_FUNC_PIO0);
    gpio_set_function(PIN_LRCK, GPIO_FUNC_PIO0);

    busy_wait_ms(100);

    const pio_program_t* audio_i2s_program = nullptr;
    void(*audio_i2s_program_init)(PIO pio, uint sm, uint offset, uint data_pin, uint clock_pin_base) = nullptr;
    switch(audio_quantize_bits)
    {
        case 24:
            audio_i2s_program = &audio_i2s_24_program;
            audio_i2s_program_init = &audio_i2s_24_program_init;
            break;
        case 32:
            audio_i2s_program = &audio_i2s_32_program;
            audio_i2s_program_init = &audio_i2s_32_program_init;
            break;
        default:
            audio_i2s_program = &audio_i2s_16_program;
            audio_i2s_program_init = &audio_i2s_16_program_init;
    }

    const uint i2s_offset = pio_add_program(audio_pio, audio_i2s_program);
    printf("i2s_offset %u\n", i2s_offset);
    const uint audio_i2s_sm = pio_claim_unused_sm(audio_pio, true);
    printf("audio_i2s_sm %u\n", audio_i2s_sm);
    audio_i2s_program_init(audio_pio, audio_i2s_sm, i2s_offset, PIN_DIN, PIN_BCK);

    update_pio_frequency(audio_sample_rate, audio_i2s_sm);

    puts("Hello, world!");

    gpio_put(PIN_XSMT, true);
    busy_wait_ms(10);
    pio_sm_set_enabled(audio_pio, audio_i2s_sm, true);

    uint32_t angle = 0;
    uint32_t frequency = 1000;
    uint32_t step = ((0x8000 << 12)/audio_sample_rate*frequency)>>4; 
    printf("step %d\n", step);
    while (true)
    {
        int32_t sample = std::max(std::min(fpsin(angle>>8)*4, 0x7FFF), -0x7FFF);
        sample <<= audio_quantize_bits - 16;
        angle += step;

        uint32_t sample_32 = sample << (32 - audio_quantize_bits);
        pio_sm_put_blocking (audio_pio, audio_i2s_sm, sample_32);
        pio_sm_put_blocking (audio_pio, audio_i2s_sm, sample_32);
    }

    return 0;
}

#if 0
    // Interpolator example code
    interp_config cfg = interp_default_config();
    // Now use the various interpolator library functions for your use case
    // e.g. interp_config_clamp(&cfg, true);
    //      interp_config_shift(&cfg, 2);
    // Then set the config 
    interp_set_config(interp0, 0, &cfg);
#endif