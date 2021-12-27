#include <stdio.h>
#include <unistd.h>
#include <sys/select.h>
#include <algorithm>
#include "pico/stdlib.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "hardware/pio.h"
#include "hardware/interp.h"
#include "hardware/clocks.h"
#include "audio_i2s_16.pio.h"
#include "audio_i2s_24.pio.h"
#include "audio_i2s_32.pio.h"
#include "libhelix-mp3/pub/mp3common.h"

constexpr uint PIN_BCK = 0;
constexpr uint PIN_LRCK = 1;
constexpr uint PIN_DIN = 2;
constexpr uint PIN_SCK = 3;
constexpr uint PIN_XSMT = 4;

const PIO& audio_pio = pio0;
const uint audio_sample_rate = 44100;
const uint audio_quantize_bits = 16;

struct audio_send_buffer_t
{
    uint32_t samples[MAX_NCHAN * MAX_NGRAN * MAX_NSAMP];
    int      count = 0;
    audio_send_buffer_t* next = nullptr;
};
audio_send_buffer_t audio_send_buffer[2];
audio_send_buffer_t audio_send_buffer_silent;
audio_send_buffer_t* audio_send_buffer_request = nullptr;
audio_send_buffer_t* audio_send_buffer_free = nullptr;

audio_send_buffer_t* pull_audio_buffer(audio_send_buffer_t*& head)
{
    if(head == nullptr)
        return nullptr;

    auto buffer = head;
    head = head->next;
    buffer->next = nullptr;

    return buffer;
}
void push_audio_buffer(audio_send_buffer_t*& head, audio_send_buffer_t* next)
{
    auto top = head;
    while(top != nullptr && top->next != nullptr)
        top = top->next;
    if(top)
        top->next = next;
    else
        head = next;
}

int dma_chan = 0;

const uint32_t recvbuf_size = 4096;
uint8_t recvbuf[recvbuf_size];
short outbuf[MAX_NCHAN * MAX_NGRAN * MAX_NSAMP];


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

int read_data(uint8_t* buf, int size)
{
    int c = getchar_timeout_us(1000);
    if(c == PICO_ERROR_TIMEOUT)
        return -1;
    
    buf[0] = c;
    for(int i = 1; i < size; ++i)
    {
        c = getchar_timeout_us(0);
        if(c == PICO_ERROR_TIMEOUT)
            return i;
        buf[i] = c;
    }
    return size;
}

void dma_handler()
{
    static audio_send_buffer_t* sending_buffer = nullptr;
    
    if(sending_buffer != nullptr)
    {  
        push_audio_buffer(audio_send_buffer_free, sending_buffer);
        sending_buffer = nullptr;
    }
    auto request = pull_audio_buffer(audio_send_buffer_request); 
    if(request != nullptr)
    {
        sending_buffer = request;
    }
    else
    {
        request = &audio_send_buffer_silent;
    }

    dma_hw->ints0 = 1u << dma_chan;

    dma_channel_set_trans_count(dma_chan, 2304, false);
    dma_channel_set_read_addr(dma_chan, request->samples, false);
    dma_channel_start(dma_chan);
}

void init()
{
    const uint pinMask = bits(PIN_XSMT, PIN_SCK, PIN_BCK, PIN_DIN, PIN_LRCK);
    gpio_init_mask(pinMask);
    gpio_set_dir_out_masked(pinMask);
    gpio_clr_mask(pinMask);

    gpio_set_function(PIN_DIN, GPIO_FUNC_PIO0);
    gpio_set_function(PIN_BCK, GPIO_FUNC_PIO0);
    gpio_set_function(PIN_LRCK, GPIO_FUNC_PIO0);

    busy_wait_ms(100);

    uint audio_i2s_sm = 0;
    {
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
        audio_i2s_sm = pio_claim_unused_sm(audio_pio, true);
        printf("audio_i2s_sm %u\n", audio_i2s_sm);
        audio_i2s_program_init(audio_pio, audio_i2s_sm, i2s_offset, PIN_DIN, PIN_BCK);

        update_pio_frequency(audio_sample_rate, audio_i2s_sm);

        puts("Hello, world!");

        gpio_put(PIN_XSMT, true);
        busy_wait_ms(10);
        pio_sm_set_enabled(audio_pio, audio_i2s_sm, true);
    }

    for(int i = 0; i < std::size(audio_send_buffer) - 1; ++i)
        audio_send_buffer[i].next = &audio_send_buffer[i + 1];
    audio_send_buffer_free = &audio_send_buffer[0];
    audio_send_buffer_request = nullptr;
    
    // Configure a channel to write the same word (32 bits) repeatedly to PIO0
    // SM0's TX FIFO, paced by the data request signal from that peripheral.
    dma_chan = dma_claim_unused_channel(true);
    dma_channel_config config = dma_channel_get_default_config(dma_chan);
    channel_config_set_transfer_data_size(&config, DMA_SIZE_32);
    channel_config_set_read_increment(&config, true);
    channel_config_set_dreq(&config, DREQ_PIO0_TX0);
    dma_channel_configure(
        dma_chan,
        &config,
        &pio0_hw->txf[audio_i2s_sm], // Write address (only need to set this once)
        nullptr,             // Don't provide a read address yet
        0,                  // Write the same value many times, then halt and interrupt
        false             // Don't start yet
    );

    // Tell the DMA to raise IRQ line 0 when the channel finishes a block
    dma_channel_set_irq0_enabled(dma_chan, true);

    // Configure the processor to run dma_handler() when DMA IRQ 0 is asserted
    irq_set_exclusive_handler(DMA_IRQ_0, dma_handler);
    irq_set_enabled(DMA_IRQ_0, true);

    puts("kick dma");
    dma_handler();
}

void acceptMP3()
{
    while(true)
    {
        printf("Ready\n");

        auto mp3dec = MP3InitDecoder();
        MP3FrameInfo frameinfo;
        uint8_t* recvbuf_write_ptr = recvbuf;
        uint8_t* recvbuf_read_ptr = recvbuf;
        
        bool receive_completed = false;

        auto receive = [&](bool blocking)
        {
            auto databytes = recvbuf_write_ptr - recvbuf_read_ptr;
            auto writespace = recvbuf_size - (recvbuf_write_ptr - recvbuf);
            if(recvbuf_read_ptr != recvbuf && writespace < recvbuf_size/4)
            {
                //printf("move to top\n");
                std::move(recvbuf_read_ptr, recvbuf_write_ptr, recvbuf);
                recvbuf_read_ptr = recvbuf;
                recvbuf_write_ptr = recvbuf + databytes;
                writespace = recvbuf_size - databytes;
            }
            int bytes = 0;
            do{
                bytes = read_data(recvbuf_write_ptr, writespace);
            }while(bytes == -1 && blocking);
            if(bytes > 0)
            {
                recvbuf_write_ptr += bytes;
                databytes = recvbuf_write_ptr - recvbuf_read_ptr;
                //printf("read %u bytes\n", bytes);
            }
            return databytes;
        };

        bool datacompleted = false;
        while(true)
        {
            printf("loop\n");
            
            int databytes = receive(true);

            int offset = -1;
            while(offset < 0)
            {
                offset = MP3FindSyncWord(recvbuf_read_ptr, databytes);
                if(offset < 0)
                    databytes = receive(true);
            }
            
            //printf("MP3FindSyncWord %u\n", offset);
            recvbuf_read_ptr += offset;
            databytes -= offset;

            bool nextframe = false;
            while(!nextframe)
            {
                //printf("MP3Decode %u\n", databytes);

                auto inbuf = recvbuf_read_ptr;
                auto bytesLeft = databytes;
                auto err = MP3Decode(mp3dec, &inbuf, &bytesLeft, outbuf, 0);
                if (err) {
                    //printf("err %d\n", err);
                    bool outofdata = false;
                    switch (err) {
                        case ERR_MP3_INDATA_UNDERFLOW:
                            outofdata = true;
                            break;
                        case ERR_MP3_MAINDATA_UNDERFLOW:
                            /* do nothing - next call to decode will provide more mainData */
                            break;
                        case ERR_MP3_FREE_BITRATE_SYNC:
                        default:
                            outofdata = true;
                            break;
                    }
                    databytes = receive(true);
                }
                else
                {
                    recvbuf_read_ptr = inbuf;
                    databytes = bytesLeft;

                    MP3GetLastFrameInfo(mp3dec, &frameinfo);

                    audio_send_buffer_t* buffer = nullptr;
                    do{
                        buffer = pull_audio_buffer(audio_send_buffer_free);
                        databytes = receive(false);
                    }while(buffer == nullptr);
                    
                    for(size_t i = 0; i < std::size(outbuf); ++i){
                        buffer->samples[i] = outbuf[i]<<18;
                        // printf("%04x ", std::abs(outbuf[i]));
                        // if(i%16==0)
                        //     printf("\n");
                    }
                    //printf("\n");
                    buffer->count = frameinfo.outputSamps;

                    push_audio_buffer(audio_send_buffer_request, buffer);

                    //printf("MP3GetLastFrameInfo %d %d\n", frameinfo.bitsPerSample, frameinfo.outputSamps);
                    nextframe = true;
                }
            }
        }

        printf("finished\n");

        MP3FreeDecoder(mp3dec);
    }
}

int main()
{
    stdio_init_all();

    busy_wait_ms(1000);

    init();

    acceptMP3();

    while(true)
        tight_loop_contents();

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