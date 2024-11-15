#include <M5Core2.h>
#include "arduinoFFT.h"

bool InitI2SSpeakOrMic(int mode, int sampling_freq)
{ // Init I2S.  初始化I2S
    esp_err_t err = ESP_OK;

    i2s_driver_uninstall(
        Speak_I2S_NUMBER); // Uninstall the I2S driver.  卸载I2S驱动
    i2s_config_t i2s_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER), // Set the I2S operating mode.
                                               // 设置I2S工作模式
        .sample_rate = sampling_freq,          // Set the I2S sampling rate.  设置I2S采样率
        .bits_per_sample =
            I2S_BITS_PER_SAMPLE_16BIT, // Fixed 12-bit stereo MSB.
        // 固定为12位立体声MSB
        .channel_format =
            I2S_CHANNEL_FMT_ONLY_RIGHT, // Set the channel format. 设置频道格式
#if ESP_IDF_VERSION > ESP_IDF_VERSION_VAL(4, 1, 0)
        .communication_format =
            I2S_COMM_FORMAT_STAND_I2S, // Set the format of the communication.
                                       // 设置通讯格式
#else
        .communication_format = I2S_COMM_FORMAT_I2S,
#endif
        .intr_alloc_flags =
            ESP_INTR_FLAG_LEVEL1, // Set the interrupt flag.  设置中断的标志
        .dma_buf_count = 2,       // DMA buffer count.  DMA缓冲区计数
        .dma_buf_len = 128,       // DMA buffer length.  DMA缓冲区长度
        .use_apll = false,
        .tx_desc_auto_clear = true,
        .fixed_mclk = -1,
        .mclk_multiple = I2S_MCLK_MULTIPLE_DEFAULT,
        .bits_per_chan = I2S_BITS_PER_CHAN_DEFAULT,
    };
    if (mode == MODE_MIC)
    {
        i2s_config.mode =
            (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_PDM);
    }
    else
    {
        i2s_config.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX);
        i2s_config.use_apll = false; // I2S clock setup.  I2S时钟设置
        i2s_config.tx_desc_auto_clear =
            true; // Enables auto-cleanup descriptors for understreams.
                  // 开启欠流自动清除描述符
    }
    // Install and drive I2S.  安装并驱动I2S
    err += i2s_driver_install(Speak_I2S_NUMBER, &i2s_config, 0, NULL);

    i2s_pin_config_t tx_pin_config;

#if (ESP_IDF_VERSION > ESP_IDF_VERSION_VAL(4, 3, 0))
    tx_pin_config.mck_io_num = I2S_PIN_NO_CHANGE;
#endif
    tx_pin_config.bck_io_num =
        CONFIG_I2S_BCK_PIN;                             // Link the BCK to the CONFIG_I2S_BCK_PIN pin.
                                                        // 将BCK链接至CONFIG_I2S_BCK_PIN引脚
    tx_pin_config.ws_io_num = CONFIG_I2S_LRCK_PIN;      //          ...
    tx_pin_config.data_out_num = CONFIG_I2S_DATA_PIN;   //       ...
    tx_pin_config.data_in_num = CONFIG_I2S_DATA_IN_PIN; //      ...
    err +=
        i2s_set_pin(Speak_I2S_NUMBER,
                    &tx_pin_config); // Set the I2S pin number. 设置I2S引脚编号
    err += i2s_set_clk(
        Speak_I2S_NUMBER, sampling_freq, I2S_BITS_PER_SAMPLE_16BIT,
        I2S_CHANNEL_MONO); // Set the clock and bitwidth used by I2S Rx and Tx.
                           // 设置I2S RX、Tx使用的时钟和位宽
    return true;
}

void microphoneBegin(const int sampling_freq)
{
    Wire1.begin(21, 22);
    uint8_t reg_addr = 0x94;
    uint8_t gpio_bit = 0x04;
    uint8_t data;
    data = Read8bit(reg_addr);
    data |= gpio_bit;
    Write1Byte(reg_addr, data);
    InitI2SSpeakOrMic(MODE_SPK, sampling_freq);
}

void convertDataToFloatArray(const uint8_t *data, float *float_data, const size_t data_size)
{
    for (size_t i = 0; i < data_size; i += 2)
    {
        float_data[i / 2] = (data[i] << 8) | data[i + 1];
    }
}

void fillZeroToArray(uint8_t *data, const size_t data_size)
{
    for (size_t i = 0; i < data_size; i++)
    {
        data[i] = 0;
    }
}

float doFFT(float *vR, float *vI, const size_t data_length, const int sampling_freq)
{
    ArduinoFFT<float> FFT = ArduinoFFT<float>(vR, vI, data_length, sampling_freq);
    FFT.dcRemoval();
    FFT.windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
    FFT.compute(FFT_FORWARD);
    FFT.complexToMagnitude();
    float peak = FFT.majorPeak();
    return peak;
}

int32_t calcCorr(const uint8_t *data1, const uint8_t *data2, const size_t data_size) // 16bit MSB
{
    int32_t corr = 0;
    int32_t sum1 = 0;
    int32_t sum2 = 0;
    int32_t sum12 = 0;
    int32_t sum1_2 = 0;
    int32_t sum2_2 = 0;
    for (size_t i = 0; i < data_size; i += 2)
    {
        int16_t d1 = (data1[i] << 8) | data1[i + 1];
        int16_t d2 = (data2[i] << 8) | data2[i + 1];
        sum1 += d1;
        sum2 += d2;
        sum12 += d1 * d2;
        sum1_2 += d1 * d1;
        sum2_2 += d2 * d2;
    }
    corr = (data_size * sum12 - sum1 * sum2) /
           (sqrt(data_size * sum1_2 - sum1 * sum1) *
            sqrt(data_size * sum2_2 - sum2 * sum2));
    return corr;
}