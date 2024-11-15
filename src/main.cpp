/*
*******************************************************************************
* Copyright (c) 2023 by M5Stack
*                  Equipped with M5Core2 sample source code
*                          配套  M5Core2 示例源代码
* Visit for more information: https://docs.m5stack.com/en/core/core2
* 获取更多资料请访问: https://docs.m5stack.com/zh_CN/core/core2
*
* Describe: NS4168--I2S power amplifier.  功放示例
* Date: 2023/7/31
*******************************************************************************
*/
#include <M5Core2.h>

#define LGFX_M5STACK_CORE2
#include <LovyanGFX.hpp>
#include <LGFX_AUTODETECT.hpp>

#include "util.h"

const int sampling_freq = 16000;
const size_t MAX_DATA_SIZE = DATA_SIZE * (size_t)(1.0 * sampling_freq * 2 / DATA_SIZE);
const size_t MAX_REFERENCE_DATA_SIZE = DATA_SIZE * (size_t)(0.2 * sampling_freq * 2 / DATA_SIZE);
const size_t MAX_FFT_BUF_SIZE = (size_t)(sizeof(float) * MAX_REFERENCE_DATA_SIZE / 2);

uint8_t mc_buf[MAX_DATA_SIZE];
size_t mc_buf_size = MAX_DATA_SIZE;

uint8_t reference_data[MAX_REFERENCE_DATA_SIZE];
size_t reference_data_size = 0;

float *fft_buf_r;
float *fft_buf_i;
size_t fft_buf_size = 0;

static LGFX lcd; // LGFXのインスタンスを作成。
static LGFX_Sprite sprite_status(&lcd);
static LGFX_Sprite sprite_buttonA(&lcd);
static LGFX_Sprite sprite_buttonB(&lcd);
static LGFX_Sprite sprite_buttonC(&lcd);

void DisplayInit(void)
{
    lcd.init();
    lcd.fillScreen(WHITE);
    lcd.setTextColor(BLACK);
    lcd.setTextSize(2);
    lcd.println("Sound Detector");

    sprite_status.createSprite(lcd.width(), lcd.height() / 4);
    sprite_status.fillSprite(WHITE);
    sprite_status.setTextSize(2);
    sprite_status.setTextColor(BLACK);

    sprite_buttonA.createSprite(lcd.width() / 3, lcd.height() / 8);
    sprite_buttonA.fillSprite(WHITE);
    sprite_buttonA.setTextSize(1);
    sprite_buttonA.setTextColor(BLACK);
    sprite_buttonA.println("Record");
    sprite_buttonA.pushSprite(0, lcd.height() * 7 / 8);

    sprite_buttonB.createSprite(lcd.width() / 3, lcd.height() / 8);
    sprite_buttonB.fillSprite(WHITE);
    sprite_buttonB.setTextSize(1);
    sprite_buttonB.setTextColor(BLACK);
    sprite_buttonB.println("Play\nrecord");
    sprite_buttonB.pushSprite(lcd.width() / 3, lcd.height() * 7 / 8);

    sprite_buttonC.createSprite(lcd.width() / 3, lcd.height() / 8);
    sprite_buttonC.fillSprite(WHITE);
    sprite_buttonC.setTextSize(1);
    sprite_buttonC.setTextColor(BLACK);
    sprite_buttonC.println("Play\nBuffering");
    sprite_buttonC.pushSprite(lcd.width() * 2 / 3, lcd.height() * 7 / 8);
}

void setup()
{
    M5.begin(true, true, true, true, kMBusModeOutput, false);
    microphoneBegin(sampling_freq);
    InitI2SSpeakOrMic(MODE_MIC, sampling_freq);
    M5.Axp.SetSpkEnable(true);
    DisplayInit();

    //
    // mc_buf = (uint8_t *)malloc(MAX_DATA_SIZE);
    // reference_data = (uint8_t *)malloc(MAX_REFERENCE_DATA_SIZE);
    fft_buf_r = (float *)malloc(MAX_FFT_BUF_SIZE);
    fft_buf_i = (float *)malloc(MAX_FFT_BUF_SIZE);
    // Fill 0
    fillZeroToArray(mc_buf, MAX_DATA_SIZE);
    fillZeroToArray(reference_data, MAX_REFERENCE_DATA_SIZE);
    fillZeroToArray((uint8_t *)fft_buf_r, MAX_FFT_BUF_SIZE);
    fillZeroToArray((uint8_t *)fft_buf_i, MAX_FFT_BUF_SIZE);

    Serial.println("Init done");
    delay(100); // delay 100ms.  延迟100ms
}

void loop()
{
    size_t byte_read;
    M5.update();

    // First, shift the data to the left by DATA_SIZE.
    // Then, read the data from the microphone to the right.
    memcpy(mc_buf, mc_buf + DATA_SIZE, MAX_DATA_SIZE - DATA_SIZE);
    i2s_read(
        Speak_I2S_NUMBER,
        (char *)(mc_buf + MAX_DATA_SIZE - DATA_SIZE),
        DATA_SIZE,
        &byte_read,
        portMAX_DELAY);
    //
    sprite_status.fillSprite(WHITE);
    sprite_status.setCursor(0, 0);
    sprite_status.println("Press ButtonA to record");
    Serial.println("Press ButtonA to record");
    // Calculate the correlation between the reference data and the current data.
    fillZeroToArray((uint8_t *)fft_buf_r, MAX_FFT_BUF_SIZE);
    fillZeroToArray((uint8_t *)fft_buf_i, MAX_FFT_BUF_SIZE);
    convertDataToFloatArray(reference_data, fft_buf_r, MAX_REFERENCE_DATA_SIZE);
    float peak = doFFT(fft_buf_r, fft_buf_i, MAX_FFT_BUF_SIZE / sizeof(float), sampling_freq);
    // sprite_status.printf("Peak: %f\n", peak);
    // Serial.printf("Peak: %f\n", peak);
    sprite_status.pushSprite(0, lcd.height() / 4);

    if (M5.BtnA.isPressed())
    {
        sprite_status.setCursor(0, 0);
        sprite_status.fillSprite(WHITE);
        sprite_status.drawString("Release ButtonA to stop recording", 0, 0);
        sprite_status.pushSprite(0, lcd.height() / 4);

        M5.Axp.SetVibration(true); // Open the vibration.   开启震动马达
        delay(100);
        M5.Axp.SetVibration(false);

        InitI2SSpeakOrMic(MODE_MIC, sampling_freq);
        int data_offset = 0;
        while (1)
        {
            i2s_read(Speak_I2S_NUMBER,
                     (char *)(reference_data + data_offset), DATA_SIZE,
                     &byte_read, (100 / portTICK_RATE_MS));
            data_offset += DATA_SIZE;
            M5.update();
            if (data_offset + DATA_SIZE > MAX_REFERENCE_DATA_SIZE || !M5.BtnA.isPressed())
                break;
        }
        reference_data_size = data_offset;

        sprite_status.setCursor(0, 0);
        sprite_status.fillSprite(WHITE);
        sprite_status.printf("Recording finished. duration: %lf\n", data_offset * 0.5 / sampling_freq);
        sprite_status.pushSprite(0, lcd.height() / 4);

        M5.Axp.SetVibration(true); // Open the vibration.   开启震动马达
        delay(10);
        M5.Axp.SetVibration(false);
        delay(10);
        M5.Axp.SetVibration(true); // Open the vibration.   开启震动马达
        delay(10);
        M5.Axp.SetVibration(false);
        delay(1000);

        sprite_status.setCursor(0, 0);
        sprite_status.fillSprite(WHITE);
        sprite_status.drawString("Playing the recorded sound", 0, 0);
        sprite_status.pushSprite(0, lcd.height() / 4);
        size_t bytes_written;
        InitI2SSpeakOrMic(MODE_SPK, sampling_freq);
        i2s_write(Speak_I2S_NUMBER, reference_data, data_offset, &bytes_written, portMAX_DELAY);
        InitI2SSpeakOrMic(MODE_MIC, sampling_freq);
    }
    else if (M5.BtnB.isPressed() and not M5.BtnB.wasPressed())
    {
        sprite_status.setCursor(0, 0);
        sprite_status.fillSprite(WHITE);
        sprite_status.drawString("Playing the recorded sound", 0, 0);
        sprite_status.pushSprite(0, lcd.height() / 4);
        size_t bytes_written;
        InitI2SSpeakOrMic(MODE_SPK, sampling_freq);
        i2s_write(Speak_I2S_NUMBER, reference_data, reference_data_size, &bytes_written, portMAX_DELAY);
        InitI2SSpeakOrMic(MODE_MIC, sampling_freq);
    }
    else if (M5.BtnC.isPressed() and not M5.BtnC.wasPressed())
    {
        sprite_status.setCursor(0, 0);
        sprite_status.fillSprite(WHITE);
        sprite_status.drawString("Playing buffering sound", 0, 0);
        sprite_status.pushSprite(0, lcd.height() / 4);
        size_t bytes_written;
        InitI2SSpeakOrMic(MODE_SPK, sampling_freq);
        i2s_write(Speak_I2S_NUMBER, mc_buf, MAX_DATA_SIZE, &bytes_written, portMAX_DELAY);
        InitI2SSpeakOrMic(MODE_MIC, sampling_freq);
    }
}
