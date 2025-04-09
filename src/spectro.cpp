#include <LIDARLite.h>
#include <Arduino.h>
#include <lib_xcore>
#include <STM32FreeRTOS.h>
#include <Wire.h>

#define UBLOX_CUSTOM_MAX_WAIT (250u)

#define ADC_BITS (8)
#define ADC_DIVIDER static_cast<float>((1 << ADC_BITS) - 1)

#define SPEC_TRG PA_0
#define SPEC_ST PA_1
#define SPEC_CLK PB_12
#define SPEC_VIDEO PA_4
#define WHITE_LED PA_5
#define LASER_404 PA_6

#define SPEC_CHANNELS 288     // จำนวนช่องข้อมูลจากเซ็นเซอร์
#define WATER_THRESHOLD 0.20f // ระดับ threshold สำหรับการตรวจจับน้ำ
#define WATER_CHANNEL_START 0 // ช่วง channel ที่เกี่ยวข้องกับน้ำ
#define WATER_CHANNEL_END 150

float spec[SPEC_CHANNELS];

extern void readSpectrometer();

bool detectWater();

void taskReadSpec(void *)
{
    for (;;)
    {
        taskENTER_CRITICAL();
        readSpectrometer();
        data.water_detect = detectWater();
        data.voltage = analogRead(digitalPinToAnalogInput(pinNametoDigitalPin(SPEC_VIDEO))) * (5.0 / 1023.0); // คำนวณแรงดัน
        taskEXIT_CRITICAL();
        DELAY(1000);
    }
}

void setup()
{
    Serial.begin(460800);

    Wire.begin();

    pinMode(pinNametoDigitalPin(SPEC_CLK), OUTPUT);
    pinMode(pinNametoDigitalPin(SPEC_ST), OUTPUT);
    pinMode(pinNametoDigitalPin(LASER_404), OUTPUT);
    pinMode(pinNametoDigitalPin(WHITE_LED), OUTPUT);
    pinMode(pinNametoDigitalPin(SPEC_VIDEO), INPUT_ANALOG);
    analogReadResolution(ADC_BITS);

    digitalWriteFast(SPEC_CLK, HIGH); // ตั้งค่าเริ่มต้นให้ SPEC_CLK สูง
    digitalWriteFast(SPEC_ST, LOW);   // ตั้งค่าเริ่มต้นให้ SPEC_ST ต่ำ
    digitalWriteFast(WHITE_LED, LOW);

    lidar.begin(0, true);
    lidar.configure(0);

    if (gps.begin(0x42, UBLOX_CUSTOM_MAX_WAIT))
    {
        gps.setI2COutput(COM_TYPE_UBX, VAL_LAYER_RAM_BBR, UBLOX_CUSTOM_MAX_WAIT);
        gps.setNavigationFrequency(5, VAL_LAYER_RAM_BBR, UBLOX_CUSTOM_MAX_WAIT);
        gps.setAutoPVT(true, VAL_LAYER_RAM_BBR, UBLOX_CUSTOM_MAX_WAIT);
        gps.setDynamicModel(DYN_MODEL_AUTOMOTIVE, VAL_LAYER_RAM_BBR, UBLOX_CUSTOM_MAX_WAIT);
    }

    xTaskCreate(taskReadGpsLidar, "", 2048, nullptr, 2, nullptr);
    xTaskCreate(taskReadSpec, "", 4096, nullptr, 2, nullptr);
    xTaskCreate(averageData, "", 1024, nullptr, 2, nullptr);
    xTaskCreate(taskPrint, "", 1024, nullptr, 2, nullptr);
    xTaskCreate(taskSend, "", 1024, nullptr, 2, nullptr);
    vTaskStartScheduler();
}