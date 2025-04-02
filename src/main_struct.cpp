#include <Arduino.h>
#include <lib_xcore>
#include <LIDARLite.h>
#include <Wire.h>
#include <SparkFun_u-blox_GNSS_v3.h>
#include <STM32FreeRTOS.h>

// hamamatsu Definitions

#define UBLOX_CUSTOM_MAX_WAIT (250u)

#define SPEC_TRG PA_0
#define SPEC_ST PA_1
#define SPEC_CLK PB_12
#define SPEC_VIDEO PB_0   
#define WHITE_LED PA_5
#define LASER_404 PA_6

#define SPEC_CHANNELS 288      // จำนวนช่องข้อมูลจากเซ็นเซอร์
#define WATER_THRESHOLD 250    // ระดับ threshold สำหรับการตรวจจับน้ำ
#define WATER_CHANNEL_START 50 // ช่วง channel ที่เกี่ยวข้องกับน้ำ
#define WATER_CHANNEL_END 100

constexpr int AVERAGE_SAMPLES = 10;

// Devices

HardwareSerial rfd900x(PA_3, PA_2);
LIDARLite lidar;
SFE_UBLOX_GNSS gps;

uint16_t spec[SPEC_CHANNELS];

// Data

struct Data
{
    uint16_t avg_spec[16];
    float gps_lat;
    float gps_lon;
    float gps_alt;
    float d_lidar;
    float Voltage;
    bool water_detect;
};

Data data;

void readSpectrometer();

void printData();

bool detectWater();

void taskReadGps(void *)
{
    for (;;)
    {
        data.gps_lat = static_cast<double>(gps.getLatitude(UBLOX_CUSTOM_MAX_WAIT)) * 1.e-7;
        data.gps_lon = static_cast<double>(gps.getLongitude(UBLOX_CUSTOM_MAX_WAIT)) * 1.e-7;
        data.gps_alt = static_cast<double>(gps.getAltitude(UBLOX_CUSTOM_MAX_WAIT)) * 1.e-3f; // Ellipsoid
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void taskReadLidar(void *)
{
    for (;;)
    {
        float total = 0;
        for (size_t i = 0; i < AVERAGE_SAMPLES; i++)
        {
            float distance = static_cast<float>(lidar.distance()); // Get distance in cm
            total += distance;
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        data.d_lidar = total / static_cast<float>(AVERAGE_SAMPLES);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void taskReadSpec(void *)
{
    for (;;)
    {
        readSpectrometer();
        if (detectWater()) {
            data.water_detect = "Water Detected";
        } else {
            data.water_detect = "No Water Detected";
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void checkVoltage(void * ) 
{
    for(;;)
    {
        data.Voltage = analogRead(SPEC_VIDEO) * (5.0 / 1023.0); // คำนวณแรงดัน
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void taskPrint(void *) 
{
    for (;;)
    {
        printData(); //water channels
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void taskSend(void *)
{
    for (;;)
    {
        rfd900x.write(reinterpret_cast<uint8_t *>(&data), sizeof(Data));
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void setup()
{
    Serial.begin(460800);

    rfd900x.begin(57600);

    Wire.begin();

    pinMode(SPEC_CLK, OUTPUT);
    pinMode(SPEC_ST, OUTPUT);
    pinMode(LASER_404, OUTPUT);
    pinMode(WHITE_LED, OUTPUT);

    digitalWrite(SPEC_CLK, HIGH); // ตั้งค่าเริ่มต้นให้ SPEC_CLK สูง
    digitalWrite(SPEC_ST, LOW);   // ตั้งค่าเริ่มต้นให้ SPEC_ST ต่ำ

    lidar.begin(0, true); // Initialize LIDAR with I2C
    lidar.configure(0);   // Use default configuration

    if (gps.begin(0x42, UBLOX_CUSTOM_MAX_WAIT))
    {
        gps.setI2COutput(COM_TYPE_UBX, VAL_LAYER_RAM_BBR, UBLOX_CUSTOM_MAX_WAIT);
        gps.setNavigationFrequency(5, VAL_LAYER_RAM_BBR, UBLOX_CUSTOM_MAX_WAIT);
        gps.setAutoPVT(true, VAL_LAYER_RAM_BBR, UBLOX_CUSTOM_MAX_WAIT);
        gps.setDynamicModel(DYN_MODEL_AUTOMOTIVE, VAL_LAYER_RAM_BBR, UBLOX_CUSTOM_MAX_WAIT);
    }

    xTaskCreate(taskReadGps, "", 1024, nullptr, 2, nullptr);
    xTaskCreate(taskReadLidar, "", 1024, nullptr, 2, nullptr);
    xTaskCreate(taskReadSpec, "", 1024, nullptr, 2, nullptr);
    xTaskCreate(taskPrint, "", 1024, nullptr, 2, nullptr);
    xTaskCreate(taskSend, "", 1024, nullptr, 2, nullptr);
    xTaskCreate(checkVoltage, "", 1024, nullptr, 2, nullptr);

    vTaskStartScheduler();
}

void readSpectrometer()
{
    static constexpr int delayTime = 1; // หน่วงเวลาระหว่างการอ่าน
    digitalWrite(WHITE_LED, HIGH);
    digitalWrite(SPEC_CLK, LOW);
    delayMicroseconds(delayTime);
    digitalWrite(SPEC_CLK, HIGH);
    delayMicroseconds(delayTime);
    digitalWrite(SPEC_CLK, LOW);
    digitalWrite(SPEC_ST, HIGH); // ส่ง Start pulse
    delayMicroseconds(delayTime);

    // ทำ clock cycle 15 ครั้ง
    for (int i = 0; i < 15; i++)
    {
        digitalWrite(SPEC_CLK, HIGH);
        delayMicroseconds(delayTime);
        digitalWrite(SPEC_CLK, LOW);
        delayMicroseconds(delayTime);
    }

    digitalWrite(SPEC_ST, LOW); // ปิด Start pulse

    // ทำ clock cycle 85 ครั้ง
    for (int i = 0; i < 85; i++)
    {
        digitalWrite(SPEC_CLK, HIGH);
        delayMicroseconds(delayTime);
        digitalWrite(SPEC_CLK, LOW);
        delayMicroseconds(delayTime);
    }

    digitalWrite(SPEC_CLK, HIGH);
    delayMicroseconds(delayTime);
    digitalWrite(SPEC_CLK, LOW);
    delayMicroseconds(delayTime);

    // อ่านค่าจาก SPEC_VIDEO
    for (int i = 0; i < SPEC_CHANNELS; i++)
    {
        uint32_t sum = 0; // สำหรับค่าเฉลี่ย
        for (int j = 0; j < AVERAGE_SAMPLES; j++)
        {
            sum += analogRead(SPEC_VIDEO); // อ่านค่า ADC หลายครั้ง
        }
        spec[i] = sum / AVERAGE_SAMPLES; // คำนวณค่าเฉลี่ย
        digitalWrite(SPEC_CLK, HIGH);
        delayMicroseconds(delayTime);
        digitalWrite(SPEC_CLK, LOW);
        delayMicroseconds(delayTime);
    }

    digitalWrite(SPEC_ST, HIGH);

    // Clock cycle อีก 7 ครั้ง
    for (int i = 0; i < 7; i++)
    {
        digitalWrite(SPEC_CLK, HIGH);
        delayMicroseconds(delayTime);
        digitalWrite(SPEC_CLK, LOW);
        delayMicroseconds(delayTime);
    }

    digitalWrite(SPEC_CLK, HIGH);
    delayMicroseconds(delayTime);
}

bool detectWater()
{
    int waterSignal = 0;

    // ตรวจสอบค่าช่องข้อมูลที่เกี่ยวข้อง
    for (int i = WATER_CHANNEL_START; i <= WATER_CHANNEL_END; i++)
    {
        if (spec[i] > WATER_THRESHOLD)
        {
            waterSignal++;
        }
    }

    // หากค่าช่องที่เกิน Threshold มากกว่าครึ่งของช่วง ให้ถือว่าตรวจจับน้ำได้
    return waterSignal > (WATER_CHANNEL_END - WATER_CHANNEL_START) / 2;
}

void printData()
{
    Serial.print("LiDAR: ");
    Serial.println(data.d_lidar);
    Serial.print("Lat: ");
    Serial.println(data.gps_lat);
    Serial.print("Long: ");
    Serial.println(data.gps_lon);
    Serial.print("Voltage at SPEC_VIDEO: ");
    Serial.print(data.Voltage);
    Serial.println(" V");

    for (int i = 0; i < SPEC_CHANNELS; i++)
    {
        Serial.print(spec[i]);
        Serial.print(',');
    }
    Serial.println();

    // Serial.write(reinterpret_cast<uint8_t *>(&data), sizeof(Data));
}

void loop()
{}