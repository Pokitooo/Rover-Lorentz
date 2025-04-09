#include <Arduino.h>
#include <lib_xcore>
#include <STM32FreeRTOS.h>
#include <Wire.h>
#include <LIDARLite.h>
#include <SparkFun_u-blox_GNSS_v3.h>

// hamamatsu Definitions

#define UBLOX_CUSTOM_MAX_WAIT (250u)

#define ADC_BITS (8)
#define ADC_DIVIDER static_cast<float>((1 << ADC_BITS) - 1)

#define SPEC_TRG PA_0
#define SPEC_ST PA_1
#define SPEC_CLK PB_12
#define SPEC_VIDEO PA_4
#define WHITE_LED PA_5
#define LASER_404 PA_6

#define SPEC_CHANNELS 288      // จำนวนช่องข้อมูลจากเซ็นเซอร์
#define WATER_THRESHOLD 0.20f    // ระดับ threshold สำหรับการตรวจจับน้ำ
#define WATER_CHANNEL_START 0 // ช่วง channel ที่เกี่ยวข้องกับน้ำ
#define WATER_CHANNEL_END 150

#define DELAY(MS) vTaskDelay(pdMS_TO_TICKS(MS))

const int groupSize = 18;
const int numGroups = SPEC_CHANNELS / groupSize; // =16
constexpr int AVERAGE_SAMPLES = 10;

// Devices

HardwareSerial rfd900x(PA_3, PA_2);
LIDARLite lidar;
SFE_UBLOX_GNSS gps;

// Variables

float prevSpeed = 0.0;
unsigned int prevTime = 0;
uint8_t sat = 0;
float spec[SPEC_CHANNELS];

// Data

struct Data
{
    uint8_t counter;
    uint32_t timeStamp;
    float gps_lat;
    float gps_lon;
    float gps_alt;
    float d_lidar;
    float velocity;
    float acceleration;
    float voltage;
    bool water_detect;
    uint16_t avgSpec[16];
};

Data data;

extern void readSpectrometer();

extern void printData();

bool detectWater();

void taskReadGpsLidar(void *)
{
    for (;;)
    {
        // gps
        data.gps_lat = static_cast<double>(gps.getLatitude(UBLOX_CUSTOM_MAX_WAIT)) * 1.e-7;
        data.gps_lon = static_cast<double>(gps.getLongitude(UBLOX_CUSTOM_MAX_WAIT)) * 1.e-7;
        data.gps_alt = static_cast<float>(gps.getAltitude(UBLOX_CUSTOM_MAX_WAIT)) * 1.e-3f; // Ellipsoid
        data.timeStamp = gps.getUnixEpoch(UBLOX_CUSTOM_MAX_WAIT);
        data.velocity = gps.getGroundSpeed(UBLOX_CUSTOM_MAX_WAIT) / 1000.0;
        sat = gps.getSIV(UBLOX_CUSTOM_MAX_WAIT);

        // Acceleration Calculation
        unsigned long currentTime = millis();

        if (prevTime != 0)
        {
            float deltaTime = (currentTime - prevTime) / 1000.0; // Convert ms to seconds

            if (deltaTime > 0)
            {
                data.acceleration = (data.velocity - prevSpeed) / deltaTime;
            }
        }
        prevSpeed = data.velocity;
        prevTime = currentTime;

        // lidar
        float total = 0;
        for (size_t i = 0; i < AVERAGE_SAMPLES; i++)
        {
            float distance = static_cast<float>(lidar.distance()); // cm
            total += distance;
            DELAY(10);
        }
        data.d_lidar = total / static_cast<float>(AVERAGE_SAMPLES);

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void taskReadSpec(void *)
{
    for (;;)
    {
        taskENTER_CRITICAL();
        readSpectrometer();
        data.water_detect = detectWater();
        taskEXIT_CRITICAL();

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void averageData(void *)
{
    for (;;)
    {
        for (int i = 0; i < numGroups; i++)
        {
            float sum = 0;

            for (int j = 0; j < groupSize; j++)
            {
                sum += spec[i * groupSize + j];
            }

            data.avgSpec[i] = (sum / groupSize) * ADC_DIVIDER;
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void checkVoltage(void *)
{
    for (;;)
    {
        data.voltage = analogRead(digitalPinToAnalogInput(pinNametoDigitalPin(SPEC_VIDEO))) * (5.0 / 1023.0); // คำนวณแรงดัน
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void taskPrint(void *)
{
    for (;;)
    {
        printData();
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void taskSend(void *)
{
    for (;;)
    {
        // rfd900x.write(reinterpret_cast<uint8_t *>(&data), sizeof(Data));
        rfd900x.print("H");
        rfd900x.print(data.counter);
        rfd900x.print(", ");
        rfd900x.print(data.timeStamp);
        rfd900x.print(", ");
        rfd900x.print(data.gps_lat, 6);
        rfd900x.print(", ");
        rfd900x.print(data.gps_lon, 6);
        rfd900x.print(", ");
        rfd900x.print(data.gps_alt);
        rfd900x.print(", ");
        rfd900x.print(data.d_lidar);
        rfd900x.print(", ");
        rfd900x.print(data.velocity);
        rfd900x.print(", ");
        rfd900x.print(data.acceleration);
        rfd900x.print(", ");
        rfd900x.print(data.water_detect);
        rfd900x.print(", ");
        rfd900x.print(data.voltage);
        rfd900x.print(", ");

        for (int i = 0; i < 16; i++)
        {
            rfd900x.print(data.avgSpec[i]);
            if (i < 15)
            {
                rfd900x.print(", ");
            }
        }
        rfd900x.println("A");

        data.counter++;
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void setup()
{
    Serial.begin(460800);

    rfd900x.begin(57600);

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
    xTaskCreate(checkVoltage, "", 1024, nullptr, 2, nullptr);
    xTaskCreate(taskPrint, "", 1024, nullptr, 2, nullptr);
    xTaskCreate(taskSend, "", 1024, nullptr, 2, nullptr);
    vTaskStartScheduler();
}

void readSpectrometer()
{
    static constexpr int delayTime = 1; // หน่วงเวลาระหว่างการอ่าน

    //digitalWriteFast(WHITE_LED, HIGH);
    digitalWriteFast(SPEC_CLK, LOW);
    delayMicroseconds(delayTime);
    digitalWriteFast(SPEC_CLK, HIGH);
    delayMicroseconds(delayTime);
    digitalWriteFast(SPEC_CLK, LOW);
    digitalWriteFast(SPEC_ST, HIGH); // ส่ง Start pulse
    delayMicroseconds(delayTime);

    // ทำ clock cycle 15 ครั้ง
    for (int i = 0; i < 15; i++)
    {
        digitalWriteFast(SPEC_CLK, HIGH);
        delayMicroseconds(delayTime);
        digitalWriteFast(SPEC_CLK, LOW);
        delayMicroseconds(delayTime);
    }

    digitalWriteFast(SPEC_ST, LOW); // ปิด Start pulse

    // ทำ clock cycle 85 ครั้ง
    for (int i = 0; i < 85; i++)
    {
        digitalWriteFast(SPEC_CLK, HIGH);
        delayMicroseconds(delayTime);
        digitalWriteFast(SPEC_CLK, LOW);
        delayMicroseconds(delayTime);
    }

    digitalWriteFast(SPEC_CLK, HIGH);
    delayMicroseconds(delayTime);
    digitalWriteFast(SPEC_CLK, LOW);
    delayMicroseconds(delayTime);

    // อ่านค่าจาก SPEC_VIDEO
    for (int i = 0; i < SPEC_CHANNELS; i++)
    {

        delayMicroseconds(delayTime);
        spec[i] = analogRead(digitalPinToAnalogInput(pinNametoDigitalPin(SPEC_VIDEO))) / ADC_DIVIDER; // อ่านค่า ADC หลายครั้ง
        digitalWriteFast(SPEC_CLK, HIGH);
        delayMicroseconds(delayTime);
        digitalWriteFast(SPEC_CLK, LOW);
        delayMicroseconds(delayTime);
    }

    digitalWriteFast(SPEC_ST, HIGH);

    // Clock cycle อีก 7 ครั้ง
    for (int i = 0; i < 7; i++)
    {
        digitalWriteFast(SPEC_CLK, HIGH);
        delayMicroseconds(delayTime);
        digitalWriteFast(SPEC_CLK, LOW);
        delayMicroseconds(delayTime);
    }

    digitalWriteFast(SPEC_CLK, HIGH);
    delayMicroseconds(delayTime);
}

bool detectWater()
{
    int waterSignal = 0;
    float avg = 0.f;

    // ตรวจสอบค่าช่องข้อมูลที่เกี่ยวข้อง
    for (int i = WATER_CHANNEL_START; i < WATER_CHANNEL_END; i++)
    {
        waterSignal += spec[i] > WATER_THRESHOLD;
        avg += spec[i];
    }

    Serial.print("WATER AVG = ");
    Serial.println(avg / (WATER_CHANNEL_END - WATER_CHANNEL_START));

    // หากค่าช่องที่เกิน Threshold มากกว่าครึ่งของช่วง ให้ถือว่าตรวจจับน้ำได้
    return waterSignal > (WATER_CHANNEL_END - WATER_CHANNEL_START) / 2;
}

void printData()
{
    Serial.println("Sensor Data:");
    Serial.print("Counter: ");
    Serial.println(data.counter);
    Serial.print("Timestamp: ");
    Serial.println(data.timeStamp);
    Serial.print("Satellite Available: ");
    Serial.println(sat);
    Serial.print("GPS Latitude: ");
    Serial.println(data.gps_lat, 6);
    Serial.print("GPS Longitude: ");
    Serial.println(data.gps_lon, 6);
    Serial.print("GPS Altitude: ");
    Serial.println(data.gps_alt);
    Serial.print("LIDAR Distance: ");
    Serial.print(data.d_lidar);
    Serial.println(" cm");
    Serial.print("Velocity: ");
    Serial.println(data.velocity);
    Serial.print("Acceleration: ");
    Serial.println(data.acceleration);
    Serial.print("Voltage: ");
    Serial.println(data.voltage);
    Serial.print("Water Detected: ");
    Serial.println(data.water_detect ? "Yes" : "No");

    Serial.print("Full Spectral: ");
    for (int i = 0; i < SPEC_CHANNELS; i++)
    {
        Serial.print(spec[i]);
        if (i < SPEC_CHANNELS - 1)
        {
            Serial.print(',');
        }
    }
    Serial.println();

    Serial.print("Spectral Averages: ");
    for (int i = 0; i < numGroups; i++)
    {
        Serial.print(data.avgSpec[i]);
        if (i < numGroups - 1)
        {
            Serial.print(", ");
        }
    }
    Serial.println();

    Serial.println("----------------------");
}

void loop()
{
}