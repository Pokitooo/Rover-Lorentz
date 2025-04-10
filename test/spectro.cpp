#include <Arduino.h>

constexpr size_t ADC_RES = 12;
constexpr float ADC_DIV = (1 << ADC_RES) - 1;

constexpr PinName SPEC_ST = PinName::PA_1;
constexpr PinName SPEC_CLK = PinName::PB_12;
constexpr PinName SPEC_VIDEO = PinName::PA_4;
constexpr PinName WHITE_LED = PinName::PA_5;

constexpr size_t SPEC_CHANNELS = 288;

uint16_t data[SPEC_CHANNELS] = {};

void readSpectrometer();

void setup() {
    Serial.begin(115200);

    pinMode(pinNametoDigitalPin(SPEC_CLK), OUTPUT);
    pinMode(pinNametoDigitalPin(SPEC_ST), OUTPUT);
    pinMode(pinNametoDigitalPin(WHITE_LED), OUTPUT);
    pinMode(pinNametoDigitalPin(SPEC_VIDEO), INPUT_ANALOG);

    analogReadResolution(ADC_RES);

    digitalWriteFast(SPEC_CLK, 1);
    digitalWriteFast(SPEC_ST, 0);
}

void loop() {
    readSpectrometer();

    for (const auto &v: data) {
        Serial.print(static_cast<float>(v) / ADC_DIV, 6);
        Serial.print(',');
    }
    Serial.println();

    delay(1000);
}

void readSpectrometer() {
    static constexpr uint32_t delayTime = 1; // delay time

    // Start clock cycle and set start pulse to signal start
    digitalWriteFast(SPEC_CLK, 0);
    delayMicroseconds(delayTime);
    digitalWriteFast(SPEC_CLK, 1);
    delayMicroseconds(delayTime);
    digitalWriteFast(SPEC_CLK, 0);
    digitalWriteFast(SPEC_ST, 1);
    delayMicroseconds(delayTime);

    //Sample for a period of time
    for (size_t i = 0; i < 15; ++i) {
        digitalWriteFast(SPEC_CLK, 1);
        delayMicroseconds(delayTime);
        digitalWriteFast(SPEC_CLK, 0);
        delayMicroseconds(delayTime);
    }

    //Set SPEC_ST to 0
    digitalWriteFast(SPEC_ST, 0);

    //Sample for a period of time
    for (size_t i = 0; i < 85; ++i) {
        digitalWriteFast(SPEC_CLK, 1);
        delayMicroseconds(delayTime);
        digitalWriteFast(SPEC_CLK, 0);
        delayMicroseconds(delayTime);
    }

    //One more clock pulse before the actual read
    digitalWriteFast(SPEC_CLK, 1);
    delayMicroseconds(delayTime);
    digitalWriteFast(SPEC_CLK, 0);
    delayMicroseconds(delayTime);

    //Read from SPEC_VIDEO
    for (auto &i: data) {
        i = analogRead(SPEC_VIDEO);
        digitalWriteFast(SPEC_CLK, 1);
        delayMicroseconds(delayTime);
        digitalWriteFast(SPEC_CLK, 0);
        delayMicroseconds(delayTime);
    }

    //Set SPEC_ST to 1
    digitalWriteFast(SPEC_ST, 1);

    //Sample for a small amount of time
    for (size_t i = 0; i < 7; ++i) {
        digitalWriteFast(SPEC_CLK, 1);
        delayMicroseconds(delayTime);
        digitalWriteFast(SPEC_CLK, 0);
        delayMicroseconds(delayTime);
    }

    digitalWriteFast(SPEC_CLK, 1);
    delayMicroseconds(delayTime);
}