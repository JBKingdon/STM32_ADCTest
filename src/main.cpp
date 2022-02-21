#include <Arduino.h>
#include "1AUDfilterInt.h"
// #include "1AUDfilter.h"
#include "main.h"

// #include <TFT_eSPI.h>
// #include "Setup_240x240_ST7789.h"

// TFT_eSPI tft = TFT_eSPI();

extern "C"
{
void EXTI2_TSC_IRQHandler()
{
    while(true) {}
}
}


uint16_t adcBuffer[ADC_BUFFER_SIZE] = {0};
uint16_t filteredValues[4] = {0};

#ifdef F103
#include "F103Init.h"
#elif defined(F411)
#include "F411Init.h"
#elif defined(F303)

#include "F303Init.h"

#else
#error "Define one of F401 or F411"
#endif

enum samplingState
{
    S_IDLE,
    S_SAMPLING,
    S_HALF,
    S_DATA_READY
};

volatile samplingState state = S_IDLE;

// long sStart, sEnd;

float minCutoff =  50;  // Hz
float maxCutoff = 150;  // Hz
float beta = 0.01f;
// slewLimit is now steps per second and gets auto converted when sample rate changes
float slewLimit = 100000.0f;

float sampleRate = PER_CHANNEL_SAMPLE_RATE;    // Hz

float dCutoff = 50.0f;      // Hz

// OneAUDfilter aud_rollF(minCutoff, maxCutoff, beta, sampleRate, dCutoff, slewLimit);
// OneAUDfilter aud_pitchF(minCutoff, maxCutoff, beta, sampleRate, dCutoff, slewLimit);
// OneAUDfilter aud_yawF(minCutoff, maxCutoff, beta, sampleRate, dCutoff, slewLimit);
// OneAUDfilter aud_throttleF(minCutoff, maxCutoff, beta, sampleRate, dCutoff, slewLimit);

OneAUDfilterInt aud_roll(minCutoff, maxCutoff, int(1.0f/beta), sampleRate, dCutoff, slewLimit);
OneAUDfilterInt aud_pitch(minCutoff, maxCutoff, int(1.0f/beta), sampleRate, dCutoff, slewLimit);
OneAUDfilterInt aud_yaw(minCutoff, maxCutoff, int(1.0f/beta), sampleRate, dCutoff, slewLimit);
OneAUDfilterInt aud_throttle(minCutoff, maxCutoff, int(1.0f/beta), sampleRate, dCutoff, slewLimit);

void setup() 
{
    delay(1000);
    Serial.begin(460800); // For debug, defaults to usb if usb is enabled in platformio.ini

    // Serial6.begin(460800);  // to main processor

    // flash the led for startup
    pinMode(PC13, OUTPUT);
    // for(int i=0; i<2; i++)
    while(true)
    {
        digitalWrite(PC13, LOW);
        delay(500);
        digitalWrite(PC13, HIGH);
        delay(500);
    }

    Serial.println("Calling adcInit");
    adcInit();
    Serial.println("adcInit done");

    // Timer setup via HAL isn't working yet, so over-ride it with high level calls
    const uint pin = PA8;

    TIM_TypeDef *Instance = (TIM_TypeDef *)pinmap_peripheral(digitalPinToPinName(pin), PinMap_PWM);
    uint32_t channel = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(pin), PinMap_PWM));

    // Configure and start PWM
    HardwareTimer *MyTim = new HardwareTimer(Instance);
    MyTim->setPWM(channel, pin, PER_CHANNEL_SAMPLE_RATE, 50);

    // tft.init();
    // // tft.setRotation(2);

    // tft.fillScreen(TFT_BLUE);
    // tft.fillScreen(TFT_BLUE);

    // tft.setTextDatum(TC_DATUM);
    // tft.setTextColor(TFT_WHITE);
    // tft.setTextFont(4);

    // tft.drawString("Hello World. Again", TFT_WIDTH/2, 100);

}


void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{
    state = S_HALF;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    // static uint ledState = 0;

    // sEnd = micros();
    state = S_DATA_READY;
    // Serial.println("cplt");

    // digitalWrite(PC13, ledState);
    // ledState = !ledState;
}

void loop() 
{
    // Serial.println("hello");
    // delay(2000);

    // measure int vs fp filter performance

    // F411 982us / 1024 samples
    // uint32_t t0 = micros();
    // for(int sample=0; sample<ADC_BUFFER_SIZE; sample++) {
    //     aud_pitch.update(adcBuffer[sample]);
    // }
    // uint32_t t1 = micros();

    // Serial.printf("int filter: %u\n", (t1-t0));

    // // F411 1120us / 1024 samples
    // t0 = micros();
    // for(int sample=0; sample<ADC_BUFFER_SIZE; sample++) {
    //     aud_pitchF.update(adcBuffer[sample]);
    // }
    // t1 = micros();

    // Serial.printf("float filter: %u\n", (t1-t0));


    static uint32_t lastDebug = 0;
    static uint32_t nSamples = 0;
    static uint32_t fStart;
    static uint32_t fLastEnd = 0;
    static unsigned long totalFilterMicros = 0;

    // HAL_StatusTypeDef result;

    switch (state) {
        case S_DATA_READY: // fall through
        case S_HALF:
        {
            fStart = micros();
            // F103 4 updates take ~9us using the integer impl, 108us with float
            // F411 16 samples = 24us with integers.
            int sampleOffset;
            if (state == S_HALF) {
                // process the first half of the buffer
                sampleOffset = 0;
            } else {
                // process the 2nd half of the buffer
                sampleOffset = ADC_BUFFER_SIZE / 2;
            }
            state = S_IDLE; // TODO these names don't quite fit anymore, find better ones
            for(int sample=0; sample<ADC_GROUP_SIZE; sample++) {
                const uint base = sampleOffset + (4 * sample);
                filteredValues[0] = aud_pitch.update(adcBuffer[base]);  // moving the asignments to after the loop is slower at groupsize 4
                filteredValues[1] = aud_roll.update(adcBuffer[base + 1]);
                filteredValues[2] = aud_throttle.update(adcBuffer[base + 2]);
                filteredValues[3] = aud_yaw.update(adcBuffer[base + 3]);
            }

            // if the state changed while we were running the filters then we're not keeping up
            if (state != S_IDLE) {
                Serial.print("ERROR: overload, last margin ");
                Serial.println(fStart - fLastEnd);
            }
            nSamples++;
            // Serial.print("margin "); 
            // Serial.println(fStart - fLastEnd);
            fLastEnd = micros();
            totalFilterMicros += (fLastEnd - fStart);
            // Serial.printf("ftime %u us\n", fLastEnd - fStart);
            // Serial.print(adcBuffer[0]); Serial.print(' '); Serial.println(filteredValues[0]);
            break;
        }
        default:
            break;
    }

    uint32_t now = millis();
    if (now > lastDebug + 1000) {
      uint32_t sps = nSamples * 1000 / (now - lastDebug);
      uint32_t aveT = totalFilterMicros/nSamples;
      totalFilterMicros = 0;
      nSamples = 0;
      lastDebug = now;
      Serial.printf("%d sgps (%d sps) Ave t/group=%u\n", sps, sps * ADC_GROUP_SIZE * 4, aveT);
    //   tft.setCursor(0, 35, 4);
    //   tft.setTextColor(TFT_WHITE, TFT_BLUE);
    //   tft.print(aveT);
      for(int i=0; i<NCHANNELS; i++) {
          Serial.print(adcBuffer[i]); Serial.print(' '); Serial.print(filteredValues[i]); Serial.print(", ");
      }
      Serial.println();
    //   tft.print(' ');
    //   tft.print(filteredValues[0]);
    //   tft.print("   ");
    }

}
