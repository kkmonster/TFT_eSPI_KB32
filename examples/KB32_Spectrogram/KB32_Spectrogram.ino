

#define MIC_pin 35 // IN4

#define INTERVAL_Hz 2500.0f
#define FFT_N 512 // Must be a power of 2
#define TOTAL_TIME ((float)FFT_N/INTERVAL_Hz) //The time in which data was captured. This is equal to FFT_N/sampling_freq
#define SOUND_THRESHOLD 0.03f


#include <arduino.h>
#include "FFT.h" // include the library
#include <TFT_eSPI_KB32.h> // Hardware-specific library

TFT_eSPI_KB32 tft = TFT_eSPI_KB32();

float fft_input[FFT_N];
float fft_output[FFT_N];

float max_magnitude = 0;
float fundamental_freq = 0;
float backgroundSound = 0;

uint32_t time_interval = 0;
uint16_t fft_input_index = 0;
uint16_t x, y;
volatile uint8_t display_mode = 0;

char print_buf[300];

fft_config_t* real_fft_plan;

float  readMic() {
    return (float)analogRead(MIC_pin) / 4096.0f * 3.3f;
}

void calibrate() {
    int i = 0;
    backgroundSound = 0;
    while (i < 200) {
        if (micros() - time_interval >= (1000000L / INTERVAL_Hz)) {
            time_interval = micros();
            backgroundSound += readMic();
            i++;
        }
    }
    backgroundSound /= 200;
    Serial.print("Background sound level is ");
    Serial.println(backgroundSound);
}

void IRAM_ATTR S1_pressed() {
    display_mode = 0;
}

void IRAM_ATTR S2_pressed() {
    display_mode = 1;
}

void setup() {

    Serial.begin(115200);

    pinMode(MIC_pin, INPUT);
    pinMode(16, INPUT_PULLUP); //S1
    pinMode(14, INPUT_PULLUP); //s2

    attachInterrupt(16, S1_pressed, FALLING);
    attachInterrupt(14, S2_pressed, FALLING);

    real_fft_plan = fft_init(FFT_N, FFT_REAL, FFT_FORWARD, fft_input, fft_output);

    Serial.println("Initialized LCD");
    tft.init();   // initialize a ST7735S chip
    tft.fillScreen(TFT_BLACK);
    tft.Set_brightness(16);
}

void loop() {

    // calibrate();

    // float sound_mag = 0;

    // while (sound_mag < SOUND_THRESHOLD)
    //     sound_mag = abs(readMic() - backgroundSound);

    // Serial.println(sound_mag);

    while (fft_input_index < FFT_N) {

        if (micros() - time_interval >= (1000000L / INTERVAL_Hz)) {
            time_interval = micros();

            real_fft_plan->input[fft_input_index] = readMic();
            fft_input_index++;
        }
    }

    if (fft_input_index == FFT_N) {
        fft_input_index = 0;

        long int t1 = micros();
        // Execute transformation
        fft_execute(real_fft_plan);

        // Print the output
        max_magnitude = 0;

        x = 0;
        for (int k = 1; k < real_fft_plan->size / 2; k++)
        {
            /*The real part of a magnitude at a frequency is followed by the corresponding imaginary part in the output*/
            float mag = sqrt(pow(real_fft_plan->output[2 * k], 2) + pow(real_fft_plan->output[2 * k + 1], 2)) / 1;
            float freq = k * 1.0 / TOTAL_TIME;
            //    sprintf(print_buf,"%f Hz : %f", freq, mag);
            //    Serial.println(print_buf);
            if (mag > max_magnitude)
            {
                max_magnitude = mag;
                fundamental_freq = freq;
            }

            if (display_mode == 1) {
                if (y + 1 == 80)
                    tft.drawFastHLine(0, 0, 159, TFT_WHITE);
                else
                    tft.drawFastHLine(0, y + 1, 159, TFT_WHITE);

                if (x < 160) {
                    tft.drawPixel(x, y, mag_2_color(mag * 80));
                    x++;
                }
            }
            else {
                y = 0;
                if (x < 160) {
                    tft.drawFastVLine(x, 0, 80 - (mag * 100), TFT_BLACK);
                    tft.drawFastVLine(x, 80 - (mag * 100), (mag * 100), mag_2_color(mag * 80));
                    x++;
                }
            }

        }
        y++;

        if (y >= 80)
            y = 0;

        long int t2 = micros();

        Serial.println();
        /*Multiply the magnitude of the DC component with (1/FFT_N) to obtain the DC component*/
        sprintf(print_buf, "DC component : %f V\n", (real_fft_plan->output[0]) / FFT_N);  // DC is at [0]
        Serial.println(print_buf);

        /*Multiply the magnitude at all other frequencies with (2/FFT_N) to obtain the amplitude at that frequency*/
        sprintf(print_buf, "Fundamental Freq : %f Hz\t Mag: %f V\n", fundamental_freq, (max_magnitude) * 2 / FFT_N);
        Serial.println(print_buf);

        Serial.print("Time taken: ");
        Serial.print((t2 - t1) * 1.0 / 1000);
        Serial.println(" milliseconds!\n\n\n");

    }

}

uint16_t mag_2_color(float mag) {

    if (mag > 160)
        mag = 160;

    byte red = 0;
    byte green = 0;
    byte blue = 0;
    byte state = 0;
    uint16_t colour = 0;

    for (int i = 0; i < mag * 1; i++) {

        switch (state) {
        case 0:
            green += 2;
            if (green == 64) {
                green = 63;
                state = 1;
            }
            break;
        case 1:
            red--;
            if (red == 255) {
                red = 0;
                state = 2;
            }
            break;
        case 2:
            blue++;
            if (blue == 32) {
                blue = 31;
                state = 3;
            }
            break;
        case 3:
            green -= 2;
            if (green == 255) {
                green = 0;
                state = 4;
            }
            break;
        case 4:
            red++;
            if (red == 32) {
                red = 31;
                state = 5;
            }
            break;
        case 5:
            blue--;
            if (blue == 255) {
                blue = 0;
                state = 0;
            }
            break;
        }
        colour = red << 11 | green << 5 | blue;
    }

    return colour;
}
