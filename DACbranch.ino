// IMPORT IMPORTANT LIBRARY
#include "BluetoothA2DPSink.h"
#include "driver/i2s.h"
#include "arduinoFFT.h"
#include "esp_heap_caps.h" //check ram left

#include <Adafruit_NeoPixel.h> //DRIVER LED

#ifdef __AVR__
#include <avr/power.h> // Required for 16 MHz Adafruit Trinket
#endif

#define LED_COLUMN 12
#define LED_ROW 8
#define PINLED 15
Adafruit_NeoPixel pixels(LED_COLUMN *LED_ROW, PINLED, NEO_GRB + NEO_KHZ800);

#define BTLED 2
BluetoothA2DPSink a2dp_sink; // Initialize Library
#define SAMPLE_RATE 44100

// config your I2S PIN
i2s_pin_config_t pin_config = {
    .mck_io_num = 3,                 // MCLK pin, ONLY USE gpio 0,1,or 3 otherwise not work
    .bck_io_num = 19,                // BCK/SCK pin
    .ws_io_num = 21,                 // LRCK/WS pin
    .data_out_num = 18,              // DATA out pin
    .data_in_num = I2S_PIN_NO_CHANGE // Not used
};

// Custom I2S configuration
i2s_config_t i2s_config_stereo = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),                    // TX only
    .sample_rate = SAMPLE_RATE,                                             // Sample rate
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,                           // Bits per sample
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,                           // Channel format
    .communication_format = (i2s_comm_format_t)(I2S_COMM_FORMAT_STAND_I2S), // Communication format
    .intr_alloc_flags = 0,                                                  // Interrupt allocation
    .dma_buf_count = 8,                                                     // DMA buffer count
    .dma_buf_len = 1024,                                                    // DMA buffer length
    .use_apll = true,                                                       // Use APLL
    .tx_desc_auto_clear = true,                                             // Auto clear tx descriptor on underflow
    .fixed_mclk = 0                                                         // Konfigurasi MCLK ke 11.2896 MHz
};

uint8_t audiobuff[4096]; // copy of 8 bit data
uint32_t audiolen;       // length of data
bool buffgo = false;     // start the buff task
bool showlog = true;     // print performance data

void audio_data_callback(const uint8_t *data, uint32_t len) // BT data on 8bit format
{
  size_t i2s_bytes_written;
  i2s_write(I2S_NUM_0, data, len, &i2s_bytes_written, portMAX_DELAY);

  if (!buffgo)
  {
    memcpy(audiobuff, data, len);
    audiolen = len;
    buffgo = true;
  }

  static unsigned long cbMillis = millis();
  static int cbCount = 0;

  if (showlog)
  {
    cbCount++;

    if (millis() - cbMillis >= 1000)
    { // Check every second
      Serial.printf("Audio Packet / s: %d\n", cbCount);
      cbCount = 0;
      cbMillis = millis();
    }
  }
  Serial.flush();
}

void convert_8_16_bit(const uint8_t *input, int16_t *output, uint32_t len)
{
  uint32_t num_samples = len / 4; // Each sample consists of 4 bytes (LSBL, MSBL, LSBR, MSBR)

  for (uint32_t i = 0; i < num_samples; i++)
  {
    int16_t left = (input[i * 4 + 1] << 8) | input[i * 4];      // Left channel
    int16_t right = (input[i * 4 + 3] << 8) | input[i * 4 + 2]; // Right channel

    output[i * 2] = left;      // Store left channel
    output[i * 2 + 1] = right; // Store right channel
  }
}

void convert8_16_mono(const uint8_t *input, int16_t *output, uint32_t len)
{
  uint32_t num_samples = len / 4; // Each sample consists of 4 bytes (LSBL, MSBL, LSBR, MSBR)

  for (uint32_t i = 0; i < num_samples; i++)
  {
    int16_t left = (input[i * 4 + 1] << 8) | input[i * 4]; // Left channel
    output[i] = left;                                      // Store only the left channel
  }
}

int16_t audio[2048]; //audio input for FFT
const uint16_t fft_samples = 1024;
double vReal[fft_samples];
double vImag[fft_samples];

ArduinoFFT<double> FFT = ArduinoFFT<double>(vReal, vImag, fft_samples, SAMPLE_RATE);

bool fftgo = false;
bool fftserial = false; //send fft result to serial instead of led
#define BANDSIZE 2
#define STARTBIN 1
#define BANDSIZE2 12
#define STARTBIN2 23
#define maxval 500

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>START OF FFT TASK<<<<<<<<<<<<<<<<<<
void fftTask(void *pvParameters){
  while (1)
  {
    if (fftgo){
      // for (int i = 0; i < fft_samples; i++) {
      //   Serial.println(vReal[i]);
      // }
      //check if audio is correct, uncomment this!
      FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward); // Apply Hamming window
      FFT.compute(FFTDirection::Forward); // Perform forward FFT
      FFT.complexToMagnitude(); // Compute magnitude

      double ledmatrix [LED_COLUMN];
      
      for (int i = 0; i < 3; i++) { //led column 1-3 for bass
        double sum = 0;
        for (int j = 0; j < BANDSIZE; j++) { //per band
          int bin = i * BANDSIZE + j + STARTBIN; //sum of bin in range
          sum += vReal[bin];
        }
        ledmatrix[i] = (ledmatrix[i]+sqrt(sum))/2;
      }
      for (int i = 3; i < LED_COLUMN; i++) { // led column 4-12
        double sum = 0;
        for (int j = 0; j < BANDSIZE2; j++) { // per band
          int bin = (i - 3) * BANDSIZE2 + j + STARTBIN2; // sum of bin in range
          sum += vReal[bin];
        }
        ledmatrix[i] = (ledmatrix[i] + sqrt(sum)) / 2;
      }

      if (!fftserial) {
        for (int col = 0; col < LED_COLUMN; col++) {
          // Hitung jumlah baris yang menyala berdasarkan rasio ledmatrix/maxval
          int rows_on = (int)((ledmatrix[col] / maxval) * LED_ROW);
          if (rows_on > LED_ROW) rows_on = LED_ROW;
          if (rows_on < 0) rows_on = 0;

          // Hitung intensitas (0-255), clamp jika lebih dari 255
          int intensity = (int)((ledmatrix[col] / maxval) * 255);
          if (intensity > 255) intensity = 255;
          if (intensity < 0) intensity = 0;
          int percent = (int)((ledmatrix[col] / maxval) * 100);
          int r,g,b = 0;
          // Nyalakan LED dari bawah ke atas
          for (int row = 0; row < LED_ROW; row++) {
            if (percent < 25){
              r,g,b = intensity;
            }
            else if(percent <50){
              r = intensity;
              g = intensity;
              b = (percent-25)/25*intensity;
            }
            else if (percent < 75){
              r = intensity;
              g = (percent-50)/25*intensity;
              b = 0;
            }
            else{
              r = intensity;
              g = 0;
              b = 0;
            }
            int pixel_index = col * LED_ROW + row; // urutan bawah ke atas
            if (row < rows_on) {
              pixels.setPixelColor(pixel_index, pixels.Color(g, r, b)); // kuning
            } else {
              pixels.setPixelColor(pixel_index, pixels.Color(0, 0, 0)); // mati
            }
          }
        }
        pixels.show();
      }

      if (fftserial){
        for (int i = 0; i < LED_COLUMN; i++) {
          Serial.print(ledmatrix[i]);
          Serial.print(" ");
        }
        Serial.println();
      }
      static unsigned long ffttimer = millis();
      static int fftCount = 0;
      if (showlog){
        fftCount++;
        if (millis() - ffttimer >= 1000)
        {
          Serial.printf("FFT task / s: %d\n", fftCount);
          fftCount = 0;
          ffttimer = millis();
          Serial.printf("HEAP: %d B, Internal: %d B, PSRAM: %d B\n", ESP.getFreeHeap(), heap_caps_get_free_size(MALLOC_CAP_INTERNAL), heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
        }
      }

      fftgo = false;
    }
    vTaskDelay(1 / portTICK_PERIOD_MS);
  }
  
}
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>END OF FFT TASK<<<<<<<<<<<<<<<<<<<<<<



//>>>>>>>>>>>>>>>>>>>>>>>>>> START OF BUFFTASK <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

void buffTask(void *pvParameters)
{
  while (true)
  {
    if (buffgo)
    {
      uint32_t monolen = audiolen / 4; // Each sample consists of 4 bytes (LSBL, MSBL, LSBR, MSBR)
      int16_t data16mono[monolen];     // uint16 buffer is half length of uint8 buffer (L,R)
      convert8_16_mono(audiobuff, data16mono, audiolen);

      if (!fftgo){
        memmove(audio, audio + monolen, (2048 - monolen) * sizeof(int16_t));     // Shift audio data forward by monolen
        memcpy(audio + (2048 - monolen), data16mono, monolen * sizeof(int16_t)); // Copy data16mono to the end of audio
        for (int i = 0; i < fft_samples; i++)
        {
          vReal[i] = audio[i];
          vImag[i] = 0;
        }
        fftgo = true;
      }
      else{
        memmove(audio, audio + monolen, (2048 - monolen) * sizeof(int16_t));     // Shift audio data forward by monolen
        memcpy(audio + (2048 - monolen), data16mono, monolen * sizeof(int16_t)); // Copy data16mono to the end of audio
      }
      
      static unsigned long bufftimer = millis();
      static int buffCount = 0;

      if (showlog){
        buffCount++;
        if (millis() - bufftimer >= 1000)
        { 
          Serial.printf("Buffer task / s: %d\n", buffCount);
          buffCount = 0;
          bufftimer = millis();
        }
      }
      buffgo = false; //ready to take another data
    }
    Serial.flush();
    vTaskDelay(1 / portTICK_PERIOD_MS);
  }
}

//>>>>>>>>>>>>>>>>>>>>>>>>>> END OF BUFFTASK <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<



//>>>>>>>>>>>>>>>>>>>>>>>>>> START OF SETUP HERE <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void setup()
{
#if defined(__AVR_ATtiny85__) && (F_CPU == 16000000)
  clock_prescale_set(clock_div_1);
#endif
  Serial.begin(115200); // disable if not used please, since pin RX/gpio 3 is used for MCLK
  pixels.begin();
  pinMode(BTLED, OUTPUT); // Set BTLED as output

  i2s_driver_install(I2S_NUM_0, &i2s_config_stereo, 0, NULL); // install i2s config
  i2s_set_pin(I2S_NUM_0, &pin_config);                        // configure pin

  a2dp_sink.set_stream_reader(audio_data_callback, false); // if audio received, run this program
  a2dp_sink.set_auto_reconnect(true);                      // remember last device and try to connect
  a2dp_sink.start("Harman/Kardon Genie");                  // Speaker name on BT

  xTaskCreatePinnedToCore(
      buffTask,   // Task function
      "buffTask", // Name of the task
      4000,       // Stack size
      NULL,       // Task input parameter
      3,          // Priority of the task
      NULL,       // Task handle
      0           // Core where the task should run
  );

  xTaskCreatePinnedToCore(
      fftTask,   // Task function
      "fftTask", // Name of the task
      2500,       // Stack size
      NULL,       // Task input parameter
      3,          // Priority of the task
      NULL,       // Task handle
      0           // Core where the task should run
  );

  for (int i = 0; i < LED_COLUMN *LED_ROW; i++)
  { // pixels.Color() takes GRB values, from 0,0,0 up to 255,255,255
    pixels.setPixelColor(i, pixels.Color(150, 0, 0));
    pixels.show(); // Send the updated pixel colors to the hardware.
    delay(1);      // Pause before next pass through loop
  }
}
//>>>>>>>>>>>>>>>>>>>>>>>>>> END OF SETUP <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

void loop()
{
  if (a2dp_sink.is_connected())
  {
    digitalWrite(BTLED, HIGH); // Turn on LED when connected
  }
  else
  {
    digitalWrite(BTLED, LOW); // Turn off LED when disconnected
  }
}
