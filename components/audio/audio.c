#include <freertos/FreeRTOS.h>
#include <driver/i2s.h>
#include <math.h>
#include <esp_log.h>
#include <lvgl.h>
#include "audio.h"

static const char *TAG2 = "AUDIO";

// Broches I2S codées en dur
#define AUDIO_I2S_BCK_IO  42 // BCLK
#define AUDIO_I2S_LRCK_IO 2  // WS (Word Select)
#define AUDIO_I2S_DO_IO   41 // DOUT (Data Out)

// Configuration I2S
#define I2S_SAMPLE_RATE 16000 // 16 kHz pour des sons simples
#define I2S_BUFFER_SIZE 512   // Taille du tampon
#define I2S_CHANNEL I2S_NUM_0

// Initialisation I2S
void audio_init(void) {
    i2s_config_t i2s_config = {
        .mode = I2S_MODE_MASTER | I2S_MODE_TX, // Mode maître, transmission
        .sample_rate = I2S_SAMPLE_RATE,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT, // Stéréo
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = 8,
        .dma_buf_len = I2S_BUFFER_SIZE,
        .use_apll = false,
        .tx_desc_auto_clear = true,
    };

    i2s_pin_config_t pin_config = {
        .bck_io_num = AUDIO_I2S_BCK_IO,   // GPIO 42
        .ws_io_num = AUDIO_I2S_LRCK_IO,   // GPIO 2
        .data_out_num = AUDIO_I2S_DO_IO,  // GPIO 41
        .data_in_num = I2S_PIN_NO_CHANGE, // Pas d'entrée
    };

    ESP_ERROR_CHECK(i2s_driver_install(I2S_CHANNEL, &i2s_config, 0, NULL));
    ESP_ERROR_CHECK(i2s_set_pin(I2S_CHANNEL, &pin_config));
    ESP_LOGI(TAG2, "I2S initialisé");
}

// Générer une tonalité (onde sinusoïdale)
void generate_tone(int16_t *buffer, size_t samples, float frequency, float amplitude) {
    for (size_t i = 0; i < samples; i++) {
        float t = (float)i / I2S_SAMPLE_RATE;
        buffer[i] = (int16_t)(amplitude * sin(2.0 * M_PI * frequency * t) * 32767.0);
    }
}

// Jouer un bip pendant une durée (en ms)
void play_beep(uint32_t duration_ms) {
    int16_t buffer[I2S_BUFFER_SIZE];
    uint32_t samples_per_ms = I2S_SAMPLE_RATE / 1000;
    uint32_t total_samples = duration_ms * samples_per_ms;
    uint32_t samples_written = 0;

    ESP_LOGI(TAG2, "Jouer bip de %lu ms", duration_ms);

    while (samples_written < total_samples) {
        uint32_t samples_to_write = (total_samples - samples_written) > I2S_BUFFER_SIZE ? 
                                    I2S_BUFFER_SIZE : (total_samples - samples_written);
        generate_tone(buffer, samples_to_write, 1000.0, 0.5);
        size_t bytes_written;
        i2s_write(I2S_CHANNEL, buffer, samples_to_write * sizeof(int16_t), &bytes_written, portMAX_DELAY);
        samples_written += samples_to_write;
        lv_timer_handler(); // Maintenir LVGL réactif
    }
}

// Jouer un motif d'avertissement (bip-bip)
void play_warning(void) {
    play_beep(200); // Premier bip
    vTaskDelay(pdMS_TO_TICKS(100)); // Pause
    play_beep(200); // Second bip
    ESP_LOGI(TAG2, "Avertissement joué");
}

// Jouer une musique de démarrage
void play_startup_tune(void) {
    int16_t buffer[I2S_BUFFER_SIZE];
    uint32_t samples_per_ms = I2S_SAMPLE_RATE / 1000;
    uint32_t note_duration_ms = 200; // Durée de chaque note
    uint32_t pause_duration_ms = 50; // Pause entre notes
    uint32_t total_samples = note_duration_ms * samples_per_ms;
    float notes[] = {261.63, 329.63, 392.00, 523.25}; // C4, E4, G4, C5

    ESP_LOGI(TAG2, "Jouer musique de démarrage");

    for (int i = 0; i < 4; i++) {
        uint32_t samples_written = 0;
        while (samples_written < total_samples) {
            uint32_t samples_to_write = (total_samples - samples_written) > I2S_BUFFER_SIZE ? 
                                        I2S_BUFFER_SIZE : (total_samples - samples_written);
            generate_tone(buffer, samples_to_write, notes[i], 0.5);
            size_t bytes_written;
            i2s_write(I2S_CHANNEL, buffer, samples_to_write * sizeof(int16_t), &bytes_written, portMAX_DELAY);
            samples_written += samples_to_write;
            lv_timer_handler(); // Maintenir LVGL réactif
        }
        vTaskDelay(pdMS_TO_TICKS(pause_duration_ms)); // Pause entre notes
    }
}