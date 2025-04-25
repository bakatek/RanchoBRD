#include <freertos/FreeRTOS.h>
#include <driver/i2s_std.h>
#include <math.h>
#include <esp_log.h>
#include <lvgl.h>
#include "audio.h"
#include "worried_pcm.h"
#include "critical_pcm.h"
#include "startup_pcm.h"
#include "phares_pcm.h"
#include "clignotant_pcm.h"
#include "pincfg.h"

static const char *TAG2 = "AUDIO";

// Configuration I2S
#define I2S_SAMPLE_RATE 44100 // 44,1 kHz pour qualité CD
#define I2S_BUFFER_SIZE 256   // Taille du tampon réduite

static i2s_chan_handle_t tx_handle = NULL;

// Initialisation I2S en mono
void audio_init(void) {
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    chan_cfg.auto_clear = true;
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, &tx_handle, NULL));

    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(I2S_SAMPLE_RATE),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = AUDIO_I2S_BCK_IO,
            .ws = AUDIO_I2S_LRCK_IO,
            .dout = AUDIO_I2S_DO_IO,
            .din = I2S_GPIO_UNUSED,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false,
            },
        },
    };
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(tx_handle, &std_cfg));
    ESP_ERROR_CHECK(i2s_channel_enable(tx_handle));
    ESP_LOGI(TAG2, "I2S initialisé en mono");
}

// Générer une tonalité (onde sinusoïdale) pour bips
void generate_tone(int16_t *buffer, size_t samples, float frequency, float amplitude) {
    for (size_t i = 0; i < samples; i++) {
        float t = (float)i / I2S_SAMPLE_RATE;
        buffer[i] = (int16_t)(amplitude * sin(2.0 * M_PI * frequency * t) * 32767.0);
    }
}

// Jouer un bip pendant une durée (en ms)
void play_beep(uint32_t duration_ms) {
    static int16_t buffer[I2S_BUFFER_SIZE];
    uint32_t samples_per_ms = I2S_SAMPLE_RATE / 1000;
    uint32_t total_samples = duration_ms * samples_per_ms;
    uint32_t samples_written = 0;

    ESP_LOGI(TAG2, "Jouer bip de %lu ms", duration_ms);

    while (samples_written < total_samples) {
        uint32_t samples_to_write = (total_samples - samples_written) > I2S_BUFFER_SIZE ? 
                                    I2S_BUFFER_SIZE : (total_samples - samples_written);
        generate_tone(buffer, samples_to_write, 1000.0, 0.5);
        size_t bytes_written;
        i2s_channel_write(tx_handle, buffer, samples_to_write * sizeof(int16_t), &bytes_written, portMAX_DELAY);
        samples_written += samples_to_write;
        // Appeler lv_timer_handler toutes les 10 ms
        uint32_t current_ms = samples_written / (I2S_SAMPLE_RATE / 1000);
        static uint32_t last_lvgl_update = 0;
        if (current_ms - last_lvgl_update >= 10) {
            lv_timer_handler();
            last_lvgl_update = current_ms;
        }
    }
}

// Jouer un motif d'avertissement (bip-bip)
void play_warning(void) {
    play_beep(200); // Premier bip
    vTaskDelay(pdMS_TO_TICKS(100)); // Pause
    play_beep(200); // Second bip
    ESP_LOGI(TAG2, "Avertissement joué");
}


// Jouer la musique de démarrage (bong PCM)
void play_startup_tune(void) {
    static int16_t buffer[I2S_BUFFER_SIZE];
    uint32_t samples_written = 0;
    uint32_t last_lvgl_update = 0;
    ESP_LOGI(TAG2, "Jouer musique de démarrage style Mac");

    while (samples_written < startup_pcm_size) {
        uint32_t samples_to_write = (startup_pcm_size - samples_written) > I2S_BUFFER_SIZE ? 
                                    I2S_BUFFER_SIZE : (startup_pcm_size - samples_written);
        // Copier directement les échantillons PCM (mono)
        for (uint32_t i = 0; i < samples_to_write; i++) {
            buffer[i] = startup_pcm[samples_written + i];
        }
        size_t bytes_written;
        i2s_channel_write(tx_handle, buffer, samples_to_write * sizeof(int16_t), &bytes_written, portMAX_DELAY);
        samples_written += samples_to_write;

        // Mettre à jour LVGL toutes les 10 ms
        uint32_t current_ms = samples_written / (I2S_SAMPLE_RATE / 1000);
        if (current_ms - last_lvgl_update >= 10) {
            lv_timer_handler();
            last_lvgl_update = current_ms;
        }
    }
}

// Jouer la musique des phares
void play_phares_tune(void) {
    static int16_t buffer[I2S_BUFFER_SIZE];
    uint32_t samples_written = 0;
    uint32_t last_lvgl_update = 0;
    ESP_LOGI(TAG2, "Jouer musique d'activation Phares");

    while (samples_written < phares_pcm_size) {
        uint32_t samples_to_write = (phares_pcm_size - samples_written) > I2S_BUFFER_SIZE ? 
                                    I2S_BUFFER_SIZE : (phares_pcm_size - samples_written);
        // Copier directement les échantillons PCM (mono)
        for (uint32_t i = 0; i < samples_to_write; i++) {
            buffer[i] = phares_pcm[samples_written + i];
        }
        size_t bytes_written;
        i2s_channel_write(tx_handle, buffer, samples_to_write * sizeof(int16_t), &bytes_written, portMAX_DELAY);
        samples_written += samples_to_write;

        // Mettre à jour LVGL toutes les 10 ms
        uint32_t current_ms = samples_written / (I2S_SAMPLE_RATE / 1000);
        if (current_ms - last_lvgl_update >= 10) {
            lv_timer_handler();
            last_lvgl_update = current_ms;
        }
    }
}

// Jouer la musique d'alerte
void play_worried_tune(void) {
    static int16_t buffer[I2S_BUFFER_SIZE];
    uint32_t samples_written = 0;
    uint32_t last_lvgl_update = 0;
    ESP_LOGI(TAG2, "Jouer musique d'inquietude");

    while (samples_written < worried_pcm_size) {
        uint32_t samples_to_write = (worried_pcm_size - samples_written) > I2S_BUFFER_SIZE ? 
                                    I2S_BUFFER_SIZE : (worried_pcm_size - samples_written);
        // Copier directement les échantillons PCM (mono)
        for (uint32_t i = 0; i < samples_to_write; i++) {
            buffer[i] = worried_pcm[samples_written + i];
        }
        size_t bytes_written;
        i2s_channel_write(tx_handle, buffer, samples_to_write * sizeof(int16_t), &bytes_written, portMAX_DELAY);
        samples_written += samples_to_write;

        // Mettre à jour LVGL toutes les 10 ms
        uint32_t current_ms = samples_written / (I2S_SAMPLE_RATE / 1000);
        if (current_ms - last_lvgl_update >= 10) {
            lv_timer_handler();
            last_lvgl_update = current_ms;
        }
    }
}

// Jouer la musique d'erreur critique
void play_critical_tune(void) {
    static int16_t buffer[I2S_BUFFER_SIZE];
    uint32_t samples_written = 0;
    uint32_t last_lvgl_update = 0;
    ESP_LOGI(TAG2, "Jouer musique critique");

    while (samples_written < critical_pcm_size) {
        uint32_t samples_to_write = (critical_pcm_size - samples_written) > I2S_BUFFER_SIZE ? 
                                    I2S_BUFFER_SIZE : (critical_pcm_size - samples_written);
        // Copier directement les échantillons PCM (mono)
        for (uint32_t i = 0; i < samples_to_write; i++) {
            buffer[i] = critical_pcm[samples_written + i];
        }
        size_t bytes_written;
        i2s_channel_write(tx_handle, buffer, samples_to_write * sizeof(int16_t), &bytes_written, portMAX_DELAY);
        samples_written += samples_to_write;

        // Mettre à jour LVGL toutes les 10 ms
        uint32_t current_ms = samples_written / (I2S_SAMPLE_RATE / 1000);
        if (current_ms - last_lvgl_update >= 10) {
            lv_timer_handler();
            last_lvgl_update = current_ms;
        }
    }
}

// Jouer la musique clignotant
void play_clignotant_tune(void) {
    static int16_t buffer[I2S_BUFFER_SIZE];
    uint32_t samples_written = 0;
    uint32_t last_lvgl_update = 0;
    ESP_LOGI(TAG2, "Jouer musique clignotant");

    while (samples_written < clignotant_pcm_size) {
        uint32_t samples_to_write = (clignotant_pcm_size - samples_written) > I2S_BUFFER_SIZE ? 
                                    I2S_BUFFER_SIZE : (clignotant_pcm_size - samples_written);
        // Copier directement les échantillons PCM (mono)
        for (uint32_t i = 0; i < samples_to_write; i++) {
            buffer[i] = clignotant_pcm[samples_written + i];
        }
        size_t bytes_written;
        i2s_channel_write(tx_handle, buffer, samples_to_write * sizeof(int16_t), &bytes_written, portMAX_DELAY);
        samples_written += samples_to_write;

        // Mettre à jour LVGL toutes les 10 ms
        uint32_t current_ms = samples_written / (I2S_SAMPLE_RATE / 1000);
        if (current_ms - last_lvgl_update >= 10) {
            lv_timer_handler();
            last_lvgl_update = current_ms;
        }
    }
}

