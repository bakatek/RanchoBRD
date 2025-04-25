#ifndef AUDIO_H
#define AUDIO_H

#include <stdint.h>

void audio_init(void);
void play_beep(uint32_t duration_ms);
void play_warning(void);
void play_startup_tune(void);
void play_clignotant(void);
void play_critical(void);
void play_phares(void);
void play_worried(void);

#endif