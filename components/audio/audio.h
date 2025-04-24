#include <stdint.h>

#ifndef AUDIO_H
#define AUDIO_H

void audio_init(void);
void play_beep(uint32_t duration_ms);
void play_warning(void);
void play_startup_tune(void);

#endif