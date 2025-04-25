// Broches I2S codées en dur
#define AUDIO_I2S_BCK_IO  42 // BCLK
#define AUDIO_I2S_LRCK_IO 2  // WS (Word Select)
#define AUDIO_I2S_DO_IO   41 // DOUT (Data Out)

// Paramètres I2C pour DS3231
#define I2C_MASTER_SCL_IO 18        // GPIO pour SCL
#define I2C_MASTER_SDA_IO 17        // GPIO pour SDA
#define I2C_MASTER_NUM I2C_NUM_1    // Port I2C
#define I2C_MASTER_FREQ_HZ 100000   // Fréquence I2C (100 kHz)
#define I2C_TIMEOUT_MS 1000
#define DS3231_I2C_ADDR 0x68        // Adresse I2C du DS3231

// Paramètres I2C pour PCF8575
#define PCF8575_I2C_ADDR 0x20 // Adresse I2C par défaut du PCF8575 (A0-A2 à GND)
#define PCF8575_WRITE_ADDR (PCF8575_I2C_ADDR << 1) // Adresse pour écriture
#define PCF8575_READ_ADDR ((PCF8575_I2C_ADDR << 1) | 1) // Adresse pour lecture

// Définir les broches du PCF8575 pour chaque icône (exemple)
#define PCF8575_PIN_BATTERY  0  // P0 pour l'icône batterie
#define PCF8575_PIN_HUILE    1  // P1 pour l'icône huile
#define PCF8575_PIN_FREINR   2  // P2 pour l'icône frein rouge
#define PCF8575_PIN_FREINO   3  // P3 pour l'icône frein orange
#define PCF8575_PIN_DEGIVRE  4  // P4 pour l'icône dégivre
#define PCF8575_PIN_ESSENCE  5  // P5 pour l'icône essence
#define PCF8575_PIN_PHARE    6  // P6 pour l'icône phare
#define PCF8575_PIN_PLEINPHARE 7 // P7 pour l'icône plein phare
#define PCF8575_PIN_CLIGNOTANT 8 // P8 pour l'icône clignotant