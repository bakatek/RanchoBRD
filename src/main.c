//je travaille sur platformio sur un esp32s3 Wroom 1 N16R8 platform espidf avec le code suivant :
/*
je travaille sur platformio sur un esp32s3 Wroom 1 N16R8 platform espidf

l'écran prend place dans le tableau de bord d'une voiture.

Listing de tous les fichiers du projet.
DIR /S /B | FIND /V "\.pio\" | FIND /V "\.vscode\" | FIND /V "\.git\"


3.5 Inch JC3248W535C

Name                        Describe
Display color               RGB 65K color
SKU                         Capacitive touch：JC3248W535C_I_Y
Size                        3.5 inch
Type                        TFT
Driver chip                 AXS15231B
Resolution                  320*480(Pixel)
Effective display area      73.4* 49.0(mm)
Module size                 94.5*62.0(mm)
View                        IPS
Operating temperature       -20℃~70℃
storage temperature         -30℃~80℃
Operating Voltage           5V
Power consumption           About 150mA
Product weight              About 80g
*/


#include <lvgl.h>
#include <math.h>
#include <time.h>
#include <sys/time.h>
#include "pincfg.h"
#include "esp_bsp.h"
#include "display.h"
#include <esp_log.h>   // Add this line to include the header file that declares ESP_LOGI
#include <driver/i2c.h> // Ajout pour l'I2C
#include <esp_heap_caps.h> // Ajout pour l'allocation dans la PSRAM
#include <esp_psram.h> // Ajout pour les fonctions PSRAM
#include <esp_wifi.h>
#include <nvs_flash.h>
//#include <esp_bt.h>
#include "pictos.h"
#include "audio.h"
#include "audio.c"
#include "../../wifiCred.h"

//#include "icon_battery.c"
//extern const lv_img_dsc_t icon_battery; // If declared in a separate file

// Paramètres de la montre
#define FULL_CANVAS_WIDTH 480
#define FULL_CANVAS_HEIGHT 320

#define CANVAS_WIDTH 300
#define CANVAS_HEIGHT 300
#define CLOCK_RADIUS CANVAS_HEIGHT/2
#define HOUR_HAND_LENGTH CANVAS_HEIGHT/2 - 60
#define MINUTE_HAND_LENGTH CANVAS_HEIGHT/2 -30
#define SECOND_HAND_LENGTH CANVAS_HEIGHT/2 -5
#define LVGL_PORT_ROTATION_DEGREE (90)

// Variables pour la gestion Wi-Fi
static int s_retry_num = 0;
static EventGroupHandle_t s_wifi_event_group;
static const int WIFI_CONNECTED_BIT = BIT0;
static const int WIFI_FAIL_BIT = BIT1;


// Prototypes des fonctions PCF8575
static esp_err_t pcf8575_init(void);
static esp_err_t pcf8575_write(uint16_t value);
static esp_err_t pcf8575_read(uint16_t *value);

/* Optimisations */

/* Optimisations */

static const char *TAG = "RANCHO_BRD";

// Structure pour stocker les points des aiguilles
typedef struct {
    lv_point_t hour_end;
    lv_point_t minute_end;
    lv_point_t second_end;
} clock_hands_t;

static lv_obj_t *background_canvas;
static lv_color_t *bg_cbuf = NULL; // Buffer pour le canvas de fond
static lv_obj_t *theCanvas;
static lv_color_t *cbuf = NULL; // Pointeur pour le buffer alloué dynamiquement
static lv_obj_t *date_label = NULL; // Étiquette pour le jour du mois
static lv_obj_t *date_window = NULL; // Rectangle pour la fenêtre de date
static int last_day = -1; // Dernier jour affiché (-1 pour forcer la première mise à jour)

// Gestionnaire d'événements Wi-Fi
static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        ESP_LOGI(TAG, "Wi-Fi démarré, tentative de connexion...");
        lv_timer_handler();
        esp_wifi_connect();
        lv_timer_handler();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < WIFI_MAX_RETRY) {
            ESP_LOGI(TAG, "Déconnexion Wi-Fi, tentative de reconnexion (%d/%d)...", s_retry_num + 1, WIFI_MAX_RETRY);
            lv_timer_handler();
            esp_wifi_connect();
            lv_timer_handler();
            s_retry_num++;
        } else {
            ESP_LOGE(TAG, "Échec de connexion Wi-Fi après %d tentatives", WIFI_MAX_RETRY);
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
            vTaskDelay(pdMS_TO_TICKS(WIFI_RECONNECT_DELAY_MS));
            s_retry_num = 0;
            lv_timer_handler();
            esp_wifi_connect(); // Nouvelle tentative après délai
            lv_timer_handler();
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "Adresse IP obtenue: " IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

// Initialisation Wi-Fi
static void wifi_init(void) {
    // Initialiser NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Créer le groupe d'événements
    s_wifi_event_group = xEventGroupCreate();

    // Initialiser la pile réseau
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    // Configurer le Wi-Fi
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // Enregistrer le gestionnaire d'événements
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL));

    // Configurer les paramètres du point d'accès
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASSWORD,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "Initialisation Wi-Fi terminée, connexion à %s...", WIFI_SSID);

    // Attendre la connexion ou l'échec
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT, pdFALSE, pdFALSE, portMAX_DELAY);
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "Connecté au Wi-Fi");
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGE(TAG, "Échec initial de connexion au Wi-Fi, reconnexion automatique activée");
    }
}


void update_dashboard_icons(uint16_t pcf8575_state) {
    // Activer ou désactiver les icônes en fonction des bits du PCF8575
    if (pcf8575_state & (1 << PCF8575_PIN_BATTERY)) {
        lv_obj_clear_flag(icon_battery_obj, LV_OBJ_FLAG_HIDDEN);
    } else {
        lv_obj_add_flag(icon_battery_obj, LV_OBJ_FLAG_HIDDEN);
    }
    if (pcf8575_state & (1 << PCF8575_PIN_HUILE)) {
        lv_obj_clear_flag(icon_huile_obj, LV_OBJ_FLAG_HIDDEN);
    } else {
        lv_obj_add_flag(icon_huile_obj, LV_OBJ_FLAG_HIDDEN);
    }
    if (pcf8575_state & (1 << PCF8575_PIN_FREINR)) {
        lv_obj_clear_flag(icon_freinR_obj, LV_OBJ_FLAG_HIDDEN);
    } else {
        lv_obj_add_flag(icon_freinR_obj, LV_OBJ_FLAG_HIDDEN);
    }
    if (pcf8575_state & (1 << PCF8575_PIN_FREINO)) {
        lv_obj_clear_flag(icon_freinO_obj, LV_OBJ_FLAG_HIDDEN);
    } else {
        lv_obj_add_flag(icon_freinO_obj, LV_OBJ_FLAG_HIDDEN);
    }
    if (pcf8575_state & (1 << PCF8575_PIN_DEGIVRE)) {
        lv_obj_clear_flag(icon_degivre_obj, LV_OBJ_FLAG_HIDDEN);
    } else {
        lv_obj_add_flag(icon_degivre_obj, LV_OBJ_FLAG_HIDDEN);
    }
    if (pcf8575_state & (1 << PCF8575_PIN_ESSENCE)) {
        lv_obj_clear_flag(icon_essence_obj, LV_OBJ_FLAG_HIDDEN);
    } else {
        lv_obj_add_flag(icon_essence_obj, LV_OBJ_FLAG_HIDDEN);
    }
    if (pcf8575_state & (1 << PCF8575_PIN_PHARE)) {
        lv_obj_clear_flag(icon_phare_obj, LV_OBJ_FLAG_HIDDEN);
    } else {
        lv_obj_add_flag(icon_phare_obj, LV_OBJ_FLAG_HIDDEN);
    }
    if (pcf8575_state & (1 << PCF8575_PIN_PLEINPHARE)) {
        lv_obj_clear_flag(icon_pleinPhare_obj, LV_OBJ_FLAG_HIDDEN);
    } else {
        lv_obj_add_flag(icon_pleinPhare_obj, LV_OBJ_FLAG_HIDDEN);
    }
    if (pcf8575_state & (1 << PCF8575_PIN_CLIGNOTANT)) {
        lv_obj_clear_flag(icon_clignotant_obj, LV_OBJ_FLAG_HIDDEN);
    } else {
        lv_obj_add_flag(icon_clignotant_obj, LV_OBJ_FLAG_HIDDEN);
    }
}

// Fonction pour scanner les périphériques I2C
static void i2c_scan(void) {
    ESP_LOGI("I2C_STUFF", "Démarrage du scan I2C...");
    bool device_found = false;
    for (uint8_t addr = 0x08; addr <= 0x7F; addr++) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);
        esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, I2C_TIMEOUT_MS / portTICK_PERIOD_MS);
        i2c_cmd_link_delete(cmd);

        if (ret == ESP_OK) {
            ESP_LOGI("I2C_STUFF", "Périphérique trouvé à l'adresse: 0x%02X", addr);
            device_found = true;
            if (addr == DS3231_I2C_ADDR) {
                ESP_LOGI("I2C_STUFF", "DS3231 détecté à l'adresse 0x%02X", addr);
            } else if (addr == PCF8575_I2C_ADDR) {
                ESP_LOGI("I2C_STUFF", "PCF8575 détecté à l'adresse 0x%02X", addr);
            }
        } else {
            ESP_LOGD("I2C_STUFF", "Aucun périphérique à l'adresse 0x%02X: %s", addr, esp_err_to_name(ret));
        }
    }
    if (!device_found) {
        ESP_LOGW("I2C_STUFF", "Aucun périphérique I2C détecté. Vérifiez SDA (GPIO%d) et SCL (GPIO%d).",
                 I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO);
    }
}

// Fonction pour initialiser l'I2C
static esp_err_t i2c_master_init(void) {
    ESP_LOGI("I2C_STUFF", "Initialisation I2C...");
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    esp_err_t ret = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (ret != ESP_OK) {
        ESP_LOGE("I2C_STUFF", "Échec de la configuration I2C: %s", esp_err_to_name(ret));
        return ret;
    }
    ret = i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
    if (ret != ESP_OK) {
        ESP_LOGE("I2C_STUFF", "Échec de l'installation du pilote I2C: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI("I2C_STUFF", "Bus I2C initialisé (SDA=GPIO%d, SCL=GPIO%d, freq=%d Hz)",
             I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO, I2C_MASTER_FREQ_HZ);
    return ESP_OK;
}


// Fonction pour convertir BCD en décimal
static uint8_t bcd_to_dec(uint8_t bcd) {
    return ((bcd >> 4) * 10) + (bcd & 0x0F);
}

// Fonction pour lire l'heure du DS3231
static esp_err_t ds3231_read_time(struct tm *timeinfo) {
    esp_err_t ret;
    uint8_t data[7]; // Buffer pour secondes, minutes, heures, jour, date, mois, année

    // Créer une commande I2C pour lire les registres 0x00 à 0x06
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (DS3231_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0x00, true); // Commencer à l'adresse 0x00 (secondes)
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (DS3231_I2C_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, 7, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, I2C_TIMEOUT_MS / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    if (ret != ESP_OK) {
        ESP_LOGE("DS3231", "Échec de la lecture de l'heure: %s", esp_err_to_name(ret));
        return ret;
    }

    // Convertir les données BCD en décimal et remplir la structure tm
    timeinfo->tm_sec = bcd_to_dec(data[0]);
    timeinfo->tm_min = bcd_to_dec(data[1]);
    timeinfo->tm_hour = bcd_to_dec(data[2] & 0x3F); // Masquer les bits de format 24h/12h
    timeinfo->tm_mday = bcd_to_dec(data[4]);
    timeinfo->tm_mon = bcd_to_dec(data[5]) - 1; // tm_mon est de 0 à 11
    timeinfo->tm_year = bcd_to_dec(data[6]) + 2000 - 1900; // tm_year est années depuis 1900
    timeinfo->tm_wday = bcd_to_dec(data[3]) - 1; // Jour de la semaine (0-6, Dim-Sam)
    timeinfo->tm_isdst = -1; // Pas d'info sur l'heure d'été

    ESP_LOGI("DS3231", "Heure lue: %02d:%02d:%02d %02d/%02d/%04d",
             timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_sec,
             timeinfo->tm_mday, timeinfo->tm_mon + 1, timeinfo->tm_year + 1900);

    return ESP_OK;
}

// Fonction pour synchroniser l'heure système avec le DS3231
static esp_err_t ds3231_sync_time(void) {
    struct tm timeinfo;
    esp_err_t ret = ds3231_read_time(&timeinfo);
    if (ret != ESP_OK) {
        return ret;
    }

    // Convertir struct tm en time_t
    time_t t = mktime(&timeinfo);
    if (t == -1) {
        ESP_LOGE("DS3231", "Échec de la conversion de l'heure");
        return ESP_FAIL;
    }

    // Mettre à jour l'heure système
    struct timeval tv = {
        .tv_sec = t,
        .tv_usec = 0
    };
    ret = settimeofday(&tv, NULL);
    if (ret != 0) {
        ESP_LOGE("DS3231", "Échec de la mise à jour de l'heure système");
        return ESP_FAIL;
    }

    ESP_LOGI("DS3231", "Heure système synchronisée avec le DS3231");
    return ESP_OK;
}

// Initialiser le PCF8575
static esp_err_t pcf8575_init(void) {
    ESP_LOGI("PCF8575", "Initialisation du PCF8575 à l'adresse 0x%02X", PCF8575_I2C_ADDR);
    // Pas besoin de configuration supplémentaire, le PCF8575 est prêt après la mise sous tension
    // On peut écrire un état initial (toutes les broches à 1 pour les entrées)
    uint16_t initial_state = 0xFFFF; // Toutes les broches à 1 (entrées ou sorties inactives)
    esp_err_t ret = pcf8575_write(initial_state);
    if (ret != ESP_OK) {
        ESP_LOGE("PCF8575", "Échec de l'initialisation: %s", esp_err_to_name(ret));
    }
    return ret;
}

// Écrire sur les broches du PCF8575
static esp_err_t pcf8575_write(uint16_t value) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, PCF8575_WRITE_ADDR, true);
    i2c_master_write_byte(cmd, (value & 0xFF), true); // Octet bas
    i2c_master_write_byte(cmd, (value >> 8), true);   // Octet haut
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, I2C_TIMEOUT_MS / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        ESP_LOGE("PCF8575", "Échec de l'écriture: %s", esp_err_to_name(ret));
    }
    return ret;
}

// Lire les broches du PCF8575
static esp_err_t pcf8575_read(uint16_t *value) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, PCF8575_READ_ADDR, true);
    uint8_t low_byte, high_byte;
    i2c_master_read_byte(cmd, &low_byte, I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, &high_byte, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, I2C_TIMEOUT_MS / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    if (ret == ESP_OK) {
        *value = (high_byte << 8) | low_byte;
    } else {
        ESP_LOGE("PCF8575", "Échec de la lecture: %s", esp_err_to_name(ret));
    }
    return ret;
}

// Fonction pour calculer la position de l'aiguille des heures
void get_hour_hand_end(int hour, int minute, int center_x, int center_y, int length, lv_point_t *point) {
    float hour_angle = (hour % 12) * 30.0f + (minute / 60.0f) * 30.0f;
    float hour_angle_rad = hour_angle * (M_PI / 180.0f) - M_PI / 2.0f;
    point->x = center_x + length * cosf(hour_angle_rad);
    point->y = center_y + length * sinf(hour_angle_rad);
}

// Fonction pour calculer la position de l'aiguille des minutes
void get_minute_hand_end(int minute, int second, int center_x, int center_y, int length, lv_point_t *point) {
    float minute_angle = minute * 6.0f + (second / 60.0f) * 6.0f;
    float minute_angle_rad = minute_angle * (M_PI / 180.0f) - M_PI / 2.0f;
    point->x = center_x + length * cosf(minute_angle_rad);
    point->y = center_y + length * sinf(minute_angle_rad);
}

// Fonction pour calculer la position de l'aiguille des secondes
/*void get_second_hand_end(int second, int center_x, int center_y, int length, lv_point_t *point) {
    float second_angle = second * 6.0f;
    float second_angle_rad = second_angle * (M_PI / 180.0f) - M_PI / 2.0f;
    point->x = center_x + length * cosf(second_angle_rad);
    point->y = center_y + length * sinf(second_angle_rad);
}*/

// Fonction pour calculer la position de l'aiguille des secondes
void get_second_hand_end(int second, int millisecond, int center_x, int center_y, int length, lv_point_t *point) {
    float second_angle = second * 6.0f + (millisecond / 1000.0f) * 6.0f;
    float second_angle_rad = second_angle * (M_PI / 180.0f) - M_PI / 2.0f;
    point->x = center_x + length * cosf(second_angle_rad);
    point->y = center_y + length * sinf(second_angle_rad);
}

// Fonction pour dessiner la montre
// Fonction pour dessiner la montre
void draw_clock(clock_hands_t *hands) {
    // Obtenir l'heure actuelle pour le jour du mois
    struct timeval tv_now;
    gettimeofday(&tv_now, NULL);
    time_t now = tv_now.tv_sec;
    struct tm *timeinfo = localtime(&now);
    int day = timeinfo->tm_mday;

    // Mettre à jour l'étiquette de la date uniquement si le jour a changé
    if (day != last_day) {
        char date_str[3];
        snprintf(date_str, sizeof(date_str), "%d", day);
        lv_label_set_text(date_label, date_str);
        last_day = day; // Mémoriser le nouveau jour
        ESP_LOGI("CLOCK", "Date mise à jour : %s", date_str);
    }

    // Effacer le canvas
    lv_canvas_fill_bg(theCanvas, lv_color_hex(0x000000), LV_OPA_TRANSP); // Fond transparent
    

    // Dessiner le cadran (cercle)
    lv_draw_arc_dsc_t arc_dsc;
    lv_draw_arc_dsc_init(&arc_dsc);
    arc_dsc.color = lv_color_white();
    arc_dsc.width = 2;
    lv_canvas_draw_arc(theCanvas, CANVAS_WIDTH / 2, CANVAS_HEIGHT / 2, CLOCK_RADIUS, 0, 360, &arc_dsc);

    // Dessiner les marqueurs des heures (toutes les 5 minutes)
    for (int i = 0; i < 60; i++) {
        float angle = i * 6.0f * (M_PI / 180.0f) - M_PI / 2.0f;
        int length = (i % 5 == 0) ? 10 : 5; // Marqueurs plus longs pour les heures
        lv_point_t outer = {
            CANVAS_WIDTH / 2 + CLOCK_RADIUS * cosf(angle),
            CANVAS_HEIGHT / 2 + CLOCK_RADIUS * sinf(angle)
        };
        lv_point_t inner = {
            CANVAS_WIDTH / 2 + (CLOCK_RADIUS - length) * cosf(angle),
            CANVAS_HEIGHT / 2 + (CLOCK_RADIUS - length) * sinf(angle)
        };
        lv_draw_line_dsc_t line_dsc;
        lv_draw_line_dsc_init(&line_dsc);
        line_dsc.color = lv_color_white();
        line_dsc.width = (i % 5 == 0) ? 3 : 1;
        lv_point_t points[2] = {inner, outer};
        lv_canvas_draw_line(theCanvas, points, 2, &line_dsc);
    }

    // Ajouter les numéros des heures (1 à 12)
    lv_draw_label_dsc_t label_dsc;
    lv_draw_label_dsc_init(&label_dsc);
    label_dsc.color = lv_color_white();
    label_dsc.font = &lv_font_montserrat_16; // Utiliser une police disponible
    label_dsc.align = LV_TEXT_ALIGN_CENTER;

    for (int i = 1; i <= 12; i++) {
        float angle = (i * 30.0f - 90.0f) * (M_PI / 180.0f); // -90 pour commencer à 12h
        // Positionner les chiffres légèrement à l'intérieur du cadran
        int text_radius = CLOCK_RADIUS - 20; // Ajuster pour placer les chiffres
        int x = CANVAS_WIDTH / 2 + text_radius * cosf(angle);
        int y = CANVAS_HEIGHT / 2 + text_radius * sinf(angle);

        char num_str[3];
        snprintf(num_str, sizeof(num_str), "%d", i);
        lv_canvas_draw_text(theCanvas, x - 10, y - 10, 20, &label_dsc, num_str);
    }

    // Dessiner l'aiguille des heures
    lv_draw_line_dsc_t hour_dsc;
    lv_draw_line_dsc_init(&hour_dsc);
    hour_dsc.color = lv_color_white();
    hour_dsc.width = 6;
    lv_point_t hour_points[2] = {{CANVAS_WIDTH / 2, CANVAS_HEIGHT / 2}, hands->hour_end};
    lv_canvas_draw_line(theCanvas, hour_points, 2, &hour_dsc);

    // Dessiner l'aiguille des minutes
    lv_draw_line_dsc_t minute_dsc;
    lv_draw_line_dsc_init(&minute_dsc);
    minute_dsc.color = lv_color_white();
    minute_dsc.width = 4;
    lv_point_t minute_points[2] = {{CANVAS_WIDTH / 2, CANVAS_HEIGHT / 2}, hands->minute_end};
    lv_canvas_draw_line(theCanvas, minute_points, 2, &minute_dsc);

    // Dessiner l'aiguille des secondes
    lv_draw_line_dsc_t second_dsc;
    lv_draw_line_dsc_init(&second_dsc);
    second_dsc.color = lv_color_hex(0xff0000);
    second_dsc.width = 2;
    lv_point_t second_points[2] = {{CANVAS_WIDTH / 2, CANVAS_HEIGHT / 2}, hands->second_end};
    lv_canvas_draw_line(theCanvas, second_points, 2, &second_dsc);

    // Dessiner le centre de la montre
    lv_draw_arc_dsc_t center_dsc;
    lv_draw_arc_dsc_init(&center_dsc);
    center_dsc.color = lv_color_white();
    lv_canvas_draw_arc(theCanvas, CANVAS_WIDTH / 2, CANVAS_HEIGHT / 2, 5, 0, 360, &center_dsc);
}


// Fonction de mise à jour de la montre (appelée par le timer)
static void clock_timer_cb2() {
    // Lire l'état des broches du PCF8575
    uint16_t pcf8575_state = 0;
    esp_err_t ret = pcf8575_read(&pcf8575_state);
    if (ret != ESP_OK) {
        ESP_LOGE("PCF8575", "Erreur de lecture des broches: %s", esp_err_to_name(ret));
        pcf8575_state = 0; // Valeur par défaut en cas d'erreur
    }/* else {
        ESP_LOGI("PCF8575", "État des broches: 0x%04X", pcf8575_state); // Log pour débogage
    }*/

    // Mettre à jour les icônes
    update_dashboard_icons(pcf8575_state);

    // Obtenir l'heure actuelle
    struct timeval tv_now;
    gettimeofday(&tv_now, NULL);

    // Obtenir l'heure actuelle
    time_t now = tv_now.tv_sec;
    struct tm *timeinfo = localtime(&now);
    int hour = timeinfo->tm_hour;
    int minute = timeinfo->tm_min;
    int second = timeinfo->tm_sec;
    int millisecond = tv_now.tv_usec / 1000;

    // Calculer les positions des aiguilles
    clock_hands_t hands;
    get_hour_hand_end(hour, minute, CANVAS_WIDTH / 2, CANVAS_HEIGHT / 2, HOUR_HAND_LENGTH, &hands.hour_end);
    get_minute_hand_end(minute, second, CANVAS_WIDTH / 2, CANVAS_HEIGHT / 2, MINUTE_HAND_LENGTH, &hands.minute_end);
    //get_second_hand_end(second, CANVAS_WIDTH / 2, CANVAS_HEIGHT / 2, SECOND_HAND_LENGTH, &hands.second_end);
    get_second_hand_end(second, millisecond, CANVAS_WIDTH / 2, CANVAS_HEIGHT / 2, SECOND_HAND_LENGTH, &hands.second_end);

    // Dessiner la montre
    draw_clock(&hands);

    //debug
    //ESP_LOGI("PCF8575", "État des broches: 0x%04X", pcf8575_state);
}

// Fonction d'initialisation de la montre
void clock_init(void) {
    // Allouer le buffer pour le canvas de fond dans la PSRAM
    size_t bg_buf_size = LV_CANVAS_BUF_SIZE_TRUE_COLOR(FULL_CANVAS_WIDTH, FULL_CANVAS_HEIGHT) * sizeof(lv_color_t);
    bg_cbuf = (lv_color_t *)heap_caps_malloc(bg_buf_size, MALLOC_CAP_SPIRAM);
    if (bg_cbuf == NULL) {
        ESP_LOGE("CLOCK", "Échec de l'allocation du buffer de fond dans la PSRAM");
        return;
    }

    // Créer le canvas de fond
    background_canvas = lv_canvas_create(lv_scr_act());
    lv_canvas_set_buffer(background_canvas, bg_cbuf, FULL_CANVAS_WIDTH, FULL_CANVAS_HEIGHT, LV_IMG_CF_TRUE_COLOR);
    lv_obj_align(background_canvas, LV_ALIGN_CENTER, 0, 0);
    lv_canvas_fill_bg(background_canvas, lv_color_hex(0x009A76), LV_OPA_COVER); // Fond de la même couleur que l'écran

    // Allouer le buffer pour le canvas de l'horloge avec lv_mem_alloc
    size_t buf_size = LV_CANVAS_BUF_SIZE_TRUE_COLOR_ALPHA(CANVAS_WIDTH, CANVAS_HEIGHT) * sizeof(lv_color_t);
    cbuf = (lv_color_t *)lv_mem_alloc(buf_size);
    if (cbuf == NULL) {
        ESP_LOGE("CLOCK", "Échec de l'allocation du buffer dans la PSRAM");
        heap_caps_free(bg_cbuf);
        return;
    }

    // Initialize the buffer to transparent
    memset(cbuf, 0, buf_size);

    if (esp_psram_is_initialized()) {
        init_icon_battery();
        init_icon_huile();
        init_icon_freinR();
        init_icon_freinO();
        init_icon_degivre();
        init_icon_essence();
        init_icon_phare();
        init_icon_pleinPhare();
        init_icon_clignotant();
    } else {
        ESP_LOGE("CLOCK", "PSRAM not available for icons");
    }

    // Créer la fenêtre de date (rectangle)
    date_window = lv_obj_create(background_canvas);
    lv_obj_set_size(date_window, 40, 24); // Taille de la fenêtre
    lv_obj_set_pos(date_window, FULL_CANVAS_WIDTH / 2 + 80, FULL_CANVAS_HEIGHT / 2 - 12); // Position à 3h
    lv_obj_set_style_bg_color(date_window, lv_color_hex(0x000000), LV_PART_MAIN); // Fond noir
    lv_obj_set_style_border_color(date_window, lv_color_white(), LV_PART_MAIN); // Bordure blanche
    lv_obj_set_style_border_width(date_window, 1, LV_PART_MAIN);
    lv_obj_set_style_radius(date_window, 4, LV_PART_MAIN); // Coins arrondis

    // Créer l'étiquette pour le jour du mois
    date_label = lv_label_create(date_window);
    lv_label_set_text(date_label, "1"); // Valeur initiale
    lv_obj_align(date_label, LV_ALIGN_CENTER, 0, 0); // Centrer dans la fenêtre
    lv_obj_set_style_text_color(date_label, lv_color_white(), LV_PART_MAIN);
    lv_obj_set_style_text_font(date_label, &lv_font_montserrat_16, LV_PART_MAIN);

    // Create image objects
    icon_battery_obj = lv_img_create(background_canvas);
    lv_img_set_src(icon_battery_obj, &icon_battery);
    lv_obj_set_pos(icon_battery_obj, 0, 1); // Top-left
    lv_obj_add_flag(icon_battery_obj, LV_OBJ_FLAG_HIDDEN); // Hidden by default

    // Create image objects
    icon_huile_obj = lv_img_create(background_canvas);
    lv_img_set_src(icon_huile_obj, &icon_huile);
    lv_obj_set_pos(icon_huile_obj, 0, 34*1+1); // Top-left
    lv_obj_add_flag(icon_huile_obj, LV_OBJ_FLAG_HIDDEN); // Hidden by default


    // Create image objects
    icon_freinR_obj = lv_img_create(background_canvas);
    lv_img_set_src(icon_freinR_obj, &icon_freinR);
    lv_obj_set_pos(icon_freinR_obj, 0, 34*2+1); // Top-left
    lv_obj_add_flag(icon_freinR_obj, LV_OBJ_FLAG_HIDDEN); // Hidden by default


    // Create image objects
    icon_freinO_obj = lv_img_create(background_canvas);
    lv_img_set_src(icon_freinO_obj, &icon_freinO);
    lv_obj_set_pos(icon_freinO_obj, 0, 34*3+1); // Top-left
    lv_obj_add_flag(icon_freinO_obj, LV_OBJ_FLAG_HIDDEN); // Hidden by default

    // Create image objects
    icon_essence_obj = lv_img_create(background_canvas);
    lv_img_set_src(icon_essence_obj, &icon_essence);
    lv_obj_set_pos(icon_essence_obj, 0, 34*4+1); // Top-left
    lv_obj_add_flag(icon_essence_obj, LV_OBJ_FLAG_HIDDEN); // Hidden by default

    // Create image objects
    icon_degivre_obj = lv_img_create(background_canvas);
    lv_img_set_src(icon_degivre_obj, &icon_degivre);
    lv_obj_set_pos(icon_degivre_obj, 0, 34*5+1); // Top-left
    lv_obj_add_flag(icon_degivre_obj, LV_OBJ_FLAG_HIDDEN); // Hidden by default

    // Create image objects
    icon_clignotant_obj = lv_img_create(background_canvas);
    lv_img_set_src(icon_clignotant_obj, &icon_clignotant);
    lv_obj_set_pos(icon_clignotant_obj, 400, 1); // Top-left
    lv_obj_add_flag(icon_clignotant_obj, LV_OBJ_FLAG_HIDDEN); // Hidden by default

    // Create image objects
    icon_phare_obj = lv_img_create(background_canvas);
    lv_img_set_src(icon_phare_obj, &icon_phare);
    lv_obj_set_pos(icon_phare_obj, 400, 34*1+1); // Top-left
    lv_obj_add_flag(icon_phare_obj, LV_OBJ_FLAG_HIDDEN); // Hidden by default

    // Create image objects
    icon_pleinPhare_obj = lv_img_create(background_canvas);
    lv_img_set_src(icon_pleinPhare_obj, &icon_pleinPhare);
    lv_obj_set_pos(icon_pleinPhare_obj, 400, 34*2+1); // Top-left
    lv_obj_add_flag(icon_pleinPhare_obj, LV_OBJ_FLAG_HIDDEN); // Hidden by default





    // Créer un canvas
    theCanvas = lv_canvas_create(lv_scr_act());
    lv_canvas_set_buffer(theCanvas, cbuf, CANVAS_WIDTH, CANVAS_HEIGHT, LV_IMG_CF_TRUE_COLOR_ALPHA);
    lv_obj_align(theCanvas, LV_ALIGN_CENTER, 0, 0);
    lv_canvas_fill_bg(theCanvas, lv_color_hex(0x000000), LV_OPA_TRANSP); // Fond transparent

    // Dessiner la montre initialement
    clock_timer_cb2();
}


void setup(void){
    bsp_display_cfg_t cfg = {
        .lvgl_port_cfg = ESP_LVGL_PORT_INIT_CONFIG(),
        .buffer_size = EXAMPLE_LCD_QSPI_H_RES * EXAMPLE_LCD_QSPI_V_RES,
        
  #if LVGL_PORT_ROTATION_DEGREE == 90
        .rotate = LV_DISP_ROT_90,
  #elif LVGL_PORT_ROTATION_DEGREE == 270
        .rotate = LV_DISP_ROT_270,
  #elif LVGL_PORT_ROTATION_DEGREE == 180
        .rotate = LV_DISP_ROT_180,
  #elif LVGL_PORT_ROTATION_DEGREE == 0
        .rotate = LV_DISP_ROT_NONE,
  #endif
    };
  
    bsp_display_start_with_config(&cfg);
    bsp_display_backlight_on();
    bsp_display_brightness_set(10);
    lv_obj_set_style_bg_color(lv_scr_act(), lv_color_hex(0x009A76), LV_PART_MAIN);
    ESP_LOGI(TAG, "Create UI");
    /* Lock the mutex due to the LVGL APIs are not thread-safe */
    bsp_display_lock(0);
  
    bsp_display_unlock();
    lv_timer_handler();

}

// Point d'entrée pour ESP32 (exemple pour ESP-IDF)
void app_main(void) {


    // Désactiver Bluetooth
    //esp_bt_controller_disable();

    // Vérifier que la PSRAM est disponible
    if (esp_psram_is_initialized()) {
        ESP_LOGI(TAG, "PSRAM initialisée, taille: %zu bytes", esp_psram_get_size());
    } else {
        ESP_LOGE(TAG, "Échec de l'initialisation de la PSRAM");
    }

    // Initialiser l'I2C pour le DS3231
    esp_err_t ret = i2c_master_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Échec de l'initialisation I2C: %s", esp_err_to_name(ret));
        return;
    }
    
    // Scanner les périphériques I2C pour détecter le DS3231
    i2c_scan();

    // Initialiser le PCF8575
    ret = pcf8575_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Échec de l'initialisation du PCF8575");
    }

    // Synchroniser l'heure avec le DS3231
    ret = ds3231_sync_time();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Échec de la synchronisation de l'heure avec DS3231");
    }



    // Initialiser LVGL et le pilote d'affichage (à configurer selon votre écran)
    //lv_init();
    setup();
    // Configurer votre pilote d'affichage ici (par exemple, ILI9341, ST7789)
    // Exemple : lvgl_driver_init(); (dépend de votre configuration)

    // Initialiser la montre
    clock_init();

    bool insideLoop = false;

    // Initialiser l'audio
    audio_init();
        
    // Jouer la musique de démarrage
    //play_startup_tune();

    // Boucle principale
    while (1) {
        bsp_display_lock(0);
        clock_timer_cb2();
        lv_timer_handler();
        bsp_display_unlock();
        vTaskDelay(100 / portTICK_PERIOD_MS);
        if (!insideLoop){
            // Jouer la musique de démarrage
            play_startup_tune();

            // Initialiser le Wi-Fi
            wifi_init();
        }
        insideLoop = true;

        // Resynchroniser toutes les 3600 secondes (1 heure)
        static uint32_t last_sync = 0;
        uint32_t current_time = esp_log_timestamp() / 1000; // Temps en secondes
        if (current_time - last_sync >= 3600) {
            esp_err_t ret = ds3231_sync_time();
            if (ret == ESP_OK) {
                ESP_LOGI("DS3231", "Resynchronisation périodique réussie");
            } else {
                ESP_LOGE("DS3231", "Échec de la resynchronisation");
            }
            last_sync = current_time;
        }
    }

    // Libérer le buffer à la fin (optionnel)
    if (cbuf != NULL) {
        heap_caps_free(cbuf);
    }
}