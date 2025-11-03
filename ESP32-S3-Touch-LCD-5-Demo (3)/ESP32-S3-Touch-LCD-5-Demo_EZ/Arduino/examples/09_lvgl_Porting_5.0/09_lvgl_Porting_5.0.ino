//==========================================================================
// Includes
//==========================================================================
#include <Arduino.h>
#include <ArduinoJson.h>
#include <WiFi.h>
#include <esp_display_panel.hpp>
#include <lvgl.h>
#include <SD.h>
#include <SPI.h>
#include <vector>
#include <string.h>
#include "lvgl_v8_port.h"
#include "waveshare_sd_card.h"
#include <HardwareSerial.h>

//==========================================================================
// Блок для объявления изображений
//==========================================================================
LV_IMG_DECLARE(symbol_flask);
LV_IMG_DECLARE(symbol_pipette);
LV_IMG_DECLARE(symbol_molecule);
LV_IMG_DECLARE(symbol_tooth);
LV_IMG_DECLARE(symbol_drill);
LV_IMG_DECLARE(symbol_uv_lamp);
LV_IMG_DECLARE(symbol_wild);
LV_IMG_DECLARE(symbol_scatter);

//==========================================================================
// Global Definitions & Fonts
//==========================================================================
using namespace esp_panel::drivers;
using namespace esp_panel::board;

const bool ENABLE_SPLASH_SCREEN = true; // true - показывать заствку, false - НЕ показывать заствку
#define LVGL_HEAP_SIZE (96 * 1024)
#define FILE_CONTENT_BUFFER_SIZE 131072 // 128 KB
const char* config_file_path = "/cfg.json"; 
const int PROFILES_PER_PAGE = 12;

LV_FONT_DECLARE(montserrat_rus_16);
LV_FONT_DECLARE(montserrat_rus_18);
LV_FONT_DECLARE(montserrat_rus_22);


//==========================================================================
// Data Structures
//==========================================================================
// Режим 1: Глазурь
struct LabModeGlaze {
    float uv_on_sec;                // 0.1 - 3.0, default 2.0
    float uv_off_sec;               // 0.1 - 3.0, default 2.0
    bool use_monomer_blow;          // <<< ДОБАВЛЕНО
    int monomer_blow_min;           // 1 - 10, default 1
    int uv_mode;                    // <<< ДОБАВЛЕНО (0=Type1, 1=Type2, 2=Both)
    int uv_exposure_sec;            // 10 - 200, default 10
    bool use_cooling;
    bool use_nitrogen;              
    int nitrogen_target_percent;
    int nitrogen_boost_sec;
};
// Режим 2: Ремонт модели
struct LabModeRepair {
    int countdown_sec;              // 5 - 60, default 5
    int uv_exposure_sec;            // 5 - 60, default 5
};

// Режим 3: Повышение прочности
struct LabModeStrength {
    int chamber_temp_c;             // 50 - 80, default 50
    int hold_time_min;              // 30 - 180, default 30
    bool use_cooling;
    int uv_pulse_duration_sec;      // Длительность импульса УФ в секундах (1-10, default 1)
    int uv_pulse_interval_min;      // Интервал между импульсами в минутах (1-10, default 1)
};

// Режим 4: Термокамера
struct LabModeThermal {
    int chamber_temp_c;             // 40 - 60, default 40
    int hold_time_min;              // 5 - 30, default 5
};

// Режим 5: Осветление композита
struct LabModeLighten {
    int chamber_temp_c;             // 80 (читается из файла, но не редактируется в UI)
    int hold_time_min;              // 10 - 30, default 10
    bool use_cooling;
};

// Режим 6: Затемнение композита
struct LabModeDarken {
    int uv_exposure_min;            // 1 - 15, default 1
    bool use_cooling;
};

// Главная структура, которая объединяет все режимы
struct LaboratorySettingsData {
    LabModeGlaze glaze;
    LabModeRepair repair;
    LabModeStrength strength;
    LabModeThermal thermal;
    LabModeLighten lighten;
    LabModeDarken darken;
};
struct ProfileData {
    int id;
    char name[201];
    bool thermal_chamber_enabled;
    int thermal_chamber_temp;
    int heat_exchange_hold_sec;
    bool nitrogen_use_enabled;
    int nitrogen_target_percent;
    int nitrogen_boost_sec;
    int primary_uv_exposure_sec;
    int secondary_uv_exposure_sec;
    bool chamber_cooling_enabled;
    int  primary_uv_mode; // 0=Type1, 1=Type2, 2=Both
    float primary_uv_flicker_on_sec;
    int  secondary_uv_mode; // 0=Type1, 1=Type2, 2=Both
    int  tertiary_uv_exposure_sec;
    int  tertiary_uv_mode; // 0=Type1, 1=Type2, 2=Both
    int  post_cooling_air_purge_sec;
};

struct GlobalSettingsData {
    bool is_first_run;
    bool is_heater_error;
    bool nitrogen_system_enabled;
    bool compressed_air_system_enabled;
    int language;
    int theme;
    int screen_timeout_mode;
};


typedef struct struct_message {
    char command[32];
    bool uv_405_on;
    bool uv_430_on;
    bool status_flag;
    int value_int;
    float value_float;
    int value_flickering_405;
    int value_flickering_430;
    char text_payload[100];
} struct_message;

struct SplashScreenData {
    lv_obj_t* screen;
    lv_obj_t* title;
};
// --- Вспомогательный класс-аллокатор для ArduinoJson v7 для работы с PSRAM ---
struct SpiRamAllocator {
  void* allocate(size_t size) {
    return heap_caps_malloc(size, MALLOC_CAP_SPIRAM);
  }
  void deallocate(void* pointer) {
    heap_caps_free(pointer);
  }
  void* reallocate(void* ptr, size_t new_size) {
    return heap_caps_realloc(ptr, new_size, MALLOC_CAP_SPIRAM);
  }
};
//==========================================================================
// Global Variables
//==========================================================================

// Указатели на тройные кнопки
lv_obj_t* btn_primary_1, *btn_primary_2, *btn_primary_3;
lv_obj_t* btn_secondary_1, *btn_secondary_2, *btn_secondary_3;
lv_obj_t* btn_tertiary_1, *btn_tertiary_2, *btn_tertiary_3;

// --- System & Core Objects ---
Board* board = nullptr;
esp_expander::CH422G* ch422g = nullptr;
#define RX1_PIN 15 // желтый права
#define TX1_PIN 44 // зелёный слева
HardwareSerial MySerial1(2);
static uint8_t lvgl_heap[LVGL_HEAP_SIZE];
static const char* uv_btnm_map[] = {"1", "2", "3", ""};
static lv_style_t style_my_text_16;
static lv_style_t style_my_text_18;
static lv_style_t style_my_text_18_white;
static lv_style_t style_just_font_18;
static lv_font_t font_18_with_fallback;
static lv_style_t style_my_text_22;
uint32_t EVENT_REFRESH_PROFILES;
static lv_timer_t* help_blink_timer = nullptr;

// --- Цвета для кнопок UV-режима ---
// Светлая тема
lv_color_t light_uv_btn1_active, light_uv_btn1_inactive;
lv_color_t light_uv_btn2_active, light_uv_btn2_inactive;
lv_color_t light_uv_btn3_active, light_uv_btn3_inactive;
// Темная тема
lv_color_t dark_uv_btn1_active, dark_uv_btn1_inactive;
lv_color_t dark_uv_btn2_active, dark_uv_btn2_inactive;
lv_color_t dark_uv_btn3_active, dark_uv_btn3_inactive;

// --- Стили для тем ---
// Светлая тема
static lv_style_t style_light_bg;
static lv_style_t style_light_text;
static lv_style_t style_light_spinner_bg;
static lv_style_t style_light_spinner_indic;
static lv_style_t style_light_btn;
static lv_style_t style_light_block_border;
static lv_style_t style_light_block_bg;
static lv_style_t style_light_block_header;
static lv_style_t style_light_tile_bg;
static lv_style_t style_light_tile_border; 
static lv_style_t style_light_arrow; 
static lv_style_t style_light_subheader;    
static lv_style_t style_light_textarea;     
static lv_style_t style_light_column_header;
// Темная тема
static lv_style_t style_dark_bg;
static lv_style_t style_dark_text;
static lv_style_t style_dark_spinner_bg;
static lv_style_t style_dark_spinner_indic;
static lv_style_t style_dark_btn;
static lv_style_t style_dark_block_border;
static lv_style_t style_dark_block_bg;
static lv_style_t style_dark_block_header; 
static lv_style_t style_block_header;
static lv_style_t style_dark_tile_bg;
static lv_style_t style_dark_tile_border;  
static lv_style_t style_dark_arrow;    
static lv_style_t style_dark_subheader;  
static lv_style_t style_dark_textarea;     
static lv_style_t style_dark_column_header; 
static lv_style_t style_casino_cell;
static lv_style_t style_info_box;

// --- Application State & Data ---
bool profile_save_action_pending = false; 
ProfileData pending_profile_to_save;      
bool sd_card_initialized = false;
bool needs_list_refresh = false;
bool is_on_splash_screen = false;
int current_profile_next_id = 1;
int current_profile_list_page = 0;
int total_profile_pages = 0;
GlobalSettingsData current_global_settings;
LaboratorySettingsData current_lab_settings;
ProfileData current_active_profile_data;
std::vector<ProfileData> all_profiles_data;
std::vector<String> service_keys;
char* file_content_buffer = NULL;
int current_selected_profile_id = -1;
char decision_text[50];
char full_status_text[100];
bool heater_decision = false;
bool cooling_decision = false;
bool is_lab_mode_running = false;

// Указатели на виджеты плиток профилей для пула объектов
lv_obj_t* profile_tiles[PROFILES_PER_PAGE];
lv_obj_t* profile_tile_icons[PROFILES_PER_PAGE];
lv_obj_t* profile_tile_labels[PROFILES_PER_PAGE];

bool lab_settings_action_pending = false;
int next_lab_action = 0; // 0 - нет действия, 1 - назад, 2..7 - старт режимов 1..6
bool lab_screen_load_pending = false;
int pending_lab_mode_id = 0;


bool is_nitrogen_test_active = false;
bool is_air_test_active = false;
volatile bool nitrogen_error_dialog_pending = false; // Флаг для показа окна

volatile bool waiting_for_stop_ack = false; // Флаг, что мы ждем подтверждения остановки
unsigned long stop_ack_timeout_start = 0;   // Таймер для защиты от зависания
const unsigned long STOP_ACK_TIMEOUT_MS = 1500; // Таймаут ожидания = 3 секунды

// --- Глобальный JSON документ для всех операций отправки ---
StaticJsonDocument<1024> command_json_doc;

// <<< ДОБАВЛЯЕМ ГЛОБАЛЬНЫЙ ДОКУМЕНТ ДЛЯ CFG.JSON >>>
// Используем using для удобства, как у тебя
using SpiRamJsonDocument = BasicJsonDocument<SpiRamAllocator>;
// static SpiRamJsonDocument config_doc(FILE_CONTENT_BUFFER_SIZE); // Выделяем память один раз при старте

//==========================================================================
// Блок для переменных игры в казик
//==========================================================================
// --- Переменные и структуры для игры "SpectraSlots" ---
#define GRID_COLS 5
#define GRID_ROWS 4
#define SYMBOL_COUNT 8 // Количество уникальных символов (0-7)
#define REEL_LENGTH 30

bool main_process_running = false;
// Состояние игры
enum GameState {
    READY_TO_SPIN,
    ANIMATING
};
GameState game_state = READY_TO_SPIN;
bool is_cascade_active = false;

// Структура для одной ячейки на поле
struct GameSymbol {
    int type;           // ID символа
    lv_obj_t* img_obj;  // Указатель на объект картинки
//==========================================================================
// ИЗМЕНЕНИЕ 3: Добавь эти два флага
//==========================================================================
    bool is_winning;    // Флаг для подсветки выигрыша
    bool to_be_removed; // Флаг для удаления в каскаде
    lv_color_t win_color;
//==========================================================================
};

GameSymbol game_grid[GRID_COLS][GRID_ROWS];
int player_balance = 1000;
int current_bet = 1;
int current_spin_win = 0;

// Указатели на UI элементы игры для обновления
lv_obj_t* label_balance_value;
lv_obj_t* label_bet_value;
lv_obj_t* label_win_value;
lv_obj_t* btn_spin;
//==========================================================================

// --- Timers & Process Control ---
unsigned long splash_screen_start_time, hold_timer_start, heater_error_timer, air_error_timer;
unsigned long nitrogen_error_timer, primary_uv_timer_start, secondary_uv_timer_start;
unsigned long process_start_time_ms = 0;   
int current_stage_total_seconds = 0;  
float temp_at_heater_error_check_start = 0.0f;
float temp_at_air_error_check_start = 0.0f;
float o2_at_error_check_start = 0.0f;
volatile int choice_dialog_result = 0;
float current_stage_target_value = 0.0f; // Для температуры или % O2
float current_stage_start_value = 0.0f;  // Начальное значение при старте этапа
String current_process_stage = "";

// --- System Screen Timeout ---
// const unsigned long SCREEN_TIMEOUT_MS = 10000; // <<< TEST: 10 секунд для простого теста
unsigned long last_interaction_time = 0;       // Время последнего касания экрана
bool is_screen_on = true;                      // Флаг состояния подсветки экрана
// bool ignore_input_until_release = false;
unsigned long input_disable_end_time = 0; // Время, когда нужно снова включить тач
bool input_is_disabled = false; 
// unsigned long shield_hide_time = 0;

//-------------------------------------------------
// LVGL UI Object Pointers
//-------------------------------------------------
// --- Screens & Main Containers ---
lv_obj_t * screen_splash, *screen_main_app, *screen_profile_details, *screen_profile_edit;
lv_obj_t * screen_loading;                      // <--- ДОБАВЬТЕ ЭТУ СТРОКУ
lv_obj_t * label_loading_text;                  // <--- ДОБАВЬТЕ ЭТУ СТРОКУ
lv_obj_t * screen_settings, *screen_process_execution, *screen_service_lock, *screen_secret_game;
lv_obj_t * screen_laboratory; 
lv_obj_t * main_screen_content_container;
lv_obj_t* screen_help;
lv_obj_t* screen_to_return_after_process = NULL;

lv_obj_t* input_shield = NULL;

lv_obj_t* screen_test_nitrogen;
lv_obj_t* label_nitrogen_test_header;
lv_obj_t* btn_nitrogen_test_press;
lv_obj_t* label_btn_nitrogen_test_press;
lv_obj_t* btn_nitrogen_test_back;
lv_obj_t* label_btn_nitrogen_test_back;
lv_obj_t* screen_test_air;
lv_obj_t* label_air_test_header;
lv_obj_t* btn_air_test_press;
lv_obj_t* label_btn_air_test_press;
lv_obj_t* btn_air_test_back;
lv_obj_t* label_btn_air_test_back;

// --- Laboratory Mode Screen Widgets ---
lv_obj_t* label_lab_header;
lv_obj_t* btn_lab_back;
lv_obj_t* label_btn_lab_back;
lv_obj_t* label_lab_tile_1;
lv_obj_t* label_lab_tile_2;
lv_obj_t* label_lab_tile_3;
lv_obj_t* label_lab_tile_4;
lv_obj_t* label_lab_tile_5;
lv_obj_t* label_lab_tile_6;

// --- Lab Mode: Glaze Screen Widgets ---
// Указатели на виджеты экрана "Глазурь"

// Главные элементы
lv_obj_t* screen_lab_glaze;
lv_obj_t* label_glaze_header;
lv_obj_t* btn_glaze_help;

// Поля ввода
lv_obj_t* ta_glaze_uv_on;
lv_obj_t* ta_glaze_uv_off;
lv_obj_t* ta_glaze_monomer_blow;
lv_obj_t* ta_glaze_uv_exposure;
lv_obj_t* ta_glaze_nitrogen_target;
lv_obj_t* ta_glaze_nitrogen_boost;

// Переключатели
lv_obj_t* sw_glaze_cooling;
lv_obj_t* sw_glaze_nitrogen;
lv_obj_t* sw_glaze_monomer_blow; 

// Контейнеры и метки (для доступа и перевода)
lv_obj_t* nitrogen_glaze_container; // Контейнер для настроек азота
lv_obj_t* monomer_blow_container;
lv_obj_t* label_glaze_title_flicker;
lv_obj_t* label_glaze_uv_on;
lv_obj_t* label_glaze_uv_off;
lv_obj_t* label_glaze_title_timers;
lv_obj_t* label_glaze_monomer_blow_switch;
lv_obj_t* label_glaze_monomer_blow;
lv_obj_t* label_glaze_uv_mode_title;
lv_obj_t* label_glaze_uv_mode_status;
lv_obj_t* label_glaze_uv_exposure;
lv_obj_t* label_glaze_title_aux;
lv_obj_t* label_glaze_cooling;
lv_obj_t* label_glaze_nitrogen;
lv_obj_t* label_glaze_nitrogen_target;
lv_obj_t* label_glaze_nitrogen_boost;
lv_obj_t* label_glaze_nitrogen_warning;

// Кнопки-переключатели UV-режима
lv_obj_t* btn_glaze_uv_1;       
lv_obj_t* btn_glaze_uv_2;          
lv_obj_t* btn_glaze_uv_3; 

// Кнопки в подвале
lv_obj_t* btn_glaze_start;
lv_obj_t* label_btn_glaze_start;
lv_obj_t* btn_glaze_back;
lv_obj_t* label_btn_glaze_back;

// --- Lab Mode: Repair Screen Widgets ---
lv_obj_t* screen_lab_repair;
lv_obj_t* label_repair_header;
lv_obj_t* btn_repair_help;
lv_obj_t* label_repair_info_text; // Для текста-предупреждения
lv_obj_t* label_repair_title_timers;
lv_obj_t* label_repair_countdown;
lv_obj_t* ta_repair_countdown;
lv_obj_t* label_repair_uv_exposure;
lv_obj_t* ta_repair_uv_exposure;
// lv_obj_t* label_repair_arrow;         
lv_obj_t* label_repair_arrow_text;     
lv_obj_t* btn_repair_start;
lv_obj_t* label_btn_repair_start;
lv_obj_t* btn_repair_back;
lv_obj_t* label_btn_repair_back;

// --- Lab Mode: Strength Boost Screen Widgets ---
lv_obj_t* screen_lab_strength;
lv_obj_t* label_strength_header;
lv_obj_t* btn_strength_help;
lv_obj_t* label_strength_title_thermo;
lv_obj_t* label_strength_temp;
lv_obj_t* ta_strength_temp;
lv_obj_t* label_strength_hold_time;
lv_obj_t* ta_strength_hold_time;
lv_obj_t* label_strength_cooling;
lv_obj_t* sw_strength_cooling;
lv_obj_t* label_strength_title_uv;
lv_obj_t* label_strength_uv_pulse_duration;
lv_obj_t* ta_strength_uv_pulse_duration;
lv_obj_t* label_strength_uv_pulse_interval;
lv_obj_t* ta_strength_uv_pulse_interval;
lv_obj_t* label_strength_info_uv_text;
lv_obj_t* btn_strength_start;
lv_obj_t* label_btn_strength_start;
lv_obj_t* btn_strength_back;
lv_obj_t* label_btn_strength_back;

// --- Lab Mode: Thermal Chamber Screen Widgets ---
lv_obj_t* screen_lab_thermal;
lv_obj_t* label_thermal_header;
lv_obj_t* btn_thermal_help;
lv_obj_t* label_thermal_title_thermo;
lv_obj_t* label_thermal_temp;
lv_obj_t* ta_thermal_temp;
lv_obj_t* label_thermal_info; // Для информационного текста
lv_obj_t* label_thermal_hold_time;
lv_obj_t* ta_thermal_hold_time;
lv_obj_t* btn_thermal_start;
lv_obj_t* label_btn_thermal_start;
lv_obj_t* btn_thermal_back;
lv_obj_t* label_btn_thermal_back;

// --- Lab Mode: Lighten Screen Widgets ---
lv_obj_t* screen_lab_lighten;
lv_obj_t* label_lighten_header;
lv_obj_t* btn_lighten_help;
lv_obj_t* label_lighten_title_thermo;
lv_obj_t* label_lighten_temp_fixed;
lv_obj_t* label_lighten_hold_time;
lv_obj_t* ta_lighten_hold_time;
lv_obj_t* label_lighten_cooling;
lv_obj_t* sw_lighten_cooling;
lv_obj_t* btn_lighten_start;
lv_obj_t* label_btn_lighten_start;
lv_obj_t* btn_lighten_back;
lv_obj_t* label_btn_lighten_back;

// --- Lab Mode: Darken Screen Widgets ---
lv_obj_t* screen_lab_darken;
lv_obj_t* label_darken_header;
lv_obj_t* btn_darken_help;
lv_obj_t* label_darken_title_params;
lv_obj_t* label_darken_uv_exposure;
lv_obj_t* ta_darken_uv_exposure;
lv_obj_t* label_darken_cooling;
lv_obj_t* sw_darken_cooling;
lv_obj_t* btn_darken_start;
lv_obj_t* label_btn_darken_start;
lv_obj_t* btn_darken_back;
lv_obj_t* label_btn_darken_back;

// --- Dialogs, Keyboards & Modal Elements ---
lv_obj_t * screen_confirm_delete_dialog, *label_confirm_delete_text;
lv_obj_t* label_confirm_delete_title;
lv_obj_t* label_confirm_btn_cancel;
lv_obj_t* label_confirm_btn_delete;
lv_obj_t * screen_info_dialog, *label_info_dialog_text, *btn_info_dialog_ok;
lv_obj_t * screen_choice_dialog, *btn_choice_dialog_skip, *btn_choice_dialog_cancel;
lv_obj_t * kb_edit_numeric, *kb_edit_alpha, *kb_service_code;
lv_obj_t * overlay_modal_input_bg, *modal_input_container, *modal_input_title_label;
lv_obj_t * ta_modal_input, *current_target_ta;
lv_obj_t* screen_keyboard;
lv_obj_t* ta_keyboard_proxy;
lv_obj_t* screen_to_return_to = NULL;

// --- Main Screen Widgets ---
lv_obj_t * list_header_label_main, *list_profiles_main, *ta_profile_input_main;
lv_obj_t * label_status_msg_main, *ta_dummy_for_new_profile;
lv_obj_t * btn_profiles_prev, *btn_profiles_next, *btn_secret_trigger;
lv_obj_t* label_btn_add_main;
lv_obj_t* label_btn_lab_main;
lv_obj_t* label_btn_settings_main;

lv_obj_t* btn_add_main;
lv_obj_t* btn_lab_main;
lv_obj_t* btn_settings_main;

// --- Profile Details Screen Widgets ---
lv_obj_t * label_detail_view_profile_name, *label_detail_view_id;
lv_obj_t * label_detail_view_thermal_chamber_enabled, *label_detail_view_thermal_chamber;
lv_obj_t * label_detail_view_nitrogen, *label_detail_view_primary_uv;
lv_obj_t * label_detail_view_secondary_uv, *label_detail_view_chamber_cooling;
lv_obj_t * label_detail_view_tertiary_uv;
lv_obj_t* label_detail_header;
lv_obj_t* label_detail_btn_start;
lv_obj_t* label_detail_btn_edit;
lv_obj_t* label_detail_btn_delete;
lv_obj_t* label_detail_btn_close;

// --- Profile Edit Screen Widgets ---
// Header
lv_obj_t* ta_edit_profile_name;
lv_obj_t* btn_help_section;
lv_obj_t* label_btn_help_section;

// Main Layout Containers & Headers
lv_obj_t* header_container;
lv_obj_t* main_content_container;
lv_obj_t* left_column;
lv_obj_t* right_column;
lv_obj_t* footer_container;
lv_obj_t* block_uv_all_stages;

lv_obj_t* row_uv_primary;
lv_obj_t* row_uv_secondary;
lv_obj_t* row_uv_tertiary;

lv_obj_t* header_uv_params;
lv_obj_t* header_poly_params;

// UV Params (Left Column)
lv_obj_t* block_uv_primary;
lv_obj_t* label_uv_primary_title;
lv_obj_t* ta_edit_primary_uv;
lv_obj_t* ta_edit_flicker_rate;
lv_obj_t* btnm_primary_uv_mode;
lv_obj_t* label_primary_uv_mode_status;

lv_obj_t* block_uv_secondary;
lv_obj_t* label_uv_secondary_title;
lv_obj_t* ta_edit_secondary_uv;
lv_obj_t* btnm_secondary_uv_mode;
lv_obj_t* label_secondary_uv_mode_status;

lv_obj_t* block_uv_tertiary;
lv_obj_t* label_uv_tertiary_title;
lv_obj_t* ta_edit_tertiary_uv;
lv_obj_t* btnm_tertiary_uv_mode;
lv_obj_t* label_tertiary_uv_mode_status;

// Polymerization Params (Right Column)
lv_obj_t* block_gases;
lv_obj_t* sw_edit_nitrogen;
lv_obj_t* nitrogen_elements_container;
lv_obj_t* ta_edit_nitrogen_target;
lv_obj_t* ta_edit_nitrogen_boost;
lv_obj_t* sw_edit_chamber_cooling;
lv_obj_t* cooling_elements_container;  
lv_obj_t* ta_edit_post_cooling_purge_sec; 
lv_obj_t* label_edit_post_cooling_purge_title;

lv_obj_t* block_thermal;
lv_obj_t* sw_edit_thermal_chamber_enable;
lv_obj_t* thermal_elements_container;
lv_obj_t* ta_edit_thermal_temp;
lv_obj_t* ta_edit_heat_hold;

// Footer Buttons
lv_obj_t* btn_save_changes;
lv_obj_t* label_btn_save;
lv_obj_t* btn_cancel_edit;
lv_obj_t* label_btn_cancel;

// Translation Hooks (for labels inside blocks)
lv_obj_t* label_edit_name_title;
lv_obj_t* label_edit_flicker_rate_title;
lv_obj_t* label_edit_primary_uv_time_title;
lv_obj_t* label_edit_secondary_uv_time_title;
lv_obj_t* label_edit_tertiary_uv_time_title;
lv_obj_t* label_edit_nitrogen_title;
lv_obj_t* label_edit_nitrogen_target_title;
lv_obj_t* label_edit_nitrogen_boost_title;
lv_obj_t* label_edit_cooling_title;
lv_obj_t* label_edit_thermal_chamber_title;
lv_obj_t* label_edit_thermal_temp_title;
lv_obj_t* label_edit_heat_hold_title;
lv_obj_t* block_post_cooling_purge; 
lv_obj_t* label_post_cooling_title; 
lv_obj_t* btnm_post_cooling_purge;

// --- Help Screen Widgets ---
lv_obj_t* label_help_title;
lv_obj_t* label_help_content;
lv_obj_t* btn_help_close;
lv_obj_t* label_btn_help_close;

// --- Process Execution Screen Widgets ---
lv_obj_t* label_process_profile_name;
lv_obj_t * label_process_status_title, *label_process_status_detail;
lv_obj_t * btn_process_cancel;
// lv_obj_t * spinner_process_execution;
lv_obj_t* arc_process_progress;
lv_obj_t* label_btn_process_cancel;
lv_obj_t * bar_process_progress;
lv_obj_t * label_process_stage_name;
// lv_obj_t* label_process_repair_info;
lv_obj_t* repair_mode_indicator_obj;

// --- Settings Screen Widgets ---
lv_obj_t * sw_settings_global_nitrogen_enabled, *sw_settings_global_air_enabled;
lv_obj_t * lang_toggle_box, *theme_toggle_box;
lv_obj_t* timeout_toggle_box;
lv_obj_t* content_grid_settings;
lv_obj_t* left_block_settings;
lv_obj_t* right_block_settings;
lv_obj_t* label_settings_timeout;
lv_obj_t* label_timeout_opt1; // 5 min
lv_obj_t* label_timeout_opt2; // 15 min
lv_obj_t* label_timeout_opt3; // 30 min
lv_obj_t* label_timeout_opt4; // 60 min
// --- ГЛОБАЛЬНЫЕ УКАЗАТЕЛИ ДЛЯ ПЕРЕВОДА ---
lv_obj_t* label_settings_title;
lv_obj_t* label_settings_nitrogen;
lv_obj_t* label_settings_air;
lv_obj_t* label_settings_language;
lv_obj_t* label_settings_lang_opt1; // ENG
lv_obj_t* label_settings_lang_opt2; // RUS
lv_obj_t* label_settings_theme;
lv_obj_t* label_settings_theme_opt1; // Light
lv_obj_t* label_settings_theme_opt2; // Dark
lv_obj_t* btn_settings_save_and_back; // Указатель на саму кнопку
lv_obj_t* btn_settings_test_nitro;
lv_obj_t* btn_settings_test_air;

// --- Service Lock Screen Widgets ---
lv_obj_t * ta_service_code_input, *label_service_lock_msg, *service_lock_modal_overlay;
lv_obj_t * ta_service_lock_modal_input, *btn_service_lock_enter;
lv_obj_t* label_splash_error = NULL;

//==========================================================================
// Function Prototypes
//==========================================================================

// --- Initialization & System ---
bool initializeSDCard();
void loadConfiguration();
bool saveConfiguration();

// --- UI Building ---
static void build_splash_screen(lv_obj_t* parent_screen);
static void build_main_app_screen(lv_obj_t* parent_screen);
static void build_laboratory_screen(lv_obj_t* parent_screen);
static void glaze_screen_switch_event_cb(lv_event_t* e);
static void build_lab_glaze_screen(lv_obj_t* parent_screen);
static void repair_screen_event_cb(lv_event_t* e);
static void build_lab_repair_screen(lv_obj_t* parent_screen);
static void strength_screen_event_cb(lv_event_t* e);
static void build_lab_strength_screen(lv_obj_t* parent_screen);
static void thermal_screen_event_cb(lv_event_t* e); 
static void build_lab_thermal_screen(lv_obj_t* parent_screen);
static void lighten_screen_event_cb(lv_event_t* e); 
static void build_lab_lighten_screen(lv_obj_t* parent_screen); 
static void darken_screen_event_cb(lv_event_t* e); 
static void build_lab_darken_screen(lv_obj_t* parent_screen); 
static void build_profile_details_screen(lv_obj_t* parent_screen);
static void build_profile_edit_screen(lv_obj_t* parent_screen);
static void build_settings_screen(lv_obj_t* parent_screen);
static void build_process_execution_screen(lv_obj_t* parent_screen);
static void build_service_lock_screen(lv_obj_t* parent_screen);
static void build_test_nitrogen_screen(lv_obj_t* parent_screen);
static void apply_theme_to_test_nitrogen_screen();
static void test_nitrogen_screen_event_cb(lv_event_t* e);
static void build_test_air_screen(lv_obj_t* parent_screen);
static void apply_theme_to_test_air_screen();
static void test_air_screen_event_cb(lv_event_t* e);
static void build_secret_game_screen(lv_obj_t* parent_screen);
static void casino_event_cb(lv_event_t* e);
static void update_casino_ui_values();
static void start_spin();
static void generate_and_display_grid();
static bool find_and_mark_wins();
static void start_cascade_animation();
static void cascade_fall_anim_finish_cb(lv_anim_t* a);
static void apply_theme_to_casino_screen();
static void apply_theme_to_loading_screen();
static void build_confirm_delete_dialog(lv_obj_t* parent_for_dialog);
static void build_info_dialog(lv_obj_t* parent_layer);
static void build_choice_dialog(lv_obj_t* parent_layer);
static void build_help_screen(lv_obj_t* parent_screen);
static void build_input_shield(void);
static void build_loading_screen(lv_obj_t* parent_screen);

// --- Custom Widget Functions ---
static void uv_mode_selector_event_cb(lv_event_t * e);
// static uint16_t get_checked_btnmatrix_id(lv_obj_t* btnm);
static void build_keyboard_screen(lv_obj_t* parent_screen);
static void keyboard_screen_bg_clicked_cb(lv_event_t* e);

// --- UI Event Handlers ---
static void profile_list_event_handler(lv_event_t * e);
static void profile_detail_start_event_cb(lv_event_t * e);
static void profile_detail_edit_btn_event_cb(lv_event_t * e);
static void profile_detail_delete_btn_event_cb(lv_event_t * e);
static void profile_detail_close_event_cb(lv_event_t * e);
static void profile_edit_save_changes_btn_event_cb(lv_event_t * e);
static void profile_edit_cancel_btn_event_cb(lv_event_t * e);
static void confirm_dialog_cancel_btn_event_cb(lv_event_t* e);
static void confirm_dialog_delete_btn_event_cb(lv_event_t* e);
static void info_dialog_ok_event_cb(lv_event_t* e);
static void choice_dialog_event_cb(lv_event_t* e);
static void process_execution_cancel_btn_event_cb(lv_event_t * e);
static void settings_screen_event_cb(lv_event_t * e);
static void btn_goto_settings_event_cb(lv_event_t* e);
static void laboratory_mode_btn_event_cb(lv_event_t * e);
static void lab_screen_back_event_cb(lv_event_t * e);
static void lab_mode_tile_event_cb(lv_event_t * e); 
static void glaze_screen_event_cb(lv_event_t* e);
static void service_lock_screen_event_cb(lv_event_t* e);
static void service_code_keyboard_event_cb(lv_event_t* e);
static void modal_input_keyboard_event_cb(lv_event_t* e);
static void modal_input_overlay_click_event_cb(lv_event_t* e);
static void thermal_temp_slider_event_cb(lv_event_t * e);
static void thermal_chamber_enable_switch_event_cb(lv_event_t * e);
static void profile_switch_value_changed_event_cb(lv_event_t * e);
static void numeric_textarea_focus_event_cb(lv_event_t * e);
static void generic_textarea_defocus_event_cb(lv_event_t * e);
static void alpha_textarea_focus_event_cb(lv_event_t* e);
static void profile_list_prev_btn_event_cb(lv_event_t* e);
static void profile_list_next_btn_event_cb(lv_event_t* e);
static void secret_button_event_cb(lv_event_t* e);
static void secret_game_back_event_cb(lv_event_t* e);
static void help_button_event_cb(lv_event_t* e);
static void help_blink_timer_cb(lv_timer_t* timer);
static void lab_mode_start_event_cb(lv_event_t* e);
void prepare_and_send_lab_command(int mode_id);
static void lab_mode_param_changed_event_cb(lv_event_t * e);
static void post_cooling_purge_event_cb(lv_event_t * e);
static void trigger_lab_action(int action_code);
static void build_profile_list_pool();

// --- Core Logic, File System & Helpers ---
void apply_current_theme_to_all_screens();
void displayProfileListPage();
void update_profile_details_screen(const ProfileData& profile_to_display);
void enter_service_lock_mode(const char* message_eng, const char* message_rus);
bool handle_save_new_profile_logic(const char* profile_input_name);
bool readFileContentToBuffer_ino(fs::FS &fs_ref, const char * path, char* buffer, size_t buffer_size);
static void show_modal_input(lv_obj_t* target_ta, lv_keyboard_mode_t kb_mode);
static void show_info_dialog(const char* title, const char* message_text);
static void show_choice_dialog(const char* title, const char* message_text);
// static void create_settings_row(lv_obj_t* parent, lv_obj_t** p_label, lv_obj_t** p_switch, const char* user_data);
static void create_custom_toggle(lv_obj_t* parent, lv_obj_t** p_label, lv_obj_t** p_toggle_box, lv_obj_t** p_lbl1, lv_obj_t** p_lbl2);
static void update_custom_toggle_ui(lv_obj_t* toggle_box, int active_index);
static void update_timeout_toggle_ui(lv_obj_t* toggle_box, int active_index);
int roundToStep(int value, int step);
void validate_numeric_input(lv_event_t * e, int min_val, int max_val);
const char* translateSystemStatus(const char* status_msg);
void validate_float_input(lv_event_t * e, float min_val, float max_val);
const char* tr(const char* text_eng);
void update_telemetry_display(float temp, float o2, int timer_rem);
void flush_serial_buffer();
void handle_lab_save_and_action();
void completeCancellationSequence();
void log_memory_status(const char* event_name);
static void prepare_and_show_lab_screen(int mode_id);
// --- НОВЫЕ ПРОТОТИПЫ ДЛЯ ЖИЗНЕННОГО ЦИКЛА ЭКРАНОВ ---
void cleanup_process_execution_screen();
static void setup_profile_edit_screen();
void cleanup_profile_edit_screen();
void load_screen(lv_obj_t* target_screen);

void handle_profile_save_action();
void trigger_profile_save_action(const ProfileData& profile_to_save);
void translate_main_screen_ui();

// Раздел 1: Вспомогательные функции 
// ==========================================================================

void translate_main_screen_ui() {
    if (!list_header_label_main) return; // Защита

    if (current_global_settings.language == 1) { // RUS
        lv_label_set_text(label_btn_add_main, LV_SYMBOL_PLUS " Добавить Профиль");
        lv_label_set_text(label_btn_lab_main, LV_SYMBOL_SETTINGS " Лаб. режим");
        lv_label_set_text(label_btn_settings_main, "Настройки");
        lv_obj_add_style(label_btn_add_main, &style_my_text_18_white, 0);
        lv_obj_add_style(label_btn_lab_main, &style_my_text_18_white, 0);  
        lv_obj_add_style(label_btn_settings_main, &style_my_text_18_white, 0); 
    } else { // ENG
        lv_label_set_text(label_btn_add_main, LV_SYMBOL_PLUS " Add Profile");
        lv_label_set_text(label_btn_lab_main, LV_SYMBOL_SETTINGS " Lab Mode");
        lv_label_set_text(label_btn_settings_main, "Settings");
        lv_obj_add_style(label_btn_add_main, &style_my_text_18_white, 0); 
        lv_obj_add_style(label_btn_lab_main, &style_my_text_18_white, 0);  
        lv_obj_add_style(label_btn_settings_main, &style_my_text_18_white, 0); 
    }
}

void apply_current_theme_to_all_screens() {
    Serial.println("--- Applying current theme to ALL UI elements ---");
    // Применяем тему к каждому экрану и диалогу
    apply_theme_to_main_app_screen();
    apply_theme_to_profile_details_screen();
    apply_theme_to_profile_edit_screen();
    apply_theme_to_settings_screen();
    apply_theme_to_laboratory_screen();
    apply_theme_to_process_screen();
    apply_theme_to_keyboard_screen();
    apply_theme_to_help_screen();
    apply_theme_to_casino_screen();
    apply_theme_to_loading_screen();

    // Отдельно для экранов лаб. режимов
    apply_theme_to_lab_glaze_screen();
    apply_theme_to_lab_repair_screen();
    apply_theme_to_lab_strength_screen();
    apply_theme_to_lab_thermal_screen();
    apply_theme_to_lab_lighten_screen();
    apply_theme_to_lab_darken_screen();

    // Отдельно для экранов тестов
    apply_theme_to_test_nitrogen_screen();
    apply_theme_to_test_air_screen();

    // Отдельно для диалогов (они на верхнем слое)
    apply_theme_to_info_dialog();
    apply_theme_to_confirm_delete_dialog();

    // Добавляем перевод для главного экрана
    translate_main_screen_ui();

    // И не забываем перерисовать список профилей, так как стили плиток теперь в apply_theme_to_main_app_screen
    if (lv_scr_act() == screen_main_app) {
        displayProfileListPage();
    }
}

static void build_profile_list_pool() {
    if (!list_profiles_main) {
        Serial.println("ERROR: Cannot build profile pool, list container is null!");
        return;
    }

    Serial.println("Building profile list object pool...");

    for (int i = 0; i < PROFILES_PER_PAGE; i++) {
        // Создаем плитку (как в твоей старой функции)
        profile_tiles[i] = lv_obj_create(list_profiles_main);
        lv_obj_set_size(profile_tiles[i], 110, 175);
        lv_obj_set_style_border_width(profile_tiles[i], 1, 0);
        lv_obj_set_style_radius(profile_tiles[i], 5, 0);
        lv_obj_set_style_pad_all(profile_tiles[i], 5, 0);
        lv_obj_set_style_pad_gap(profile_tiles[i], 5, 0);
        lv_obj_set_layout(profile_tiles[i], LV_LAYOUT_FLEX);
        lv_obj_set_flex_flow(profile_tiles[i], LV_FLEX_FLOW_COLUMN);
        lv_obj_set_flex_align(profile_tiles[i], LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
        lv_obj_add_flag(profile_tiles[i], LV_OBJ_FLAG_CLICKABLE);

        // Создаем иконку
        profile_tile_icons[i] = lv_label_create(profile_tiles[i]);
        lv_label_set_text(profile_tile_icons[i], LV_SYMBOL_FILE);
        lv_obj_set_style_text_font(profile_tile_icons[i], &lv_font_montserrat_24, 0);

        // Создаем метку для имени
        profile_tile_labels[i] = lv_label_create(profile_tiles[i]);
        lv_obj_set_width(profile_tile_labels[i], lv_pct(100));
        lv_label_set_long_mode(profile_tile_labels[i], LV_LABEL_LONG_DOT);
        lv_obj_set_style_text_align(profile_tile_labels[i], LV_TEXT_ALIGN_CENTER, 0);

        // Сразу прячем плитку. Она станет видимой, когда понадобится.
        lv_obj_add_flag(profile_tiles[i], LV_OBJ_FLAG_HIDDEN);
    }
}

// Функция очистки для экрана выполнения процесса
void cleanup_process_execution_screen() {
    Serial.println("-> Cleaning up 'Process Execution' screen...");
    
    // Останавливаем ВСЕ запущенные анимации в системе.
    // Это самый надежный способ убить "зомби" от спиннера.
    lv_anim_del_all(); 
    
    // Очищаем текстовые поля, чтобы LVGL освободил память под строки
    if(label_process_profile_name) lv_label_set_text(label_process_profile_name, "");
    if(label_process_status_title) lv_label_set_text(label_process_status_title, "");
    if(label_process_status_detail) lv_label_set_text(label_process_status_detail, "");

    // Скрываем спиннер и индикатор ремонта, чтобы они не пытались анимироваться
    // if(spinner_process_execution) lv_obj_add_flag(spinner_process_execution, LV_OBJ_FLAG_HIDDEN);
    if(repair_mode_indicator_obj) lv_obj_add_flag(repair_mode_indicator_obj, LV_OBJ_FLAG_HIDDEN);
}

// Функция очистки для экрана редактирования профиля
void cleanup_profile_edit_screen() {
    Serial.println("-> Cleaning up 'Profile Edit' screen...");
    // Главная задача здесь - удалить таймер мигания
    if (help_blink_timer) {
        lv_timer_del(help_blink_timer);
        help_blink_timer = nullptr;
    }
}

// Центральная функция для переключения экранов
void load_screen(lv_obj_t* target_screen) {
    lv_obj_t* current_screen = lv_scr_act();

    // Если мы никуда не переключаемся, ничего не делаем
    if (current_screen == target_screen) {
        return;
    }

    Serial.printf("\n--- SCREEN TRANSITION: from '%p' to '%p' ---\n", current_screen, target_screen);
    log_memory_status("Before Cleanup & Transition");

    // --- ФАЗА 1: ОЧИСТКА (EXIT CALLBACK) ---
    // Вызываем функцию очистки для экрана, С КОТОРОГО мы уходим
    if (current_screen == screen_process_execution) {
        cleanup_process_execution_screen();
    } 
    else if (current_screen == screen_profile_edit) {
        cleanup_profile_edit_screen();
    }
    // Добавьте сюда `else if` для других экранов с таймерами или анимациями в будущем

    // --- ФАЗА 2: ПЕРЕКЛЮЧЕНИЕ ЭКРАНА ---
    lv_scr_load(target_screen);

    // --- ФАЗА 3: НАСТРОЙКА (ENTER CALLBACK) ---
    // Вызываем функцию настройки для экрана, НА КОТОРЫЙ мы пришли
    if (target_screen == screen_profile_edit) {
        setup_profile_edit_screen();
    }
    // Здесь можно будет добавить `else if` для других экранов, если им нужна настройка при входе
    
    log_memory_status("After Transition & Setup");
    Serial.println("--- TRANSITION COMPLETE ---");
}

void log_memory_status(const char* event_name) {
    // heap_caps_get_free_size(MALLOC_CAP_INTERNAL) - это то же самое, что ESP.getFreeHeap() для внутренней SRAM
    size_t free_heap = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
    // heap_caps_get_largest_free_block - показывает самый большой непрерывный кусок памяти
    size_t largest_free_block = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL);
    
    Serial.println("----------------------------------------");
    Serial.printf("MEMORY STATUS after: %s\n", event_name);
    Serial.printf("  - Free Internal SRAM: %u bytes\n", free_heap);
    Serial.printf("  - Largest Free Block: %u bytes\n", largest_free_block);
#if CONFIG_SPIRAM
    size_t free_psram = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
    Serial.printf("  - Free PSRAM:         %u bytes\n", free_psram);
#endif
    Serial.println("----------------------------------------");
}

// Функция, которая будет вызываться из loop() для "тяжелой" операции сохранения профиля
void handle_profile_save_action() {
    // 1. Обновляем профиль в векторе
    for (auto& profile : all_profiles_data) {
        if (profile.id == pending_profile_to_save.id) {
            profile = pending_profile_to_save;
            break;
        }
    }
    
    // 2. Выполняем "тяжелую" операцию сохранения
    if (saveConfiguration()) {
        Serial.printf("Profile ID %d updated successfully.\n", pending_profile_to_save.id);
        current_active_profile_data = pending_profile_to_save; // Обновляем активные данные
        needs_list_refresh = true;

        // 3. После сохранения, обновляем и показываем экран деталей
        update_profile_details_screen(current_active_profile_data);
        if (screen_profile_details) {
            load_screen(screen_profile_details);
        }
    } else { 
        show_info_dialog(tr("Save Error"), "Failed to save updated config file.");
        loadConfiguration(); // В случае ошибки перезагружаем все с диска
        // Возвращаемся на главный экран, чтобы избежать рассинхронизации
        if (screen_main_app) {
            load_screen(screen_main_app);
        }
    }
}

// Функция, которую будет вызывать кнопка "Сохранить"
void trigger_profile_save_action(const ProfileData& profile_to_save) {
    // 1. Сохраняем данные, которые нужно будет записать
    pending_profile_to_save = profile_to_save;
    
    // 2. Устанавливаем флаг для loop()
    profile_save_action_pending = true;

    // 3. Показываем экран "Сохранение..."
    if (screen_loading) {
        apply_theme_to_loading_screen();
        lv_label_set_text(label_loading_text, tr("Saving..."));
        load_screen(screen_loading);
        // Небольшая задержка для отрисовки
        for (int i = 0; i < 5; i++) {
            lv_timer_handler();
            delay(5);
        }
    }
}

// Новая функция для обновления экрана деталей на основе переданных данных
void update_profile_details_screen(const ProfileData& profile_to_display) {
    // --- ОБНОВЛЕНИЕ ДАННЫХ ---
    if (label_detail_view_profile_name) lv_label_set_text(label_detail_view_profile_name, profile_to_display.name);
    if (label_detail_view_id) lv_label_set_text_fmt(label_detail_view_id, "ID: %d", profile_to_display.id);

    // --- ЛОГИКА ПЕРЕВОДА И ЗАПОЛНЕНИЯ ПОЛЕЙ ---
    if (current_global_settings.language == 1) { // RUS
        const char* on_str = "ВКЛ";
        const char* off_str = "ВЫКЛ";

        if (label_detail_view_thermal_chamber_enabled) lv_label_set_text_fmt(label_detail_view_thermal_chamber_enabled, "Термокамера: %s", profile_to_display.thermal_chamber_enabled ? on_str : off_str);
        if (label_detail_view_thermal_chamber) {
            if (profile_to_display.thermal_chamber_enabled) {
                lv_obj_clear_flag(label_detail_view_thermal_chamber, LV_OBJ_FLAG_HIDDEN);
                lv_label_set_text_fmt(label_detail_view_thermal_chamber, "Цель: %d°C, Удержание: %d с", profile_to_display.thermal_chamber_temp, profile_to_display.heat_exchange_hold_sec);
            } else {
                lv_obj_add_flag(label_detail_view_thermal_chamber, LV_OBJ_FLAG_HIDDEN);
            }
        }
        if (label_detail_view_nitrogen) {
            if (profile_to_display.nitrogen_use_enabled) {
                lv_label_set_text_fmt(label_detail_view_nitrogen, "Азот: ВКЛ (Цель: %d%%)", profile_to_display.nitrogen_target_percent);
            } else {
                lv_label_set_text(label_detail_view_nitrogen, "Азот: ВЫКЛ");
            }
        }
        if (label_detail_view_chamber_cooling) {
            if (profile_to_display.chamber_cooling_enabled) {
                lv_label_set_text_fmt(label_detail_view_chamber_cooling, "Сжатый воздух: %s (Продувка: %d с)", on_str, profile_to_display.post_cooling_air_purge_sec);
            } else {
                lv_label_set_text_fmt(label_detail_view_chamber_cooling, "Сжатый воздух: %s", off_str);
            }
        }
        if (label_detail_view_primary_uv) lv_label_set_text_fmt(label_detail_view_primary_uv, "Первичный УФ (Мерцания): %d с", profile_to_display.primary_uv_exposure_sec);
        if (label_detail_view_secondary_uv) lv_label_set_text_fmt(label_detail_view_secondary_uv, "Вторичный УФ (Статичный): %d с", profile_to_display.secondary_uv_exposure_sec);
        if (label_detail_view_tertiary_uv) lv_label_set_text_fmt(label_detail_view_tertiary_uv, "Третичный УФ (Статичный): %d с", profile_to_display.tertiary_uv_exposure_sec);

    } else { // ENG
        const char* on_str = "ON";
        const char* off_str = "OFF";

        if (label_detail_view_thermal_chamber_enabled) lv_label_set_text_fmt(label_detail_view_thermal_chamber_enabled, "Thermal Chamber: %s", profile_to_display.thermal_chamber_enabled ? on_str : off_str);
         if (label_detail_view_thermal_chamber) {
            if (profile_to_display.thermal_chamber_enabled) {
                lv_obj_clear_flag(label_detail_view_thermal_chamber, LV_OBJ_FLAG_HIDDEN);
                lv_label_set_text_fmt(label_detail_view_thermal_chamber, "Target Temp: %d°C, Hold: %d s", profile_to_display.thermal_chamber_temp, profile_to_display.heat_exchange_hold_sec);
            } else {
                lv_obj_add_flag(label_detail_view_thermal_chamber, LV_OBJ_FLAG_HIDDEN);
            }
        }
        if (label_detail_view_nitrogen) {
            if (profile_to_display.nitrogen_use_enabled) {
                lv_label_set_text_fmt(label_detail_view_nitrogen, "Nitrogen Use: ON (Target: %d%%)", profile_to_display.nitrogen_target_percent);
            } else {
                lv_label_set_text(label_detail_view_nitrogen, "Nitrogen Use: OFF");
            }
        }
        if (label_detail_view_chamber_cooling) {
            if (profile_to_display.chamber_cooling_enabled) {
                lv_label_set_text_fmt(label_detail_view_chamber_cooling, "Compressed air: %s (Purge: %ds)", on_str, profile_to_display.post_cooling_air_purge_sec);
            } else {
                lv_label_set_text_fmt(label_detail_view_chamber_cooling, "Compressed air: %s", off_str);
            }
        }
        if (label_detail_view_primary_uv) lv_label_set_text_fmt(label_detail_view_primary_uv, "Primary UV: %d s", profile_to_display.primary_uv_exposure_sec);
        if (label_detail_view_secondary_uv) lv_label_set_text_fmt(label_detail_view_secondary_uv, "Secondary UV: %d s", profile_to_display.secondary_uv_exposure_sec);
        if (label_detail_view_tertiary_uv) lv_label_set_text_fmt(label_detail_view_tertiary_uv, "Tertiary UV: %d s", profile_to_display.tertiary_uv_exposure_sec);
    }

    // --- ПЕРЕВОД СТАТИЧНЫХ ЭЛЕМЕНТОВ ---
    if (current_global_settings.language == 1) { // RUS
        lv_label_set_text(label_detail_header, "Имя и параметры профиля:");
        lv_label_set_text(label_detail_btn_start, "Старт");
        lv_label_set_text(label_detail_btn_edit, "Редактировать");
        lv_label_set_text(label_detail_btn_delete, "Удалить");
        lv_label_set_text(label_detail_btn_close, "Закрыть");
    } else { // ENG
        lv_label_set_text(label_detail_header, "Profile name and details:");
        lv_label_set_text(label_detail_btn_start, "Start");
        lv_label_set_text(label_detail_btn_edit, "Edit");
        lv_label_set_text(label_detail_btn_delete, "Delete");
        lv_label_set_text(label_detail_btn_close, "Close");
    }

    lv_obj_add_style(label_detail_btn_start, &style_my_text_18_white, 0);
    lv_obj_add_style(label_detail_btn_edit, &style_my_text_18_white, 0);
    lv_obj_add_style(label_detail_btn_delete, &style_my_text_18_white, 0);
    lv_obj_add_style(label_detail_btn_close, &style_my_text_18_white, 0);

    // Применение темы
    apply_theme_to_profile_details_screen();
}
// ==========================================================================
// НОВЫЕ ЦЕНТРАЛЬНЫЕ ФУНКЦИИ ДЛЯ РАБОТЫ С cfg.json
// ==========================================================================

bool saveConfiguration() {
    SpiRamJsonDocument config_doc(FILE_CONTENT_BUFFER_SIZE);
    Serial.println("-> Saving ALL configuration to cfg.json...");
    if (!sd_card_initialized) {
        Serial.println("   ERROR: SD card not ready.");
        return false;
    }
    lvgl_port_lock(-1);

    config_doc.clear();

    // --- 1. Секция global_settings ---
    JsonObject global = config_doc.createNestedObject("global_settings");
    global["is_first_run"] = current_global_settings.is_first_run;
    global["is_heater_error"] = current_global_settings.is_heater_error;
    global["nitrogen_system_enabled"] = current_global_settings.nitrogen_system_enabled;
    global["compressed_air_system_enabled"] = current_global_settings.compressed_air_system_enabled;
    global["language"] = current_global_settings.language;
    global["theme"] = current_global_settings.theme;
    global["screen_timeout_mode"] = current_global_settings.screen_timeout_mode;

    // --- 2. Секция laboratory_modes ---
    JsonObject lab = config_doc.createNestedObject("laboratory_modes");
    JsonObject glaze = lab.createNestedObject("glaze");
    glaze["uv_on_sec"] = current_lab_settings.glaze.uv_on_sec;
    glaze["uv_off_sec"] = current_lab_settings.glaze.uv_off_sec;
    glaze["use_monomer_blow"] = current_lab_settings.glaze.use_monomer_blow;
    glaze["monomer_blow_min"] = current_lab_settings.glaze.monomer_blow_min;
    glaze["uv_mode"] = current_lab_settings.glaze.uv_mode;
    glaze["uv_exposure_sec"] = current_lab_settings.glaze.uv_exposure_sec;
    glaze["use_cooling"] = current_lab_settings.glaze.use_cooling;
    glaze["use_nitrogen"] = current_lab_settings.glaze.use_nitrogen;
    glaze["nitrogen_target_percent"] = current_lab_settings.glaze.nitrogen_target_percent;
    glaze["nitrogen_boost_sec"] = current_lab_settings.glaze.nitrogen_boost_sec;
    
    JsonObject repair = lab.createNestedObject("repair");
    repair["countdown_sec"] = current_lab_settings.repair.countdown_sec;
    repair["uv_exposure_sec"] = current_lab_settings.repair.uv_exposure_sec;

    JsonObject strength = lab.createNestedObject("strength");
    strength["chamber_temp_c"] = current_lab_settings.strength.chamber_temp_c;
    strength["hold_time_min"] = current_lab_settings.strength.hold_time_min;
    strength["use_cooling"] = current_lab_settings.strength.use_cooling;
    strength["uv_pulse_duration_sec"] = current_lab_settings.strength.uv_pulse_duration_sec;
    strength["uv_pulse_interval_min"] = current_lab_settings.strength.uv_pulse_interval_min;

    JsonObject thermal = lab.createNestedObject("thermal");
    thermal["chamber_temp_c"] = current_lab_settings.thermal.chamber_temp_c;
    thermal["hold_time_min"] = current_lab_settings.thermal.hold_time_min;
    
    JsonObject lighten = lab.createNestedObject("lighten");
    lighten["chamber_temp_c"] = current_lab_settings.lighten.chamber_temp_c;
    lighten["hold_time_min"] = current_lab_settings.lighten.hold_time_min;
    lighten["use_cooling"] = current_lab_settings.lighten.use_cooling;

    JsonObject darken = lab.createNestedObject("darken");
    darken["uv_exposure_min"] = current_lab_settings.darken.uv_exposure_min;
    darken["use_cooling"] = current_lab_settings.darken.use_cooling;

    // --- 3. Секция user_profiles ---
    JsonArray profiles = config_doc.createNestedArray("user_profiles");
    for (const auto& profile : all_profiles_data) {
        JsonObject p_obj = profiles.createNestedObject();
        p_obj["id"] = profile.id;
        p_obj["name"] = profile.name;
        p_obj["thermal_chamber_enabled"] = profile.thermal_chamber_enabled;
        p_obj["thermal_chamber_temp"] = profile.thermal_chamber_temp;
        p_obj["heat_exchange_hold_sec"] = profile.heat_exchange_hold_sec;
        p_obj["nitrogen_use_enabled"] = profile.nitrogen_use_enabled;
        p_obj["nitrogen_target_percent"] = profile.nitrogen_target_percent;
        p_obj["nitrogen_boost_sec"] = profile.nitrogen_boost_sec;
        p_obj["primary_uv_exposure_sec"] = profile.primary_uv_exposure_sec;
        p_obj["secondary_uv_exposure_sec"] = profile.secondary_uv_exposure_sec;
        p_obj["chamber_cooling_enabled"] = profile.chamber_cooling_enabled;
        p_obj["primary_uv_mode"] = profile.primary_uv_mode;
        p_obj["primary_uv_flicker_on_sec"] = profile.primary_uv_flicker_on_sec;
        p_obj["secondary_uv_mode"] = profile.secondary_uv_mode;
        p_obj["tertiary_uv_exposure_sec"] = profile.tertiary_uv_exposure_sec;
        p_obj["tertiary_uv_mode"] = profile.tertiary_uv_mode;
        p_obj["post_cooling_air_purge_sec"] = profile.post_cooling_air_purge_sec;
    }

    // --- 4. Секция service_keys ---
    JsonArray keys = config_doc.createNestedArray("service_keys");
    for (const auto& key : service_keys) {
        keys.add(key);
    }
    
    // --- Безопасное сохранение ---
    const char* temp_path = "/cfg.tmp";
    File file = SD.open(temp_path, FILE_WRITE);
    if (!file) {
        Serial.println("   ERROR: Failed to open temporary config file.");
        lvgl_port_unlock();
        return false;
    }
    size_t bytes_written = serializeJson(config_doc, file);
    file.close();
    
    if (bytes_written == 0) {
        Serial.println("   ERROR: Failed to write data to temporary config file.");
        lvgl_port_unlock();
        return false;
    }

    SD.remove(config_file_path);
    SD.rename(temp_path, config_file_path);

    Serial.printf("   SUCCESS: Configuration saved to %s (%d bytes)\n", config_file_path, bytes_written);
    lvgl_port_unlock();
    return true;
}

void loadConfiguration() {
    SpiRamJsonDocument config_doc(FILE_CONTENT_BUFFER_SIZE);
    Serial.println("-> Loading all configuration from cfg.json...");
    lvgl_port_lock(-1);

    // Очистка старых данных в памяти
    all_profiles_data.clear();
    all_profiles_data.reserve(50); // Резервируем место под 50 профилей
    service_keys.clear();
    current_profile_next_id = 1;

    if (!sd_card_initialized) {
        Serial.println("   ERROR: SD card not initialized. Cannot load config.");
        lvgl_port_unlock();
        return;
    }

    if (!SD.exists(config_file_path)) {
        Serial.println("   cfg.json not found. Creating default configuration from scratch...");
        // --- БЛОК ДЛЯ ПЕРВОГО ЗАПУСКА ---

        // 1. Устанавливаем значения по умолчанию для глобальных настроек
        current_global_settings.is_first_run = false; // Сразу ставим false, так как файл будет создан
        current_global_settings.is_heater_error = false;
        current_global_settings.nitrogen_system_enabled = false;
        current_global_settings.compressed_air_system_enabled = false;
        current_global_settings.language = 1; // RUS
        current_global_settings.theme = 1;    // Dark
        current_global_settings.screen_timeout_mode = 0; // 5 min

        // 2. Устанавливаем значения по умолчанию для лабораторных режимов
        current_lab_settings.glaze = {2.0f, 2.0f, true, 1, 2, 10, false, true, 97, 1};
        current_lab_settings.repair = {5, 5};
        current_lab_settings.strength = {50, 30, false, 1, 1};
        current_lab_settings.thermal = {40, 5};
        current_lab_settings.lighten = {80, 10, false};
        current_lab_settings.darken = {1, false};
        
        // 3. Создаем профиль по умолчанию
        ProfileData default_profile;
        memset(&default_profile, 0, sizeof(ProfileData));
        default_profile.id = 1;
        strncpy(default_profile.name, "WaveDent Base", sizeof(default_profile.name) - 1);
        default_profile.thermal_chamber_enabled = true;
        default_profile.thermal_chamber_temp = 75;
        default_profile.heat_exchange_hold_sec = 60;
        default_profile.nitrogen_use_enabled = true;
        default_profile.nitrogen_target_percent = 98;
        default_profile.nitrogen_boost_sec = 3;
        default_profile.primary_uv_exposure_sec = 60;
        default_profile.secondary_uv_exposure_sec = 30;
        default_profile.chamber_cooling_enabled = true;
        default_profile.primary_uv_mode = 2;
        default_profile.primary_uv_flicker_on_sec = 0.2;
        default_profile.secondary_uv_mode = 1;
        default_profile.tertiary_uv_exposure_sec = 10;
        default_profile.tertiary_uv_mode = 1;
        default_profile.post_cooling_air_purge_sec = 20;
        all_profiles_data.push_back(default_profile);
        current_profile_next_id = 2;

        // 4. Генерируем сервисные ключи в памяти
        const char* default_keys[] = {
            "8614", "5483", "7419", "2057", "1928", "8370", "8107", "9531", "4172", "8425",
            "7936", "8512", "3270", "4829", "4310", "2834", "5719", "8542", "7759", "1036",
            "1584", "2471", "9538", "4821", "7309", "7056", "2184", "4802", "8249", "8591",
            "3805", "1853", "9217", "2830", "5429", "4918", "7124", "1937", "8201", "6731",
            "7409", "5082", "1429", "5320", "8915", "1504", "1520", "3981", "1479", "6210",
            "7812", "7904", "5109", "2971", "9130", "6621", "1035", "1872", "2906", "5625",
            "4719", "3184", "5590", "2517", "1852", "6520", "2167", "5421", "5813", "3712",
            "1592", "4720", "4018", "5921", "2017", "2019", "3751", "9148", "4862", "1408",
            "8206", "9210", "3710", "5139", "9370", "1305", "6109", "2501", "8271", "9714",
            "1703", "3921", "1730", "2805", "7412", "2958", "6142", "7316", "5731", "1472"
        };
        for(const char* key : default_keys) {
            service_keys.push_back(String(key));
        }

        // 5. Сохраняем все это в новый файл cfg.json
        saveConfiguration();
        lvgl_port_unlock();
        return; // Выходим, т.к. все данные уже в памяти
    }

    // --- ЕСЛИ ФАЙЛ СУЩЕСТВУЕТ, ЧИТАЕМ ЕГО ---
    if (!readFileContentToBuffer_ino(SD, config_file_path, file_content_buffer, FILE_CONTENT_BUFFER_SIZE)) {
        Serial.println("   ERROR: Failed to read cfg.json content.");
        lvgl_port_unlock();
        return;
    }

    config_doc.clear();
    DeserializationError error = deserializeJson(config_doc, file_content_buffer);

    if (error) {
        Serial.printf("   ERROR: Failed to parse cfg.json: %s\n", error.c_str());
        lvgl_port_unlock();
        return;
    }

    // --- "Раскладываем" данные по переменным ---
    JsonObject global = config_doc["global_settings"];
    current_global_settings.is_first_run = global["is_first_run"] | true;
    current_global_settings.is_heater_error = global["is_heater_error"] | false;
    current_global_settings.nitrogen_system_enabled = global["nitrogen_system_enabled"] | false;
    current_global_settings.compressed_air_system_enabled = global["compressed_air_system_enabled"] | false;
    current_global_settings.language = global["language"] | 1;
    current_global_settings.theme = global["theme"] | 1;
    current_global_settings.screen_timeout_mode = global["screen_timeout_mode"] | 0;

    JsonObject lab = config_doc["laboratory_modes"];
    JsonObject glaze = lab["glaze"];
    current_lab_settings.glaze.uv_on_sec = glaze["uv_on_sec"] | 2.0f;
    current_lab_settings.glaze.uv_off_sec = glaze["uv_off_sec"] | 2.0f;
    current_lab_settings.glaze.use_monomer_blow = glaze["use_monomer_blow"] | true;
    current_lab_settings.glaze.monomer_blow_min = glaze["monomer_blow_min"] | 1;
    current_lab_settings.glaze.uv_mode = glaze["uv_mode"] | 2;
    current_lab_settings.glaze.uv_exposure_sec = glaze["uv_exposure_sec"] | 10;
    current_lab_settings.glaze.use_cooling = glaze["use_cooling"] | false;
    current_lab_settings.glaze.use_nitrogen = glaze["use_nitrogen"] | false;
    current_lab_settings.glaze.nitrogen_target_percent = glaze["nitrogen_target_percent"] | 99; 
    current_lab_settings.glaze.nitrogen_boost_sec = glaze["nitrogen_boost_sec"] | 1;

    JsonObject repair = lab["repair"];
    current_lab_settings.repair.countdown_sec = repair["countdown_sec"] | 5;
    current_lab_settings.repair.uv_exposure_sec = repair["uv_exposure_sec"] | 5;
    
    JsonObject strength = lab["strength"];
    current_lab_settings.strength.chamber_temp_c = strength["chamber_temp_c"] | 50;
    current_lab_settings.strength.hold_time_min = strength["hold_time_min"] | 30;
    current_lab_settings.strength.use_cooling = strength["use_cooling"] | false;
    current_lab_settings.strength.uv_pulse_duration_sec = strength["uv_pulse_duration_sec"] | 1;
    current_lab_settings.strength.uv_pulse_interval_min = strength["uv_pulse_interval_min"] | 1;
            
    JsonObject thermal = lab["thermal"];
    current_lab_settings.thermal.chamber_temp_c = thermal["chamber_temp_c"] | 40;
    current_lab_settings.thermal.hold_time_min = thermal["hold_time_min"] | 5;

    JsonObject lighten = lab["lighten"];
    current_lab_settings.lighten.chamber_temp_c = lighten["chamber_temp_c"] | 80;
    current_lab_settings.lighten.hold_time_min = lighten["hold_time_min"] | 10;
    current_lab_settings.lighten.use_cooling = lighten["use_cooling"] | false;

    JsonObject darken = lab["darken"];
    current_lab_settings.darken.uv_exposure_min = darken["uv_exposure_min"] | 1;
    current_lab_settings.darken.use_cooling = darken["use_cooling"] | false;

    JsonArray profiles = config_doc["user_profiles"];
    int max_id_found = 0;
    for (JsonObject p_obj : profiles) {
        ProfileData profile;
        profile.id = p_obj["id"] | -1;
        const char* name_ptr = p_obj["name"] | "Unnamed";
        strncpy(profile.name, name_ptr, sizeof(profile.name) - 1);
        profile.thermal_chamber_enabled = p_obj["thermal_chamber_enabled"] | false;
        profile.thermal_chamber_temp = p_obj["thermal_chamber_temp"] | 40;
        profile.heat_exchange_hold_sec = p_obj["heat_exchange_hold_sec"] | 60;
        profile.nitrogen_use_enabled = p_obj["nitrogen_use_enabled"] | false;
        profile.nitrogen_target_percent = p_obj["nitrogen_target_percent"] | 95;
        profile.nitrogen_boost_sec = p_obj["nitrogen_boost_sec"] | 1;
        profile.primary_uv_exposure_sec = p_obj["primary_uv_exposure_sec"] | 30;
        profile.secondary_uv_exposure_sec = p_obj["secondary_uv_exposure_sec"] | 60;
        profile.chamber_cooling_enabled = p_obj["chamber_cooling_enabled"] | false;
        profile.primary_uv_mode = p_obj["primary_uv_mode"] | 1;
        profile.primary_uv_flicker_on_sec = p_obj["primary_uv_flicker_on_sec"] | 0.2f;
        profile.secondary_uv_mode = p_obj["secondary_uv_mode"] | 1;
        profile.tertiary_uv_exposure_sec = p_obj["tertiary_uv_exposure_sec"] | 60;
        profile.tertiary_uv_mode = p_obj["tertiary_uv_mode"] | 1;
        profile.post_cooling_air_purge_sec = p_obj["post_cooling_air_purge_sec"] | 0;
        
        all_profiles_data.push_back(profile);
        if (profile.id > max_id_found) {
            max_id_found = profile.id;
        }
    }
    current_profile_next_id = max_id_found + 1;

    JsonArray keys = config_doc["service_keys"];
    for (JsonVariant v : keys) {
        service_keys.push_back(v.as<String>());
    }

    Serial.println("   SUCCESS: Configuration loaded.");
    lvgl_port_unlock();
}

// Эта функция будет вызываться из loop(), когда пользователь видит экран загрузки для лабораторных режимов
void handle_lab_save_and_action() {
    // 1. Выполняем "тяжелую" операцию сохранения
    saveConfiguration();

    // 2. Выполняем следующее действие на основе сохраненного кода
    int action = next_lab_action;
    next_lab_action = 0; // Сразу сбрасываем

    if (action == 1) { // 1 = Назад
        if (screen_laboratory) {
            load_screen(screen_laboratory);
        }
    } else if (action >= 2 && action <= 7) { // 2-7 = Старт режимов
        // mode_id на 1 меньше, чем код действия
        int mode_id = action - 1; 
        prepare_and_send_lab_command(mode_id);
    }
}
static void trigger_lab_action(int action_code) {
    // 1. Сохраняем, что нужно сделать
    next_lab_action = action_code;

    // 2. Устанавливаем флаг для loop()
    lab_settings_action_pending = true;

    // 3. Показываем экран загрузки (аналогично вашему коду)
    if (screen_loading) {
        apply_theme_to_loading_screen();
        lv_label_set_text(label_loading_text, tr("Saving..."));
        load_screen(screen_loading);

        for (int i = 0; i < 5; i++) {
            lv_timer_handler();
            delay(5);
        }
    }
}

void flush_serial_buffer() {
    // Ждем небольшую паузу, чтобы все возможные данные успели прийти
    delay(80); 
    while (MySerial1.available() > 0) {
        MySerial1.read();
    }
    Serial.println("UART RX Buffer Flushed.");
}
void completeCancellationSequence() {
    Serial.println("--- Completing cancellation sequence on UI board ---");
    
    // 1. Сбрасываем все флаги состояния
    waiting_for_stop_ack = false;
    // main_process_running = false;
    // is_lab_mode_running = false;
    current_process_stage = "";

    // 2. Возвращаемся на предыдущий экран
    if (screen_to_return_after_process) {
        load_screen(screen_to_return_after_process);
    } else {
        // Запасной вариант, если указатель по какой-то причине пуст
        load_screen(screen_main_app);
    }

    // 3. Снова делаем кнопку "Отмена" активной для следующего запуска
    if (btn_process_cancel) {
        lv_obj_clear_state(btn_process_cancel, LV_STATE_DISABLED);
    }

    // 4. Очищаем входящий буфер UART от возможного мусора
    flush_serial_buffer();

    log_memory_status("Process Cancellation");
}
// --- СЦЕНА ТЕСТА АЗОТА ---
static void build_test_nitrogen_screen(lv_obj_t* parent_screen) {
    screen_test_nitrogen = parent_screen;
    lv_obj_clear_flag(screen_test_nitrogen, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_t* main_container = lv_obj_create(screen_test_nitrogen);
    lv_obj_set_size(main_container, lv_pct(100), lv_pct(100));
    lv_obj_center(main_container);
    lv_obj_set_flex_flow(main_container, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_all(main_container, 0, 0);
    lv_obj_set_style_border_width(main_container, 0, 0);
    lv_obj_set_style_radius(main_container, 0, 0);
    lv_obj_set_style_pad_ver(main_container, 20, 0);
    lv_obj_set_style_pad_hor(main_container, 15, 0);
    lv_obj_set_style_pad_gap(main_container, 20, 0);
    label_nitrogen_test_header = lv_label_create(main_container);
    lv_obj_add_style(label_nitrogen_test_header, &style_my_text_22, 0);
    lv_obj_t* content_area = lv_obj_create(main_container);
    lv_obj_remove_style_all(content_area);
    lv_obj_set_flex_grow(content_area, 1);
    lv_obj_set_width(content_area, lv_pct(100));
    btn_nitrogen_test_press = lv_btn_create(content_area);
    lv_obj_set_size(btn_nitrogen_test_press, lv_pct(100), lv_pct(100));
    lv_obj_add_event_cb(btn_nitrogen_test_press, test_nitrogen_screen_event_cb, LV_EVENT_CLICKED, (void*)"TOGGLE_SUPPLY");
    label_btn_nitrogen_test_press = lv_label_create(btn_nitrogen_test_press);
    lv_obj_add_style(label_btn_nitrogen_test_press, &style_my_text_22, 0);
    lv_obj_center(label_btn_nitrogen_test_press);
    btn_nitrogen_test_back = lv_btn_create(main_container);
    lv_obj_set_width(btn_nitrogen_test_back, lv_pct(60));
    lv_obj_align(btn_nitrogen_test_back, LV_ALIGN_CENTER, 0, 0);
    lv_obj_add_event_cb(btn_nitrogen_test_back, test_nitrogen_screen_event_cb, LV_EVENT_CLICKED, (void*)"BACK");
    label_btn_nitrogen_test_back = lv_label_create(btn_nitrogen_test_back);
    lv_obj_add_style(label_btn_nitrogen_test_back, &style_my_text_18_white, 0);
    lv_obj_center(label_btn_nitrogen_test_back);
}

static void apply_theme_to_loading_screen() {
    if (!screen_loading) return;
    
    // Получаем наш новый главный контейнер (он всегда первый дочерний элемент)
    lv_obj_t* main_container = lv_obj_get_child(screen_loading, 0);
    if (!main_container) return;

    if (current_global_settings.theme == 1) { // --- ТЕМНАЯ ТЕМА ---
        // Удаляем стили светлой темы
        lv_obj_remove_style(main_container, &style_light_bg, 0);
        lv_obj_remove_style(label_loading_text, &style_light_text, 0);
        // Добавляем стили темной темы
        lv_obj_add_style(main_container, &style_dark_bg, 0);
        lv_obj_add_style(label_loading_text, &style_dark_text, 0);
    } else { // --- СВЕТЛАЯ ТЕМА ---
        // Удаляем стили темной темы
        lv_obj_remove_style(main_container, &style_dark_bg, 0);
        lv_obj_remove_style(label_loading_text, &style_dark_text, 0);
        // Добавляем стили светлой темы
        lv_obj_add_style(main_container, &style_light_bg, 0);
        lv_obj_add_style(label_loading_text, &style_light_text, 0);
    }
}

static void apply_theme_to_test_nitrogen_screen() {
    if (!screen_test_nitrogen) return;
    lv_obj_t* main_container = lv_obj_get_child(screen_test_nitrogen, 0);
    if (current_global_settings.theme == 1) { // --- ТЕМНАЯ ТЕМА ---
        lv_obj_add_style(main_container, &style_dark_bg, 0);
        lv_obj_add_style(label_nitrogen_test_header, &style_dark_text, 0);
        lv_obj_add_style(btn_nitrogen_test_press, &style_dark_btn, 0);
        lv_obj_add_style(label_btn_nitrogen_test_press, &style_dark_text, 0);
        lv_obj_add_style(btn_nitrogen_test_back, &style_dark_btn, 0);
    } else { // --- СВЕТЛАЯ ТЕМА ---
        lv_obj_remove_style(main_container, &style_dark_bg, 0);
        lv_obj_remove_style(label_nitrogen_test_header, &style_dark_text, 0);
        lv_obj_remove_style(btn_nitrogen_test_press, &style_dark_btn, 0);
        lv_obj_remove_style(label_btn_nitrogen_test_press, &style_dark_text, 0);
        lv_obj_remove_style(btn_nitrogen_test_back, &style_dark_btn, 0);
        
        lv_obj_add_style(main_container, &style_light_bg, 0);
        lv_obj_add_style(label_nitrogen_test_header, &style_light_text, 0);
        lv_obj_add_style(btn_nitrogen_test_press, &style_light_btn, 0);
        lv_obj_add_style(label_btn_nitrogen_test_press, &style_my_text_18_white, 0);
        lv_obj_add_style(btn_nitrogen_test_back, &style_light_btn, 0);
    }
}

// Новый обработчик для группы кнопок
static void uv_mode_btn_group_event_cb(lv_event_t * e) {
    lv_obj_t* clicked_btn = lv_event_get_target(e);
    lv_obj_t* parent = lv_obj_get_parent(clicked_btn);

    // Убираем флаг со всех кнопок в группе
    for (uint32_t i = 0; i < lv_obj_get_child_cnt(parent); i++) {
        lv_obj_clear_state(lv_obj_get_child(parent, i), LV_STATE_CHECKED);
    }
    
    // Устанавливаем флаг только на нажатую кнопку
    lv_obj_add_state(clicked_btn, LV_STATE_CHECKED);

    // Вызываем старый обработчик, чтобы обновить текстовую метку
    uv_mode_selector_event_cb(e); 
}

// Обработчик для кнопок ВНУТРИ сцены теста азота
static void test_nitrogen_screen_event_cb(lv_event_t* e) {
    lv_event_code_t code = lv_event_get_code(e);
    if (code != LV_EVENT_CLICKED) return;

    const char* user_data = (const char*)lv_event_get_user_data(e);
    command_json_doc.clear();
    String output;
    
    if (strcmp(user_data, "TOGGLE_SUPPLY") == 0) {
        is_nitrogen_test_active = !is_nitrogen_test_active; // Инвертируем состояние

        if (is_nitrogen_test_active) {
            // --- Включаем подачу ---
            command_json_doc["command"] = "TEST_VALVE_NITROGEN_OPEN";
            lv_label_set_text(label_btn_nitrogen_test_press, tr("Stop Supply"));
            // Меняем цвет кнопки на "активный" (красный)
            lv_obj_set_style_bg_color(btn_nitrogen_test_press, lv_palette_main(LV_PALETTE_RED), 0);
        } else {
            // --- Выключаем подачу ---
            command_json_doc["command"] = "TEST_VALVE_NITROGEN_CLOSE";
            lv_label_set_text(label_btn_nitrogen_test_press, tr("Start Supply"));

            // <<< ВОТ ПРАВИЛЬНОЕ ИСПРАВЛЕНИЕ >>>
            // Удаляем ТОЛЬКО локальное свойство цвета фона. Все остальные стили темы остаются!
            lv_obj_remove_local_style_prop(btn_nitrogen_test_press, LV_STYLE_BG_COLOR, 0);
        }
        serializeJson(command_json_doc, output);
        flush_serial_buffer();
        MySerial1.println(output);

    } else if (strcmp(user_data, "BACK") == 0) {
        // --- Безопасный выход: всегда выключаем подачу ---
        if (is_nitrogen_test_active) {
            is_nitrogen_test_active = false; // Сбрасываем состояние
            command_json_doc["command"] = "TEST_VALVE_NITROGEN_CLOSE";
            serializeJson(command_json_doc, output);
            MySerial1.println(output);
        }
        load_screen(screen_settings);
    }
}
// --- СЦЕНА ТЕСТА ВОЗДУХА ---
static void build_test_air_screen(lv_obj_t* parent_screen) {
    screen_test_air = parent_screen;
    lv_obj_clear_flag(screen_test_air, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_t* main_container = lv_obj_create(screen_test_air);
    lv_obj_set_size(main_container, lv_pct(100), lv_pct(100));
    lv_obj_center(main_container);
    lv_obj_set_flex_flow(main_container, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_all(main_container, 0, 0);
    lv_obj_set_style_border_width(main_container, 0, 0);
    lv_obj_set_style_radius(main_container, 0, 0);
    lv_obj_set_style_pad_ver(main_container, 20, 0);
    lv_obj_set_style_pad_hor(main_container, 15, 0);
    lv_obj_set_style_pad_gap(main_container, 20, 0);
    label_air_test_header = lv_label_create(main_container);
    lv_obj_add_style(label_air_test_header, &style_my_text_22, 0);
    lv_obj_t* content_area = lv_obj_create(main_container);
    lv_obj_remove_style_all(content_area);
    lv_obj_set_flex_grow(content_area, 1);
    lv_obj_set_width(content_area, lv_pct(100));
    btn_air_test_press = lv_btn_create(content_area);
    lv_obj_set_size(btn_air_test_press, lv_pct(100), lv_pct(100));
    lv_obj_add_event_cb(btn_air_test_press, test_air_screen_event_cb, LV_EVENT_CLICKED, (void*)"TOGGLE_SUPPLY");
    label_btn_air_test_press = lv_label_create(btn_air_test_press);
    lv_obj_add_style(label_btn_air_test_press, &style_my_text_22, 0);
    lv_obj_center(label_btn_air_test_press);
    btn_air_test_back = lv_btn_create(main_container);
    lv_obj_set_width(btn_air_test_back, lv_pct(60));
    lv_obj_align(btn_air_test_back, LV_ALIGN_CENTER, 0, 0);
    lv_obj_add_event_cb(btn_air_test_back, test_air_screen_event_cb, LV_EVENT_CLICKED, (void*)"BACK");
    label_btn_air_test_back = lv_label_create(btn_air_test_back);
    lv_obj_add_style(label_btn_air_test_back, &style_my_text_18_white, 0);
    lv_obj_center(label_btn_air_test_back);
}

static void apply_theme_to_test_air_screen() {
    if (!screen_test_air) return;
    lv_obj_t* main_container = lv_obj_get_child(screen_test_air, 0);
    if (current_global_settings.theme == 1) { // --- ТЕМНАЯ ТЕМА ---
        lv_obj_add_style(main_container, &style_dark_bg, 0);
        lv_obj_add_style(label_air_test_header, &style_dark_text, 0);
        lv_obj_add_style(btn_air_test_press, &style_dark_btn, 0);
        lv_obj_add_style(label_btn_air_test_press, &style_dark_text, 0);
        lv_obj_add_style(btn_air_test_back, &style_dark_btn, 0);
    } else { // --- СВЕТЛАЯ ТЕМА ---
        lv_obj_remove_style(main_container, &style_dark_bg, 0);
        lv_obj_remove_style(label_air_test_header, &style_dark_text, 0);
        lv_obj_remove_style(btn_air_test_press, &style_dark_btn, 0);
        lv_obj_remove_style(label_btn_air_test_press, &style_dark_text, 0);
        lv_obj_remove_style(btn_air_test_back, &style_dark_btn, 0);
        
        lv_obj_add_style(main_container, &style_light_bg, 0);
        lv_obj_add_style(label_air_test_header, &style_light_text, 0);
        lv_obj_add_style(btn_air_test_press, &style_light_btn, 0);
        lv_obj_add_style(label_btn_air_test_press, &style_my_text_18_white, 0);
        lv_obj_add_style(btn_air_test_back, &style_light_btn, 0);
    }
}


// Обработчик для кнопок ВНУТРИ сцены теста воздуха
static void test_air_screen_event_cb(lv_event_t* e) {
    lv_event_code_t code = lv_event_get_code(e);
    if (code != LV_EVENT_CLICKED) return;
    
    const char* user_data = (const char*)lv_event_get_user_data(e);
    command_json_doc.clear();
    String output;

    if (strcmp(user_data, "TOGGLE_SUPPLY") == 0) {
        is_air_test_active = !is_air_test_active; // Инвертируем состояние

        if (is_air_test_active) {
            // --- Включаем подачу ---
            command_json_doc["command"] = "TEST_VALVE_AIR_OPEN";
            lv_label_set_text(label_btn_air_test_press, tr("Stop Supply"));
            lv_obj_set_style_bg_color(btn_air_test_press, lv_palette_main(LV_PALETTE_RED), 0);
        } else {
            // --- Выключаем подачу ---
            command_json_doc["command"] = "TEST_VALVE_AIR_CLOSE";
            lv_label_set_text(label_btn_air_test_press, tr("Start Supply"));
            
            // <<< И ЗДЕСЬ ТОЖЕ ПРАВИЛЬНОЕ ИСПРАВЛЕНИЕ >>>
            lv_obj_remove_local_style_prop(btn_air_test_press, LV_STYLE_BG_COLOR, 0);
        }
        serializeJson(command_json_doc, output);
        MySerial1.println(output);

    } else if (strcmp(user_data, "BACK") == 0) {
        // --- Безопасный выход: всегда выключаем подачу ---
        if (is_air_test_active) {
            is_air_test_active = false; // Сбрасываем состояние
            command_json_doc["command"] = "TEST_VALVE_AIR_CLOSE";
            serializeJson(command_json_doc, output);
            MySerial1.println(output);
        }
        load_screen(screen_settings);
    }
}
//

// Конвертирует режим таймаута (0-3) в миллисекунды
unsigned long get_current_screen_timeout_ms() {
    switch (current_global_settings.screen_timeout_mode) {
        case 0:  return 5 * 60 * 1000UL;  // 5 минут
        case 1:  return 15 * 60 * 1000UL; // 15 минут
        case 2:  return 30 * 60 * 1000UL; // 30 минут
        case 3:  return 60 * 60 * 1000UL; // 60 минут
        default: return 5 * 60 * 1000UL;  // По умолчанию 5 минут, на всякий случай
    }
}

void update_telemetry_display(float temp, float o2, int timer_rem) {
    // Специальная обработка для режима "Ремонт модели"
    if (is_lab_mode_running && screen_to_return_after_process == screen_lab_repair) {
        String repair_telemetry_text = "";
        // Для этого режима показываем ТОЛЬКО оставшееся время
        if (timer_rem >= 0) {
            repair_telemetry_text = String(tr("Time left:")) + " " + String(timer_rem) + " " + tr("s");
        }
        lv_label_set_text(label_process_status_detail, repair_telemetry_text.c_str());
        return; // ВАЖНО: Выходим из функции, чтобы не показывать остальную телеметрию
    }

    String telemetry_line1 = ""; // Строка для температуры и ее цели
    String telemetry_line2 = ""; // Строка для кислорода и его цели
    String telemetry_line3 = ""; // Строка для таймера

    // --- Шаг 1: Всегда отображаем текущую температуру ---
    telemetry_line1 += String(tr("Temp:")) + " " + String(temp, 1) + " C";

    // --- Шаг 2: Проверяем, есть ли целевая температура для ТЕКУЩЕГО режима ---
    int target_temp = 0;

    // --- НАЧАЛО ИЗМЕНЕНИЙ ---
    if (is_lab_mode_running) {
        // Используем надежную проверку экрана, с которого был запущен процесс,
        // а не текст заголовка, который постоянно меняется.
        if (screen_to_return_after_process == screen_lab_strength) {
            target_temp = current_lab_settings.strength.chamber_temp_c;
        } else if (screen_to_return_after_process == screen_lab_thermal) {
            target_temp = current_lab_settings.thermal.chamber_temp_c;
        } else if (screen_to_return_after_process == screen_lab_lighten) {
            target_temp = current_lab_settings.lighten.chamber_temp_c;
        }
    } else { // Если запущен профиль (эта логика была правильной)
        if (current_active_profile_data.thermal_chamber_enabled) {
            target_temp = current_active_profile_data.thermal_chamber_temp;
        }
    }
    
    if (target_temp > 0) {
        telemetry_line1 += ", " + String(tr("Target:")) + " " + String(target_temp) + " C";
    }

    // --- Шаг 3: Проверяем, нужно ли отображать кислород ---
    if (current_global_settings.nitrogen_system_enabled) {
        telemetry_line2 += String(tr("Oxygen:")) + " " + String(o2, 1) + "%";

        // --- Шаг 4: Проверяем, есть ли цель по кислороду для ТЕКУЩЕГО режима ---
        bool nitrogen_target_enabled = false;
        int nitrogen_target_percent = 0;
        
        if (is_lab_mode_running) {
            // Азот используется только в режиме "Глазурь". Проверяем это так же надежно.
            if (screen_to_return_after_process == screen_lab_glaze) {
                if (current_lab_settings.glaze.use_nitrogen) {
                    nitrogen_target_enabled = true;
                    nitrogen_target_percent = current_lab_settings.glaze.nitrogen_target_percent;
                }
            }
        } else { // Для профилей (эта логика была правильной)
            if (current_active_profile_data.nitrogen_use_enabled) {
                nitrogen_target_enabled = true;
                nitrogen_target_percent = current_active_profile_data.nitrogen_target_percent;
            }
        }
        // --- КОНЕЦ ИЗМЕНЕНИЙ ---

        if (nitrogen_target_enabled && nitrogen_target_percent > 0) {
            // Цель по азоту N2% означает, что кислорода O2 должно быть < (100 - N2)%
            // Но для простоты обычно ставят цель по кислороду, например < 5% или < 1%.
            // Здесь я оставлю вашу логику, она показывает цель по O2, вычисленную из цели по N2.
            float target_o2 = 100.0f - nitrogen_target_percent;
            telemetry_line2 += ", " + String(tr("Target O2:")) + " <" + String(target_o2, 1) + "%";
        }
    }

    // --- Шаг 5: Формируем строку с таймером ---
    if (timer_rem >= 0) {
        telemetry_line3 = String(tr("Time left:")) + " " + String(timer_rem) + " " + tr("s");
    }

    // --- Финальная сборка и отображение ---
    String full_telemetry_text = telemetry_line1;
    if (telemetry_line2.length() > 0) {
        full_telemetry_text += "\n" + telemetry_line2;
    }
    if (telemetry_line3.length() > 0) {
        // Добавим отступ для лучшей читаемости
        full_telemetry_text += "\n\n" + telemetry_line3;
    }

    lv_label_set_text(label_process_status_detail, full_telemetry_text.c_str());
}

const char* tr(const char* text_eng) {
    // Если язык не русский (0 = ENG) или строка пустая, просто возвращаем оригинал
    if (current_global_settings.language != 1 || text_eng == NULL) {
        return text_eng;
    }

    // --- СЛОВАРЬ ПЕРЕВОДОВ ---

    // --- Заголовки диалоговых окон ---
    if (strcmp(text_eng, "Loading...") == 0) return "Загрузка...";
    if (strcmp(text_eng, "Saving...") == 0) return "Сохранение...";
    if (strcmp(text_eng, "Cancelling...") == 0) return "Отмена...";
    if (strcmp(text_eng, "Error") == 0) return "Ошибка";
    if (strcmp(text_eng, "Input Error") == 0) return "Ошибка ввода";
    if (strcmp(text_eng, "Save Error") == 0) return "Ошибка сохранения";
    if (strcmp(text_eng, "Info") == 0) return "Инфо";
    if (strcmp(text_eng, "Setting Disabled") == 0) return "Система отключена";
    if (strcmp(text_eng, "Help: Glaze Mode") == 0) return "Справка: Режим Глазурь";
    if (strcmp(text_eng, "Help: Model Repair") == 0) return "Справка: Ремонт Модели";
    if (strcmp(text_eng, "Help: Strength Boost") == 0) return "Справка: Повышение прочности";
    if (strcmp(text_eng, "Help: Thermal Chamber") == 0) return "Справка: Термокамера";
    if (strcmp(text_eng, "Help: Composite Lightening") == 0) return "Справка: Осветление композита";
    if (strcmp(text_eng, "Help: Composite Darkening") == 0) return "Справка: Затемнение композита";
    if (strcmp(text_eng, "System components cooling") == 0) return "Охлаждение компонентов системы";


    // --- Сообщения об ошибках и уведомления ---
    if (strcmp(text_eng, "Profile name cannot be empty!") == 0) return "Имя профиля не может быть пустым!";
    if (strcmp(text_eng, "Without nitrogen supply, the surface will be sticky!") == 0) return "Без подачи азота поверхность будет липкой!";
    if (strcmp(text_eng, "Profile name is too long!") == 0) return "Имя профиля слишком длинное!";
    if (strcmp(text_eng, "SD Card not ready!") == 0) return "SD-карта не готова!";
    if (strcmp(text_eng, "Nitrogen system is disabled in Global Settings.\nPlease connect the valve and enable it in Settings.") == 0) return "Система азота отключена в общих настройках.\nПодключите клапан и включите ее в Настройках.";
    if (strcmp(text_eng, "Compressed Air (Chamber Cooling) is disabled in Global Settings.\nPlease connect it and enable in Settings.") == 0) return "Подача сжатого воздуха (охлаждение) отключена в общих настройках.\nПодключите ее и включите в Настройках.";
    if (strcmp(text_eng, "Compressed Air system is disabled in Global Settings.") == 0) return "Система сжатого воздуха отключена в общих настройках.";
    if (strcmp(text_eng, "Nitrogen system is disabled in Global Settings.") == 0) return "Система азота отключена в общих настройках.";
    if (strcmp(text_eng, "Could not load profile data for editing.") == 0) return "Не удалось загрузить данные профиля для редактирования.";
    if (strcmp(text_eng, "Failed to prepare data for saving.") == 0) return "Не удалось подготовить данные для сохранения.";
    if (strcmp(text_eng, "Nitrogen system is disabled in Global Settings.\nPlease connect the valve and enable it in Settings.") == 0) return "Система азота отключена в общих настройках.\nПодключите клапан и включите ее в Настройках.";
    if (strcmp(text_eng, LV_SYMBOL_DOWN "\nBring the model to the UV diode located below the screen.") == 0) return LV_SYMBOL_DOWN "\nПоднесите модель к УФ диоду, расположенному под экраном.";

    if (strcmp(text_eng, "Compressed Air (Chamber Cooling) is disabled in Global Settings.\nPlease connect it and enable in Settings.") == 0) return "Подача сжатого воздуха (охлаждение) отключена в общих настройках.\nПодключите ее и включите в Настройках.";
    if (strcmp(text_eng, "Compressed Air system is disabled in Global Settings.") == 0) return "Система сжатого воздуха отключена в общих настройках.";
    if (strcmp(text_eng, "Nitrogen system is disabled in Global Settings.") == 0) return "Система азота отключена в общих настройках.";
    
    // --- Тексты справки для лаб. режимов ---
    if (strcmp(text_eng, "A mode for applying the final glossy layer. Adjust the UV flicker rate and stage times to achieve a perfect shine.") == 0) return "Режим для нанесения финального глянцевого слоя. Настройте частоту мерцания УФ и время этапов для достижения идеального блеска.";
    if (strcmp(text_eng, "This mode is designed for local repair and curing of the composite. Set a timer for preparation and the exposure time.") == 0) return "Этот режим предназначен для локального ремонта и отверждения композита. Установите таймер для подготовки и время засветки.";
    if (strcmp(text_eng, "Thermal post-processing with UV pulses to create additional cross-links in the polymer structure, making it harder and more wear-resistant.") == 0) return "Термическая постобработка с УФ-импульсами для создания дополнительных поперечных связей в структуре полимера, что делает его более твердым и износостойким.";
    if (strcmp(text_eng, "A simple mode for heating the chamber to a set temperature and holding it for a specified time. Used for various laboratory needs.") == 0) return "Режим простого нагрева камеры до заданной температуры и удержания в течение указанного времени. Используется для различных лабораторных нужд.";
    if (strcmp(text_eng, "This mode lightens composite materials by holding them at a fixed high temperature.") == 0) return "Этот режим осветляет композитные материалы путем выдержки при фиксированной высокой температуре.";
    if (strcmp(text_eng, "This mode darkens composite materials using UV exposure.") == 0) return "Этот режим затемняет композитные материалы с помощью УФ-облучения.";

    // --- Экран блокировки ---
    if (strcmp(text_eng, "DEVICE LOCKED") == 0) return "УСТРОЙСТВО ЗАБЛОКИРОВАНО";
    if (strcmp(text_eng, "Critical Error!\nHeating element failure.\nDevice is locked.") == 0) return "Критическая ошибка!\nОтказ нагревательного элемента.\nУстройство заблокировано.";
    if (strcmp(text_eng, "Enter code here...") == 0) return "Введите код...";
    if (strcmp(text_eng, "ENTER") == 0) return "ВВОД";
    if (strcmp(text_eng, "Invalid code! Please try again.") == 0) return "Неверный код! Попробуйте еще раз.";

    // --- Экран процесса ---
    if (strcmp(text_eng, "Countdown") == 0) return "Обратный отсчет";
    if (strcmp(text_eng, "Bring the model to the indicator") == 0) return "Поднесите модель под указатель";
    if (strcmp(text_eng, "Target O2:") == 0) return "Цель O2:";
    if (strcmp(text_eng, "Back") == 0) return "Назад";
    if (strcmp(text_eng, "Temp:") == 0) return "Температура:";
    if (strcmp(text_eng, "Oxygen:") == 0) return "Кислород:";
    if (strcmp(text_eng, "Time left:") == 0) return "Осталось:";
    if (strcmp(text_eng, "Target:") == 0) return "Цель:";
    if (strcmp(text_eng, "s") == 0) return "с"; // Для секунд, если понадобится

    // --- Кнопки диалогов ---
    if (strcmp(text_eng, "OK") == 0) return "Понятно";
    if (strcmp(text_eng, "Test") == 0) return "Тест";
    if (strcmp(text_eng, "Start Supply") == 0) return "Включить подачу";
    if (strcmp(text_eng, "Stop Supply") == 0) return "Выключить подачу";
    if (strcmp(text_eng, "Nitrogen Supply Test") == 0) return "Тест подачи Азота";
    if (strcmp(text_eng, "Compressed Air Test") == 0) return "Тест подачи сжатого воздуха";
    if (strcmp(text_eng, "Press to supply") == 0) return "Нажми для подачи";
    if (strcmp(text_eng, "Screen auto-off:") == 0) return "Автоотключение экрана:";
    if (strcmp(text_eng, "5 min") == 0) return "5 мин";
    if (strcmp(text_eng, "15 min") == 0) return "15 мин";
    if (strcmp(text_eng, "30 min") == 0) return "30 мин";
    if (strcmp(text_eng, "60 min") == 0) return "60 мин";
    if (strcmp(text_eng, "Cancel") == 0) return "Отмена";
    if (strcmp(text_eng, "Skip") == 0) return "Пропустить";

    // --- Добавляем в конец или в логически подходящий блок ---
    if (strcmp(text_eng, "Nitrogen System Error") == 0) return "Ошибка системы азота";
    if (strcmp(text_eng, "The oxygen level is not decreasing. Continue the process without nitrogen?") == 0) return "Уровень кислорода не снижается. Продолжить процесс без продувки азотом?";
    if (strcmp(text_eng, "Continue without N2") == 0) return "Продолжить без азота";
    if (strcmp(text_eng, "Cancel Process") == 0) return "Завершить процесс";
    if (strcmp(text_eng, "Continuing process...") == 0) return "Продолжение процесса...";
    if (strcmp(text_eng, "Nitrogen purge stage skipped.") == 0) return "Этап продувки азотом пропущен.";

    if (strcmp(text_eng, "PROCESS_COMPLETE_WITH_COOLING_WARNING") == 0) return "Процесс завершен c предупреждением";
    if (strcmp(text_eng, "Warning: The compressed air supply may have failed during the process. Please check the system in Settings -> Test.") == 0) return "Внимание: Во время процесса мог произойти сбой подачи сжатого воздуха. Пожалуйста, проверьте систему в Настройках -> Тест.";

    // Если перевод не найден, возвращаем оригинальную английскую строку
    return text_eng;
}

void validate_float_input(lv_event_t * e, float min_val, float max_val) {
    lv_obj_t * ta = lv_event_get_target(e);
    const char* txt = lv_textarea_get_text(ta);
    float val = atof(txt);
    
    bool changed = false;
    if (val < min_val) {
        val = min_val;
        changed = true;
    } else if (val > max_val) {
        val = max_val;
        changed = true;
    }

    if (changed) {
        char buf[10];
        snprintf(buf, sizeof(buf), "%.1f", val);
        lv_textarea_set_text(ta, buf);
        // <<<--- ДОБАВЛЕНО: Стабилизируем курсор и здесь ---<<<
        lv_textarea_set_cursor_pos(ta, LV_TEXTAREA_CURSOR_LAST);
        Serial.printf("Float input validated and corrected to: %.1f\n", val);
    }
}

const char* translateSystemStatus(const char* status_msg) {
    // --- Статусы для профилей (уже были) ---
    if (strcmp(status_msg, "ACK:PROCESS_STARTED") == 0) return "Процесс запущен";
    if (strcmp(status_msg, "STATUS:PREPARATION_OK") == 0) return "Подготовка завершена";
    if (strcmp(status_msg, "STATUS:HEATING_STARTED") == 0) return "Этап: Нагрев";
    if (strcmp(status_msg, "STATUS:COOLING_STARTED") == 0) return "Этап: Охлаждение";
    if (strcmp(status_msg, "STATUS:HOLDING_TEMPERATURE") == 0) return "Этап: Удержание температуры";
    if (strcmp(status_msg, "STATUS:NITROGEN_PURGE_STARTED") == 0) return "Этап: Продувка азотом";
    if (strcmp(status_msg, "STATUS:PRIMARY_UV_STARTED") == 0) return "Этап: Первичный УФ (Мерцания)";
    if (strcmp(status_msg, "STATUS:SECONDARY_UV_STARTED") == 0) return "Этап: Вторичный УФ (Статичный)";
    if (strcmp(status_msg, "STATUS:POST_COOLING_STARTED") == 0) return "Этап: Финальное охлаждение";
    if (strcmp(status_msg, "STATUS:TERTIARY_UV_STARTED") == 0) return "Этап: Третичный УФ (Статичный)";
    if (strcmp(status_msg, "PROCESS_COMPLETE") == 0) return "Процесс завершен";
    if (strcmp(status_msg, "PROCESS_COMPLETE_WITH_COOLING_WARNING") == 0) return "Процесс завершен c предупреждением";
    if (strcmp(status_msg, "ERROR:DOOR_IS_OPEN") == 0) return "ОШИБКА: Дверь открыта!";
    if (strcmp(status_msg, "FATAL_ERROR:HEATER_FAILURE") == 0) return "КРИТИЧЕСКАЯ ОШИБКА: Нагреватель";
    if (strcmp(status_msg, "WARN:COOLING_SKIPPED") == 0) return "ВНИМАНИЕ: Охлаждение пропущено";
    if (strcmp(status_msg, "WARN:NITROGEN_SKIPPED") == 0) return "ВНИМАНИЕ: Продувка азотом пропущена";

    // --- НОВЫЕ СТАТУСЫ ДЛЯ ЛАБОРАТОРНЫХ РЕЖИМОВ ---
    if (strcmp(status_msg, "STATUS:Pre-cooling chamber...") == 0) return "Этап: Предв. охлаждение";
    if (strcmp(status_msg, "STATUS:Blowing monomer...") == 0) return "Этап: Сушка мономера";
    if (strcmp(status_msg, "STATUS:Purging with nitrogen...") == 0) return "Этап: Продувка азотом";
    if (strcmp(status_msg, "STATUS:UV flickering started...") == 0) return "Этап: УФ-мерцание";
    if (strcmp(status_msg, "STATUS:Repair UV is ON") == 0) return "Этап: УФ-засветка для ремонта";
    if (strcmp(status_msg, "STATUS:Heating chamber...") == 0) return "Этап: Нагрев камеры";
    if (strcmp(status_msg, "STATUS:Holding temperature...") == 0) return "Этап: Удержание температуры";
    if (strcmp(status_msg, "STATUS:Holding temperature with UV pulses...") == 0) return "Этап: Удержание t° с УФ-импульсами";
    if (strcmp(status_msg, "STATUS:Static UV exposure...") == 0) return "Этап: Статичная УФ-засветка";
    if (strcmp(status_msg, "STATUS:Final cooling...") == 0) return "Этап: Финальное охлаждение";
    if (strcmp(status_msg, "STATUS:System components cooling") == 0) return "Этап: Охлаждение компонентов"; 

    // Если перевод не найден, возвращаем оригинальное сообщение
    return status_msg;
}
static void update_custom_toggle_ui(lv_obj_t* toggle_box, int active_index) {
    if (!toggle_box) return;

    lv_obj_t* btn1 = lv_obj_get_child(toggle_box, 0);
    lv_obj_t* btn2 = lv_obj_get_child(toggle_box, 1);
    if (!btn1 || !btn2) return;

    bool is_dark_theme = (current_global_settings.theme == 1);
    lv_color_t inactive_bg = is_dark_theme ? lv_color_hex(0x2C2C2C) : lv_palette_lighten(LV_PALETTE_GREY, 2);
    lv_color_t inactive_text = is_dark_theme ? lv_color_white() : lv_color_black();
    
    // <<< --- ИЗМЕНЕНИЕ ЗДЕСЬ --- >>>
    lv_color_t active_color = is_dark_theme ? lv_color_hex(0xff05b8) : lv_palette_main(LV_PALETTE_BLUE);

    // Первая кнопка
    if (active_index == 0) {
        lv_obj_set_style_bg_color(btn1, active_color, 0); // <<< Используем переменную
        lv_obj_set_style_text_color(lv_obj_get_child(btn1, 0), lv_color_white(), 0);
    } else {
        lv_obj_set_style_bg_color(btn1, inactive_bg, 0);
        lv_obj_set_style_text_color(lv_obj_get_child(btn1, 0), inactive_text, 0);
    }

    // Вторая кнопка
    if (active_index == 1) {
        lv_obj_set_style_bg_color(btn2, active_color, 0); // <<< Используем переменную
        lv_obj_set_style_text_color(lv_obj_get_child(btn2, 0), lv_color_white(), 0);
    } else {
        lv_obj_set_style_bg_color(btn2, inactive_bg, 0);
        lv_obj_set_style_text_color(lv_obj_get_child(btn2, 0), inactive_text, 0);
    }
}
void handleFatalError(const char* message) {
    Serial.println("-------------------- FATAL ERROR --------------------");
    Serial.println(message); Serial.println("System halted.");
    Serial.println("-----------------------------------------------------");
    while (1) { delay(1000); }
}
bool readFileContentToBuffer_ino(fs::FS &fs_ref, const char * path, char* buffer, size_t buffer_size) {
    Serial.printf("INO: Reading file to char buffer: %s (buffer_size: %u)\n", path, buffer_size);
    if (buffer_size == 0) { Serial.println("INO: readFileContentToBuffer_ino - Zero buffer size!"); return false; }
    memset(buffer, 0, buffer_size); 
    if (!sd_card_initialized) { Serial.println("INO: readFileContentToBuffer_ino - SD card not initialized!"); strncpy(buffer, "(SD Error: Not Init)", buffer_size - 1); buffer[buffer_size-1] = '\0'; return false; }
    File file = fs_ref.open(path, FILE_READ);
    if (!file) { Serial.printf("INO: Failed to open file for reading: %s\n", path); strncpy(buffer, "(File Open Error)", buffer_size - 1); buffer[buffer_size-1] = '\0'; return false; }
    size_t file_actual_size = file.size();
    size_t bytes_to_read = (file_actual_size < buffer_size -1) ? file_actual_size : (buffer_size - 2); 
    size_t bytes_read = 0;
    if (file_actual_size > 0) { bytes_read = file.readBytes(buffer, bytes_to_read); buffer[bytes_read] = '\0'; }
    else { Serial.println("INO: File is reported as empty by file.size()."); strncpy(buffer, "(File is empty)", buffer_size - 1); buffer[buffer_size-1] = '\0'; }
    file.close();
    if (bytes_read == 0 && file_actual_size > 0) { Serial.println("INO: Read 0 bytes, but file was not empty. Possible read issue."); }
    else if (bytes_read > 0) { Serial.printf("INO: Read %d bytes.\n", bytes_read); }
    if (file_actual_size > bytes_to_read && bytes_read > 0 && bytes_read < buffer_size -1) { 
        Serial.println("INO: File content was truncated."); const char* truncated_msg = "\n...(truncated)";
        if (strlen(buffer) + strlen(truncated_msg) < buffer_size) { strcat(buffer, truncated_msg); }
    }
    return true; 
}

int roundToStep(int value, int step) { if (step == 0) return value; return ((value + step / 2) / step) * step; }
void validate_numeric_input(lv_event_t * e, int min_val, int max_val) {
    lv_obj_t * ta = lv_event_get_target(e); const char* txt = lv_textarea_get_text(ta); int val = atoi(txt); 
    bool changed = false; if (val < min_val) { val = min_val; changed = true; } else if (val > max_val) { val = max_val; changed = true; }
    if (changed) { 
        char buf[10]; 
        snprintf(buf, sizeof(buf), "%d", val); 
        lv_textarea_set_text(ta, buf); 
        // <<<--- ДОБАВЛЕНО: Стабилизируем курсор, чтобы убрать визуальный глюк ---<<<
        lv_textarea_set_cursor_pos(ta, LV_TEXTAREA_CURSOR_LAST);
        Serial.printf("Input validated and corrected to: %d\n", val); 
    }
}

// Удаляет использованный ключ и перезаписывает файл
void remove_and_save_service_keys(String key_to_remove) {
    bool key_found = false;

    // Ищем и удаляем ключ из вектора в памяти
    for (auto it = service_keys.begin(); it != service_keys.end(); ++it) {
        if (*it == key_to_remove) {
            it = service_keys.erase(it);
            key_found = true;
            break;
        }
    }

    if (!key_found) {
        Serial.println("Key to remove not found in memory. No changes made.");
        return;
    }

    // Просто вызываем центральную функцию сохранения
    if (saveConfiguration()) {
        Serial.printf("Key '%s' removed and config saved. %d keys remaining.\n", key_to_remove.c_str(), service_keys.size());
    } else {
        Serial.println("ERROR: Failed to save config after removing a key.");
        // В критической ситуации можно попробовать восстановить ключ в памяти, но пока это излишне
    }
}

void enter_service_lock_mode(const char* message_eng, const char* message_rus) {
    Serial.printf("ENTERING SERVICE LOCK MODE. Reason: %s\n", message_eng);

    // 1. Устанавливаем флаг ошибки
    current_global_settings.is_heater_error = true;
    
    // 2. НЕМЕДЛЕННО СОХРАНЯЕМ НАСТРОЙКИ, ЧТОБЫ ЗАФИКСИРОВАТЬ БЛОКИРОВКУ
    saveConfiguration();
    
    // 3. Останавливаем все процессы на УП
    main_process_running = false;
    is_lab_mode_running = false; // Добавим сброс и этого флага для полной чистоты
    current_process_stage = "";
    
    // 4. Отправляем команду аварийной остановки по UART
    command_json_doc.clear();
    command_json_doc["command"] = "EMERGENCY_STOP";
    String output;
    serializeJson(command_json_doc, output);
    MySerial1.println(output);
    Serial.println("Sent UART command: EMERGENCY_STOP");
    
    // 5. Переключаемся на экран блокировки
    if(screen_service_lock) {
        load_screen(screen_service_lock);
    }
    
    // 6. ОЧИЩАЕМ БУФЕР UART ПОСЛЕ ВСЕХ ДЕЙСТВИЙ
    // Даем ПП микроскопическую паузу на отправку ACK, если она успеет, и чистим всё.
    flush_serial_buffer(); 
}

static void laboratory_mode_btn_event_cb(lv_event_t * e) {
    if (lv_event_get_code(e) == LV_EVENT_CLICKED) {
        Serial.println("Laboratory Mode button clicked. Loading lab screen and translating UI...");
        
        // Создаем массивы с переводами для удобства
        const char* tile_names_rus[] = {"Глазурь", "Ремонт модели", "Повышение прочности", "Термокамера", "Осветление", "Затемнение"};
        const char* tile_names_eng[] = {"Glaze", "Model Repair", "Strength Boost", "Thermal Chamber", "Lighten", "Darken"};
        
        // Создаем массив с нашими глобальными указателями на метки
        lv_obj_t* all_tile_labels[] = {
            label_lab_tile_1, label_lab_tile_2, label_lab_tile_3,
            label_lab_tile_4, label_lab_tile_5, label_lab_tile_6
        };

        // Обновляем текст на элементах перед переходом
        if (current_global_settings.language == 1) { // RUS
            lv_label_set_text(label_lab_header, "Лабораторный режим");
            lv_label_set_text(label_btn_lab_back, "Назад");
            
            for(int i = 0; i < 6; i++) {
                if (all_tile_labels[i]) {
                    lv_label_set_text(all_tile_labels[i], tile_names_rus[i]);
                    // Применяем стиль с поддержкой кириллицы
                    lv_obj_add_style(all_tile_labels[i], &style_my_text_18, 0);
                }
            }

        } else { // ENG
            lv_label_set_text(label_lab_header, "Laboratory Mode");
            lv_label_set_text(label_btn_lab_back, "Back");
            
            for(int i = 0; i < 6; i++) {
                if (all_tile_labels[i]) {
                    lv_label_set_text(all_tile_labels[i], tile_names_eng[i]);
                    // Применяем тот же стиль, чтобы шрифты были одинаковыми
                    lv_obj_add_style(all_tile_labels[i], &style_my_text_18, 0);
                }
            }
        }
        
        // Переходим на новый экран
        if (screen_laboratory) {
            apply_theme_to_laboratory_screen();
            load_screen(screen_laboratory);
        }
    }
}


static void thermal_chamber_enable_switch_event_cb(lv_event_t * e) {
    lv_obj_t* sw = lv_event_get_target(e);
    bool is_enabled = lv_obj_has_state(sw, LV_STATE_CHECKED);

    if (is_enabled) {
        Serial.println("Thermal chamber ENABLED by user in edit screen.");
        if (thermal_elements_container) {
            // Делаем контейнер полностью видимым (непрозрачным)
            lv_obj_set_style_opa(thermal_elements_container, LV_OPA_COVER, 0);
            // Разрешаем клики по нему и его дочерним элементам
            lv_obj_clear_flag(thermal_elements_container, LV_OBJ_FLAG_CLICKABLE);
        }
    } else {
        Serial.println("Thermal chamber DISABLED by user in edit screen.");
        if (thermal_elements_container) {
            // Делаем контейнер полностью прозрачным (он останется на своем месте!)
            lv_obj_set_style_opa(thermal_elements_container, LV_OPA_TRANSP, 0);
            // Запрещаем клики, чтобы случайно не нажать на невидимое поле ввода
            lv_obj_add_flag(thermal_elements_container, LV_OBJ_FLAG_CLICKABLE);
        }
    }
}

static void profile_detail_start_event_cb(lv_event_t * e) {
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;

    log_memory_status("START Button Click");

    Serial.println("--- START button clicked. Preparing full process command. ---");

    screen_to_return_after_process = screen_profile_details; 
    is_lab_mode_running = false;

    

    // 1. Переходим на экран процесса
    if (screen_process_execution) {
        lv_label_set_text(label_process_profile_name, current_active_profile_data.name);
        lv_label_set_text(label_process_status_title, tr("Starting Process..."));
        lv_label_set_text(label_process_status_detail, tr("Sending profile to controller..."));
        lv_label_set_text(label_btn_process_cancel, tr("Cancel"));
        
        lv_obj_remove_local_style_prop(lv_obj_get_child(screen_process_execution, 0), LV_STYLE_BG_COLOR, 0);
        // lv_obj_clear_flag(spinner_process_execution, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(repair_mode_indicator_obj, LV_OBJ_FLAG_HIDDEN);
        apply_theme_to_process_screen(); 

        load_screen(screen_process_execution);
        lv_obj_clear_flag(btn_process_cancel, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_state(btn_process_cancel, LV_STATE_DISABLED);
        // if(spinner_process_execution) lv_obj_clear_flag(spinner_process_execution, LV_OBJ_FLAG_HIDDEN);
    }

    // 2. Используем ГЛОБАЛЬНЫЙ JsonDocument
    command_json_doc.clear(); // Очищаем его перед использованием
    command_json_doc["command"] = "START_PROCESS";
    
    // 3. Вкладываем все параметры профиля в объект "params"
    JsonObject params = command_json_doc.createNestedObject("params");
    params["thermal_chamber_enabled"] = current_active_profile_data.thermal_chamber_enabled;
    params["thermal_chamber_temp"] = current_active_profile_data.thermal_chamber_temp;
    params["heat_exchange_hold_sec"] = current_active_profile_data.heat_exchange_hold_sec;
    params["nitrogen_use_enabled"] = current_active_profile_data.nitrogen_use_enabled;
    params["nitrogen_target_percent"] = current_active_profile_data.nitrogen_target_percent;
    params["nitrogen_boost_sec"] = current_active_profile_data.nitrogen_boost_sec;
    params["primary_uv_exposure_sec"] = current_active_profile_data.primary_uv_exposure_sec;
    params["primary_uv_mode"] = current_active_profile_data.primary_uv_mode;
    params["primary_uv_flicker_on_sec"] = current_active_profile_data.primary_uv_flicker_on_sec;
    params["secondary_uv_exposure_sec"] = current_active_profile_data.secondary_uv_exposure_sec;
    params["secondary_uv_mode"] = current_active_profile_data.secondary_uv_mode;
    params["tertiary_uv_exposure_sec"] = current_active_profile_data.tertiary_uv_exposure_sec;
    params["tertiary_uv_mode"] = current_active_profile_data.tertiary_uv_mode;
    params["chamber_cooling_enabled"] = current_active_profile_data.chamber_cooling_enabled;
    params["post_cooling_air_purge_sec"] = current_active_profile_data.post_cooling_air_purge_sec;

    // 4. Сериализуем JSON в строку и отправляем
    String output;
    serializeJson(command_json_doc, output);
    flush_serial_buffer();
    MySerial1.println(output);

    Serial.println("Full process command sent:");
    Serial.println(output);

    // 5. Устанавливаем флаг, что процесс запущен
    main_process_running = true;
}

// Раздел 2: Функции обновления UI и обработчики событий LVGL
// ==========================================================================
static void update_timeout_toggle_ui(lv_obj_t* toggle_box, int active_index) {
    if (!toggle_box) return;

    for(int i = 0; i < 4; i++) {
        lv_obj_t* btn = lv_obj_get_child(toggle_box, i);
        if(!btn) continue;
        
        bool is_dark_theme = (current_global_settings.theme == 1);
        
        // <<< --- ИЗМЕНЕНИЕ ЗДЕСЬ --- >>>
        lv_color_t active_color = is_dark_theme ? lv_color_hex(0xff05b8) : lv_palette_main(LV_PALETTE_BLUE);
        
        if(i == active_index) {
            // Активная кнопка
            lv_obj_set_style_bg_color(btn, active_color, 0); // <<< Используем переменную
            lv_obj_set_style_text_color(lv_obj_get_child(btn, 0), lv_color_white(), 0);
        } else {
            // Неактивные кнопки
            lv_color_t inactive_bg = is_dark_theme ? lv_color_hex(0x2C2C2C) : lv_palette_lighten(LV_PALETTE_GREY, 2);
            lv_color_t inactive_text = is_dark_theme ? lv_color_white() : lv_color_black();
            lv_obj_set_style_bg_color(btn, inactive_bg, 0);
            lv_obj_set_style_text_color(lv_obj_get_child(btn, 0), inactive_text, 0);
        }
    }
}

void apply_theme_to_keyboard_screen() {
    if (!screen_keyboard) return;

    // Массив из указателей на все клавиатуры для удобства
    lv_obj_t* keyboards[] = { kb_edit_alpha, kb_edit_numeric, kb_service_code };

    // =====================================================================
    // <<< --- СТИЛИ ДЛЯ СПЕЦИАЛЬНЫХ КНОПОК (создаем их один раз) --- >>>
    // =====================================================================
    static lv_style_t style_keyboard_ok_btn_dark;
    static bool dark_style_inited = false;
    if(!dark_style_inited) {
        lv_style_init(&style_keyboard_ok_btn_dark);
        lv_style_set_bg_color(&style_keyboard_ok_btn_dark, lv_palette_main(LV_PALETTE_GREEN));
        dark_style_inited = true;
    }

    static lv_style_t style_keyboard_ok_btn_light;
    static bool light_style_inited = false;
    if(!light_style_inited) {
        lv_style_init(&style_keyboard_ok_btn_light);
        lv_style_set_bg_color(&style_keyboard_ok_btn_light, lv_palette_main(LV_PALETTE_GREEN));
        light_style_inited = true;
    }


    // =====================================================================
    // <<< --- БЛОК ДЛЯ ТЕМНОЙ ТЕМЫ --- >>>
    // =====================================================================
    if (current_global_settings.theme == 1) { // --- ТЕМНАЯ ТЕМА ---
        // ... (код для фона и поля ввода остается без изменений) ...
        lv_obj_set_style_bg_color(screen_keyboard, lv_color_hex(0x1C1C1C), 0);
        lv_obj_add_style(ta_keyboard_proxy, &style_dark_textarea, 0);
        lv_obj_set_style_bg_color(ta_keyboard_proxy, lv_color_hex(0x424242), 0);
        lv_obj_set_style_bg_opa(screen_keyboard, LV_OPA_COVER, 0);
        
        // Стилизуем все клавиатуры
        for (auto kb : keyboards) {
            if (!kb) continue;
            
            // Удаляем стиль для светлых кнопок "OK", если он был
            lv_obj_remove_style(kb, &style_keyboard_ok_btn_light, LV_PART_ITEMS | LV_STATE_CHECKED);

            // 1. Фон всей клавиатуры (подложка)
            lv_obj_set_style_bg_color(kb, lv_color_hex(0x2C2C2C), LV_PART_MAIN);
            
            // 2. Стили для ОБЫЧНЫХ кнопок (LV_PART_ITEMS в обычном состоянии)
            lv_obj_set_style_bg_color(kb, lv_color_hex(0x424242), LV_PART_ITEMS); 
            lv_obj_set_style_text_color(kb, lv_color_white(), LV_PART_ITEMS);
            
            // 3. Стиль для кнопок В МОМЕНТ НАЖАТИЯ
            lv_obj_set_style_bg_color(kb, lv_color_hex(0xa802a6), LV_PART_ITEMS | LV_STATE_PRESSED);

            // 4. <<< --- ВОТ ОНО! ПРИМЕНЯЕМ СТИЛЬ ДЛЯ КНОПКИ "OK" --- >>>
            // Этот стиль перекроет цвет фона для кнопок, у которых есть состояние CHECKED
            // lv_obj_add_style(kb, &style_keyboard_ok_btn_dark, LV_PART_ITEMS | LV_STATE_CHECKED);
        }

    // =====================================================================
    // <<< --- БЛОК ДЛЯ СВЕТЛОЙ ТЕМЫ --- >>>
    // =====================================================================
    } else { // --- СВЕТЛАЯ ТЕМА ---
        // ... (код для фона и поля ввода остается без изменений) ...
        lv_obj_set_style_bg_color(screen_keyboard, lv_color_hex(0x333333), 0);
        lv_obj_remove_style(ta_keyboard_proxy, &style_dark_textarea, 0);
        lv_obj_add_style(ta_keyboard_proxy, &style_light_textarea, 0);

        // Стилизуем все клавиатуры для светлой темы
        for (auto kb : keyboards) {
            if (!kb) continue;

            // Удаляем стиль для темных кнопок "OK"
            lv_obj_remove_style(kb, &style_keyboard_ok_btn_dark, LV_PART_ITEMS | LV_STATE_CHECKED);

            // 1. Фон всей клавиатуры (подложка)
            lv_obj_set_style_bg_color(kb, lv_color_hex(0xD3D3D3), LV_PART_MAIN);
            
            // 2. Стили для ОБЫЧНЫХ кнопок (LV_PART_ITEMS в обычном состоянии)
            lv_obj_set_style_bg_color(kb, lv_color_white(), LV_PART_ITEMS);
            lv_obj_set_style_text_color(kb, lv_color_black(), LV_PART_ITEMS);
            
            // 3. Сбрасываем стиль нажатия
            lv_obj_remove_style(kb, NULL, LV_PART_ITEMS | LV_STATE_PRESSED);

            // 4. <<< --- И ЗДЕСЬ ТОЖЕ ПРИМЕНЯЕМ СТИЛЬ ДЛЯ КНОПКИ "OK" --- >>>
            // lv_obj_add_style(kb, &style_keyboard_ok_btn_light, LV_PART_ITEMS | LV_STATE_CHECKED);
        }
    }
}
void apply_theme_to_info_dialog() {
    if (!screen_info_dialog) return;

    if (current_global_settings.theme == 1) { // --- ТЕМНАЯ ТЕМА ---
        // Устанавливаем стили для темной темы
        lv_obj_set_style_bg_color(screen_info_dialog, lv_color_hex(0x2C2C2C), 0);
        lv_obj_set_style_border_color(screen_info_dialog, lv_palette_darken(LV_PALETTE_GREY, 1), 0);
        lv_obj_add_style(label_info_dialog_text, &style_dark_text, 0);
        lv_obj_add_style(btn_info_dialog_ok, &style_dark_btn, 0);

    } else { // --- СВЕТЛАЯ ТЕМА ---
        // Возвращаем стили для светлой темы
        lv_obj_set_style_bg_color(screen_info_dialog, lv_color_white(), 0);
        lv_obj_set_style_border_color(screen_info_dialog, lv_palette_main(LV_PALETTE_GREY), 0);
        lv_obj_remove_style(label_info_dialog_text, &style_dark_text, 0);
        lv_obj_remove_style(btn_info_dialog_ok, &style_dark_btn, 0);
    }
}
void apply_theme_to_confirm_delete_dialog() {
    if (!screen_confirm_delete_dialog) return;

    // Находим кнопки по их дочерним меткам, так как у нас нет на них глобальных указателей
    lv_obj_t* btn_cancel = lv_obj_get_parent(label_confirm_btn_cancel);
    lv_obj_t* btn_delete = lv_obj_get_parent(label_confirm_btn_delete);

    if (current_global_settings.theme == 1) { // --- ТЕМНАЯ ТЕМА ---
        // Устанавливаем стили для темной темы
        lv_obj_set_style_bg_color(screen_confirm_delete_dialog, lv_color_hex(0x2C2C2C), 0);
        lv_obj_set_style_border_color(screen_confirm_delete_dialog, lv_palette_darken(LV_PALETTE_GREY, 1), 0);
        lv_obj_add_style(label_confirm_delete_title, &style_dark_text, 0);
        lv_obj_add_style(label_confirm_delete_text, &style_dark_text, 0);
        
        // Кнопки
        lv_obj_add_style(btn_cancel, &style_dark_btn, 0);
        // Красную кнопку не трогаем, она всегда должна быть красной
        // lv_obj_add_style(btn_delete, &style_dark_btn, 0); 

    } else { // --- СВЕТЛАЯ ТЕМА ---
        // Возвращаем стили для светлой темы
        lv_obj_set_style_bg_color(screen_confirm_delete_dialog, lv_color_white(), 0);
        lv_obj_set_style_border_color(screen_confirm_delete_dialog, lv_palette_main(LV_PALETTE_GREY), 0);
        lv_obj_remove_style(label_confirm_delete_title, &style_dark_text, 0);
        lv_obj_remove_style(label_confirm_delete_text, &style_dark_text, 0);

        // Кнопки
        lv_obj_remove_style(btn_cancel, &style_dark_btn, 0);
        // Красную кнопку не трогаем
    }
}
void apply_theme_to_help_screen() {
    if (!screen_help) return;

    // Находим главный контейнер
    lv_obj_t* main_container = lv_obj_get_child(screen_help, 0);
    if (!main_container) return;
    
    // Находим контейнер для текста, чтобы задать ему фон
    lv_obj_t* text_container = lv_obj_get_child(main_container, 1);

    if (current_global_settings.theme == 1) { // --- ТЕМНАЯ ТЕМА ---
        // Удаляем стили светлой темы
        lv_obj_remove_style(main_container, &style_light_bg, 0);
        lv_obj_remove_style(label_help_title, &style_light_text, 0);
        lv_obj_remove_style(label_help_content, &style_light_text, 0);
        lv_obj_remove_style(btn_help_close, &style_light_btn, 0);
        if (text_container) lv_obj_set_style_bg_opa(text_container, LV_OPA_TRANSP, 0); // Убираем фон, если был

        // Добавляем стили темной темы
        lv_obj_add_style(main_container, &style_dark_bg, 0);
        lv_obj_add_style(label_help_title, &style_dark_text, 0);
        lv_obj_add_style(label_help_content, &style_dark_text, 0);
        lv_obj_add_style(btn_help_close, &style_dark_btn, 0);
        if (text_container) {
             lv_obj_set_style_bg_color(text_container, lv_color_hex(0x2C2C2C), 0); // Темно-серый фон
             lv_obj_set_style_bg_opa(text_container, LV_OPA_COVER, 0);
        }

    } else { // --- СВЕТЛАЯ ТЕМА ---
        // Удаляем стили темной темы
        lv_obj_remove_style(main_container, &style_dark_bg, 0);
        lv_obj_remove_style(label_help_title, &style_dark_text, 0);
        lv_obj_remove_style(label_help_content, &style_dark_text, 0);
        lv_obj_remove_style(btn_help_close, &style_dark_btn, 0);
        if (text_container) lv_obj_set_style_bg_opa(text_container, LV_OPA_TRANSP, 0); // Убираем фон, если был

        // Добавляем стили светлой темы
        lv_obj_add_style(main_container, &style_light_bg, 0);
        lv_obj_add_style(label_help_title, &style_light_text, 0);
        lv_obj_add_style(label_help_content, &style_light_text, 0);
        lv_obj_add_style(btn_help_close, &style_light_btn, 0);
        if (text_container) {
            lv_obj_set_style_bg_color(text_container, lv_color_white(), 0); // Белый фон
            lv_obj_set_style_bg_opa(text_container, LV_OPA_COVER, 0);
        }
    }
}
void apply_theme_to_settings_screen() {
    if (!screen_settings) return;

    // Находим все нужные объекты
    lv_obj_t* content_container = lv_obj_get_child(screen_settings, 0);
    if (!content_container) return;

    lv_obj_t* content_grid = lv_obj_get_child(content_container, 1);
    lv_obj_t* left_block = lv_obj_get_child(content_grid, 0);
    lv_obj_t* right_block = lv_obj_get_child(content_grid, 2);

    lv_obj_t* btn_lang1 = lv_obj_get_child(lang_toggle_box, 0);
    lv_obj_t* btn_lang2 = lv_obj_get_child(lang_toggle_box, 1);
    lv_obj_t* btn_theme1 = lv_obj_get_child(theme_toggle_box, 0);
    lv_obj_t* btn_theme2 = lv_obj_get_child(theme_toggle_box, 1);

    if (current_global_settings.theme == 1) { // --- ТЕМНАЯ ТЕМА ---
        // Удаляем стили светлой темы
        lv_obj_remove_style(content_container, &style_light_bg, 0);
        lv_obj_remove_style(label_settings_title, &style_light_text, 0);
        lv_obj_remove_style(label_settings_nitrogen, &style_light_text, 0);
        lv_obj_remove_style(label_settings_air, &style_light_text, 0);
        lv_obj_remove_style(label_settings_language, &style_light_text, 0);
        lv_obj_remove_style(label_settings_theme, &style_light_text, 0);
        lv_obj_remove_style(btn_settings_save_and_back, &style_light_btn, 0);

        // Добавляем стили темной темы
        lv_obj_add_style(content_container, &style_dark_bg, 0);
        lv_obj_add_style(label_settings_title, &style_dark_text, 0);
        lv_obj_add_style(label_settings_nitrogen, &style_dark_text, 0);
        lv_obj_add_style(label_settings_air, &style_dark_text, 0);
        lv_obj_add_style(label_settings_language, &style_dark_text, 0);
        lv_obj_add_style(label_settings_theme, &style_dark_text, 0);
        lv_obj_add_style(btn_settings_save_and_back, &style_dark_btn, 0);
        lv_obj_add_style(label_settings_timeout, &style_dark_text, 0);

        lv_obj_add_style(left_block_settings, &style_dark_block_bg, 0);
        lv_obj_add_style(right_block_settings, &style_dark_block_bg, 0);
        lv_obj_add_style(btn_settings_test_nitro, &style_dark_btn, 0);
        lv_obj_add_style(btn_settings_test_air, &style_dark_btn, 0);

        // <<< НАЧАЛО БЛОКА ДЛЯ СТАНДАРТНЫХ ПЕРЕКЛЮЧАТЕЛЕЙ (ТЕМНАЯ ТЕМА) >>>
        lv_obj_t* switches[] = { sw_settings_global_nitrogen_enabled, sw_settings_global_air_enabled };
        for (auto sw : switches) {
            if (!sw) continue;
            // Убираем стандартные цвета темы, чтобы они не мешали
            lv_obj_remove_style(sw, NULL, LV_PART_INDICATOR | LV_STATE_CHECKED);
            
            // Фон для ВЫКЛЮЧЕННОГО состояния
            lv_obj_set_style_bg_color(sw, lv_color_hex(0x1C1C1C), LV_PART_MAIN);
            
            // Фон для ВКЛЮЧЕННОГО состояния
            lv_obj_set_style_bg_color(sw, lv_color_hex(0xff05b8), LV_PART_MAIN | LV_STATE_CHECKED);
        }
        // <<< КОНЕЦ БЛОКА >>>

        // Стилизуем кастомные переключатели для ТЕМНОЙ темы
        lv_obj_set_style_border_color(lang_toggle_box, lv_palette_main(LV_PALETTE_GREY), 0);
        lv_obj_set_style_border_color(theme_toggle_box, lv_palette_main(LV_PALETTE_GREY), 0);
        lv_obj_set_style_bg_color(btn_lang1, lv_color_hex(0x2C2C2C), 0);
        lv_obj_set_style_text_color(label_settings_lang_opt1, lv_color_white(), 0);
        lv_obj_set_style_bg_color(btn_lang2, lv_color_hex(0x2C2C2C), 0);
        lv_obj_set_style_text_color(label_settings_lang_opt2, lv_color_white(), 0);
        lv_obj_set_style_bg_color(btn_theme1, lv_color_hex(0x2C2C2C), 0);
        lv_obj_set_style_text_color(label_settings_theme_opt1, lv_color_white(), 0);
        lv_obj_set_style_bg_color(btn_theme2, lv_color_hex(0x2C2C2C), 0);
        lv_obj_set_style_text_color(label_settings_theme_opt2, lv_color_white(), 0);

    } else { // --- СВЕТЛАЯ ТЕМА ---
        // Удаляем стили темной темы
        lv_obj_remove_style(content_container, &style_dark_bg, 0);
        lv_obj_remove_style(label_settings_title, &style_dark_text, 0);
        lv_obj_remove_style(label_settings_nitrogen, &style_dark_text, 0);
        lv_obj_remove_style(label_settings_air, &style_dark_text, 0);
        lv_obj_remove_style(label_settings_language, &style_dark_text, 0);
        lv_obj_remove_style(label_settings_theme, &style_dark_text, 0);
        lv_obj_remove_style(btn_settings_save_and_back, &style_dark_btn, 0);
        lv_obj_remove_style(label_settings_timeout, &style_dark_text, 0);

        // Добавляем стили светлой темы
        lv_obj_add_style(content_container, &style_light_bg, 0);
        lv_obj_add_style(label_settings_title, &style_light_text, 0);
        lv_obj_add_style(label_settings_nitrogen, &style_light_text, 0);
        lv_obj_add_style(label_settings_air, &style_light_text, 0);
        lv_obj_add_style(label_settings_language, &style_light_text, 0);
        lv_obj_add_style(label_settings_theme, &style_light_text, 0);
        lv_obj_add_style(btn_settings_save_and_back, &style_light_btn, 0);
        lv_obj_add_style(label_settings_timeout, &style_light_text, 0);
        
        lv_obj_remove_style(left_block_settings, &style_dark_block_bg, 0);
        lv_obj_remove_style(right_block_settings, &style_dark_block_bg, 0);
        lv_obj_add_style(btn_settings_test_nitro, &style_light_btn, 0);
        lv_obj_add_style(btn_settings_test_air, &style_light_btn, 0);

        // <<< НАЧАЛО БЛОКА ДЛЯ СТАНДАРТНЫХ ПЕРЕКЛЮЧАТЕЛЕЙ (СВЕТЛАЯ ТЕМА) >>>
        lv_obj_t* switches[] = { sw_settings_global_nitrogen_enabled, sw_settings_global_air_enabled };
        for (auto sw : switches) {
            if (!sw) continue;
            // Явно задаем стандартные цвета для светлой темы
            lv_obj_set_style_bg_color(sw, lv_color_hex(0xF0F0F0), LV_PART_MAIN); // Фон для ВЫКЛЮЧЕННОГО состояния
            lv_obj_set_style_bg_color(sw, lv_palette_main(LV_PALETTE_BLUE), LV_PART_MAIN | LV_STATE_CHECKED); // Фон для ВКЛЮЧЕННОГО
        }
        // <<< КОНЕЦ БЛОКА >>>

        // Стилизуем кастомные переключатели для СВЕТЛОЙ темы
        lv_obj_set_style_border_color(lang_toggle_box, lv_palette_main(LV_PALETTE_BLUE), 0);
        lv_obj_set_style_border_color(theme_toggle_box, lv_palette_main(LV_PALETTE_BLUE), 0);
        lv_obj_set_style_bg_color(btn_lang1, lv_palette_lighten(LV_PALETTE_GREY, 2), 0);
        lv_obj_set_style_text_color(label_settings_lang_opt1, lv_color_black(), 0);
        lv_obj_set_style_bg_color(btn_lang2, lv_palette_lighten(LV_PALETTE_GREY, 2), 0);
        lv_obj_set_style_text_color(label_settings_lang_opt2, lv_color_black(), 0);
        lv_obj_set_style_bg_color(btn_theme1, lv_palette_lighten(LV_PALETTE_GREY, 2), 0);
        lv_obj_set_style_text_color(label_settings_theme_opt1, lv_color_black(), 0);
        lv_obj_set_style_bg_color(btn_theme2, lv_palette_lighten(LV_PALETTE_GREY, 2), 0);
        lv_obj_set_style_text_color(label_settings_theme_opt2, lv_color_black(), 0);
    }
    
    // Этот код остается без изменений
    update_custom_toggle_ui(lang_toggle_box, current_global_settings.language);
    update_custom_toggle_ui(theme_toggle_box, current_global_settings.theme);
}
void apply_theme_to_profile_edit_screen() {
    if (!screen_profile_edit) return;

    lv_obj_t* main_container = lv_obj_get_child(screen_profile_edit, 0);
    if (!main_container) return;

    // Создаем массив из переключателей для удобства
    // lv_obj_t* btn_matrices[] = { btnm_primary_uv_mode, btnm_secondary_uv_mode, btnm_tertiary_uv_mode, btnm_post_cooling_purge };
    
    if (current_global_settings.theme == 1) { // --- ТЕМНАЯ ТЕМА ---
        // Удаляем стили светлой темы
        lv_obj_remove_style(main_container, &style_light_bg, 0);
        lv_obj_remove_style(label_edit_name_title, &style_light_text, 0);
        lv_obj_remove_style(ta_edit_profile_name, &style_light_textarea, 0);
        lv_obj_remove_style(header_uv_params, &style_light_column_header, 0);
        lv_obj_remove_style(header_poly_params, &style_light_column_header, 0);
        lv_obj_remove_style(block_uv_all_stages, &style_light_block_border, 0);
        lv_obj_remove_style(block_uv_all_stages, &style_light_block_bg, 0);
        lv_obj_remove_style(block_gases, &style_light_block_border, 0);
        lv_obj_remove_style(block_gases, &style_light_block_bg, 0);
        lv_obj_remove_style(block_thermal, &style_light_block_border, 0);
        lv_obj_remove_style(block_thermal, &style_light_block_bg, 0);
        lv_obj_remove_style(label_uv_primary_title, &style_light_text, 0);
        lv_obj_remove_style(label_primary_uv_mode_status, &style_light_text, 0);
        lv_obj_remove_style(label_uv_secondary_title, &style_light_text, 0);
        lv_obj_remove_style(label_secondary_uv_mode_status, &style_light_text, 0);
        lv_obj_remove_style(label_uv_tertiary_title, &style_light_text, 0);
        lv_obj_remove_style(label_tertiary_uv_mode_status, &style_light_text, 0);
        lv_obj_remove_style(label_edit_flicker_rate_title, &style_light_text, 0);
        lv_obj_remove_style(label_edit_primary_uv_time_title, &style_light_text, 0);
        lv_obj_remove_style(label_edit_secondary_uv_time_title, &style_light_text, 0);
        lv_obj_remove_style(label_edit_tertiary_uv_time_title, &style_light_text, 0);
        lv_obj_remove_style(label_edit_nitrogen_title, &style_light_text, 0);
        lv_obj_remove_style(label_edit_nitrogen_target_title, &style_light_text, 0);
        lv_obj_remove_style(label_edit_nitrogen_boost_title, &style_light_text, 0);
        lv_obj_remove_style(label_edit_cooling_title, &style_light_text, 0);
        lv_obj_remove_style(label_edit_thermal_chamber_title, &style_light_text, 0);
        lv_obj_remove_style(label_edit_thermal_temp_title, &style_light_text, 0);
        lv_obj_remove_style(label_edit_heat_hold_title, &style_light_text, 0);
        lv_obj_remove_style(label_post_cooling_title, &style_light_text, 0);
        lv_obj_remove_style(btn_save_changes, &style_light_btn, 0);
        lv_obj_remove_style(btn_cancel_edit, &style_light_btn, 0);

        // Добавляем стили темной темы
        lv_obj_add_style(main_container, &style_dark_bg, 0);
        lv_obj_add_style(label_edit_name_title, &style_dark_text, 0);
        lv_obj_add_style(ta_edit_profile_name, &style_dark_textarea, 0);
        lv_obj_add_style(header_uv_params, &style_dark_column_header, 0);
        lv_obj_add_style(header_poly_params, &style_dark_column_header, 0);
        lv_obj_add_style(block_uv_all_stages, &style_dark_block_border, 0);
        lv_obj_add_style(block_uv_all_stages, &style_dark_block_bg, 0);
        lv_obj_add_style(block_gases, &style_dark_block_border, 0);
        lv_obj_add_style(block_gases, &style_dark_block_bg, 0);
        lv_obj_add_style(block_thermal, &style_dark_block_border, 0);
        lv_obj_add_style(block_thermal, &style_dark_block_bg, 0);
        lv_obj_add_style(label_uv_primary_title, &style_dark_text, 0);
        lv_obj_add_style(label_primary_uv_mode_status, &style_dark_text, 0);
        lv_obj_add_style(label_uv_secondary_title, &style_dark_text, 0);
        lv_obj_add_style(label_secondary_uv_mode_status, &style_dark_text, 0);
        lv_obj_add_style(label_uv_tertiary_title, &style_dark_text, 0);
        lv_obj_add_style(label_tertiary_uv_mode_status, &style_dark_text, 0);
        lv_obj_add_style(label_edit_flicker_rate_title, &style_dark_text, 0);
        lv_obj_add_style(label_edit_primary_uv_time_title, &style_dark_text, 0);
        lv_obj_add_style(label_edit_secondary_uv_time_title, &style_dark_text, 0);
        lv_obj_add_style(label_edit_tertiary_uv_time_title, &style_dark_text, 0);
        lv_obj_add_style(label_edit_nitrogen_title, &style_dark_text, 0);
        lv_obj_add_style(label_edit_nitrogen_target_title, &style_dark_text, 0);
        lv_obj_add_style(label_edit_nitrogen_boost_title, &style_dark_text, 0);
        lv_obj_add_style(label_edit_cooling_title, &style_dark_text, 0);
        lv_obj_add_style(label_edit_thermal_chamber_title, &style_dark_text, 0);
        lv_obj_add_style(label_edit_thermal_temp_title, &style_dark_text, 0);
        lv_obj_add_style(label_edit_heat_hold_title, &style_dark_text, 0);
        lv_obj_add_style(label_post_cooling_title, &style_dark_text, 0);
        lv_obj_add_style(btn_save_changes, &style_dark_btn, 0);
        lv_obj_add_style(btn_cancel_edit, &style_dark_btn, 0);
        
        lv_obj_add_style(ta_edit_primary_uv, &style_dark_textarea, 0);
        lv_obj_add_style(ta_edit_flicker_rate, &style_dark_textarea, 0);
        lv_obj_add_style(ta_edit_secondary_uv, &style_dark_textarea, 0);
        lv_obj_add_style(ta_edit_tertiary_uv, &style_dark_textarea, 0);
        lv_obj_add_style(ta_edit_nitrogen_target, &style_dark_textarea, 0);
        lv_obj_add_style(ta_edit_nitrogen_boost, &style_dark_textarea, 0);
        lv_obj_add_style(ta_edit_thermal_temp, &style_dark_textarea, 0);
        lv_obj_add_style(ta_edit_heat_hold, &style_dark_textarea, 0);

        // <<< Устанавливаем ТЕМНЫЙ фон для строк с переключателями >>>
        lv_obj_set_style_bg_color(row_uv_primary, lv_color_hex(0x2C2C2C), 0);
        lv_obj_set_style_bg_opa(row_uv_primary, LV_OPA_COVER, 0);
        lv_obj_set_style_bg_color(row_uv_secondary, lv_color_hex(0x2C2C2C), 0);
        lv_obj_set_style_bg_opa(row_uv_secondary, LV_OPA_COVER, 0); 
        lv_obj_set_style_bg_color(row_uv_tertiary, lv_color_hex(0x2C2C2C), 0);
        lv_obj_set_style_bg_opa(row_uv_tertiary, LV_OPA_COVER, 0);

        // --- Стилизуем строки и переключатели ---
        lv_obj_set_style_bg_color(row_uv_primary, lv_color_hex(0x2C2C2C), 0);
        lv_obj_set_style_bg_opa(row_uv_primary, LV_OPA_COVER, 0);
        lv_obj_set_style_bg_color(row_uv_secondary, lv_color_hex(0x2C2C2C), 0);
        lv_obj_set_style_bg_opa(row_uv_secondary, LV_OPA_COVER, 0);
        lv_obj_set_style_bg_color(row_uv_tertiary, lv_color_hex(0x2C2C2C), 0);
        lv_obj_set_style_bg_opa(row_uv_tertiary, LV_OPA_COVER, 0);

        // for (auto btnm : btn_matrices) {
        //     if (!btnm) continue;
        //     // Неактивные кнопки
        //     lv_obj_set_style_bg_color(btnm, lv_color_hex(0x424242), LV_PART_ITEMS);
        //     lv_obj_set_style_text_color(btnm, lv_color_white(), LV_PART_ITEMS);
        //     // Активная кнопка
        //     lv_obj_set_style_bg_color(btnm, lv_color_hex(0xff05b8), LV_PART_ITEMS | LV_STATE_CHECKED);
        //     lv_obj_set_style_text_color(btnm, lv_color_white(), LV_PART_ITEMS | LV_STATE_CHECKED);
        // }

        // НОВЫЙ БЛОК СТИЛИЗАЦИИ КНОПОК (ТЕМНАЯ ТЕМА)
        lv_obj_t* all_btns[] = {
            btn_primary_1, btn_primary_2, btn_primary_3,
            btn_secondary_1, btn_secondary_2, btn_secondary_3,
            btn_tertiary_1, btn_tertiary_2, btn_tertiary_3
        };

        for (int i = 0; i < 9; i++) {
            int btn_type = i % 3; // 0 для кнопки "1", 1 для "2", 2 для "3"
            if (btn_type == 0) {
                lv_obj_set_style_bg_color(all_btns[i], dark_uv_btn1_inactive, LV_STATE_DEFAULT);
                lv_obj_set_style_bg_color(all_btns[i], dark_uv_btn1_active, LV_STATE_CHECKED);
            } else if (btn_type == 1) {
                lv_obj_set_style_bg_color(all_btns[i], dark_uv_btn2_inactive, LV_STATE_DEFAULT);
                lv_obj_set_style_bg_color(all_btns[i], dark_uv_btn2_active, LV_STATE_CHECKED);
            } else {
                lv_obj_set_style_bg_color(all_btns[i], dark_uv_btn3_inactive, LV_STATE_DEFAULT);
                lv_obj_set_style_bg_color(all_btns[i], dark_uv_btn3_active, LV_STATE_CHECKED);
            }
        }

        // --- Стилизуем стандартные переключатели ---
        lv_obj_t* switches[] = { sw_edit_nitrogen, sw_edit_chamber_cooling, sw_edit_thermal_chamber_enable };
        for (auto sw : switches) {
            if (!sw) continue;
            // Убираем стандартные цвета темы
            lv_obj_remove_style(sw, NULL, LV_PART_INDICATOR | LV_STATE_CHECKED);
            
            // Фон для ВЫКЛЮЧЕННОГО состояния
            lv_obj_set_style_bg_color(sw, lv_color_hex(0x333333), LV_PART_MAIN);
            
            // Фон для ВКЛЮЧЕННОГО состояния
            lv_obj_set_style_bg_color(sw, lv_color_hex(0xff05b8), LV_PART_MAIN | LV_STATE_CHECKED);
        }

    } else { // --- СВЕТЛАЯ ТЕМА ---
        // Удаляем стили темной темы
        lv_obj_remove_style(main_container, &style_dark_bg, 0);
        lv_obj_remove_style(label_edit_name_title, &style_dark_text, 0);
        lv_obj_remove_style(ta_edit_profile_name, &style_dark_textarea, 0);
        lv_obj_remove_style(header_uv_params, &style_dark_column_header, 0);
        lv_obj_remove_style(header_poly_params, &style_dark_column_header, 0);
        lv_obj_remove_style(block_uv_all_stages, &style_dark_block_border, 0);
        lv_obj_remove_style(block_uv_all_stages, &style_dark_block_bg, 0);
        lv_obj_remove_style(block_gases, &style_dark_block_border, 0);
        lv_obj_remove_style(block_gases, &style_dark_block_bg, 0);
        lv_obj_remove_style(block_thermal, &style_dark_block_border, 0);
        lv_obj_remove_style(block_thermal, &style_dark_block_bg, 0);
        lv_obj_remove_style(label_uv_primary_title, &style_dark_text, 0);
        lv_obj_remove_style(label_primary_uv_mode_status, &style_dark_text, 0);
        lv_obj_remove_style(label_uv_secondary_title, &style_dark_text, 0);
        lv_obj_remove_style(label_secondary_uv_mode_status, &style_dark_text, 0);
        lv_obj_remove_style(label_uv_tertiary_title, &style_dark_text, 0);
        lv_obj_remove_style(label_tertiary_uv_mode_status, &style_dark_text, 0);
        lv_obj_remove_style(label_edit_flicker_rate_title, &style_dark_text, 0);
        lv_obj_remove_style(label_edit_primary_uv_time_title, &style_dark_text, 0);
        lv_obj_remove_style(label_edit_secondary_uv_time_title, &style_dark_text, 0);
        lv_obj_remove_style(label_edit_tertiary_uv_time_title, &style_dark_text, 0);
        lv_obj_remove_style(label_edit_nitrogen_title, &style_dark_text, 0);
        lv_obj_remove_style(label_edit_nitrogen_target_title, &style_dark_text, 0);
        lv_obj_remove_style(label_edit_nitrogen_boost_title, &style_dark_text, 0);
        lv_obj_remove_style(label_edit_cooling_title, &style_dark_text, 0);
        lv_obj_remove_style(label_edit_thermal_chamber_title, &style_dark_text, 0);
        lv_obj_remove_style(label_edit_thermal_temp_title, &style_dark_text, 0);
        lv_obj_remove_style(label_edit_heat_hold_title, &style_dark_text, 0);
        lv_obj_remove_style(label_post_cooling_title, &style_dark_text, 0);
        lv_obj_remove_style(btn_save_changes, &style_dark_btn, 0);
        lv_obj_remove_style(btn_cancel_edit, &style_dark_btn, 0);

        lv_obj_remove_style(ta_edit_primary_uv, &style_dark_textarea, 0);
        lv_obj_remove_style(ta_edit_flicker_rate, &style_dark_textarea, 0);
        lv_obj_remove_style(ta_edit_secondary_uv, &style_dark_textarea, 0);
        lv_obj_remove_style(ta_edit_tertiary_uv, &style_dark_textarea, 0);
        lv_obj_remove_style(ta_edit_nitrogen_target, &style_dark_textarea, 0);
        lv_obj_remove_style(ta_edit_nitrogen_boost, &style_dark_textarea, 0);
        lv_obj_remove_style(ta_edit_thermal_temp, &style_dark_textarea, 0);
        lv_obj_remove_style(ta_edit_heat_hold, &style_dark_textarea, 0);

        // Добавляем стили светлой темы
        lv_obj_add_style(main_container, &style_light_bg, 0);
        lv_obj_add_style(label_edit_name_title, &style_light_text, 0);
        lv_obj_add_style(ta_edit_profile_name, &style_light_textarea, 0);
        lv_obj_add_style(header_uv_params, &style_light_column_header, 0);
        lv_obj_add_style(header_poly_params, &style_light_column_header, 0);
        lv_obj_add_style(block_uv_all_stages, &style_light_block_border, 0);
        lv_obj_add_style(block_uv_all_stages, &style_light_block_bg, 0);
        lv_obj_add_style(block_gases, &style_light_block_border, 0);
        lv_obj_add_style(block_gases, &style_light_block_bg, 0);
        lv_obj_add_style(block_thermal, &style_light_block_border, 0);
        lv_obj_add_style(block_thermal, &style_light_block_bg, 0);
        lv_obj_add_style(label_uv_primary_title, &style_light_text, 0);
        lv_obj_add_style(label_primary_uv_mode_status, &style_light_text, 0);
        lv_obj_add_style(label_uv_secondary_title, &style_light_text, 0);
        lv_obj_add_style(label_secondary_uv_mode_status, &style_light_text, 0);
        lv_obj_add_style(label_uv_tertiary_title, &style_light_text, 0);
        lv_obj_add_style(label_tertiary_uv_mode_status, &style_light_text, 0);
        lv_obj_add_style(label_edit_flicker_rate_title, &style_light_text, 0);
        lv_obj_add_style(label_edit_primary_uv_time_title, &style_light_text, 0);
        lv_obj_add_style(label_edit_secondary_uv_time_title, &style_light_text, 0);
        lv_obj_add_style(label_edit_tertiary_uv_time_title, &style_light_text, 0);
        lv_obj_add_style(label_edit_nitrogen_title, &style_light_text, 0);
        lv_obj_add_style(label_edit_nitrogen_target_title, &style_light_text, 0);
        lv_obj_add_style(label_edit_nitrogen_boost_title, &style_light_text, 0);
        lv_obj_add_style(label_edit_cooling_title, &style_light_text, 0);
        lv_obj_add_style(label_edit_thermal_chamber_title, &style_light_text, 0);
        lv_obj_add_style(label_edit_thermal_temp_title, &style_light_text, 0);
        lv_obj_add_style(label_edit_heat_hold_title, &style_light_text, 0);
        lv_obj_add_style(label_post_cooling_title, &style_light_text, 0); 
        lv_obj_add_style(btn_save_changes, &style_light_btn, 0);
        lv_obj_add_style(btn_cancel_edit, &style_light_btn, 0);

        lv_obj_add_style(ta_edit_primary_uv, &style_light_textarea, 0);
        lv_obj_add_style(ta_edit_flicker_rate, &style_light_textarea, 0);
        lv_obj_add_style(ta_edit_secondary_uv, &style_light_textarea, 0);
        lv_obj_add_style(ta_edit_tertiary_uv, &style_light_textarea, 0);
        lv_obj_add_style(ta_edit_nitrogen_target, &style_light_textarea, 0);
        lv_obj_add_style(ta_edit_nitrogen_boost, &style_light_textarea, 0);
        lv_obj_add_style(ta_edit_thermal_temp, &style_light_textarea, 0);
        lv_obj_add_style(ta_edit_heat_hold, &style_light_textarea, 0);

        // <<< Устанавливаем СВЕТЛЫЙ фон для строк с переключателями >>>
        lv_obj_set_style_bg_color(row_uv_primary, lv_color_hex(0xE0E0E0), 0);
        lv_obj_set_style_bg_opa(row_uv_primary, LV_OPA_COVER, 0);
        lv_obj_set_style_bg_color(row_uv_secondary, lv_color_hex(0xE0E0E0), 0);
        lv_obj_set_style_bg_opa(row_uv_secondary, LV_OPA_COVER, 0);
        lv_obj_set_style_bg_color(row_uv_tertiary, lv_color_hex(0xE0E0E0), 0);
        lv_obj_set_style_bg_opa(row_uv_tertiary, LV_OPA_COVER, 0);

        // for (auto btnm : btn_matrices) {
        //     if (!btnm) continue;
        //     // Неактивные кнопки
        //     lv_obj_set_style_bg_color(btnm, lv_color_hex(0xE0E0E0), LV_PART_ITEMS);
        //     lv_obj_set_style_text_color(btnm, lv_color_black(), LV_PART_ITEMS);
        //     // Активная кнопка
        //     lv_obj_set_style_bg_color(btnm, lv_palette_main(LV_PALETTE_BLUE), LV_PART_ITEMS | LV_STATE_CHECKED);
        //     lv_obj_set_style_text_color(btnm, lv_color_white(), LV_PART_ITEMS | LV_STATE_CHECKED);
        // }

        // НОВЫЙ БЛОК СТИЛИЗАЦИИ КНОПОК (СВЕТЛАЯ ТЕМА)
        lv_obj_t* all_btns[] = {
            btn_primary_1, btn_primary_2, btn_primary_3,
            btn_secondary_1, btn_secondary_2, btn_secondary_3,
            btn_tertiary_1, btn_tertiary_2, btn_tertiary_3
        };

        for (int i = 0; i < 9; i++) {
            int btn_type = i % 3; // 0 для кнопки "1", 1 для "2", 2 для "3"
            if (btn_type == 0) {
                lv_obj_set_style_bg_color(all_btns[i], light_uv_btn1_inactive, LV_STATE_DEFAULT);
                lv_obj_set_style_bg_color(all_btns[i], light_uv_btn1_active, LV_STATE_CHECKED);
            } else if (btn_type == 1) {
                lv_obj_set_style_bg_color(all_btns[i], light_uv_btn2_inactive, LV_STATE_DEFAULT);
                lv_obj_set_style_bg_color(all_btns[i], light_uv_btn2_active, LV_STATE_CHECKED);
            } else {
                lv_obj_set_style_bg_color(all_btns[i], light_uv_btn3_inactive, LV_STATE_DEFAULT);
                lv_obj_set_style_bg_color(all_btns[i], light_uv_btn3_active, LV_STATE_CHECKED);
            }
        }

        // --- Сбрасываем стили переключателей к стандартным ---
        lv_obj_t* switches[] = { sw_edit_nitrogen, sw_edit_chamber_cooling, sw_edit_thermal_chamber_enable };
        for (auto sw : switches) {
            if (!sw) continue;
            // Убираем все наши кастомные стили, чтобы вернулись стандартные
            lv_obj_set_style_bg_color(sw, lv_color_hex(0xF0F0F0), LV_PART_MAIN); // Фон для ВЫКЛЮЧЕННОГО состояния
            lv_obj_set_style_bg_color(sw, lv_palette_main(LV_PALETTE_BLUE), LV_PART_MAIN | LV_STATE_CHECKED);
        }
    }
}
void apply_theme_to_profile_details_screen() {
    if (!screen_profile_details) return;

    // Находим все нужные объекты
    lv_obj_t* content_container = lv_obj_get_child(screen_profile_details, 0);
    if (!content_container) return;
    
    // Получаем кнопки через их дочерние метки
    lv_obj_t* btn_start = lv_obj_get_parent(label_detail_btn_start);
    lv_obj_t* btn_edit = lv_obj_get_parent(label_detail_btn_edit);
    lv_obj_t* btn_close = lv_obj_get_parent(label_detail_btn_close);

    if (current_global_settings.theme == 1) { // --- ТЕМНАЯ ТЕМА ---
        // Удаляем стили светлой темы
        lv_obj_remove_style(content_container, &style_light_bg, 0);
        lv_obj_remove_style(label_detail_header, &style_light_subheader, 0);
        lv_obj_remove_style(label_detail_view_profile_name, &style_light_text, 0);
        lv_obj_remove_style(label_detail_view_id, &style_light_text, 0);
        lv_obj_remove_style(label_detail_view_thermal_chamber_enabled, &style_light_text, 0);
        lv_obj_remove_style(label_detail_view_thermal_chamber, &style_light_text, 0);
        lv_obj_remove_style(label_detail_view_chamber_cooling, &style_light_text, 0);
        lv_obj_remove_style(label_detail_view_nitrogen, &style_light_text, 0);
        lv_obj_remove_style(label_detail_view_primary_uv, &style_light_text, 0);
        lv_obj_remove_style(label_detail_view_secondary_uv, &style_light_text, 0);
        lv_obj_remove_style(label_detail_view_tertiary_uv, &style_light_text, 0);
        lv_obj_remove_style(btn_start, &style_light_btn, 0);
        lv_obj_remove_style(btn_edit, &style_light_btn, 0);
        lv_obj_remove_style(btn_close, &style_light_btn, 0);

        // Добавляем стили темной темы
        lv_obj_add_style(content_container, &style_dark_bg, 0);
        lv_obj_add_style(label_detail_header, &style_dark_subheader, 0);
        lv_obj_add_style(label_detail_view_profile_name, &style_dark_text, 0);
        lv_obj_add_style(label_detail_view_id, &style_dark_text, 0);
        lv_obj_add_style(label_detail_view_thermal_chamber_enabled, &style_dark_text, 0);
        lv_obj_add_style(label_detail_view_thermal_chamber, &style_dark_text, 0);
        lv_obj_add_style(label_detail_view_chamber_cooling, &style_dark_text, 0);
        lv_obj_add_style(label_detail_view_nitrogen, &style_dark_text, 0);
        lv_obj_add_style(label_detail_view_primary_uv, &style_dark_text, 0);
        lv_obj_add_style(label_detail_view_secondary_uv, &style_dark_text, 0);
        lv_obj_add_style(label_detail_view_tertiary_uv, &style_dark_text, 0);
        lv_obj_add_style(btn_start, &style_dark_btn, 0);
        lv_obj_add_style(btn_edit, &style_dark_btn, 0);
        lv_obj_add_style(btn_close, &style_dark_btn, 0);

    } else { // --- СВЕТЛАЯ ТЕМА ---
        // Удаляем стили темной темы
        lv_obj_remove_style(content_container, &style_dark_bg, 0);
        lv_obj_remove_style(label_detail_header, &style_dark_subheader, 0);
        lv_obj_remove_style(label_detail_view_profile_name, &style_dark_text, 0);
        lv_obj_remove_style(label_detail_view_id, &style_dark_text, 0);
        lv_obj_remove_style(label_detail_view_thermal_chamber_enabled, &style_dark_text, 0);
        lv_obj_remove_style(label_detail_view_thermal_chamber, &style_dark_text, 0);
        lv_obj_remove_style(label_detail_view_chamber_cooling, &style_dark_text, 0);
        lv_obj_remove_style(label_detail_view_nitrogen, &style_dark_text, 0);
        lv_obj_remove_style(label_detail_view_primary_uv, &style_dark_text, 0);
        lv_obj_remove_style(label_detail_view_secondary_uv, &style_dark_text, 0);
        lv_obj_remove_style(label_detail_view_tertiary_uv, &style_dark_text, 0);
        lv_obj_remove_style(btn_start, &style_dark_btn, 0);
        lv_obj_remove_style(btn_edit, &style_dark_btn, 0);
        lv_obj_remove_style(btn_close, &style_dark_btn, 0);

        // Добавляем стили светлой темы
        lv_obj_add_style(content_container, &style_light_bg, 0);
        lv_obj_add_style(label_detail_header, &style_light_subheader, 0);
        lv_obj_add_style(label_detail_view_profile_name, &style_light_text, 0);
        lv_obj_add_style(label_detail_view_id, &style_light_text, 0);
        lv_obj_add_style(label_detail_view_thermal_chamber_enabled, &style_light_text, 0);
        lv_obj_add_style(label_detail_view_thermal_chamber, &style_light_text, 0);
        lv_obj_add_style(label_detail_view_chamber_cooling, &style_light_text, 0);
        lv_obj_add_style(label_detail_view_nitrogen, &style_light_text, 0);
        lv_obj_add_style(label_detail_view_primary_uv, &style_light_text, 0);
        lv_obj_add_style(label_detail_view_secondary_uv, &style_light_text, 0);
        lv_obj_add_style(label_detail_view_tertiary_uv, &style_light_text, 0);
        lv_obj_add_style(btn_start, &style_light_btn, 0);
        lv_obj_add_style(btn_edit, &style_light_btn, 0);
        lv_obj_add_style(btn_close, &style_light_btn, 0);
    }
}
void apply_theme_to_main_app_screen() {
    if (!screen_main_app || !main_screen_content_container) return;

    // --- Часть 1: Стилизация основных элементов экрана (фон, заголовок, кнопки) ---
    // Эта часть очень похожа на то, что у тебя было, и это правильно.
    // Мы явно удаляем стили одной темы и добавляем стили другой.

    if (current_global_settings.theme == 1) { // --- ТЕМНАЯ ТЕМА ---
        // Удаляем стили светлой темы с основных элементов
        lv_obj_remove_style(main_screen_content_container, &style_light_bg, 0);
        lv_obj_remove_style(list_header_label_main, &style_light_text, 0);
        lv_obj_remove_style(lv_obj_get_child(btn_profiles_prev, 0), &style_light_arrow, 0);
        lv_obj_remove_style(lv_obj_get_child(btn_profiles_next, 0), &style_light_arrow, 0);
        lv_obj_remove_style(btn_add_main, &style_light_btn, 0);
        lv_obj_remove_style(btn_lab_main, &style_light_btn, 0);
        lv_obj_remove_style(btn_settings_main, &style_light_btn, 0);
        
        // Добавляем стили темной темы к основным элементам
        lv_obj_add_style(main_screen_content_container, &style_dark_bg, 0);
        lv_obj_add_style(list_header_label_main, &style_dark_text, 0);
        lv_obj_add_style(lv_obj_get_child(btn_profiles_prev, 0), &style_dark_arrow, 0);
        lv_obj_add_style(lv_obj_get_child(btn_profiles_next, 0), &style_dark_arrow, 0);
        lv_obj_add_style(btn_add_main, &style_dark_btn, 0);
        lv_obj_add_style(btn_lab_main, &style_dark_btn, 0);
        lv_obj_add_style(btn_settings_main, &style_dark_btn, 0);

    } else { // --- СВЕТЛАЯ ТЕМА ---
        // Удаляем стили темной темы с основных элементов
        lv_obj_remove_style(main_screen_content_container, &style_dark_bg, 0);
        lv_obj_remove_style(list_header_label_main, &style_dark_text, 0);
        lv_obj_remove_style(lv_obj_get_child(btn_profiles_prev, 0), &style_dark_arrow, 0);
        lv_obj_remove_style(lv_obj_get_child(btn_profiles_next, 0), &style_dark_arrow, 0);
        lv_obj_remove_style(btn_add_main, &style_dark_btn, 0);
        lv_obj_remove_style(btn_lab_main, &style_dark_btn, 0);
        lv_obj_remove_style(btn_settings_main, &style_dark_btn, 0);
        
        // Добавляем стили светлой темы к основным элементам
        lv_obj_add_style(main_screen_content_container, &style_light_bg, 0);
        lv_obj_add_style(list_header_label_main, &style_light_text, 0);
        lv_obj_add_style(lv_obj_get_child(btn_profiles_prev, 0), &style_light_arrow, 0);
        lv_obj_add_style(lv_obj_get_child(btn_profiles_next, 0), &style_light_arrow, 0);
        lv_obj_add_style(btn_add_main, &style_light_btn, 0);
        lv_obj_add_style(btn_lab_main, &style_light_btn, 0);
        lv_obj_add_style(btn_settings_main, &style_light_btn, 0);
    }

    // --- Часть 2: Стилизация плиток из "бассейна объектов" ---
    // Это новый и самый важный блок. Он гарантирует, что стили плиток
    // не накапливаются, а полностью переопределяются при смене темы.
    for (int i = 0; i < PROFILES_PER_PAGE; i++) {
        // Получаем указатели на элементы i-ой плитки для удобства
        lv_obj_t* tile = profile_tiles[i];
        lv_obj_t* icon = profile_tile_icons[i];
        lv_obj_t* label = profile_tile_labels[i];

        // Проверяем, что указатели валидны, на всякий случай
        if (!tile || !icon || !label) continue;
        
        if (current_global_settings.theme == 1) { // --- ТЕМНАЯ ТЕМА для плиток ---
            // Сначала удаляем стили светлой темы, если они были
            lv_obj_remove_style(tile, &style_light_tile_bg, 0);
            lv_obj_remove_style(tile, &style_light_tile_border, 0);
            lv_obj_remove_style(icon, &style_light_text, 0);
            lv_obj_remove_style(label, &style_light_text, 0);
            
            // Теперь добавляем стили темной темы
            lv_obj_add_style(tile, &style_dark_tile_bg, 0);
            lv_obj_add_style(tile, &style_dark_block_border, 0); // Используем общий стиль рамки для блоков
            lv_obj_add_style(icon, &style_dark_text, 0);
            lv_obj_add_style(label, &style_dark_text, 0);

        } else { // --- СВЕТЛАЯ ТЕМА для плиток ---
            // Сначала удаляем стили темной темы, если они были
            lv_obj_remove_style(tile, &style_dark_tile_bg, 0);
            lv_obj_remove_style(tile, &style_dark_block_border, 0);
            lv_obj_remove_style(icon, &style_dark_text, 0);
            lv_obj_remove_style(label, &style_dark_text, 0);
            
            // Теперь добавляем стили светлой темы
            lv_obj_add_style(tile, &style_light_tile_bg, 0);
            lv_obj_add_style(tile, &style_light_tile_border, 0);
            lv_obj_add_style(icon, &style_light_text, 0);
            lv_obj_add_style(label, &style_light_text, 0);
        }
    }
}
void apply_theme_to_laboratory_screen() {
    if (!screen_laboratory) return;

    // Находим все нужные объекты
    lv_obj_t* content_container = lv_obj_get_child(screen_laboratory, 0);
    if (!content_container) return;
    lv_obj_t* tiles_container = lv_obj_get_child(content_container, 1);
    
    if (current_global_settings.theme == 1) { // --- ТЕМНАЯ ТЕМА ---
        // Удаляем стили светлой темы
        lv_obj_remove_style(content_container, &style_light_bg, 0);
        lv_obj_remove_style(label_lab_header, &style_light_text, 0);
        lv_obj_remove_style(btn_lab_back, &style_light_btn, 0);
        for (uint32_t i = 0; i < lv_obj_get_child_cnt(tiles_container); i++) {
            lv_obj_t* tile = lv_obj_get_child(tiles_container, i);
            lv_obj_remove_style(tile, &style_light_tile_bg, 0);
            lv_obj_remove_style(lv_obj_get_child(tile, 0), &style_light_text, 0);
        }

        // Добавляем стили темной темы
        lv_obj_add_style(content_container, &style_dark_bg, 0);
        lv_obj_add_style(label_lab_header, &style_dark_text, 0);
        lv_obj_add_style(btn_lab_back, &style_dark_btn, 0);
        for (uint32_t i = 0; i < lv_obj_get_child_cnt(tiles_container); i++) {
            lv_obj_t* tile = lv_obj_get_child(tiles_container, i);
            lv_obj_add_style(tile, &style_dark_tile_bg, 0);
            lv_obj_add_style(lv_obj_get_child(tile, 0), &style_dark_text, 0);
        }

    } else { // --- СВЕТЛАЯ ТЕМА ---
        // Удаляем стили темной темы
        lv_obj_remove_style(content_container, &style_dark_bg, 0);
        lv_obj_remove_style(label_lab_header, &style_dark_text, 0);
        lv_obj_remove_style(btn_lab_back, &style_dark_btn, 0);
        for (uint32_t i = 0; i < lv_obj_get_child_cnt(tiles_container); i++) {
            lv_obj_t* tile = lv_obj_get_child(tiles_container, i);
            lv_obj_remove_style(tile, &style_dark_tile_bg, 0);
            lv_obj_remove_style(lv_obj_get_child(tile, 0), &style_dark_text, 0);
        }
        
        // Добавляем стили светлой темы
        lv_obj_add_style(content_container, &style_light_bg, 0);
        lv_obj_add_style(label_lab_header, &style_light_text, 0);
        lv_obj_add_style(btn_lab_back, &style_light_btn, 0);
        for (uint32_t i = 0; i < lv_obj_get_child_cnt(tiles_container); i++) {
            lv_obj_t* tile = lv_obj_get_child(tiles_container, i);
            lv_obj_add_style(tile, &style_light_tile_bg, 0);
            lv_obj_add_style(lv_obj_get_child(tile, 0), &style_light_text, 0);
        }
    }
}
void apply_theme_to_lab_lighten_screen() {
    if (!screen_lab_lighten) return;

    // Находим все нужные объекты
    lv_obj_t* main_container = lv_obj_get_child(screen_lab_lighten, 0);
    if (!main_container) return;
    lv_obj_t* content_area = lv_obj_get_child(main_container, 1);
    lv_obj_t* block_thermo = lv_obj_get_child(content_area, 0);
    lv_obj_t* thermo_header = lv_obj_get_child(block_thermo, 0);
    
    if (current_global_settings.theme == 1) { // --- ТЕМНАЯ ТЕМА ---
        // Удаляем стили светлой темы
        lv_obj_remove_style(main_container, &style_light_bg, 0);
        lv_obj_remove_style(label_lighten_header, &style_light_text, 0);
        lv_obj_remove_style(block_thermo, &style_light_block_border, 0);
        lv_obj_remove_style(block_thermo, &style_light_block_bg, 0);
        lv_obj_remove_style(thermo_header, &style_light_block_header, 0);
        lv_obj_remove_style(label_lighten_temp_fixed, &style_light_text, 0);
        lv_obj_remove_style(label_lighten_hold_time, &style_light_text, 0);
        lv_obj_remove_style(label_lighten_cooling, &style_light_text, 0);
        lv_obj_remove_style(btn_lighten_start, &style_light_btn, 0);
        lv_obj_remove_style(btn_lighten_back, &style_light_btn, 0);

        // Добавляем стили темной темы
        lv_obj_add_style(main_container, &style_dark_bg, 0);
        lv_obj_add_style(label_lighten_header, &style_dark_text, 0);
        lv_obj_add_style(block_thermo, &style_dark_block_border, 0);
        lv_obj_add_style(block_thermo, &style_dark_block_bg, 0);
        lv_obj_add_style(thermo_header, &style_dark_block_header, 0);
        lv_obj_add_style(label_lighten_temp_fixed, &style_dark_text, 0);
        lv_obj_add_style(label_lighten_hold_time, &style_dark_text, 0);
        lv_obj_add_style(label_lighten_cooling, &style_dark_text, 0);
        lv_obj_add_style(btn_lighten_start, &style_dark_btn, 0);
        lv_obj_add_style(btn_lighten_back, &style_dark_btn, 0);

    } else { // --- СВЕТЛАЯ ТЕМА ---
        // Удаляем стили темной темы
        lv_obj_remove_style(main_container, &style_dark_bg, 0);
        lv_obj_remove_style(label_lighten_header, &style_dark_text, 0);
        lv_obj_remove_style(block_thermo, &style_dark_block_border, 0);
        lv_obj_remove_style(block_thermo, &style_dark_block_bg, 0);
        lv_obj_remove_style(thermo_header, &style_dark_block_header, 0);
        lv_obj_remove_style(label_lighten_temp_fixed, &style_dark_text, 0);
        lv_obj_remove_style(label_lighten_hold_time, &style_dark_text, 0);
        lv_obj_remove_style(label_lighten_cooling, &style_dark_text, 0);
        lv_obj_remove_style(btn_lighten_start, &style_dark_btn, 0);
        lv_obj_remove_style(btn_lighten_back, &style_dark_btn, 0);

        // Добавляем стили светлой темы
        lv_obj_add_style(main_container, &style_light_bg, 0);
        lv_obj_add_style(label_lighten_header, &style_light_text, 0);
        lv_obj_add_style(block_thermo, &style_light_block_border, 0);
        lv_obj_add_style(block_thermo, &style_light_block_bg, 0);
        lv_obj_add_style(thermo_header, &style_light_block_header, 0);
        lv_obj_add_style(label_lighten_temp_fixed, &style_light_text, 0);
        lv_obj_add_style(label_lighten_hold_time, &style_light_text, 0);
        lv_obj_add_style(label_lighten_cooling, &style_light_text, 0);
        lv_obj_add_style(btn_lighten_start, &style_light_btn, 0);
        lv_obj_add_style(btn_lighten_back, &style_light_btn, 0);
    }
}
void apply_theme_to_lab_darken_screen() {
    if (!screen_lab_darken) return;

    // Находим все нужные объекты
    lv_obj_t* main_container = lv_obj_get_child(screen_lab_darken, 0);
    if (!main_container) return;
    lv_obj_t* content_area = lv_obj_get_child(main_container, 1);
    lv_obj_t* block_params = lv_obj_get_child(content_area, 0);
    lv_obj_t* params_header = lv_obj_get_child(block_params, 0);
    
    if (current_global_settings.theme == 1) { // --- ТЕМНАЯ ТЕМА ---
        // Удаляем стили светлой темы
        lv_obj_remove_style(main_container, &style_light_bg, 0);
        lv_obj_remove_style(label_darken_header, &style_light_text, 0);
        lv_obj_remove_style(block_params, &style_light_block_border, 0);
        lv_obj_remove_style(block_params, &style_light_block_bg, 0);
        lv_obj_remove_style(params_header, &style_light_block_header, 0);
        lv_obj_remove_style(label_darken_uv_exposure, &style_light_text, 0);
        lv_obj_remove_style(label_darken_cooling, &style_light_text, 0);
        lv_obj_remove_style(btn_darken_start, &style_light_btn, 0);
        lv_obj_remove_style(btn_darken_back, &style_light_btn, 0);

        // Добавляем стили темной темы
        lv_obj_add_style(main_container, &style_dark_bg, 0);
        lv_obj_add_style(label_darken_header, &style_dark_text, 0);
        lv_obj_add_style(block_params, &style_dark_block_border, 0);
        lv_obj_add_style(block_params, &style_dark_block_bg, 0);
        lv_obj_add_style(params_header, &style_dark_block_header, 0);
        lv_obj_add_style(label_darken_uv_exposure, &style_dark_text, 0);
        lv_obj_add_style(label_darken_cooling, &style_dark_text, 0);
        lv_obj_add_style(btn_darken_start, &style_dark_btn, 0);
        lv_obj_add_style(btn_darken_back, &style_dark_btn, 0);

    } else { // --- СВЕТЛАЯ ТЕМА ---
        // Удаляем стили темной темы
        lv_obj_remove_style(main_container, &style_dark_bg, 0);
        lv_obj_remove_style(label_darken_header, &style_dark_text, 0);
        lv_obj_remove_style(block_params, &style_dark_block_border, 0);
        lv_obj_remove_style(block_params, &style_dark_block_bg, 0);
        lv_obj_remove_style(params_header, &style_dark_block_header, 0);
        lv_obj_remove_style(label_darken_uv_exposure, &style_dark_text, 0);
        lv_obj_remove_style(label_darken_cooling, &style_dark_text, 0);
        lv_obj_remove_style(btn_darken_start, &style_dark_btn, 0);
        lv_obj_remove_style(btn_darken_back, &style_dark_btn, 0);

        // Добавляем стили светлой темы
        lv_obj_add_style(main_container, &style_light_bg, 0);
        lv_obj_add_style(label_darken_header, &style_light_text, 0);
        lv_obj_add_style(block_params, &style_light_block_border, 0);
        lv_obj_add_style(block_params, &style_light_block_bg, 0);
        lv_obj_add_style(params_header, &style_light_block_header, 0);
        lv_obj_add_style(label_darken_uv_exposure, &style_light_text, 0);
        lv_obj_add_style(label_darken_cooling, &style_light_text, 0);
        lv_obj_add_style(btn_darken_start, &style_light_btn, 0);
        lv_obj_add_style(btn_darken_back, &style_light_btn, 0);
    }
}
void apply_theme_to_lab_strength_screen() {
    if (!screen_lab_strength) return;

    // Находим все нужные объекты
    lv_obj_t* main_container = lv_obj_get_child(screen_lab_strength, 0);
    if (!main_container) return;
    lv_obj_t* content_area = lv_obj_get_child(main_container, 1);
    lv_obj_t* left_col = lv_obj_get_child(content_area, 0);
    lv_obj_t* right_col = lv_obj_get_child(content_area, 1);
    lv_obj_t* thermo_header = lv_obj_get_child(left_col, 0);
    lv_obj_t* uv_header = lv_obj_get_child(right_col, 0);

    if (current_global_settings.theme == 1) { // --- ТЕМНАЯ ТЕМА ---
        // Удаляем стили светлой темы
        lv_obj_remove_style(main_container, &style_light_bg, 0);
        lv_obj_remove_style(label_strength_header, &style_light_text, 0);
        lv_obj_remove_style(left_col, &style_light_block_border, 0);
        lv_obj_remove_style(left_col, &style_light_block_bg, 0);
        lv_obj_remove_style(right_col, &style_light_block_border, 0);
        lv_obj_remove_style(right_col, &style_light_block_bg, 0);
        lv_obj_remove_style(thermo_header, &style_light_block_header, 0);
        lv_obj_remove_style(uv_header, &style_light_block_header, 0);
        lv_obj_remove_style(label_strength_temp, &style_light_text, 0);
        lv_obj_remove_style(label_strength_hold_time, &style_light_text, 0);
        lv_obj_remove_style(label_strength_cooling, &style_light_text, 0);
        lv_obj_remove_style(label_strength_uv_pulse_duration, &style_light_text, 0);
        lv_obj_remove_style(label_strength_uv_pulse_interval, &style_light_text, 0);
        lv_obj_remove_style(label_strength_info_uv_text, &style_light_text, 0);
        lv_obj_remove_style(btn_strength_start, &style_light_btn, 0);
        lv_obj_remove_style(btn_strength_back, &style_light_btn, 0);

        // Добавляем стили темной темы
        lv_obj_add_style(main_container, &style_dark_bg, 0);
        lv_obj_add_style(label_strength_header, &style_dark_text, 0);
        lv_obj_add_style(left_col, &style_dark_block_border, 0);
        lv_obj_add_style(left_col, &style_dark_block_bg, 0);
        lv_obj_add_style(right_col, &style_dark_block_border, 0);
        lv_obj_add_style(right_col, &style_dark_block_bg, 0);
        lv_obj_add_style(thermo_header, &style_dark_block_header, 0);
        lv_obj_add_style(uv_header, &style_dark_block_header, 0);
        lv_obj_add_style(label_strength_temp, &style_dark_text, 0);
        lv_obj_add_style(label_strength_hold_time, &style_dark_text, 0);
        lv_obj_add_style(label_strength_cooling, &style_dark_text, 0);
        lv_obj_add_style(label_strength_uv_pulse_duration, &style_dark_text, 0);
        lv_obj_add_style(label_strength_uv_pulse_interval, &style_dark_text, 0);
        lv_obj_add_style(label_strength_info_uv_text, &style_dark_text, 0);
        lv_obj_add_style(btn_strength_start, &style_dark_btn, 0);
        lv_obj_add_style(btn_strength_back, &style_dark_btn, 0);

    } else { // --- СВЕТЛАЯ ТЕМА ---
        // Удаляем стили темной темы
        lv_obj_remove_style(main_container, &style_dark_bg, 0);
        lv_obj_remove_style(label_strength_header, &style_dark_text, 0);
        lv_obj_remove_style(left_col, &style_dark_block_border, 0);
        lv_obj_remove_style(left_col, &style_dark_block_bg, 0);
        lv_obj_remove_style(right_col, &style_dark_block_border, 0);
        lv_obj_remove_style(right_col, &style_dark_block_bg, 0);
        lv_obj_remove_style(thermo_header, &style_dark_block_header, 0);
        lv_obj_remove_style(uv_header, &style_dark_block_header, 0);
        lv_obj_remove_style(label_strength_temp, &style_dark_text, 0);
        lv_obj_remove_style(label_strength_hold_time, &style_dark_text, 0);
        lv_obj_remove_style(label_strength_cooling, &style_dark_text, 0);
        lv_obj_remove_style(label_strength_uv_pulse_duration, &style_dark_text, 0);
        lv_obj_remove_style(label_strength_uv_pulse_interval, &style_dark_text, 0);
        lv_obj_remove_style(label_strength_info_uv_text, &style_dark_text, 0);
        lv_obj_remove_style(btn_strength_start, &style_dark_btn, 0);
        lv_obj_remove_style(btn_strength_back, &style_dark_btn, 0);

        // Добавляем стили светлой темы
        lv_obj_add_style(main_container, &style_light_bg, 0);
        lv_obj_add_style(label_strength_header, &style_light_text, 0);
        lv_obj_add_style(left_col, &style_light_block_border, 0);
        lv_obj_add_style(left_col, &style_light_block_bg, 0);
        lv_obj_add_style(right_col, &style_light_block_border, 0);
        lv_obj_add_style(right_col, &style_light_block_bg, 0);
        lv_obj_add_style(thermo_header, &style_light_block_header, 0);
        lv_obj_add_style(uv_header, &style_light_block_header, 0);
        lv_obj_add_style(label_strength_temp, &style_light_text, 0);
        lv_obj_add_style(label_strength_hold_time, &style_light_text, 0);
        lv_obj_add_style(label_strength_cooling, &style_light_text, 0);
        lv_obj_add_style(label_strength_uv_pulse_duration, &style_light_text, 0);
        lv_obj_add_style(label_strength_uv_pulse_interval, &style_light_text, 0);
        lv_obj_add_style(label_strength_info_uv_text, &style_light_text, 0);
        lv_obj_add_style(btn_strength_start, &style_light_btn, 0);
        lv_obj_add_style(btn_strength_back, &style_light_btn, 0);
    }
}
void apply_theme_to_lab_thermal_screen() {
    if (!screen_lab_thermal) return;

    // Находим все нужные объекты по их указателям
    lv_obj_t* main_container = lv_obj_get_child(screen_lab_thermal, 0);
    if (!main_container) return;
    lv_obj_t* content_area = lv_obj_get_child(main_container, 1);
    lv_obj_t* block_thermo = lv_obj_get_child(content_area, 0);
    lv_obj_t* thermo_header = lv_obj_get_child(block_thermo, 0);
    
    if (current_global_settings.theme == 1) { // --- ТЕМНАЯ ТЕМА ---
        // Удаляем стили светлой темы
        lv_obj_remove_style(main_container, &style_light_bg, 0);
        lv_obj_remove_style(label_thermal_header, &style_light_text, 0);
        lv_obj_remove_style(block_thermo, &style_light_block_border, 0);
        lv_obj_remove_style(block_thermo, &style_light_block_bg, 0);
        lv_obj_remove_style(thermo_header, &style_light_block_header, 0);
        lv_obj_remove_style(label_thermal_temp, &style_light_text, 0);
        lv_obj_remove_style(label_thermal_info, &style_light_text, 0);
        lv_obj_remove_style(label_thermal_hold_time, &style_light_text, 0);
        lv_obj_remove_style(btn_thermal_start, &style_light_btn, 0);
        lv_obj_remove_style(btn_thermal_back, &style_light_btn, 0);

        // Добавляем стили темной темы
        lv_obj_add_style(main_container, &style_dark_bg, 0);
        lv_obj_add_style(label_thermal_header, &style_dark_text, 0);
        lv_obj_add_style(block_thermo, &style_dark_block_border, 0);
        lv_obj_add_style(block_thermo, &style_dark_block_bg, 0);
        lv_obj_add_style(thermo_header, &style_dark_block_header, 0);
        lv_obj_add_style(label_thermal_temp, &style_dark_text, 0);
        lv_obj_add_style(label_thermal_info, &style_dark_text, 0);
        lv_obj_add_style(label_thermal_hold_time, &style_dark_text, 0);
        lv_obj_add_style(btn_thermal_start, &style_dark_btn, 0);
        lv_obj_add_style(btn_thermal_back, &style_dark_btn, 0);

    } else { // --- СВЕТЛАЯ ТЕМА ---
        // Удаляем стили темной темы
        lv_obj_remove_style(main_container, &style_dark_bg, 0);
        lv_obj_remove_style(label_thermal_header, &style_dark_text, 0);
        lv_obj_remove_style(block_thermo, &style_dark_block_border, 0);
        lv_obj_remove_style(block_thermo, &style_dark_block_bg, 0);
        lv_obj_remove_style(thermo_header, &style_dark_block_header, 0);
        lv_obj_remove_style(label_thermal_temp, &style_dark_text, 0);
        lv_obj_remove_style(label_thermal_info, &style_dark_text, 0);
        lv_obj_remove_style(label_thermal_hold_time, &style_dark_text, 0);
        lv_obj_remove_style(btn_thermal_start, &style_dark_btn, 0);
        lv_obj_remove_style(btn_thermal_back, &style_dark_btn, 0);

        // Добавляем стили светлой темы
        lv_obj_add_style(main_container, &style_light_bg, 0);
        lv_obj_add_style(label_thermal_header, &style_light_text, 0);
        lv_obj_add_style(block_thermo, &style_light_block_border, 0);
        lv_obj_add_style(block_thermo, &style_light_block_bg, 0);
        lv_obj_add_style(thermo_header, &style_light_block_header, 0);
        lv_obj_add_style(label_thermal_temp, &style_light_text, 0);
        lv_obj_add_style(label_thermal_info, &style_light_text, 0);
        lv_obj_add_style(label_thermal_hold_time, &style_light_text, 0);
        lv_obj_add_style(btn_thermal_start, &style_light_btn, 0);
        lv_obj_add_style(btn_thermal_back, &style_light_btn, 0);
    }
}
void apply_theme_to_lab_glaze_screen() {
    if (!screen_lab_glaze) return;

    // Находим все нужные объекты по их указателям
    lv_obj_t* main_container = lv_obj_get_child(screen_lab_glaze, 0);
    if (!main_container) return;
    lv_obj_t* content_area = lv_obj_get_child(main_container, 1);
    lv_obj_t* left_col = lv_obj_get_child(content_area, 0);
    lv_obj_t* right_col = lv_obj_get_child(content_area, 1);
    lv_obj_t* block_flicker = lv_obj_get_child(left_col, 0);
    lv_obj_t* block_timers = lv_obj_get_child(left_col, 1);
    lv_obj_t* block_aux = lv_obj_get_child(right_col, 0);

    if (current_global_settings.theme == 1) { // --- ТЕМНАЯ ТЕМА ---
        // Удаляем стили светлой темы
        lv_obj_remove_style(main_container, &style_light_bg, 0);
        lv_obj_remove_style(label_glaze_header, &style_light_text, 0);
        lv_obj_remove_style(block_flicker, &style_light_block_border, 0);
        lv_obj_remove_style(block_flicker, &style_light_block_bg, 0);
        lv_obj_remove_style(block_timers, &style_light_block_border, 0);
        lv_obj_remove_style(block_timers, &style_light_block_bg, 0);
        lv_obj_remove_style(block_aux, &style_light_block_border, 0);
        lv_obj_remove_style(block_aux, &style_light_block_bg, 0);
        lv_obj_remove_style(label_glaze_title_flicker, &style_light_block_header, 0);
        lv_obj_remove_style(label_glaze_title_timers, &style_light_block_header, 0);
        lv_obj_remove_style(label_glaze_title_aux, &style_light_block_header, 0);
        lv_obj_remove_style(label_glaze_uv_on, &style_light_text, 0);
        lv_obj_remove_style(label_glaze_uv_off, &style_light_text, 0);
        lv_obj_remove_style(label_glaze_monomer_blow, &style_light_text, 0);
        lv_obj_remove_style(label_glaze_uv_exposure, &style_light_text, 0);
        lv_obj_remove_style(label_glaze_cooling, &style_light_text, 0);
        lv_obj_remove_style(label_glaze_nitrogen, &style_light_text, 0);
        lv_obj_remove_style(label_glaze_nitrogen_target, &style_light_text, 0);
        lv_obj_remove_style(btn_glaze_start, &style_light_btn, 0);
        lv_obj_remove_style(btn_glaze_back, &style_light_btn, 0);
        // <<< ДОБАВЛЕНО: Удаление светлого стиля для новых элементов >>>
        lv_obj_remove_style(label_glaze_monomer_blow_switch, &style_light_text, 0);
        lv_obj_remove_style(label_glaze_uv_mode_title, &style_light_text, 0);
        lv_obj_remove_style(label_glaze_uv_mode_status, &style_light_text, 0);


        // Добавляем стили темной темы
        lv_obj_add_style(main_container, &style_dark_bg, 0);
        lv_obj_add_style(label_glaze_header, &style_dark_text, 0);
        lv_obj_add_style(block_flicker, &style_dark_block_border, 0);
        lv_obj_add_style(block_flicker, &style_dark_block_bg, 0);
        lv_obj_add_style(block_timers, &style_dark_block_border, 0);
        lv_obj_add_style(block_timers, &style_dark_block_bg, 0);
        lv_obj_add_style(block_aux, &style_dark_block_border, 0);
        lv_obj_add_style(block_aux, &style_dark_block_bg, 0);
        lv_obj_add_style(label_glaze_title_flicker, &style_dark_block_header, 0);
        lv_obj_add_style(label_glaze_title_timers, &style_dark_block_header, 0);
        lv_obj_add_style(label_glaze_title_aux, &style_dark_block_header, 0);
        lv_obj_add_style(label_glaze_uv_on, &style_dark_text, 0);
        lv_obj_add_style(label_glaze_uv_off, &style_dark_text, 0);
        lv_obj_add_style(label_glaze_monomer_blow, &style_dark_text, 0);
        lv_obj_add_style(label_glaze_uv_exposure, &style_dark_text, 0);
        lv_obj_add_style(label_glaze_cooling, &style_dark_text, 0);
        lv_obj_add_style(label_glaze_nitrogen, &style_dark_text, 0);
        lv_obj_add_style(label_glaze_nitrogen_target, &style_dark_text, 0);
        lv_obj_add_style(label_glaze_monomer_blow_switch, &style_dark_text, 0);
        lv_obj_add_style(label_glaze_nitrogen_boost, &style_dark_text, 0);
        lv_obj_add_style(btn_glaze_start, &style_dark_btn, 0);
        lv_obj_add_style(btn_glaze_back, &style_dark_btn, 0);
        // <<< ДОБАВЛЕНО: Добавление темного стиля для новых элементов >>>
        lv_obj_add_style(label_glaze_monomer_blow_switch, &style_dark_text, 0);
        lv_obj_add_style(label_glaze_uv_mode_title, &style_dark_text, 0);
        lv_obj_add_style(label_glaze_uv_mode_status, &style_dark_text, 0);
        
    } else { // --- СВЕТЛАЯ ТЕМА ---
        // Удаляем стили темной темы
        lv_obj_remove_style(main_container, &style_dark_bg, 0);
        lv_obj_remove_style(label_glaze_header, &style_dark_text, 0);
        lv_obj_remove_style(block_flicker, &style_dark_block_border, 0);
        lv_obj_remove_style(block_flicker, &style_dark_block_bg, 0);
        lv_obj_remove_style(block_timers, &style_dark_block_border, 0);
        lv_obj_remove_style(block_timers, &style_dark_block_bg, 0);
        lv_obj_remove_style(block_aux, &style_dark_block_border, 0);
        lv_obj_remove_style(block_aux, &style_dark_block_bg, 0);
        lv_obj_remove_style(label_glaze_title_flicker, &style_dark_block_header, 0);
        lv_obj_remove_style(label_glaze_title_timers, &style_dark_block_header, 0);
        lv_obj_remove_style(label_glaze_title_aux, &style_dark_block_header, 0);
        lv_obj_remove_style(label_glaze_uv_on, &style_dark_text, 0);
        lv_obj_remove_style(label_glaze_uv_off, &style_dark_text, 0);
        lv_obj_remove_style(label_glaze_monomer_blow, &style_dark_text, 0);
        lv_obj_remove_style(label_glaze_uv_exposure, &style_dark_text, 0);
        lv_obj_remove_style(label_glaze_cooling, &style_dark_text, 0);
        lv_obj_remove_style(label_glaze_nitrogen, &style_dark_text, 0);
        lv_obj_remove_style(label_glaze_nitrogen_target, &style_dark_text, 0);
        lv_obj_remove_style(label_glaze_monomer_blow_switch, &style_dark_text, 0);
        lv_obj_remove_style(label_glaze_nitrogen_boost, &style_dark_text, 0);
        lv_obj_remove_style(btn_glaze_start, &style_dark_btn, 0);
        lv_obj_remove_style(btn_glaze_back, &style_dark_btn, 0);
        // <<< ДОБАВЛЕНО: Удаление темного стиля для новых элементов >>>
        lv_obj_remove_style(label_glaze_monomer_blow_switch, &style_dark_text, 0);
        lv_obj_remove_style(label_glaze_uv_mode_title, &style_dark_text, 0);
        lv_obj_remove_style(label_glaze_uv_mode_status, &style_dark_text, 0);


        // Добавляем стили светлой темы
        lv_obj_add_style(main_container, &style_light_bg, 0);
        lv_obj_add_style(label_glaze_header, &style_light_text, 0);
        lv_obj_add_style(block_flicker, &style_light_block_border, 0);
        lv_obj_add_style(block_flicker, &style_light_block_bg, 0);
        lv_obj_add_style(block_timers, &style_light_block_border, 0);
        lv_obj_add_style(block_timers, &style_light_block_bg, 0);
        lv_obj_add_style(block_aux, &style_light_block_border, 0);
        lv_obj_add_style(block_aux, &style_light_block_bg, 0);
        lv_obj_add_style(label_glaze_title_flicker, &style_light_block_header, 0);
        lv_obj_add_style(label_glaze_title_timers, &style_light_block_header, 0);
        lv_obj_add_style(label_glaze_title_aux, &style_light_block_header, 0);
        lv_obj_add_style(label_glaze_uv_on, &style_light_text, 0);
        lv_obj_add_style(label_glaze_uv_off, &style_light_text, 0);
        lv_obj_add_style(label_glaze_monomer_blow, &style_light_text, 0);
        lv_obj_add_style(label_glaze_uv_exposure, &style_light_text, 0);
        lv_obj_add_style(label_glaze_cooling, &style_light_text, 0);
        lv_obj_add_style(label_glaze_nitrogen, &style_light_text, 0);
        lv_obj_add_style(label_glaze_nitrogen_target, &style_light_text, 0);
        lv_obj_add_style(label_glaze_monomer_blow_switch, &style_light_text, 0);
        lv_obj_add_style(label_glaze_nitrogen_boost, &style_light_text, 0);
        lv_obj_add_style(btn_glaze_start, &style_light_btn, 0);
        lv_obj_add_style(btn_glaze_back, &style_light_btn, 0);
        // <<< ДОБАВЛЕНО: Добавление светлого стиля для новых элементов >>>
        lv_obj_add_style(label_glaze_monomer_blow_switch, &style_light_text, 0);
        lv_obj_add_style(label_glaze_uv_mode_title, &style_light_text, 0);
        lv_obj_add_style(label_glaze_uv_mode_status, &style_light_text, 0);
    }
    
    // Стилизация нового тройного переключателя UV
    lv_obj_t* all_btns[] = { btn_glaze_uv_1, btn_glaze_uv_2, btn_glaze_uv_3 };

    if (current_global_settings.theme == 1) { // --- ТЕМНАЯ ТЕМА ---
        for (int i = 0; i < 3; i++) {
            int btn_type = i; // 0 для кнопки "1", 1 для "2", 2 для "3"
            if (btn_type == 0) {
                lv_obj_set_style_bg_color(all_btns[i], dark_uv_btn1_inactive, LV_STATE_DEFAULT);
                lv_obj_set_style_bg_color(all_btns[i], dark_uv_btn1_active, LV_STATE_CHECKED);
            } else if (btn_type == 1) {
                lv_obj_set_style_bg_color(all_btns[i], dark_uv_btn2_inactive, LV_STATE_DEFAULT);
                lv_obj_set_style_bg_color(all_btns[i], dark_uv_btn2_active, LV_STATE_CHECKED);
            } else {
                lv_obj_set_style_bg_color(all_btns[i], dark_uv_btn3_inactive, LV_STATE_DEFAULT);
                lv_obj_set_style_bg_color(all_btns[i], dark_uv_btn3_active, LV_STATE_CHECKED);
            }
        }
    } else { // --- СВЕТЛАЯ ТЕМА ---
         for (int i = 0; i < 3; i++) {
            int btn_type = i;
            if (btn_type == 0) {
                lv_obj_set_style_bg_color(all_btns[i], light_uv_btn1_inactive, LV_STATE_DEFAULT);
                lv_obj_set_style_bg_color(all_btns[i], light_uv_btn1_active, LV_STATE_CHECKED);
            } else if (btn_type == 1) {
                lv_obj_set_style_bg_color(all_btns[i], light_uv_btn2_inactive, LV_STATE_DEFAULT);
                lv_obj_set_style_bg_color(all_btns[i], light_uv_btn2_active, LV_STATE_CHECKED);
            } else {
                lv_obj_set_style_bg_color(all_btns[i], light_uv_btn3_inactive, LV_STATE_DEFAULT);
                lv_obj_set_style_bg_color(all_btns[i], light_uv_btn3_active, LV_STATE_CHECKED);
            }
        }
    }
}

void apply_theme_to_lab_repair_screen() {
    if (!screen_lab_repair) return;

    // Находим все нужные объекты по их указателям
    lv_obj_t* main_container = lv_obj_get_child(screen_lab_repair, 0);
    if (!main_container) return;
    lv_obj_t* content_area = lv_obj_get_child(main_container, 1);
    lv_obj_t* block_timers = lv_obj_get_child(content_area, 0);
    lv_obj_t* block_warning = lv_obj_get_child(content_area, 1);
    lv_obj_t* timers_header = lv_obj_get_child(block_timers, 0);
    lv_obj_t* warning_header = lv_obj_get_child(block_warning, 0);
    
    if (current_global_settings.theme == 1) { // --- ТЕМНАЯ ТЕМА ---
        // Удаляем стили светлой темы
        lv_obj_remove_style(main_container, &style_light_bg, 0);
        lv_obj_remove_style(label_repair_header, &style_light_text, 0);
        lv_obj_remove_style(block_timers, &style_light_block_border, 0);
        lv_obj_remove_style(block_timers, &style_light_block_bg, 0);
        lv_obj_remove_style(block_warning, &style_light_block_border, 0);
        lv_obj_remove_style(block_warning, &style_light_block_bg, 0);
        lv_obj_remove_style(timers_header, &style_light_block_header, 0);
        lv_obj_remove_style(label_repair_countdown, &style_light_text, 0);
        lv_obj_remove_style(label_repair_uv_exposure, &style_light_text, 0);
        lv_obj_remove_style(label_repair_info_text, &style_light_text, 0);
        lv_obj_remove_style(label_repair_arrow_text, &style_light_text, 0);
        lv_obj_remove_style(btn_repair_start, &style_light_btn, 0);
        lv_obj_remove_style(btn_repair_back, &style_light_btn, 0);

        // Добавляем стили темной темы
        lv_obj_add_style(main_container, &style_dark_bg, 0);
        lv_obj_add_style(label_repair_header, &style_dark_text, 0);
        lv_obj_add_style(block_timers, &style_dark_block_border, 0);
        lv_obj_add_style(block_timers, &style_dark_block_bg, 0);
        lv_obj_add_style(block_warning, &style_dark_block_border, 0);
        lv_obj_add_style(block_warning, &style_dark_block_bg, 0);
        lv_obj_add_style(timers_header, &style_dark_block_header, 0);
        // warning_header НЕ трогаем, он всегда желтый с черным текстом
        lv_obj_add_style(label_repair_countdown, &style_dark_text, 0);
        lv_obj_add_style(label_repair_uv_exposure, &style_dark_text, 0);
        lv_obj_add_style(label_repair_info_text, &style_dark_text, 0);
        lv_obj_add_style(label_repair_arrow_text, &style_dark_text, 0);
        lv_obj_add_style(btn_repair_start, &style_dark_btn, 0);
        lv_obj_add_style(btn_repair_back, &style_dark_btn, 0);

    } else { // --- СВЕТЛАЯ ТЕМА ---
        // Удаляем стили темной темы
        lv_obj_remove_style(main_container, &style_dark_bg, 0);
        lv_obj_remove_style(label_repair_header, &style_dark_text, 0);
        lv_obj_remove_style(block_timers, &style_dark_block_border, 0);
        lv_obj_remove_style(block_timers, &style_dark_block_bg, 0);
        lv_obj_remove_style(block_warning, &style_dark_block_border, 0);
        lv_obj_remove_style(block_warning, &style_dark_block_bg, 0);
        lv_obj_remove_style(timers_header, &style_dark_block_header, 0);
        lv_obj_remove_style(label_repair_countdown, &style_dark_text, 0);
        lv_obj_remove_style(label_repair_uv_exposure, &style_dark_text, 0);
        lv_obj_remove_style(label_repair_info_text, &style_dark_text, 0);
        lv_obj_remove_style(label_repair_arrow_text, &style_dark_text, 0);
        lv_obj_remove_style(btn_repair_start, &style_dark_btn, 0);
        lv_obj_remove_style(btn_repair_back, &style_dark_btn, 0);

        // Добавляем стили светлой темы
        lv_obj_add_style(main_container, &style_light_bg, 0);
        lv_obj_add_style(label_repair_header, &style_light_text, 0);
        lv_obj_add_style(block_timers, &style_light_block_border, 0);
        lv_obj_add_style(block_timers, &style_light_block_bg, 0);
        lv_obj_add_style(block_warning, &style_light_block_border, 0);
        lv_obj_add_style(block_warning, &style_light_block_bg, 0);
        lv_obj_add_style(timers_header, &style_light_block_header, 0);
        // warning_header НЕ трогаем, он всегда желтый с черным текстом
        lv_obj_add_style(label_repair_countdown, &style_light_text, 0);
        lv_obj_add_style(label_repair_uv_exposure, &style_light_text, 0);
        lv_obj_add_style(label_repair_info_text, &style_light_text, 0);
        lv_obj_add_style(label_repair_arrow_text, &style_light_text, 0);
        lv_obj_add_style(btn_repair_start, &style_light_btn, 0);
        lv_obj_add_style(btn_repair_back, &style_light_btn, 0);
    }
}
void apply_theme_to_process_screen() {
    if (!screen_process_execution) return;

    // Находим наш главный контейнер. Он всегда первый дочерний элемент экрана.
    lv_obj_t* content_container = lv_obj_get_child(screen_process_execution, 0);
    if (!content_container) return;

    if (current_global_settings.theme == 1) { // --- ТЕМНАЯ ТЕМА ---
        // Удаляем стили светлой темы
        lv_obj_remove_style(content_container, &style_light_bg, 0);
        lv_obj_remove_style(label_process_profile_name, &style_light_text, 0);
        lv_obj_remove_style(label_process_status_title, &style_light_text, 0);
        lv_obj_remove_style(label_process_status_detail, &style_light_text, 0);
        // lv_obj_remove_style(spinner_process_execution, &style_light_spinner_bg, LV_PART_MAIN);
        // lv_obj_remove_style(spinner_process_execution, &style_light_spinner_indic, LV_PART_INDICATOR);
        lv_obj_remove_style(btn_process_cancel, &style_light_btn, 0);

        // Добавляем стили темной темы
        lv_obj_add_style(content_container, &style_dark_bg, 0);
        lv_obj_set_style_radius(content_container, 0, 0);
        lv_obj_add_style(label_process_profile_name, &style_dark_text, 0);
        lv_obj_add_style(label_process_status_title, &style_dark_text, 0);
        lv_obj_add_style(label_process_status_detail, &style_dark_text, 0);
        // lv_obj_add_style(spinner_process_execution, &style_dark_spinner_bg, LV_PART_MAIN);
        // lv_obj_add_style(spinner_process_execution, &style_dark_spinner_indic, LV_PART_INDICATOR);
        lv_obj_add_style(btn_process_cancel, &style_dark_btn, 0);
        lv_canvas_fill_bg(repair_mode_indicator_obj, lv_color_black(), LV_OPA_TRANSP);
        lv_point_t main_points[] = { {0, 0}, {239, 0}, {120, 99} };
        lv_draw_rect_dsc_t main_dsc;
        lv_draw_rect_dsc_init(&main_dsc);
        main_dsc.bg_color = lv_color_hex(0xff05b8);
        main_dsc.bg_opa = LV_OPA_COVER;
        lv_canvas_draw_polygon(repair_mode_indicator_obj, main_points, 3, &main_dsc);
        
        lv_draw_rect_dsc_t corner_dsc;
        lv_draw_rect_dsc_init(&corner_dsc);
        corner_dsc.bg_color = lv_color_black(); // ЧЕРНЫЙ ЦВЕТ
        corner_dsc.bg_opa = LV_OPA_COVER;
        
        lv_coord_t cut_size = 120;
        lv_point_t corner_points_left[] = { {0, -21}, {0, 99}, {120, 99} };
        lv_canvas_draw_polygon(repair_mode_indicator_obj, corner_points_left, 3, &corner_dsc);
        lv_point_t corner_points_right[] = { {239, -21}, {239, 99}, {119, 99} };
        lv_canvas_draw_polygon(repair_mode_indicator_obj, corner_points_right, 3, &corner_dsc);

    } else { // --- СВЕТЛАЯ ТЕМА ---
        // Удаляем стили темной темы
        lv_obj_remove_style(content_container, &style_dark_bg, 0);
        lv_obj_set_style_radius(content_container, 0, 0);
        lv_obj_remove_style(label_process_profile_name, &style_dark_text, 0);
        lv_obj_remove_style(label_process_status_title, &style_dark_text, 0);
        lv_obj_remove_style(label_process_status_detail, &style_dark_text, 0);
        // lv_obj_remove_style(spinner_process_execution, &style_dark_spinner_bg, LV_PART_MAIN);
        // lv_obj_remove_style(spinner_process_execution, &style_dark_spinner_indic, LV_PART_INDICATOR);
        lv_obj_remove_style(btn_process_cancel, &style_dark_btn, 0);

        // Добавляем стили светлой темы
        lv_obj_add_style(content_container, &style_light_bg, 0);
        lv_obj_add_style(label_process_profile_name, &style_light_text, 0);
        lv_obj_add_style(label_process_status_title, &style_light_text, 0);
        lv_obj_add_style(label_process_status_detail, &style_light_text, 0);
        // lv_obj_add_style(spinner_process_execution, &style_light_spinner_bg, LV_PART_MAIN);
        // lv_obj_add_style(spinner_process_execution, &style_light_spinner_indic, LV_PART_INDICATOR);
        lv_obj_add_style(btn_process_cancel, &style_light_btn, 0);
        lv_canvas_fill_bg(repair_mode_indicator_obj, lv_color_black(), LV_OPA_TRANSP);
        lv_point_t main_points[] = { {0, 0}, {239, 0}, {120, 99} };
        lv_draw_rect_dsc_t main_dsc;
        lv_draw_rect_dsc_init(&main_dsc);
        main_dsc.bg_color = lv_color_hex(0xff05b8);
        main_dsc.bg_opa = LV_OPA_COVER;
        lv_canvas_draw_polygon(repair_mode_indicator_obj, main_points, 3, &main_dsc);
        
        lv_draw_rect_dsc_t corner_dsc;
        lv_draw_rect_dsc_init(&corner_dsc);
        corner_dsc.bg_color = lv_color_hex(0xF0F0F0); 
        corner_dsc.bg_opa = LV_OPA_COVER;
        
        lv_coord_t cut_size = 120;
        lv_point_t corner_points_left[] = { {0, -21}, {0, 99}, {120, 99} };
        lv_canvas_draw_polygon(repair_mode_indicator_obj, corner_points_left, 3, &corner_dsc);
        lv_point_t corner_points_right[] = { {239, -21}, {239, 99}, {119, 99} };
        lv_canvas_draw_polygon(repair_mode_indicator_obj, corner_points_right, 3, &corner_dsc);
    }
}
static void keyboard_screen_bg_clicked_cb(lv_event_t* e) {
    // Эта магия проверяет, что клик был именно по фону (screen_keyboard),
    // а не по его дочерним элементам (клавиатуре или полю ввода).
    if (lv_event_get_target(e) != lv_event_get_current_target(e)) {
        return; // Кликнули по дочернему элементу, ничего не делаем
    }
    
    Serial.println("Keyboard screen background clicked, cancelling input.");

    // Ищем, какая из клавиатур сейчас видна
    lv_obj_t* visible_keyboard = NULL;
    if (kb_edit_alpha && !lv_obj_has_flag(kb_edit_alpha, LV_OBJ_FLAG_HIDDEN)) {
        visible_keyboard = kb_edit_alpha;
    } else if (kb_edit_numeric && !lv_obj_has_flag(kb_edit_numeric, LV_OBJ_FLAG_HIDDEN)) {
        visible_keyboard = kb_edit_numeric;
    } else if (kb_service_code && !lv_obj_has_flag(kb_service_code, LV_OBJ_FLAG_HIDDEN)) {
        visible_keyboard = kb_service_code;
    }

    // Если нашли видимую клавиатуру, отправляем ей событие "Отмена".
    // Это вызовет modal_input_keyboard_event_cb или service_code_keyboard_event_cb,
    // которые уже умеют правильно закрывать экран.
    if (visible_keyboard) {
        lv_event_send(visible_keyboard, LV_EVENT_CANCEL, NULL);
    }
}
static void build_keyboard_screen(lv_obj_t* parent_screen) {
    screen_keyboard = parent_screen;
    lv_obj_set_style_bg_color(screen_keyboard, lv_color_hex(0x333333), 0);
    lv_obj_add_event_cb(screen_keyboard, keyboard_screen_bg_clicked_cb, LV_EVENT_CLICKED, NULL);

    // --- Прокси-поле для ввода ---
    ta_keyboard_proxy = lv_textarea_create(screen_keyboard);
    lv_obj_set_width(ta_keyboard_proxy, lv_pct(90));
    lv_textarea_set_one_line(ta_keyboard_proxy, true);
    lv_obj_align(ta_keyboard_proxy, LV_ALIGN_TOP_MID, 0, 10);
    lv_obj_add_style(ta_keyboard_proxy, &style_my_text_18, 0);
    lv_obj_add_state(ta_keyboard_proxy, LV_STATE_FOCUSED);
    lv_obj_set_style_pad_all(ta_keyboard_proxy, 10, 0);


    // --- Создаем и настраиваем все три клавиатуры ---

    // 1. Алфавитная клавиатура
    kb_edit_alpha = lv_keyboard_create(screen_keyboard);
    lv_obj_add_flag(kb_edit_alpha, LV_OBJ_FLAG_HIDDEN);
    lv_keyboard_set_textarea(kb_edit_alpha, ta_keyboard_proxy);
    lv_obj_add_event_cb(kb_edit_alpha, modal_input_keyboard_event_cb, LV_EVENT_ALL, NULL);
    lv_obj_set_height(kb_edit_alpha, lv_pct(70));

    // 2. Цифровая клавиатура
    kb_edit_numeric = lv_keyboard_create(screen_keyboard);
    lv_obj_add_flag(kb_edit_numeric, LV_OBJ_FLAG_HIDDEN);
    lv_keyboard_set_textarea(kb_edit_numeric, ta_keyboard_proxy);
    lv_obj_add_event_cb(kb_edit_numeric, modal_input_keyboard_event_cb, LV_EVENT_ALL, NULL);
    lv_keyboard_set_mode(kb_edit_numeric, LV_KEYBOARD_MODE_NUMBER);
    lv_obj_set_height(kb_edit_numeric, lv_pct(70));

    // 3. Сервисная клавиатура
    kb_service_code = lv_keyboard_create(screen_keyboard);
    lv_obj_add_flag(kb_service_code, LV_OBJ_FLAG_HIDDEN);
    lv_keyboard_set_textarea(kb_service_code, ta_keyboard_proxy);
    lv_obj_add_event_cb(kb_service_code, service_code_keyboard_event_cb, LV_EVENT_ALL, NULL);
    lv_keyboard_set_mode(kb_service_code, LV_KEYBOARD_MODE_TEXT_UPPER);
    lv_obj_set_height(kb_service_code, lv_pct(70));
}

// Обработчик для кнопки "Назад" с экрана лаб. режима
static void glaze_screen_event_cb(lv_event_t* e) {
    const char* user_data = (const char*)lv_event_get_user_data(e);
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;

    if (strcmp(user_data, "show_help") == 0) {
        if (current_global_settings.language == 1) { // RUS
            show_info_dialog("Справка: Режим Глазурь", "Режим для нанесения финального глянцевого слоя. Настройте частоту мерцания УФ и время этапов для достижения идеального блеска.");
        } else { // ENG
            show_info_dialog("Help: Glaze Mode", "A mode for applying the final glossy layer. Adjust the UV flicker rate and stage times to achieve a perfect shine.");
        }
    } else if (strcmp(user_data, "back") == 0) {
        if (help_blink_timer) { lv_timer_del(help_blink_timer); help_blink_timer = nullptr; }
        trigger_lab_action(1); // 1 = Назад
    } else if (strcmp(user_data, "start") == 0) {
        trigger_lab_action(2);
    }
}
static void glaze_uv_mode_save_cb(lv_event_t * e) {
    lv_obj_t* clicked_btn = lv_event_get_target(e);
    
    // Определяем, какая кнопка была нажата (0, 1 или 2)
    int new_mode = 0;
    if (clicked_btn == btn_glaze_uv_1) {
        new_mode = 0;
    } else if (clicked_btn == btn_glaze_uv_2) {
        new_mode = 1;
    } else if (clicked_btn == btn_glaze_uv_3) {
        new_mode = 2;
    }

    // Обновляем структуру данных и сохраняем
    if (current_lab_settings.glaze.uv_mode != new_mode) {
        current_lab_settings.glaze.uv_mode = new_mode;
        Serial.printf("Glaze UV mode changed to %d. Saving settings...\n", new_mode);
        // saveLaboratorySettings();
    }
}

static void lab_screen_back_event_cb(lv_event_t * e) {
    if (lv_event_get_code(e) == LV_EVENT_CLICKED) {
        Serial.println("Back from Lab Mode to Main Menu.");
        if (screen_main_app) {
            load_screen(screen_main_app);
        }
    }
}

// Эта "тяжелая" функция вызывается из loop() для подготовки и показа экрана лабораторного режима
static void prepare_and_show_lab_screen(int mode_id) {
    if (help_blink_timer) {
        lv_timer_del(help_blink_timer);
        help_blink_timer = nullptr;
    }
    
    switch(mode_id) {
        case 1: { // Глазурь
            if(!screen_lab_glaze) break;

            // 1. Загружаем актуальные данные в UI
            char buf[10];
            snprintf(buf, sizeof(buf), "%.1f", current_lab_settings.glaze.uv_on_sec);
            lv_textarea_set_text(ta_glaze_uv_on, buf);
            snprintf(buf, sizeof(buf), "%.1f", current_lab_settings.glaze.uv_off_sec);
            lv_textarea_set_text(ta_glaze_uv_off, buf);
            snprintf(buf, sizeof(buf), "%d", current_lab_settings.glaze.monomer_blow_min);
            lv_textarea_set_text(ta_glaze_monomer_blow, buf);
            snprintf(buf, sizeof(buf), "%d", current_lab_settings.glaze.uv_exposure_sec);
            lv_textarea_set_text(ta_glaze_uv_exposure, buf);
            snprintf(buf, sizeof(buf), "%d", current_lab_settings.glaze.nitrogen_target_percent);
            lv_textarea_set_text(ta_glaze_nitrogen_target, buf);
            snprintf(buf, sizeof(buf), "%d", current_lab_settings.glaze.nitrogen_boost_sec);
            lv_textarea_set_text(ta_glaze_nitrogen_boost, buf);            

            if (current_lab_settings.glaze.use_monomer_blow) lv_obj_add_state(sw_glaze_monomer_blow, LV_STATE_CHECKED);
            else lv_obj_clear_state(sw_glaze_monomer_blow, LV_STATE_CHECKED);
            
            bool is_monomer_blow_active = lv_obj_has_state(sw_glaze_monomer_blow, LV_STATE_CHECKED);
            if (is_monomer_blow_active) {
                lv_obj_set_style_opa(monomer_blow_container, LV_OPA_COVER, 0);
                lv_obj_clear_flag(monomer_blow_container, LV_OBJ_FLAG_CLICKABLE);
            } else {
                lv_obj_set_style_opa(monomer_blow_container, LV_OPA_TRANSP, 0);
                lv_obj_add_flag(monomer_blow_container, LV_OBJ_FLAG_CLICKABLE);
            }

            lv_obj_t* glaze_uv_btns[] = {btn_glaze_uv_1, btn_glaze_uv_2, btn_glaze_uv_3};
            for(int i=0; i<3; i++) lv_obj_clear_state(glaze_uv_btns[i], LV_STATE_CHECKED);
            int uv_mode_idx = current_lab_settings.glaze.uv_mode;
            if (uv_mode_idx >= 0 && uv_mode_idx < 3) {
                lv_obj_add_state(glaze_uv_btns[uv_mode_idx], LV_STATE_CHECKED);
                lv_event_send(glaze_uv_btns[uv_mode_idx], LV_EVENT_CLICKED, NULL); 
            }
            
            if (current_lab_settings.glaze.use_cooling) lv_obj_add_state(sw_glaze_cooling, LV_STATE_CHECKED);
            else lv_obj_clear_state(sw_glaze_cooling, LV_STATE_CHECKED);
            
            if (current_lab_settings.glaze.use_nitrogen) lv_obj_add_state(sw_glaze_nitrogen, LV_STATE_CHECKED);
            else lv_obj_clear_state(sw_glaze_nitrogen, LV_STATE_CHECKED);
            
            if (!current_global_settings.compressed_air_system_enabled) lv_obj_clear_state(sw_glaze_cooling, LV_STATE_CHECKED);
            if (!current_global_settings.nitrogen_system_enabled) lv_obj_clear_state(sw_glaze_nitrogen, LV_STATE_CHECKED);

            bool is_nitrogen_active = lv_obj_has_state(sw_glaze_nitrogen, LV_STATE_CHECKED);
            if (is_nitrogen_active) {
                lv_obj_set_style_opa(nitrogen_glaze_container, LV_OPA_COVER, 0);
                lv_obj_clear_flag(nitrogen_glaze_container, LV_OBJ_FLAG_CLICKABLE);
                lv_obj_add_flag(label_glaze_nitrogen_warning, LV_OBJ_FLAG_HIDDEN);
            } else {
                lv_obj_set_style_opa(nitrogen_glaze_container, LV_OPA_TRANSP, 0);
                lv_obj_add_flag(nitrogen_glaze_container, LV_OBJ_FLAG_CLICKABLE);
                lv_obj_clear_flag(label_glaze_nitrogen_warning, LV_OBJ_FLAG_HIDDEN);
            }

            if (current_global_settings.language == 1) {
                lv_label_set_text(label_glaze_header, "Режим: Глазурь");
                lv_label_set_text(lv_obj_get_child(btn_glaze_help, 0), "Справка по разделу ?");
                lv_label_set_text(label_glaze_title_flicker, "Настройка такта мерцания");
                lv_label_set_text(label_glaze_uv_on, "Вкл. диода (0.1-3с)");
                lv_label_set_text(label_glaze_uv_off, "Выкл. диода (0.1-3с)");
                lv_label_set_text(label_glaze_title_timers, "Настройка таймеров");
                lv_label_set_text(label_glaze_uv_mode_title, "Тип:");
                lv_label_set_text(label_glaze_monomer_blow, "Время (1-10мин)");
                lv_label_set_text(label_glaze_uv_exposure, "Работа УФ (10-200с)");
                lv_label_set_text(label_glaze_title_aux, "Вспомогательные параметры");
                lv_label_set_text(label_glaze_monomer_blow_switch, "Обдув мономера");
                lv_label_set_text(label_glaze_cooling, "Быстрое охлаждение");
                lv_label_set_text(label_glaze_nitrogen, "Использование азота");
                lv_label_set_text(label_glaze_nitrogen_target, "Цель N2 (%)");
                lv_label_set_text(label_glaze_nitrogen_boost, "Доп. подача (1-5с)");
                lv_label_set_text(label_btn_glaze_start, "Старт");
                lv_label_set_text(label_btn_glaze_back, "Назад");
            } else {
                lv_label_set_text(label_glaze_header, "Mode: Glaze");
                lv_label_set_text(lv_obj_get_child(btn_glaze_help, 0), "Section Help ?");
                lv_label_set_text(label_glaze_title_flicker, "Flicker Takt Setup");
                lv_label_set_text(label_glaze_uv_on, "Diode ON (0.1-3s)");
                lv_label_set_text(label_glaze_uv_off, "Diode OFF (0.1-3s)");
                lv_label_set_text(label_glaze_title_timers, "Timers Setup");
                lv_label_set_text(label_glaze_uv_mode_title, "Type:");
                lv_label_set_text(label_glaze_monomer_blow, "Blow (1-10min)");
                lv_label_set_text(label_glaze_uv_exposure, "UV Work (10-200s)");
                lv_label_set_text(label_glaze_title_aux, "Auxiliary Parameters");
                lv_label_set_text(label_glaze_monomer_blow_switch, "Monomer Blow");
                lv_label_set_text(label_glaze_cooling, "Fast Cooling");
                lv_label_set_text(label_glaze_nitrogen, "Use Nitrogen");
                lv_label_set_text(label_glaze_nitrogen_target, "N2 Target (%)");
                lv_label_set_text(label_glaze_nitrogen_boost, "Boost (1-5s)");
                lv_label_set_text(label_btn_glaze_start, "Start");
                lv_label_set_text(label_btn_glaze_back, "Back");
            }
            
            help_blink_timer = lv_timer_create(help_blink_timer_cb, 1, btn_glaze_help);
            apply_theme_to_lab_glaze_screen();
            load_screen(screen_lab_glaze);
            break;
        } // <<<--- ИСПРАВЛЕНИЕ: ДОБАВЛЕНА ЭТА СКОБКА
        case 2: { // Ремонт модели
            if(!screen_lab_repair) break;

            char buf[10];
            snprintf(buf, sizeof(buf), "%d", current_lab_settings.repair.countdown_sec);
            lv_textarea_set_text(ta_repair_countdown, buf);
            snprintf(buf, sizeof(buf), "%d", current_lab_settings.repair.uv_exposure_sec);
            lv_textarea_set_text(ta_repair_uv_exposure, buf);

            if (current_global_settings.language == 1) {
                lv_label_set_text(label_repair_header, "Режим: Ремонт модели");
                lv_label_set_text(lv_obj_get_child(btn_repair_help, 0), "Справка ?");
                lv_obj_t* warning_header = lv_obj_get_child(label_repair_info_text->parent, 0);
                lv_label_set_text(warning_header, "ВНИМАНИЕ!");
                lv_label_set_text(label_repair_info_text, "Берегите глаза! Надевайте защитные очки и перчатки!");
                lv_label_set_text(label_repair_title_timers, "Настройки таймеров");
                lv_label_set_text(label_repair_countdown, "Таймер обратного отсчета (5-60с):");
                lv_label_set_text(label_repair_uv_exposure, "Таймер работы УФ диода (5-60с):");
                lv_label_set_text(label_repair_arrow_text, "Поднесите модель к УФ диоду, который расположен под экраном засветки.");
                lv_label_set_text(label_btn_repair_start, "Старт");
                lv_label_set_text(label_btn_repair_back, "Назад");
            } else {
                lv_label_set_text(label_repair_header, "Mode: Model Repair");
                lv_label_set_text(lv_obj_get_child(btn_repair_help, 0), "Help ?");
                lv_obj_t* warning_header = lv_obj_get_child(label_repair_info_text->parent, 0);
                lv_label_set_text(warning_header, "WARNING!");
                lv_label_set_text(label_repair_info_text, "Protect your eyes! Wear safety glasses and gloves!");
                lv_label_set_text(label_repair_title_timers, "Timers Setup");
                lv_label_set_text(label_repair_countdown, "Countdown Timer (5-60s):");
                lv_label_set_text(label_repair_uv_exposure, "UV Diode Timer (5-60s):");
                lv_label_set_text(label_repair_arrow_text, "Bring the model to the UV diode");
                lv_label_set_text(label_btn_repair_start, "Start");
                lv_label_set_text(label_btn_repair_back, "Back");
            }

            help_blink_timer = lv_timer_create(help_blink_timer_cb, 1, btn_repair_help);
            apply_theme_to_lab_repair_screen();
            load_screen(screen_lab_repair);
            break;
        } // <<<--- ИСПРАВЛЕНИЕ: ДОБАВЛЕНА ЭТА СКОБКА
        case 3: { // Повышение прочности
            if(!screen_lab_strength) break;

            char buf[10];
            snprintf(buf, sizeof(buf), "%d", current_lab_settings.strength.chamber_temp_c);
            lv_textarea_set_text(ta_strength_temp, buf);
            snprintf(buf, sizeof(buf), "%d", current_lab_settings.strength.hold_time_min);
            lv_textarea_set_text(ta_strength_hold_time, buf);
            snprintf(buf, sizeof(buf), "%d", current_lab_settings.strength.uv_pulse_duration_sec);
            lv_textarea_set_text(ta_strength_uv_pulse_duration, buf);
            snprintf(buf, sizeof(buf), "%d", current_lab_settings.strength.uv_pulse_interval_min);
            lv_textarea_set_text(ta_strength_uv_pulse_interval, buf);

            if (current_lab_settings.strength.use_cooling) lv_obj_add_state(sw_strength_cooling, LV_STATE_CHECKED);
            else lv_obj_clear_state(sw_strength_cooling, LV_STATE_CHECKED);

            if (!current_global_settings.compressed_air_system_enabled) lv_obj_clear_state(sw_strength_cooling, LV_STATE_CHECKED);
            
            if (current_global_settings.language == 1) {
                lv_label_set_text(label_strength_header, "Режим: Повышение прочности");
                lv_label_set_text(lv_obj_get_child(btn_strength_help, 0), "Справка ?");
                lv_label_set_text(label_strength_title_thermo, "Настройка термокамеры");
                lv_label_set_text(label_strength_temp, "Температура (50-80C):");
                lv_label_set_text(label_strength_hold_time, "Удержание (30-180мин):");
                lv_label_set_text(label_strength_cooling, "Быстрое охлаждение");
                lv_label_set_text(label_strength_title_uv, "Настройка УФ-импульсов");
                lv_label_set_text(label_strength_uv_pulse_duration, "Длительность (1-10с):");
                lv_label_set_text(label_strength_uv_pulse_interval, "Интервал (1-10мин):");
                lv_label_set_text_fmt(label_strength_info_uv_text, "Подача УФ на %d сек каждые %d мин удержания.", current_lab_settings.strength.uv_pulse_duration_sec, current_lab_settings.strength.uv_pulse_interval_min);
                lv_label_set_text(label_btn_strength_start, "Старт");
                lv_label_set_text(label_btn_strength_back, "Назад");
            } else {
                lv_label_set_text(label_strength_header, "Mode: Strength Boost");
                lv_label_set_text(lv_obj_get_child(btn_strength_help, 0), "Help ?");
                lv_label_set_text(label_strength_title_thermo, "Thermal Chamber Setup");
                lv_label_set_text(label_strength_temp, "Temperature (50-80C):");
                lv_label_set_text(label_strength_hold_time, "Hold Time (30-180min):");
                lv_label_set_text(label_strength_cooling, "Fast Cooling");
                lv_label_set_text(label_strength_title_uv, "UV Pulse Setup");
                lv_label_set_text(label_strength_uv_pulse_duration, "Pulse duration (1-10s):");
                lv_label_set_text(label_strength_uv_pulse_interval, "Pulse interval (1-10m):");
                lv_label_set_text_fmt(label_strength_info_uv_text, "UV pulse for %d sec every %d min of hold time.", current_lab_settings.strength.uv_pulse_duration_sec, current_lab_settings.strength.uv_pulse_interval_min);
                lv_label_set_text(label_btn_strength_start, "Start");
                lv_label_set_text(label_btn_strength_back, "Back");
            }
            
            help_blink_timer = lv_timer_create(help_blink_timer_cb, 1, btn_strength_help);
            apply_theme_to_lab_strength_screen();
            load_screen(screen_lab_strength);
            break;
        } // <<<--- ИСПРАВЛЕНИЕ: ДОБАВЛЕНА ЭТА СКОБКА
        case 4: { // Термокамера
            if(!screen_lab_thermal) break;

            char buf[10];
            snprintf(buf, sizeof(buf), "%d", current_lab_settings.thermal.chamber_temp_c);
            lv_textarea_set_text(ta_thermal_temp, buf);
            snprintf(buf, sizeof(buf), "%d", current_lab_settings.thermal.hold_time_min);
            lv_textarea_set_text(ta_thermal_hold_time, buf);

            if (current_global_settings.language == 1) {
                lv_label_set_text(label_thermal_header, "Режим: Термокамера");
                lv_label_set_text(lv_obj_get_child(btn_thermal_help, 0), "Справка ?");
                lv_label_set_text(label_thermal_title_thermo, "Настройка термокамеры");
                lv_label_set_text(label_thermal_temp, "Температура в термокамере (40-60C):");
                lv_label_set_text(label_thermal_info, "Рекомендуемая t для большинства смол: 50C");
                lv_label_set_text(label_thermal_hold_time, "Время удержания (5-30мин):");
                lv_label_set_text(label_btn_thermal_start, "Старт");
                lv_label_set_text(label_btn_thermal_back, "Назад");
            } else {
                lv_label_set_text(label_thermal_header, "Mode: Thermal Chamber");
                lv_label_set_text(lv_obj_get_child(btn_thermal_help, 0), "Help ?");
                lv_label_set_text(label_thermal_title_thermo, "Thermal Chamber Setup");
                lv_label_set_text(label_thermal_temp, "Chamber Temperature (40-60C):");
                lv_label_set_text(label_thermal_info, "Recommended temperature for most resins: 50C");
                lv_label_set_text(label_thermal_hold_time, "Hold Time (5-30min):");
                lv_label_set_text(label_btn_thermal_start, "Start");
                lv_label_set_text(label_btn_thermal_back, "Back");
            }
            
            help_blink_timer = lv_timer_create(help_blink_timer_cb, 1, btn_thermal_help);
            apply_theme_to_lab_thermal_screen();
            load_screen(screen_lab_thermal);
            break;
        } // <<<--- ИСПРАВЛЕНИЕ: ДОБАВЛЕНА ЭТА СКОБКА
        case 5: { // Осветление
            if(!screen_lab_lighten) break;

            char buf[10];
            snprintf(buf, sizeof(buf), "%d", current_lab_settings.lighten.hold_time_min);
            lv_textarea_set_text(ta_lighten_hold_time, buf);

            if (current_lab_settings.lighten.use_cooling) lv_obj_add_state(sw_lighten_cooling, LV_STATE_CHECKED);
            else lv_obj_clear_state(sw_lighten_cooling, LV_STATE_CHECKED);
            
            if (!current_global_settings.compressed_air_system_enabled) lv_obj_clear_state(sw_lighten_cooling, LV_STATE_CHECKED);

            if (current_global_settings.language == 1) {
                lv_label_set_text(label_lighten_header, "Режим: Осветление композита");
                lv_label_set_text(lv_obj_get_child(btn_lighten_help, 0), "Справка ?");
                lv_label_set_text(label_lighten_title_thermo, "Настройки термокамеры");
                lv_label_set_text_fmt(label_lighten_temp_fixed, "Температура в камере: %dC (фиксированно)", current_lab_settings.lighten.chamber_temp_c);
                lv_label_set_text(label_lighten_hold_time, "Время удержания (10-30мин):");
                lv_label_set_text(label_lighten_cooling, "Быстрое охлаждение (сжатый воздух)");
                lv_label_set_text(label_btn_lighten_start, "Старт");
                lv_label_set_text(label_btn_lighten_back, "Назад");
            } else {
                lv_label_set_text(label_lighten_header, "Mode: Composite Lightening");
                lv_label_set_text(lv_obj_get_child(btn_lighten_help, 0), "Help ?");
                lv_label_set_text(label_lighten_title_thermo, "Thermal Chamber Setup");
                lv_label_set_text_fmt(label_lighten_temp_fixed, "Chamber Temperature: %dC (fixed)", current_lab_settings.lighten.chamber_temp_c);
                lv_label_set_text(label_lighten_hold_time, "Hold Time (10-30min):");
                lv_label_set_text(label_lighten_cooling, "Fast Cooling (Compressed Air)");
                lv_label_set_text(label_btn_lighten_start, "Start");
                lv_label_set_text(label_btn_lighten_back, "Back");
            }
            
            help_blink_timer = lv_timer_create(help_blink_timer_cb, 1, btn_lighten_help);
            apply_theme_to_lab_lighten_screen();
            load_screen(screen_lab_lighten);
            break;
        } // <<<--- ИСПРАВЛЕНИЕ: ДОБАВЛЕНА ЭТА СКОБКА
        case 6: { // Затемнение
            if(!screen_lab_darken) break;

            char buf[10];
            snprintf(buf, sizeof(buf), "%d", current_lab_settings.darken.uv_exposure_min);
            lv_textarea_set_text(ta_darken_uv_exposure, buf);

            if (current_lab_settings.darken.use_cooling) lv_obj_add_state(sw_darken_cooling, LV_STATE_CHECKED);
            else lv_obj_clear_state(sw_darken_cooling, LV_STATE_CHECKED);

            if (!current_global_settings.compressed_air_system_enabled) lv_obj_clear_state(sw_darken_cooling, LV_STATE_CHECKED);
            
            if (current_global_settings.language == 1) {
                lv_label_set_text(label_darken_header, "Режим: Затемнение композита");
                lv_label_set_text(lv_obj_get_child(btn_darken_help, 0), "Справка ?");
                lv_label_set_text(label_darken_title_params, "Настройка параметров");
                lv_label_set_text(label_darken_uv_exposure, "Время работы УФ диодов (1-15мин):");
                lv_label_set_text(label_darken_cooling, "Быстрое охлаждение (сжатый воздух)");
                lv_label_set_text(label_btn_darken_start, "Старт");
                lv_label_set_text(label_btn_darken_back, "Назад");
            } else {
                lv_label_set_text(label_darken_header, "Mode: Composite Darkening");
                lv_label_set_text(lv_obj_get_child(btn_darken_help, 0), "Help ?");
                lv_label_set_text(label_darken_title_params, "Parameter Setup");
                lv_label_set_text(label_darken_uv_exposure, "UV Diodes Work Time (1-15min):");
                lv_label_set_text(label_darken_cooling, "Fast Cooling (Compressed Air)");
                lv_label_set_text(label_btn_darken_start, "Start");
                lv_label_set_text(label_btn_darken_back, "Back");
            }

            help_blink_timer = lv_timer_create(help_blink_timer_cb, 1, btn_darken_help);
            apply_theme_to_lab_darken_screen();
            load_screen(screen_lab_darken);
            break;
        } // <<<--- ИСПРАВЛЕНИЕ: ДОБАВЛЕНА ЭТА СКОБКА
        default: {
            char msg[100];
            snprintf(msg, sizeof(msg), "Screen for Mode #%d is not yet implemented.", mode_id);
            show_info_dialog("Info", msg);
            if (screen_laboratory) load_screen(screen_laboratory);
            break;
        }
    }
}

// Обработчик для клика по одной из 6 плиток
static void lab_mode_tile_event_cb(lv_event_t * e) {
    if (lv_event_get_code(e) == LV_EVENT_CLICKED) {
        intptr_t mode_id_ptr = (intptr_t)lv_event_get_user_data(e);
        int mode_id = (int)mode_id_ptr;
        Serial.printf("Lab Mode tile clicked: Mode #%d\n", mode_id);
        
        if (help_blink_timer) {
            lv_timer_del(help_blink_timer);
            help_blink_timer = nullptr;
        }
        
        switch(mode_id) {
            case 1: { // Глазурь
                if(!screen_lab_glaze) break;

                // 1. Загружаем актуальные данные в UI
                char buf[10];
                snprintf(buf, sizeof(buf), "%.1f", current_lab_settings.glaze.uv_on_sec);
                lv_textarea_set_text(ta_glaze_uv_on, buf);
                snprintf(buf, sizeof(buf), "%.1f", current_lab_settings.glaze.uv_off_sec);
                lv_textarea_set_text(ta_glaze_uv_off, buf);
                snprintf(buf, sizeof(buf), "%d", current_lab_settings.glaze.monomer_blow_min);
                lv_textarea_set_text(ta_glaze_monomer_blow, buf);
                snprintf(buf, sizeof(buf), "%d", current_lab_settings.glaze.uv_exposure_sec);
                lv_textarea_set_text(ta_glaze_uv_exposure, buf);
                snprintf(buf, sizeof(buf), "%d", current_lab_settings.glaze.nitrogen_target_percent);
                lv_textarea_set_text(ta_glaze_nitrogen_target, buf);
                snprintf(buf, sizeof(buf), "%d", current_lab_settings.glaze.nitrogen_boost_sec);
                lv_textarea_set_text(ta_glaze_nitrogen_boost, buf);            

                // Устанавливаем состояние переключателя Обдува
                if (current_lab_settings.glaze.use_monomer_blow) lv_obj_add_state(sw_glaze_monomer_blow, LV_STATE_CHECKED);
                else lv_obj_clear_state(sw_glaze_monomer_blow, LV_STATE_CHECKED);
                
                // Сразу применяем видимость контейнера для обдува
                bool is_monomer_blow_active = lv_obj_has_state(sw_glaze_monomer_blow, LV_STATE_CHECKED);
                if (is_monomer_blow_active) {
                    lv_obj_set_style_opa(monomer_blow_container, LV_OPA_COVER, 0);
                    lv_obj_clear_flag(monomer_blow_container, LV_OBJ_FLAG_CLICKABLE);
                } else {
                    lv_obj_set_style_opa(monomer_blow_container, LV_OPA_TRANSP, 0);
                    lv_obj_add_flag(monomer_blow_container, LV_OBJ_FLAG_CLICKABLE);
                }

                // Устанавливаем состояние тройного переключателя UV
                lv_obj_t* glaze_uv_btns[] = {btn_glaze_uv_1, btn_glaze_uv_2, btn_glaze_uv_3};
                for(int i=0; i<3; i++) lv_obj_clear_state(glaze_uv_btns[i], LV_STATE_CHECKED);
                int uv_mode_idx = current_lab_settings.glaze.uv_mode;
                if (uv_mode_idx >= 0 && uv_mode_idx < 3) {
                    lv_obj_add_state(glaze_uv_btns[uv_mode_idx], LV_STATE_CHECKED);
                    // Имитируем клик, чтобы обновился текстовый статус
                    lv_event_send(glaze_uv_btns[uv_mode_idx], LV_EVENT_CLICKED, NULL); 
                }
                
                if (current_lab_settings.glaze.use_cooling) lv_obj_add_state(sw_glaze_cooling, LV_STATE_CHECKED);
                else lv_obj_clear_state(sw_glaze_cooling, LV_STATE_CHECKED);
                
                if (current_lab_settings.glaze.use_nitrogen) lv_obj_add_state(sw_glaze_nitrogen, LV_STATE_CHECKED);
                else lv_obj_clear_state(sw_glaze_nitrogen, LV_STATE_CHECKED);
                
                if (!current_global_settings.compressed_air_system_enabled) {
                    lv_obj_clear_state(sw_glaze_cooling, LV_STATE_CHECKED);
                }
                if (!current_global_settings.nitrogen_system_enabled) {
                    lv_obj_clear_state(sw_glaze_nitrogen, LV_STATE_CHECKED);
                }

                // Управляем видимостью поля для цели азота через ПРОЗРАЧНОСТЬ
                bool is_nitrogen_active = lv_obj_has_state(sw_glaze_nitrogen, LV_STATE_CHECKED);
                if (is_nitrogen_active) {
                    lv_obj_set_style_opa(nitrogen_glaze_container, LV_OPA_COVER, 0);
                    lv_obj_clear_flag(nitrogen_glaze_container, LV_OBJ_FLAG_CLICKABLE);
                    lv_obj_add_flag(label_glaze_nitrogen_warning, LV_OBJ_FLAG_HIDDEN);
                } else {
                    lv_obj_set_style_opa(nitrogen_glaze_container, LV_OPA_TRANSP, 0);
                    lv_obj_add_flag(nitrogen_glaze_container, LV_OBJ_FLAG_CLICKABLE);
                    lv_obj_clear_flag(label_glaze_nitrogen_warning, LV_OBJ_FLAG_HIDDEN);
                }

                // 2. Переводим все надписи
                if (current_global_settings.language == 1) { // RUS
                    lv_label_set_text(label_glaze_header, "Режим: Глазурь");
                    lv_label_set_text(lv_obj_get_child(btn_glaze_help, 0), "Справка по разделу ?");
                    lv_label_set_text(label_glaze_title_flicker, "Настройка такта мерцания");
                    lv_label_set_text(label_glaze_uv_on, "Вкл. диода (0.1-3с)");
                    lv_label_set_text(label_glaze_uv_off, "Выкл. диода (0.1-3с)");
                    lv_label_set_text(label_glaze_title_timers, "Настройка таймеров");
                    lv_label_set_text(label_glaze_uv_mode_title, "Тип:");
                    lv_label_set_text(label_glaze_monomer_blow, "Время (1-10мин)");
                    lv_label_set_text(label_glaze_uv_exposure, "Работа УФ (10-200с)");
                    lv_label_set_text(label_glaze_title_aux, "Вспомогательные параметры");
                    lv_label_set_text(label_glaze_monomer_blow_switch, "Обдув мономера");
                    lv_label_set_text(label_glaze_cooling, "Быстрое охлаждение");
                    lv_label_set_text(label_glaze_nitrogen, "Использование азота");
                    lv_label_set_text(label_glaze_nitrogen_target, "Цель N2 (%)");
                    lv_label_set_text(label_glaze_nitrogen_boost, "Доп. подача (1-5с)");
                    lv_label_set_text(label_btn_glaze_start, "Старт");
                    lv_label_set_text(label_btn_glaze_back, "Назад");
                } else { // ENG
                    lv_label_set_text(label_glaze_header, "Mode: Glaze");
                    lv_label_set_text(lv_obj_get_child(btn_glaze_help, 0), "Section Help ?");
                    lv_label_set_text(label_glaze_title_flicker, "Flicker Takt Setup");
                    lv_label_set_text(label_glaze_uv_on, "Diode ON (0.1-3s)");
                    lv_label_set_text(label_glaze_uv_off, "Diode OFF (0.1-3s)");
                    lv_label_set_text(label_glaze_title_timers, "Timers Setup");
                    lv_label_set_text(label_glaze_uv_mode_title, "Type:");
                    lv_label_set_text(label_glaze_monomer_blow, "Blow (1-10min)");
                    lv_label_set_text(label_glaze_uv_exposure, "UV Work (10-200s)");
                    lv_label_set_text(label_glaze_title_aux, "Auxiliary Parameters");
                    lv_label_set_text(label_glaze_monomer_blow_switch, "Monomer Blow");
                    lv_label_set_text(label_glaze_cooling, "Fast Cooling");
                    lv_label_set_text(label_glaze_nitrogen, "Use Nitrogen");
                    lv_label_set_text(label_glaze_nitrogen_target, "N2 Target (%)");
                    lv_label_set_text(label_glaze_nitrogen_boost, "Boost (1-5s)");
                    lv_label_set_text(label_btn_glaze_start, "Start");
                    lv_label_set_text(label_btn_glaze_back, "Back");
                }

                help_blink_timer = lv_timer_create(help_blink_timer_cb, 1, btn_glaze_help);

                apply_theme_to_lab_glaze_screen();

                load_screen(screen_lab_glaze);
                break;
            }
            case 2: { // Ремонт модели
                if(!screen_lab_repair) break;

                // 1. Загружаем актуальные данные в UI
                char buf[10];
                snprintf(buf, sizeof(buf), "%d", current_lab_settings.repair.countdown_sec);
                lv_textarea_set_text(ta_repair_countdown, buf);
                snprintf(buf, sizeof(buf), "%d", current_lab_settings.repair.uv_exposure_sec);
                lv_textarea_set_text(ta_repair_uv_exposure, buf);

                // 2. Переводим все надписи
                if (current_global_settings.language == 1) { // RUS
                    lv_label_set_text(label_repair_header, "Режим: Ремонт модели");
                    lv_label_set_text(lv_obj_get_child(btn_repair_help, 0), "Справка ?");
                    
                    // --- ИЗМЕНЕНИЯ ЗДЕСЬ ---
                    lv_obj_t* warning_header = lv_obj_get_child(label_repair_info_text->parent, 0);
                    lv_label_set_text(warning_header, "ВНИМАНИЕ!");
                    lv_label_set_text(label_repair_info_text, "Берегите глаза! Надевайте защитные очки и перчатки!");
                    // -----------------------

                    lv_label_set_text(label_repair_title_timers, "Настройки таймеров");
                    lv_label_set_text(label_repair_countdown, "Таймер обратного отсчета (5-60с):");
                    lv_label_set_text(label_repair_uv_exposure, "Таймер работы УФ диода (5-60с):");
                    lv_label_set_text(label_repair_arrow_text, "Поднесите модель к УФ диоду, который расположен под экраном засветки.");
                    lv_label_set_text(label_btn_repair_start, "Старт");
                    lv_label_set_text(label_btn_repair_back, "Назад");
                } else { // ENG
                    lv_label_set_text(label_repair_header, "Mode: Model Repair");
                    lv_label_set_text(lv_obj_get_child(btn_repair_help, 0), "Help ?");

                    // --- ИЗМЕНЕНИЯ ЗДЕСЬ ---
                    lv_obj_t* warning_header = lv_obj_get_child(label_repair_info_text->parent, 0);
                    lv_label_set_text(warning_header, "WARNING!");
                    lv_label_set_text(label_repair_info_text, "Protect your eyes! Wear safety glasses and gloves!");
                    // -----------------------

                    lv_label_set_text(label_repair_title_timers, "Timers Setup");
                    lv_label_set_text(label_repair_countdown, "Countdown Timer (5-60s):");
                    lv_label_set_text(label_repair_uv_exposure, "UV Diode Timer (5-60s):");
                    lv_label_set_text(label_repair_arrow_text, "Bring the model to the UV diode");
                    lv_label_set_text(label_btn_repair_start, "Start");
                    lv_label_set_text(label_btn_repair_back, "Back");
                }

                help_blink_timer = lv_timer_create(help_blink_timer_cb, 1, btn_repair_help);

                apply_theme_to_lab_repair_screen();

                load_screen(screen_lab_repair);
                break;
            }

            case 3: { // Повышение прочности
                if(!screen_lab_strength) break;

                // 1. Загружаем актуальные данные в UI
                char buf[10];
                snprintf(buf, sizeof(buf), "%d", current_lab_settings.strength.chamber_temp_c);
                lv_textarea_set_text(ta_strength_temp, buf);
                snprintf(buf, sizeof(buf), "%d", current_lab_settings.strength.hold_time_min);
                lv_textarea_set_text(ta_strength_hold_time, buf);
                // --- НОВЫЕ ПОЛЯ ---
                snprintf(buf, sizeof(buf), "%d", current_lab_settings.strength.uv_pulse_duration_sec);
                lv_textarea_set_text(ta_strength_uv_pulse_duration, buf);
                snprintf(buf, sizeof(buf), "%d", current_lab_settings.strength.uv_pulse_interval_min);
                lv_textarea_set_text(ta_strength_uv_pulse_interval, buf);
                // -----------------

                if (current_lab_settings.strength.use_cooling) {
                    lv_obj_add_state(sw_strength_cooling, LV_STATE_CHECKED);
                } else {
                    lv_obj_clear_state(sw_strength_cooling, LV_STATE_CHECKED);
                }

                if (!current_global_settings.compressed_air_system_enabled) {
                    lv_obj_clear_state(sw_strength_cooling, LV_STATE_CHECKED);
                }
                
                // 2. Переводим все надписи
                if (current_global_settings.language == 1) { // RUS
                    lv_label_set_text(label_strength_header, "Режим: Повышение прочности");
                    lv_label_set_text(lv_obj_get_child(btn_strength_help, 0), "Справка ?");
                    lv_label_set_text(label_strength_title_thermo, "Настройка термокамеры");
                    lv_label_set_text(label_strength_temp, "Температура (50-80C):");
                    lv_label_set_text(label_strength_hold_time, "Удержание (30-180мин):");
                    lv_label_set_text(label_strength_cooling, "Быстрое охлаждение");
                    lv_label_set_text(label_strength_title_uv, "Настройка УФ-импульсов");
                    lv_label_set_text(label_strength_uv_pulse_duration, "Длительность (1-10с):");
                    lv_label_set_text(label_strength_uv_pulse_interval, "Интервал (1-10мин):");
                    lv_label_set_text_fmt(label_strength_info_uv_text, "Подача УФ на %d сек каждые %d мин удержания.", current_lab_settings.strength.uv_pulse_duration_sec, current_lab_settings.strength.uv_pulse_interval_min);
                    lv_label_set_text(label_btn_strength_start, "Старт");
                    lv_label_set_text(label_btn_strength_back, "Назад");
                } else { // ENG
                    lv_label_set_text(label_strength_header, "Mode: Strength Boost");
                    lv_label_set_text(lv_obj_get_child(btn_strength_help, 0), "Help ?");
                    lv_label_set_text(label_strength_title_thermo, "Thermal Chamber Setup");
                    lv_label_set_text(label_strength_temp, "Temperature (50-80C):");
                    lv_label_set_text(label_strength_hold_time, "Hold Time (30-180min):");
                    lv_label_set_text(label_strength_cooling, "Fast Cooling");
                    lv_label_set_text(label_strength_title_uv, "UV Pulse Setup");
                    lv_label_set_text(label_strength_uv_pulse_duration, "Pulse duration (1-10s):");
                    lv_label_set_text(label_strength_uv_pulse_interval, "Pulse interval (1-10m):");
                    lv_label_set_text_fmt(label_strength_info_uv_text, "UV pulse for %d sec every %d min of hold time.", current_lab_settings.strength.uv_pulse_duration_sec, current_lab_settings.strength.uv_pulse_interval_min);
                    lv_label_set_text(label_btn_strength_start, "Start");
                    lv_label_set_text(label_btn_strength_back, "Back");
                }
                
                help_blink_timer = lv_timer_create(help_blink_timer_cb, 1, btn_strength_help);
                apply_theme_to_lab_strength_screen();
                load_screen(screen_lab_strength);
                break;
            }

            case 4: { // Термокамера
                if(!screen_lab_thermal) break;

                // 1. Загружаем актуальные данные в UI
                char buf[10];
                snprintf(buf, sizeof(buf), "%d", current_lab_settings.thermal.chamber_temp_c);
                lv_textarea_set_text(ta_thermal_temp, buf);
                snprintf(buf, sizeof(buf), "%d", current_lab_settings.thermal.hold_time_min);
                lv_textarea_set_text(ta_thermal_hold_time, buf);

                // 2. Переводим все надписи
                if (current_global_settings.language == 1) { // RUS
                    lv_label_set_text(label_thermal_header, "Режим: Термокамера");
                    lv_label_set_text(lv_obj_get_child(btn_thermal_help, 0), "Справка ?");
                    lv_label_set_text(label_thermal_title_thermo, "Настройка термокамеры");
                    lv_label_set_text(label_thermal_temp, "Температура в термокамере (40-60C):");
                    lv_label_set_text(label_thermal_info, "Рекомендуемая t для большинства смол: 50C");
                    lv_label_set_text(label_thermal_hold_time, "Время удержания (5-30мин):");
                    lv_label_set_text(label_btn_thermal_start, "Старт");
                    lv_label_set_text(label_btn_thermal_back, "Назад");
                } else { // ENG
                    lv_label_set_text(label_thermal_header, "Mode: Thermal Chamber");
                    lv_label_set_text(lv_obj_get_child(btn_thermal_help, 0), "Help ?");
                    lv_label_set_text(label_thermal_title_thermo, "Thermal Chamber Setup");
                    lv_label_set_text(label_thermal_temp, "Chamber Temperature (40-60C):");
                    lv_label_set_text(label_thermal_info, "Recommended temperature for most resins: 50C");
                    lv_label_set_text(label_thermal_hold_time, "Hold Time (5-30min):");
                    lv_label_set_text(label_btn_thermal_start, "Start");
                    lv_label_set_text(label_btn_thermal_back, "Back");
                }
                
                help_blink_timer = lv_timer_create(help_blink_timer_cb, 1, btn_thermal_help);
                apply_theme_to_lab_thermal_screen();
                load_screen(screen_lab_thermal);
                break;
            }
            
            case 5: { // Осветление
                if(!screen_lab_lighten) break;

                // 1. Загружаем данные
                char buf[10];
                snprintf(buf, sizeof(buf), "%d", current_lab_settings.lighten.hold_time_min);
                lv_textarea_set_text(ta_lighten_hold_time, buf);

                if (current_lab_settings.lighten.use_cooling) {
                    lv_obj_add_state(sw_lighten_cooling, LV_STATE_CHECKED);
                } else {
                    lv_obj_clear_state(sw_lighten_cooling, LV_STATE_CHECKED);
                }
                
                if (!current_global_settings.compressed_air_system_enabled) {
                    lv_obj_clear_state(sw_lighten_cooling, LV_STATE_CHECKED);
                }

                // 2. Перевод
                if (current_global_settings.language == 1) { // RUS
                    lv_label_set_text(label_lighten_header, "Режим: Осветление композита");
                    lv_label_set_text(lv_obj_get_child(btn_lighten_help, 0), "Справка ?");
                    lv_label_set_text(label_lighten_title_thermo, "Настройки термокамеры");
                    lv_label_set_text_fmt(label_lighten_temp_fixed, "Температура в камере: %dC (фиксированно)", current_lab_settings.lighten.chamber_temp_c);
                    lv_label_set_text(label_lighten_hold_time, "Время удержания (10-30мин):");
                    lv_label_set_text(label_lighten_cooling, "Быстрое охлаждение (сжатый воздух)");
                    lv_label_set_text(label_btn_lighten_start, "Старт");
                    lv_label_set_text(label_btn_lighten_back, "Назад");
                } else { // ENG
                    lv_label_set_text(label_lighten_header, "Mode: Composite Lightening");
                    lv_label_set_text(lv_obj_get_child(btn_lighten_help, 0), "Help ?");
                    lv_label_set_text(label_lighten_title_thermo, "Thermal Chamber Setup");
                    lv_label_set_text_fmt(label_lighten_temp_fixed, "Chamber Temperature: %dC (fixed)", current_lab_settings.lighten.chamber_temp_c);
                    lv_label_set_text(label_lighten_hold_time, "Hold Time (10-30min):");
                    lv_label_set_text(label_lighten_cooling, "Fast Cooling (Compressed Air)");
                    lv_label_set_text(label_btn_lighten_start, "Start");
                    lv_label_set_text(label_btn_lighten_back, "Back");
                }
                
                help_blink_timer = lv_timer_create(help_blink_timer_cb, 1, btn_lighten_help);
                apply_theme_to_lab_lighten_screen();
                load_screen(screen_lab_lighten);
                break;
            }
            case 6: { // Затемнение
                if(!screen_lab_darken) break;

                // 1. Загружаем данные
                char buf[10];
                snprintf(buf, sizeof(buf), "%d", current_lab_settings.darken.uv_exposure_min);
                lv_textarea_set_text(ta_darken_uv_exposure, buf);

                if (current_lab_settings.darken.use_cooling) {
                    lv_obj_add_state(sw_darken_cooling, LV_STATE_CHECKED);
                } else {
                    lv_obj_clear_state(sw_darken_cooling, LV_STATE_CHECKED);
                }

                if (!current_global_settings.compressed_air_system_enabled) {
                    lv_obj_clear_state(sw_darken_cooling, LV_STATE_CHECKED);
                }
                
                // 2. Перевод
                if (current_global_settings.language == 1) { // RUS
                    lv_label_set_text(label_darken_header, "Режим: Затемнение композита");
                    lv_label_set_text(lv_obj_get_child(btn_darken_help, 0), "Справка ?");
                    lv_label_set_text(label_darken_title_params, "Настройка параметров");
                    lv_label_set_text(label_darken_uv_exposure, "Время работы УФ диодов (1-15мин):");
                    lv_label_set_text(label_darken_cooling, "Быстрое охлаждение (сжатый воздух)");
                    lv_label_set_text(label_btn_darken_start, "Старт");
                    lv_label_set_text(label_btn_darken_back, "Назад");
                } else { // ENG
                    lv_label_set_text(label_darken_header, "Mode: Composite Darkening");
                    lv_label_set_text(lv_obj_get_child(btn_darken_help, 0), "Help ?");
                    lv_label_set_text(label_darken_title_params, "Parameter Setup");
                    lv_label_set_text(label_darken_uv_exposure, "UV Diodes Work Time (1-15min):");
                    lv_label_set_text(label_darken_cooling, "Fast Cooling (Compressed Air)");
                    lv_label_set_text(label_btn_darken_start, "Start");
                    lv_label_set_text(label_btn_darken_back, "Back");
                }

                help_blink_timer = lv_timer_create(help_blink_timer_cb, 1, btn_darken_help);
                apply_theme_to_lab_darken_screen();
                load_screen(screen_lab_darken);
                break;
            }
            default: {
                char msg[100];
                snprintf(msg, sizeof(msg), "Screen for Mode #%d is not yet implemented.", mode_id);
                show_info_dialog("Info", msg);
                break;
            }
        }
    }
}

static void uv_mode_selector_event_cb(lv_event_t * e) {
    lv_obj_t* clicked_btn = lv_event_get_target(e);
    lv_obj_t* label = (lv_obj_t*)lv_event_get_user_data(e);
    lv_obj_t* parent_container = lv_obj_get_parent(clicked_btn);

    if (!clicked_btn || !label || !parent_container) return; // Защита от ошибок

    // Определяем ID нажатой кнопки (0, 1 или 2)
    uint32_t id = 0;
    if (clicked_btn == lv_obj_get_child(parent_container, 0)) {
        id = 0;
    } else if (clicked_btn == lv_obj_get_child(parent_container, 1)) {
        id = 1;
    } else {
        id = 2;
    }
    
    // --- Теперь мы можем обновить UI ---
    const char* text_eng;
    const char* text_rus;

    switch(id) {
        case 0: text_eng = "type low"; text_rus = "низкая волна"; break;
        case 1: text_eng = "type high"; text_rus = "высокая волна"; break;
        case 2: text_eng = "both types"; text_rus = "комбо мощь"; break;
        default: text_eng = ""; text_rus = ""; break;
    }

    if (current_global_settings.language == 1) {
        lv_label_set_text(label, text_rus);
    } else {
        lv_label_set_text(label, text_eng);
    }
}

static void help_blink_timer_cb(lv_timer_t* timer) {
    // Получаем указатель на кнопку, которую нужно анимировать, из user_data таймера
    lv_obj_t* btn = (lv_obj_t*)timer->user_data;
    
    // Проверяем, что кнопка все еще существует. Если нет, удаляем таймер.
    if (!btn || !lv_obj_is_valid(btn)) {
        lv_timer_del(timer);
        help_blink_timer = nullptr;
        return;
    }

    // Для хранения состояния мигания (0 или 1) будем использовать user_data самой кнопки.
    // Это надежнее, чем хранить состояние в таймере.
    bool is_yellow = (bool)lv_obj_get_user_data(btn);

    if (!is_yellow) { // Если кнопка не желтая, делаем ее желтой
        lv_obj_set_style_bg_color(btn, lv_palette_main(LV_PALETTE_YELLOW), 0);
        timer->period = 1000; // Следующий вызов через 1 секунду
        lv_obj_set_user_data(btn, (void*)true); // Запоминаем, что она теперь желтая
    } else { // Если кнопка желтая, возвращаем ей серый цвет
        lv_obj_set_style_bg_color(btn, lv_palette_lighten(LV_PALETTE_GREY, 2), 0);
        timer->period = 2000; // Следующий вызов через 2 секунды
        lv_obj_set_user_data(btn, (void*)false); // Запоминаем, что она теперь серая
    }
}
static void help_button_event_cb(lv_event_t* e) {
    lv_event_code_t code = lv_event_get_code(e);
    if (code != LV_EVENT_CLICKED) return;

    const char* user_data = (const char*)lv_event_get_user_data(e);

    if (strcmp(user_data, "open_help") == 0) {
        Serial.println("Help button clicked, opening help screen.");
        if (screen_help) {
            // --- Перевод и заполнение контента ---
            if (current_global_settings.language == 1) { // RUS
                lv_label_set_text(label_help_title, "Справка по редактированию");
                lv_label_set_text(label_btn_help_close, "Понятно");
                lv_label_set_text(label_help_content, 
                    "1. Имя: Название профиля.\n\n"
                    "2. Термокамера: Включает нагрев до заданной температуры (40-80°C) и удержание (30-180с).\n\n"
                    "3. Азот: Включает продувку камеры азотом до целевой концентрации (95-99%). Требует включения в 'Настройках'.\n\n"
                    "4. Сжатый воздух: Включает охлаждение камеры. Требует включения в 'Настройках'.\n\n"
                    "5. Мерцания УФ: Первый этап облучения с мерцанием. 'Тип 1' и 'Тип 2' - разные длины волн. 'Тип 1' даёт затемнение, 'Тип 2' не дает затемнения. Можно выбрать частоту мерцаний (1-5).\n\n"
                    "6. Статичный УФ (Вторичный и Третичный): Этапы облучения постоянным светом. Можно выбрать, какие типы светодиодов будут активны.");
            } else { // ENG
                lv_label_set_text(label_help_title, "Editing Help");
                lv_label_set_text(label_btn_help_close, "Got it!");
                lv_label_set_text(label_help_content, 
                    "1. Name: The name of the profile.\n\n"
                    "2. Thermal Chamber: Enables heating to a target temperature (40-80°C) and holding it (30-180s).\n\n"
                    "3. Nitrogen: Enables purging the chamber with nitrogen to a target concentration (95-99%). Must be enabled in 'Settings'.\n\n"
                    "4. Compressed Air: Enables chamber cooling. Must be enabled in 'Settings'.\n\n"
                    "5. Primary UV: The first, flickering UV exposure stage. 'Type 1' and 'Type 2' are different wavelengths. Flicker rate (1-5) can be set.\n\n"
                    "6. Static UV (Secondary & Tertiary): Constant light exposure stages. You can select which LED types are active.");
            }
            apply_theme_to_help_screen();
            load_screen(screen_help);
        }
    } else if (strcmp(user_data, "close_help") == 0) {
        Serial.println("Closing help screen.");
        if (screen_profile_edit) {
            load_screen(screen_profile_edit);
        }
    }
}


void displayProfileListPage() {
    if (!list_profiles_main) {
        Serial.println("displayProfileListPage: list_profiles_main is NULL!");
        return;
    }
    // lv_obj_clean(list_profiles_main); // <<< ГЛАВНОЕ ИЗМЕНЕНИЕ: ЭТА СТРОКА УДАЛЕНА!

    char page_info_buffer[64];

    total_profile_pages = (all_profiles_data.size() + PROFILES_PER_PAGE - 1) / PROFILES_PER_PAGE;
    if (total_profile_pages == 0) total_profile_pages = 1;

    if (current_profile_list_page >= total_profile_pages) {
        current_profile_list_page = total_profile_pages - 1;
    }
    if (current_profile_list_page < 0) current_profile_list_page = 0;

    // apply_theme_to_main_app_screen(); // Применяем тему ко всему экрану

    if (all_profiles_data.empty()) {
        // Если профилей нет, просто прячем все 12 плиток
        for (int i = 0; i < PROFILES_PER_PAGE; i++) {
            lv_obj_add_flag(profile_tiles[i], LV_OBJ_FLAG_HIDDEN);
        }
        
        // Обновляем заголовок и кнопки (твой код здесь уже был правильным)
        if(list_header_label_main) {
            if (current_global_settings.language == 1) {
                lv_label_set_text(list_header_label_main, "Главное Меню (0/0)");
            } else {
                lv_label_set_text(list_header_label_main, "Main Menu (0/0)");
            }
        }
        if(btn_profiles_prev) lv_obj_add_state(btn_profiles_prev, LV_STATE_DISABLED);
        if(btn_profiles_next) lv_obj_add_state(btn_profiles_next, LV_STATE_DISABLED);
        return;
    }

    // --- ОСНОВНАЯ ЛОГИКА ОБНОВЛЕНИЯ ПУЛА ---
    int start_index = current_profile_list_page * PROFILES_PER_PAGE;

    for (int i = 0; i < PROFILES_PER_PAGE; i++) {
        int profile_index = start_index + i;

        if (profile_index < all_profiles_data.size()) {
            // Эта плитка нужна. Обновляем ее данные и показываем.
            const ProfileData& profile = all_profiles_data[profile_index];

            // 1. Обновляем текст
            lv_label_set_text(profile_tile_labels[i], profile.name);

            // 2. Обновляем данные для обработчика клика
            lv_obj_remove_event_cb(profile_tiles[i], profile_list_event_handler); // Безопаснее сначала удалить старый
            lv_obj_add_event_cb(profile_tiles[i], profile_list_event_handler, LV_EVENT_CLICKED, (void*)(intptr_t)profile.id);

            // // 3. Применяем тему для этой конкретной плитки
            // if (current_global_settings.theme == 1) { // Темная тема
            //     lv_obj_add_style(profile_tiles[i], &style_dark_tile_bg, 0);
            //     lv_obj_add_style(profile_tiles[i], &style_dark_block_border, 0);
            //     lv_obj_add_style(profile_tile_icons[i], &style_dark_text, 0);
            //     lv_obj_add_style(profile_tile_labels[i], &style_dark_text, 0);
            // } else { // Светлая тема
            //     lv_obj_add_style(profile_tiles[i], &style_light_tile_bg, 0);
            //     lv_obj_add_style(profile_tiles[i], &style_light_tile_border, 0);
            //     lv_obj_add_style(profile_tile_icons[i], &style_light_text, 0);
            //     lv_obj_add_style(profile_tile_labels[i], &style_light_text, 0);
            // }

            // 4. Делаем плитку видимой
            lv_obj_clear_flag(profile_tiles[i], LV_OBJ_FLAG_HIDDEN);

        } else {
            // Эта плитка не нужна на текущей странице, прячем ее
            lv_obj_add_flag(profile_tiles[i], LV_OBJ_FLAG_HIDDEN);
        }
    }

    // Обновляем заголовок и состояние кнопок
    if (list_header_label_main) {
        // Просто формируем заголовок со счетчиком страниц
        const char* title_text = (current_global_settings.language == 1) ? "Главное Меню" : "Main Menu";
        const char* page_text = (current_global_settings.language == 1) ? "Стр." : "Page";
        
        snprintf(page_info_buffer, sizeof(page_info_buffer), "%s (%s %d/%d)",
            title_text,
            page_text,
            current_profile_list_page + 1,
            total_profile_pages > 0 ? total_profile_pages : 1);
        lv_label_set_text(list_header_label_main, page_info_buffer);
    }

    if (total_profile_pages > 1) {
        // Если страниц больше одной, обе кнопки активны
        if (btn_profiles_prev) lv_obj_clear_state(btn_profiles_prev, LV_STATE_DISABLED);
        if (btn_profiles_next) lv_obj_clear_state(btn_profiles_next, LV_STATE_DISABLED);
    } else {
        // Если страница всего одна, деактивируем обе кнопки
        if (btn_profiles_prev) lv_obj_add_state(btn_profiles_prev, LV_STATE_DISABLED);
        if (btn_profiles_next) lv_obj_add_state(btn_profiles_next, LV_STATE_DISABLED);
    }
}

static void profile_list_prev_btn_event_cb(lv_event_t* e) {
    if (lv_event_get_code(e) == LV_EVENT_CLICKED) {
        // Если страниц нет или всего одна, ничего не делаем
        if (total_profile_pages <= 1) return;

        if (current_profile_list_page > 0) {
            // Если мы не на первой странице, просто идем назад
            current_profile_list_page--;
        } else {
            // Если мы на первой странице (page 0), перепрыгиваем на последнюю
            current_profile_list_page = total_profile_pages - 1;
        }

        log_memory_status("Before Prev Page Update");

        // Перерисовываем страницу
        lvgl_port_lock(-1);
        displayProfileListPage();
        lvgl_port_unlock();

        log_memory_status("After Prev Page Update");
    }
}

static void profile_list_next_btn_event_cb(lv_event_t* e) {
    if (lv_event_get_code(e) == LV_EVENT_CLICKED) {
        // Если страниц нет или всего одна, ничего не делаем
        if (total_profile_pages <= 1) return;

        if (current_profile_list_page < total_profile_pages - 1) {
            // Если мы не на последней странице, просто идем вперед
            current_profile_list_page++;
        } else {
            // Если мы на последней странице, перепрыгиваем на первую (page 0)
            current_profile_list_page = 0;
        }
        
        log_memory_status("Before Next Page Update");

        // Перерисовываем страницу
        lvgl_port_lock(-1);
        displayProfileListPage();
        lvgl_port_unlock();

        log_memory_status("After Next Page Update");
    }
}

void prepare_and_send_lab_command(int mode_id) {
    Serial.printf("--- START Lab Mode (ID: %d). Applying SAFE start sequence ---\n", mode_id);
    is_lab_mode_running = true;
    current_process_stage = "";

    // Определяем, на какой экран возвращаться после процесса
    lv_obj_t* screen_to_return = screen_laboratory;
    const char* process_title_eng = "Laboratory Process";
    const char* process_title_rus = "Лабораторный процесс";

    switch(mode_id) {
        case 1: screen_to_return = screen_lab_glaze; process_title_eng = "Starting: Glaze"; process_title_rus = "Запуск: Глазурь"; break;
        case 2: screen_to_return = screen_lab_repair; process_title_eng = "Starting: Repair"; process_title_rus = "Запуск: Ремонт"; break;
        case 3: screen_to_return = screen_lab_strength; process_title_eng = "Starting: Strength Boost"; process_title_rus = "Запуск: Повышение прочности"; break;
        case 4: screen_to_return = screen_lab_thermal; process_title_eng = "Starting: Thermal Chamber"; process_title_rus = "Запуск: Термокамера"; break;
        case 5: screen_to_return = screen_lab_lighten; process_title_eng = "Starting: Lightening"; process_title_rus = "Запуск: Осветление"; break;
        case 6: screen_to_return = screen_lab_darken; process_title_eng = "Starting: Darkening"; process_title_rus = "Запуск: Затемнение"; break;
    }
    screen_to_return_after_process = screen_to_return;

    // Настраиваем экран процесса
    const char* title = (current_global_settings.language == 1) ? process_title_rus : process_title_eng;
    lv_label_set_text(label_process_status_title, title);
    lv_label_set_text(label_process_status_detail, tr("Sending command to controller..."));
    lv_obj_clear_flag(btn_process_cancel, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_state(btn_process_cancel, LV_STATE_DISABLED);
    lv_obj_remove_local_style_prop(lv_obj_get_child(screen_process_execution, 0), LV_STYLE_BG_COLOR, 0);
    if (mode_id == 2) {
        Serial.println("Configuring process screen for Repair Mode.");
        // lv_obj_add_flag(spinner_process_execution, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(repair_mode_indicator_obj, LV_OBJ_FLAG_HIDDEN);
    } else {
        Serial.println("Configuring process screen for Standard Mode.");
        // lv_obj_clear_flag(spinner_process_execution, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(repair_mode_indicator_obj, LV_OBJ_FLAG_HIDDEN);
    }
    apply_theme_to_process_screen();
    load_screen(screen_process_execution);

    // Формируем и отправляем команду, используя ГЛОБАЛЬНЫЙ JsonDocument
    command_json_doc.clear(); // Очищаем
    JsonObject params = command_json_doc.createNestedObject("params");
    command_json_doc["command"] = "START_LAB_PROCESS";
    command_json_doc["mode_id"] = mode_id;

    // Наполняем JSON
    switch(mode_id) {
        case 1: params["uv_on_sec"] = current_lab_settings.glaze.uv_on_sec; params["uv_off_sec"] = current_lab_settings.glaze.uv_off_sec; params["use_monomer_blow"] = current_lab_settings.glaze.use_monomer_blow; params["monomer_blow_min"] = current_lab_settings.glaze.monomer_blow_min; params["uv_mode"] = current_lab_settings.glaze.uv_mode; params["uv_exposure_sec"] = current_lab_settings.glaze.uv_exposure_sec; params["use_cooling"] = current_lab_settings.glaze.use_cooling; params["use_nitrogen"] = current_lab_settings.glaze.use_nitrogen; params["nitrogen_target_percent"] = current_lab_settings.glaze.nitrogen_target_percent; params["nitrogen_boost_sec"] = current_lab_settings.glaze.nitrogen_boost_sec; break;
        case 2: params["countdown_sec"] = current_lab_settings.repair.countdown_sec; params["uv_exposure_sec"] = current_lab_settings.repair.uv_exposure_sec; break;
        case 3: params["chamber_temp_c"] = current_lab_settings.strength.chamber_temp_c; params["hold_time_min"] = current_lab_settings.strength.hold_time_min; params["use_cooling"] = current_lab_settings.strength.use_cooling; params["uv_pulse_duration_sec"] = current_lab_settings.strength.uv_pulse_duration_sec; params["uv_pulse_interval_min"] = current_lab_settings.strength.uv_pulse_interval_min; break;
        case 4: params["chamber_temp_c"] = current_lab_settings.thermal.chamber_temp_c; params["hold_time_min"] = current_lab_settings.thermal.hold_time_min; break;
        case 5: params["chamber_temp_c"] = current_lab_settings.lighten.chamber_temp_c; params["hold_time_min"] = current_lab_settings.lighten.hold_time_min; params["use_cooling"] = current_lab_settings.lighten.use_cooling; break;
        case 6: params["darken_uv_exposure_min"] = current_lab_settings.darken.uv_exposure_min; params["use_cooling"] = current_lab_settings.darken.use_cooling; break;
    }
    
    String output;
    serializeJson(command_json_doc, output);
    flush_serial_buffer();
    MySerial1.println(output);
    Serial.println("Lab process command sent:");
    Serial.println(output);

    main_process_running = true;
    process_start_time_ms = millis();
}
// Меняем void на bool
bool handle_save_new_profile_logic(const char* profile_input_name) {
    // --- Этап 1: Проверки входных данных (без изменений) ---
    if (strlen(profile_input_name) == 0) {
        show_info_dialog(tr("Error"), tr("Profile name cannot be empty!"));
        return false;
    }
    if (strlen(profile_input_name) >= sizeof(ProfileData::name)) {
        show_info_dialog(tr("Error"), tr("Profile name is too long!"));
        return false;
    }
    if (!sd_card_initialized) {
        show_info_dialog(tr("Error"), tr("SD Card not ready!"));
        return false;
    }
    
    lvgl_port_lock(-1);

    // --- Этап 2: Создание нового профиля со значениями по умолчанию ---
    ProfileData new_profile;
    memset(&new_profile, 0, sizeof(ProfileData)); // Очистка
    
    new_profile.id = current_profile_next_id;
    strncpy(new_profile.name, profile_input_name, sizeof(new_profile.name) - 1);
    
    // Значения по умолчанию
    new_profile.thermal_chamber_enabled = false;
    new_profile.thermal_chamber_temp = 40;
    new_profile.heat_exchange_hold_sec = 60;
    new_profile.nitrogen_use_enabled = false;
    new_profile.nitrogen_target_percent = 95;
    new_profile.nitrogen_boost_sec = 1;
    new_profile.primary_uv_exposure_sec = 30;
    new_profile.secondary_uv_exposure_sec = 60;
    new_profile.tertiary_uv_exposure_sec = 60;
    new_profile.chamber_cooling_enabled = false;
    new_profile.post_cooling_air_purge_sec = 0;
    new_profile.primary_uv_mode = 1;
    new_profile.primary_uv_flicker_on_sec = 0.2f;
    new_profile.secondary_uv_mode = 1;
    new_profile.tertiary_uv_mode = 1;

    // --- Этап 3: Обновление данных в памяти и сохранение на SD-карту ---
    // 1. Добавляем новый профиль в наш вектор в оперативной памяти
    all_profiles_data.push_back(new_profile);

    // 2. Вызываем новую централизованную функцию сохранения
    if (!saveConfiguration()) {
        show_info_dialog(tr("Save Error"), "Failed to write profile to SD card.");
        all_profiles_data.pop_back(); // Откатываем изменение в памяти, если запись не удалась
        lvgl_port_unlock();
        return false;
    }

    // 3. Успешно! Обновляем состояние программы
    current_profile_next_id++;
    needs_list_refresh = true;
    
    lvgl_port_unlock();
    return true;
}

// Не забудь поправить и прототип функции в начале файла!
// bool handle_save_new_profile_logic(const char* profile_input_name);
static void add_new_profile_btn_event_cb(lv_event_t * e) {
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;

    Serial.println("Add New Profile button clicked. Showing modal input...");
    // Очищаем фиктивное поле перед показом, на всякий случай
    if(ta_dummy_for_new_profile) {
        lv_textarea_set_text(ta_dummy_for_new_profile, "");
    }
    // Вызываем модальный ввод, ЦЕЛЬЮ которого будет наше фиктивное поле
    show_modal_input(ta_dummy_for_new_profile, LV_KEYBOARD_MODE_TEXT_LOWER);
}

static void profile_list_event_handler(lv_event_t * e) {
    lv_event_code_t code = lv_event_get_code(e);
    if (code == LV_EVENT_CLICKED) {
        
        int profile_id = (intptr_t)lv_event_get_user_data(e);
        current_selected_profile_id = profile_id;
        Serial.printf("Clicked on profile with ID: %d. Preparing detail screen.\n", profile_id);
        
        bool profile_found = false;
        for (const auto& profile : all_profiles_data) {
            if (profile.id == profile_id) {
                current_active_profile_data = profile;
                profile_found = true;
                break;
            }
        }
        
        lvgl_port_lock(-1);
        
        if (profile_found) {
            // Вызываем централизованную функцию для обновления экрана
            update_profile_details_screen(current_active_profile_data);
        } else { 
            // Обработка редкой ошибки, если профиль не нашелся в памяти
            if (label_detail_view_profile_name) lv_label_set_text_fmt(label_detail_view_profile_name, "Error: ID %d Not Found", profile_id); 
            if (label_detail_view_id) lv_label_set_text(label_detail_view_id, "ID: N/A (Error)");
            if (label_detail_view_thermal_chamber) lv_label_set_text(label_detail_view_thermal_chamber, "");
            if (label_detail_view_nitrogen) lv_label_set_text(label_detail_view_nitrogen, "");
            if (label_detail_view_chamber_cooling) lv_label_set_text(label_detail_view_chamber_cooling, "");
            if (label_detail_view_primary_uv) lv_label_set_text(label_detail_view_primary_uv, "");
            if (label_detail_view_secondary_uv) lv_label_set_text(label_detail_view_secondary_uv, "");
        }
        
        if (screen_profile_details) { 
            load_screen(screen_profile_details); 
        }
        lvgl_port_unlock();
    }
}

// Новая функция для асинхронного заполнения экрана
static void setup_profile_edit_screen() {
    // --- Этап 1: Находим актуальные данные профиля в памяти ---
    bool profile_found = false;
    // Ищем в нашем глобальном векторе профиль с нужным ID
    for (const auto& profile : all_profiles_data) {
        if (profile.id == current_selected_profile_id) {
            // Копируем найденные данные в рабочую структуру
            current_active_profile_data = profile; 
            profile_found = true;
            break;
        }
    }

    if (!profile_found) {
        show_info_dialog(tr("Error"), tr("Could not load profile data for editing."));
        load_screen(screen_profile_details);
        return;
    }
    
    Serial.printf("--- Preparing edit screen for profile: %s (ID: %d) ---\n", current_active_profile_data.name, current_active_profile_data.id);

    // --- Этап 2: Заполнение UI (этот код остается почти без изменений) ---
    if (screen_profile_edit) {
        char num_buf[10];

        // --- Шапка ---
        lv_textarea_set_text(ta_edit_profile_name, current_active_profile_data.name);

        // --- Левая колонка (Параметры УФ) ---
        snprintf(num_buf, sizeof(num_buf), "%d", current_active_profile_data.primary_uv_exposure_sec);
        lv_textarea_set_text(ta_edit_primary_uv, num_buf);
        snprintf(num_buf, sizeof(num_buf), "%.1f", current_active_profile_data.primary_uv_flicker_on_sec);
        lv_textarea_set_text(ta_edit_flicker_rate, num_buf);
        lv_obj_t* primary_btns[] = {btn_primary_1, btn_primary_2, btn_primary_3};
        for(int i=0; i<3; i++) lv_obj_clear_state(primary_btns[i], LV_STATE_CHECKED);
        if (current_active_profile_data.primary_uv_mode >= 0 && current_active_profile_data.primary_uv_mode < 3) {
            lv_obj_add_state(primary_btns[current_active_profile_data.primary_uv_mode], LV_STATE_CHECKED);
            lv_event_send(primary_btns[current_active_profile_data.primary_uv_mode], LV_EVENT_CLICKED, NULL);
        }

        snprintf(num_buf, sizeof(num_buf), "%d", current_active_profile_data.secondary_uv_exposure_sec);
        lv_textarea_set_text(ta_edit_secondary_uv, num_buf);
        lv_obj_t* secondary_btns[] = {btn_secondary_1, btn_secondary_2, btn_secondary_3};
        for(int i=0; i<3; i++) lv_obj_clear_state(secondary_btns[i], LV_STATE_CHECKED);
        if (current_active_profile_data.secondary_uv_mode >= 0 && current_active_profile_data.secondary_uv_mode < 3) {
            lv_obj_add_state(secondary_btns[current_active_profile_data.secondary_uv_mode], LV_STATE_CHECKED);
            lv_event_send(secondary_btns[current_active_profile_data.secondary_uv_mode], LV_EVENT_CLICKED, NULL);
        }
        
        snprintf(num_buf, sizeof(num_buf), "%d", current_active_profile_data.tertiary_uv_exposure_sec);
        lv_textarea_set_text(ta_edit_tertiary_uv, num_buf);
        lv_obj_t* tertiary_btns[] = {btn_tertiary_1, btn_tertiary_2, btn_tertiary_3};
        for(int i=0; i<3; i++) lv_obj_clear_state(tertiary_btns[i], LV_STATE_CHECKED);
        if (current_active_profile_data.tertiary_uv_mode >= 0 && current_active_profile_data.tertiary_uv_mode < 3) {
            lv_obj_add_state(tertiary_btns[current_active_profile_data.tertiary_uv_mode], LV_STATE_CHECKED);
            lv_event_send(tertiary_btns[current_active_profile_data.tertiary_uv_mode], LV_EVENT_CLICKED, NULL);
        }

        // --- Правая колонка (Дополнительные параметры) ---
        snprintf(num_buf, sizeof(num_buf), "%d", current_active_profile_data.nitrogen_target_percent);
        lv_textarea_set_text(ta_edit_nitrogen_target, num_buf); 
        snprintf(num_buf, sizeof(num_buf), "%d", current_active_profile_data.nitrogen_boost_sec);
        lv_textarea_set_text(ta_edit_nitrogen_boost, num_buf);     
        if (current_active_profile_data.nitrogen_use_enabled && current_global_settings.nitrogen_system_enabled) {
            lv_obj_add_state(sw_edit_nitrogen, LV_STATE_CHECKED);
        } else {
            lv_obj_clear_state(sw_edit_nitrogen, LV_STATE_CHECKED);
        }
        
        if (current_active_profile_data.chamber_cooling_enabled && current_global_settings.compressed_air_system_enabled) {
            lv_obj_add_state(sw_edit_chamber_cooling, LV_STATE_CHECKED);
        } else {
            lv_obj_clear_state(sw_edit_chamber_cooling, LV_STATE_CHECKED);
        }

        int purge_time = current_active_profile_data.post_cooling_air_purge_sec;
        uint16_t btn_id_to_check = 0; 
        if (purge_time == 40) btn_id_to_check = 1;
        else if (purge_time == 60) btn_id_to_check = 2;
        lv_btnmatrix_set_selected_btn(btnm_post_cooling_purge, btn_id_to_check);

        snprintf(num_buf, sizeof(num_buf), "%d", current_active_profile_data.thermal_chamber_temp);
        lv_textarea_set_text(ta_edit_thermal_temp, num_buf);
        snprintf(num_buf, sizeof(num_buf), "%d", current_active_profile_data.heat_exchange_hold_sec);
        lv_textarea_set_text(ta_edit_heat_hold, num_buf);
        if (current_active_profile_data.thermal_chamber_enabled) {
            lv_obj_add_state(sw_edit_thermal_chamber_enable, LV_STATE_CHECKED);
        } else {
            lv_obj_clear_state(sw_edit_thermal_chamber_enable, LV_STATE_CHECKED);
        }

        lv_event_send(sw_edit_nitrogen, LV_EVENT_VALUE_CHANGED, NULL);
        lv_event_send(sw_edit_thermal_chamber_enable, LV_EVENT_VALUE_CHANGED, NULL);
        lv_event_send(sw_edit_chamber_cooling, LV_EVENT_VALUE_CHANGED, NULL); 

        // --- Перевод надписей ---
        // (Этот блок остается без изменений)
        if (current_global_settings.language == 1) { // RUS
            lv_label_set_text(label_edit_name_title, "Имя:");
            lv_label_set_text(label_btn_help_section, "Справка по разделу ?");
            lv_label_set_text(label_post_cooling_title, "Время продувки (сек):");
            lv_label_set_text(header_uv_params, "Параметры UV излучения");
            lv_label_set_text(label_uv_primary_title, "1 этап:");
            lv_label_set_text(label_uv_secondary_title, "2 этап:");
            lv_label_set_text(label_uv_tertiary_title, "3 этап:");
            lv_label_set_text(label_edit_flicker_rate_title, "Время ВКЛ (0.1-1.0 сек):");
            lv_label_set_text(label_edit_primary_uv_time_title, "Время работы (1-60сек)");
            lv_label_set_text(label_edit_secondary_uv_time_title, "Время работы (1-200сек)");
            lv_label_set_text(label_edit_tertiary_uv_time_title, "Время работы (1-200сек)");
            lv_label_set_text(header_poly_params, "Доп. параметры полимеризации");
            lv_label_set_text(label_edit_nitrogen_title, "Использование Азота");
            lv_label_set_text(label_edit_nitrogen_target_title, "Концентрация N2 (90-99%):");
            lv_label_set_text(label_edit_nitrogen_boost_title, "Доп. подача (1-5с):");
            lv_label_set_text(label_edit_cooling_title, "Подача сжатого воздуха ");
            lv_label_set_text(label_edit_thermal_chamber_title, "Термокамера");
            lv_label_set_text(label_edit_thermal_temp_title, "T нагрева (30-80C):");
            lv_label_set_text(label_edit_heat_hold_title, "Удержание T (30-180сек): ");
            lv_label_set_text(label_btn_save, "Сохранить");
            lv_label_set_text(label_btn_cancel, "Отмена");
        } else { // ENG
            lv_label_set_text(label_edit_name_title, "Name:");
            lv_label_set_text(label_btn_help_section, "Section Help ?");
            lv_label_set_text(label_post_cooling_title, "Purge time (sec):");
            lv_label_set_text(header_uv_params, "UV Parameters");
            lv_label_set_text(label_uv_primary_title, "1st Stage:");
            lv_label_set_text(label_uv_secondary_title, "2nd Stage:");
            lv_label_set_text(label_uv_tertiary_title, "3rd Stage:");
            lv_label_set_text(label_edit_flicker_rate_title, "ON Time (0.1-1.0 sec):");
            lv_label_set_text(label_edit_primary_uv_time_title, "Exposure time (1-60s)");
            lv_label_set_text(label_edit_secondary_uv_time_title, "Exposure time (1-200s)");
            lv_label_set_text(label_edit_tertiary_uv_time_title, "Exposure time (1-200s)");
            lv_label_set_text(header_poly_params, "Additional Parameters");
            lv_label_set_text(label_edit_nitrogen_title, "Nitrogen Use");
            lv_label_set_text(label_edit_nitrogen_target_title, "Concentration N2 (90-99%):");
            lv_label_set_text(label_edit_nitrogen_boost_title, "Boost (1-5s):");
            lv_label_set_text(label_edit_cooling_title, "Compressed Air Use");
            lv_label_set_text(label_edit_thermal_chamber_title, "Thermal Chamber");
            lv_label_set_text(label_edit_thermal_temp_title, "Heating t (30-80C):");
            lv_label_set_text(label_edit_heat_hold_title, "Hold Time (30-180sec):");
            lv_label_set_text(label_btn_save, "Save Changes");
            lv_label_set_text(label_btn_cancel, "Cancel");
        }
        
        // --- Запуск таймера и загрузка экрана ---
        if (help_blink_timer) lv_timer_del(help_blink_timer);
        help_blink_timer = lv_timer_create(help_blink_timer_cb, 1, btn_help_section);

        apply_theme_to_profile_edit_screen();
        
        if (screen_profile_edit) {
            load_screen(screen_profile_edit);
        }
    }
}

static void profile_detail_edit_btn_event_cb(lv_event_t * e) {
    if (e && lv_event_get_code(e) != LV_EVENT_CLICKED) return;

    Serial.println("Edit button clicked. Preparing edit screen instantly...");
    
    // Просто напрямую вызываем функцию, которая готовит и показывает экран
    // Она быстрая, так как работает с данными из памяти
    setup_profile_edit_screen();
}

static void profile_detail_delete_btn_event_cb(lv_event_t * e) {
    lv_event_code_t code = lv_event_get_code(e);
    if (code == LV_EVENT_CLICKED) {
        Serial.printf("--- DELETE button clicked for profile: '%s' (ID: %d). Showing custom confirm dialog ---\n", current_active_profile_data.name, current_selected_profile_id);
        lvgl_port_lock(-1); 
        if (screen_confirm_delete_dialog && label_confirm_delete_text) {
            
            // --- БЛОК ПЕРЕВОДА ДИАЛОГА УДАЛЕНИЯ ---
            char confirm_dialog_msg_buffer[256];
            if (current_global_settings.language == 1) { // RUS
                lv_label_set_text(label_confirm_delete_title, "Подтвердите Удаление");
                lv_label_set_text(label_confirm_btn_cancel, "Отмена");
                lv_label_set_text(label_confirm_btn_delete, "Удалить");
                snprintf(confirm_dialog_msg_buffer, sizeof(confirm_dialog_msg_buffer), "Действительно удалить профиль\n'%s'?", current_active_profile_data.name); 
            } else { // ENG
                lv_label_set_text(label_confirm_delete_title, "Confirm Deletion");
                lv_label_set_text(label_confirm_btn_cancel, "Cancel");
                lv_label_set_text(label_confirm_btn_delete, "Delete");
                snprintf(confirm_dialog_msg_buffer, sizeof(confirm_dialog_msg_buffer), "Really delete profile\n'%s'?", current_active_profile_data.name); 
            }
            // ----------------------------------------

            lv_label_set_text(label_confirm_delete_text, confirm_dialog_msg_buffer);
            apply_theme_to_confirm_delete_dialog();
            lv_obj_clear_flag(screen_confirm_delete_dialog, LV_OBJ_FLAG_HIDDEN);
            lv_obj_move_foreground(screen_confirm_delete_dialog); 
            Serial.println("Custom confirm delete dialog shown.");
        } else { Serial.println("ERROR: Custom confirm delete dialog not initialized!"); }
        lvgl_port_unlock();
    }
}
static void profile_detail_close_event_cb(lv_event_t * e) {
    lv_event_code_t code = lv_event_get_code(e);
    if (code == LV_EVENT_CLICKED) {
        Serial.println("Close button on details screen clicked. Loading main screen (NO ANIMATION).");
        if (screen_main_app) { load_screen(screen_main_app); }
    }
}
static void confirm_dialog_cancel_btn_event_cb(lv_event_t* e) {
    if (screen_confirm_delete_dialog) {
        lv_obj_add_flag(screen_confirm_delete_dialog, LV_OBJ_FLAG_HIDDEN);
        Serial.println("Delete confirmation cancelled by user.");
    }
}
static void confirm_dialog_delete_btn_event_cb(lv_event_t* e) {
    Serial.printf("Deletion confirmed for profile ID: %d\n", current_selected_profile_id);
    if (screen_confirm_delete_dialog) { 
        lv_obj_add_flag(screen_confirm_delete_dialog, LV_OBJ_FLAG_HIDDEN); 
    }
    
    lvgl_port_lock(-1);

    // --- Этап 1: Удаление профиля из вектора в памяти ---
    bool found_and_erased = false;
    for (auto it = all_profiles_data.begin(); it != all_profiles_data.end(); ++it) {
        if (it->id == current_selected_profile_id) {
            all_profiles_data.erase(it);
            found_and_erased = true;
            break;
        }
    }

    if (found_and_erased) {
        // --- Этап 2: Сохранение обновленного списка на SD-карту ---
        if (!saveConfiguration()) {
            // В случае ошибки, лучше перезагрузить все профили с карты, чтобы восстановить консистентность
            show_info_dialog(tr("Save Error"), "Failed to update profiles file after deletion.");
            loadConfiguration();
        }
    } else {
        Serial.println("   WARNING: Profile to delete not found in memory vector.");
    }
    
    // --- Этап 3: Обновление UI и возврат на главный экран ---
    displayProfileListPage(); 
    if (screen_main_app) { 
        load_screen(screen_main_app); 
    } 
    
    lvgl_port_unlock(); 
}

static void thermal_temp_slider_event_cb(lv_event_t * e) {
    lv_obj_t* slider = lv_event_get_target(e);
    int slider_raw_val = lv_slider_get_value(slider);
    int actual_temp = 40 + slider_raw_val * 5; 

    if (ta_edit_thermal_temp) { 
        char temp_buffer[5]; // Буфер для строки (например, "80\0")
        snprintf(temp_buffer, sizeof(temp_buffer), "%d", actual_temp); // Форматируем число в строку
        lv_textarea_set_text(ta_edit_thermal_temp, temp_buffer); // Устанавливаем готовую строку
    }
}
static void alpha_textarea_focus_event_cb(lv_event_t* e) { 
    lv_obj_t * ta = lv_event_get_target(e);
    // Определяем, какой режим клавиатуры нужен (LOWER, UPPER, SPECIAL)
    // Для простоты пока всегда TEXT_LOWER для алфавитной
    show_modal_input(ta, LV_KEYBOARD_MODE_TEXT_LOWER); 
    Serial.println("Alpha textarea focused, showing modal input.");
}
static void numeric_textarea_focus_event_cb(lv_event_t * e) { 
    lv_obj_t * ta = lv_event_get_target(e);
    show_modal_input(ta, LV_KEYBOARD_MODE_NUMBER);
    Serial.println("Numeric textarea focused, showing modal input.");
}
static void generic_textarea_defocus_event_cb(lv_event_t * e) {
    lv_obj_t * ta = lv_event_get_target(e);

    // --- Поля с экрана Редактирования Профиля ---
    if (ta == ta_edit_primary_uv) { validate_numeric_input(e, 1, 60); } 
    else if (ta == ta_edit_secondary_uv) { validate_numeric_input(e, 1, 200); }
    else if (ta == ta_edit_tertiary_uv) { validate_numeric_input(e, 1, 200); }
    else if (ta == ta_edit_heat_hold) { validate_numeric_input(e, 30, 180); }
    else if (ta == ta_edit_flicker_rate) { validate_float_input(e, 0.1f, 1.0f); }
    else if (ta == ta_edit_thermal_temp) { validate_numeric_input(e, 30, 80); }
    else if (ta == ta_edit_nitrogen_target) { validate_numeric_input(e, 90, 99); }
    else if (ta == ta_edit_nitrogen_boost) { validate_numeric_input(e, 1, 5); }

    // --- Поля с экрана "Глазурь" ---
    else if (ta == ta_glaze_uv_on) { validate_float_input(e, 0.1f, 3.0f); }
    else if (ta == ta_glaze_uv_off) { validate_float_input(e, 0.1f, 3.0f); }
    else if (ta == ta_glaze_monomer_blow) { validate_numeric_input(e, 1, 10); }
    else if (ta == ta_glaze_uv_exposure) { validate_numeric_input(e, 10, 200); }
    else if (ta == ta_glaze_nitrogen_target) { validate_numeric_input(e, 90, 99); }
    else if (ta == ta_glaze_nitrogen_boost) { validate_numeric_input(e, 1, 5); }

    // --- Поля с экрана "Ремонт модели" ---
    else if (ta == ta_repair_countdown) { validate_numeric_input(e, 5, 60); }
    else if (ta == ta_repair_uv_exposure) { validate_numeric_input(e, 5, 60); }

    // --- Поля с экрана "Повышение прочности" ---
    else if (ta == ta_strength_temp) { validate_numeric_input(e, 50, 80); }
    else if (ta == ta_strength_hold_time) { validate_numeric_input(e, 30, 180); }
    else if (ta == ta_strength_uv_pulse_duration) { validate_numeric_input(e, 1, 10); }
    else if (ta == ta_strength_uv_pulse_interval) { validate_numeric_input(e, 1, 10); }

    // --- Поля с экрана "Термокамера" ---
    else if (ta == ta_thermal_temp) { validate_numeric_input(e, 40, 60); }
    else if (ta == ta_thermal_hold_time) { validate_numeric_input(e, 5, 30); }

    else if (ta == ta_lighten_hold_time) { validate_numeric_input(e, 10, 30); }
    else if (ta == ta_darken_uv_exposure) { validate_numeric_input(e, 1, 15); }
}

static void profile_edit_save_changes_btn_event_cb(lv_event_t * e) {
    Serial.println("Save Changes button clicked.");
    
    // --- Этап 1: Сбор данных с UI в новую структуру (без изменений) ---
    ProfileData edited_data;
    edited_data.id = current_active_profile_data.id;

    strncpy(edited_data.name, lv_textarea_get_text(ta_edit_profile_name), sizeof(edited_data.name) - 1);
    edited_data.name[sizeof(edited_data.name) - 1] = '\0'; 
    if (strlen(edited_data.name) == 0) { 
        show_info_dialog(tr("Input Error"), tr("Profile name cannot be empty."));
        return; 
    }

    edited_data.primary_uv_exposure_sec = constrain(atoi(lv_textarea_get_text(ta_edit_primary_uv)), 1, 60);
    edited_data.primary_uv_flicker_on_sec = atof(lv_textarea_get_text(ta_edit_flicker_rate));
    if (edited_data.primary_uv_flicker_on_sec < 0.1f) edited_data.primary_uv_flicker_on_sec = 0.1f;
    if (edited_data.primary_uv_flicker_on_sec > 1.0f) edited_data.primary_uv_flicker_on_sec = 1.0f;
    
    if(lv_obj_has_state(btn_primary_1, LV_STATE_CHECKED)) edited_data.primary_uv_mode = 0;
    else if(lv_obj_has_state(btn_primary_2, LV_STATE_CHECKED)) edited_data.primary_uv_mode = 1;
    else edited_data.primary_uv_mode = 2;

    edited_data.secondary_uv_exposure_sec = constrain(atoi(lv_textarea_get_text(ta_edit_secondary_uv)), 1, 200);
    if(lv_obj_has_state(btn_secondary_1, LV_STATE_CHECKED)) edited_data.secondary_uv_mode = 0;
    else if(lv_obj_has_state(btn_secondary_2, LV_STATE_CHECKED)) edited_data.secondary_uv_mode = 1;
    else edited_data.secondary_uv_mode = 2;

    edited_data.tertiary_uv_exposure_sec = constrain(atoi(lv_textarea_get_text(ta_edit_tertiary_uv)), 1, 200);
    if(lv_obj_has_state(btn_tertiary_1, LV_STATE_CHECKED)) edited_data.tertiary_uv_mode = 0;
    else if(lv_obj_has_state(btn_tertiary_2, LV_STATE_CHECKED)) edited_data.tertiary_uv_mode = 1;
    else edited_data.tertiary_uv_mode = 2;
    
    edited_data.nitrogen_use_enabled = lv_obj_has_state(sw_edit_nitrogen, LV_STATE_CHECKED);
    if (edited_data.nitrogen_use_enabled) {
        edited_data.nitrogen_target_percent = constrain(atoi(lv_textarea_get_text(ta_edit_nitrogen_target)), 90, 99);
        edited_data.nitrogen_boost_sec = constrain(atoi(lv_textarea_get_text(ta_edit_nitrogen_boost)), 1, 5);
    } else {
        edited_data.nitrogen_target_percent = current_active_profile_data.nitrogen_target_percent;
        edited_data.nitrogen_boost_sec = current_active_profile_data.nitrogen_boost_sec;
    }
    
    edited_data.chamber_cooling_enabled = lv_obj_has_state(sw_edit_chamber_cooling, LV_STATE_CHECKED);
    if (edited_data.chamber_cooling_enabled) {
        edited_data.post_cooling_air_purge_sec = current_active_profile_data.post_cooling_air_purge_sec;
    } else {
        edited_data.post_cooling_air_purge_sec = 0;
    }

    edited_data.thermal_chamber_enabled = lv_obj_has_state(sw_edit_thermal_chamber_enable, LV_STATE_CHECKED);
    if (edited_data.thermal_chamber_enabled) {
        edited_data.thermal_chamber_temp = constrain(atoi(lv_textarea_get_text(ta_edit_thermal_temp)), 30, 80);
        edited_data.heat_exchange_hold_sec = constrain(atoi(lv_textarea_get_text(ta_edit_heat_hold)), 30, 180);
    } else {
        edited_data.thermal_chamber_temp = current_active_profile_data.thermal_chamber_temp;
        edited_data.heat_exchange_hold_sec = current_active_profile_data.heat_exchange_hold_sec;
    }
    
    // --- Этап 2: Сохранение и переход ---
    if (help_blink_timer) {
        lv_timer_del(help_blink_timer);
        help_blink_timer = nullptr;
    }

    trigger_profile_save_action(edited_data);

}
static void profile_edit_cancel_btn_event_cb(lv_event_t * e) {
    Serial.println("Cancel Edit button clicked. Loading profile_details_screen without saving (NO ANIMATION).");
    if (screen_profile_details) {
        if (help_blink_timer) {
            lv_timer_del(help_blink_timer);
            help_blink_timer = nullptr;
        }
        lvgl_port_lock(-1);
        // Восстанавливаем отображение на экране деталей из current_active_profile_data (которое не было изменено в файле)
        if (label_detail_view_profile_name) lv_label_set_text(label_detail_view_profile_name, current_active_profile_data.name);
        if (label_detail_view_id) lv_label_set_text_fmt(label_detail_view_id, "ID: %d", current_active_profile_data.id);

        if (current_global_settings.language == 1) { // RUS
            const char* on_str = "ВКЛ";
            const char* off_str = "ВЫКЛ";
            lv_label_set_text(label_detail_header, "Имя и параметры профиля:");
            if (label_detail_view_thermal_chamber_enabled) lv_label_set_text_fmt(label_detail_view_thermal_chamber_enabled, "Термокамера: %s", current_active_profile_data.thermal_chamber_enabled ? on_str : off_str);
            if (label_detail_view_thermal_chamber) {
                if (current_active_profile_data.thermal_chamber_enabled) {
                    lv_obj_clear_flag(label_detail_view_thermal_chamber, LV_OBJ_FLAG_HIDDEN);
                    lv_label_set_text_fmt(label_detail_view_thermal_chamber, "Цель: %d C, Удержание: %d с", current_active_profile_data.thermal_chamber_temp, current_active_profile_data.heat_exchange_hold_sec);
                } else { lv_obj_add_flag(label_detail_view_thermal_chamber, LV_OBJ_FLAG_HIDDEN); }
            }
            if (label_detail_view_nitrogen) {
                if (current_active_profile_data.nitrogen_use_enabled) {
                    lv_label_set_text_fmt(label_detail_view_nitrogen, "Азот: ВКЛ (Цель: %d%%)", current_active_profile_data.nitrogen_target_percent);
                } else {
                    lv_label_set_text(label_detail_view_nitrogen, "Азот: ВЫКЛ");
                }
            }
            if (label_detail_view_chamber_cooling) {
                if (current_active_profile_data.chamber_cooling_enabled) {
                    // <<<--- ИСПРАВЛЕНИЕ: Добавляем %d и передаем переменную ---<<<
                    lv_label_set_text_fmt(label_detail_view_chamber_cooling, "Сжатый воздух: %s (Продувка: %dс)", on_str, current_active_profile_data.post_cooling_air_purge_sec);
                } else {
                    lv_label_set_text_fmt(label_detail_view_chamber_cooling, "Сжатый воздух: %s", off_str);
                }
            }
            if (label_detail_view_primary_uv) lv_label_set_text_fmt(label_detail_view_primary_uv, "Первичный Ультрафиолет (Мерцания): %d с", current_active_profile_data.primary_uv_exposure_sec);
            if (label_detail_view_secondary_uv) lv_label_set_text_fmt(label_detail_view_secondary_uv, "Вторичный Ультрафиолет (Статичный): %d с", current_active_profile_data.secondary_uv_exposure_sec);
            if (label_detail_view_tertiary_uv) lv_label_set_text_fmt(label_detail_view_tertiary_uv, "Третичный Ультрафиолет (Статичный): %d с", current_active_profile_data.tertiary_uv_exposure_sec);

        } else { // ENG
            const char* on_str = "ON";
            const char* off_str = "OFF";
            lv_label_set_text(label_detail_header, "Profile name and details:");
            if (label_detail_view_thermal_chamber_enabled) lv_label_set_text_fmt(label_detail_view_thermal_chamber_enabled, "Thermal Chamber: %s", current_active_profile_data.thermal_chamber_enabled ? on_str : off_str);
            if (label_detail_view_thermal_chamber) {
                if (current_active_profile_data.thermal_chamber_enabled) {
                    lv_obj_clear_flag(label_detail_view_thermal_chamber, LV_OBJ_FLAG_HIDDEN);
                    lv_label_set_text_fmt(label_detail_view_thermal_chamber, "Target Temp: %d C, Hold: %d s", current_active_profile_data.thermal_chamber_temp, current_active_profile_data.heat_exchange_hold_sec);
                } else { lv_obj_add_flag(label_detail_view_thermal_chamber, LV_OBJ_FLAG_HIDDEN); }
            }
            if (label_detail_view_nitrogen) {
                if (current_active_profile_data.nitrogen_use_enabled) {
                    lv_label_set_text_fmt(label_detail_view_nitrogen, "Nitrogen Use: ON (Target: %d%%)", current_active_profile_data.nitrogen_target_percent);
                } else {
                    lv_label_set_text(label_detail_view_nitrogen, "Nitrogen Use: OFF");
                }
            }
            if (label_detail_view_chamber_cooling) {
                if (current_active_profile_data.chamber_cooling_enabled) {
                    // <<<--- ИСПРАВЛЕНИЕ: Добавляем %d и передаем переменную ---<<<
                    lv_label_set_text_fmt(label_detail_view_chamber_cooling, "Compressed air: %s (Purge: %ds)", on_str, current_active_profile_data.post_cooling_air_purge_sec);
                } else {
                    lv_label_set_text_fmt(label_detail_view_chamber_cooling, "Compressed air: %s", off_str);
                }
            }
            if (label_detail_view_primary_uv) lv_label_set_text_fmt(label_detail_view_primary_uv, "Primary UV: %d s", current_active_profile_data.primary_uv_exposure_sec);
            if (label_detail_view_secondary_uv) lv_label_set_text_fmt(label_detail_view_secondary_uv, "Secondary UV: %d s", current_active_profile_data.secondary_uv_exposure_sec);
            if (label_detail_view_tertiary_uv) lv_label_set_text_fmt(label_detail_view_tertiary_uv, "Tertiary UV: %d s", current_active_profile_data.tertiary_uv_exposure_sec);

        }

        lvgl_port_unlock();
        load_screen(screen_profile_details);
    }
}

// Обработчик для кнопки "Cancel" на экране выполнения процесса
static void process_execution_cancel_btn_event_cb(lv_event_t * e) {
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;

    // Если мы уже в процессе отмены, ничего не делаем
    if (waiting_for_stop_ack) return;

    Serial.println("--- CANCEL button clicked. Entering 'Waiting for ACK' state. ---");

    // 1. Отправляем команду СТОП
    command_json_doc.clear();
    command_json_doc["command"] = "EMERGENCY_STOP";
    String output;
    serializeJson(command_json_doc, output);
    MySerial1.println(output);
    Serial.println("Sent command: EMERGENCY_STOP");

    // 2. Обновляем UI, чтобы показать пользователю, что идет отмена
    lvgl_port_lock(-1);
    lv_label_set_text(label_process_status_title, tr("Cancelling..."));
    lv_label_set_text(label_process_status_detail, ""); // Очищаем детали телеметрии
    lv_obj_add_state(btn_process_cancel, LV_STATE_DISABLED); // Блокируем кнопку от повторных нажатий
    lvgl_port_unlock();

    // 3. Взводим флаги для `loop()`
    waiting_for_stop_ack = true;
    stop_ack_timeout_start = millis();
    
    // ВАЖНО: Мы больше НЕ МЕНЯЕМ ЭКРАН и НЕ СБРАСЫВАЕМ флаг main_process_running здесь!
}

static void update_custom_toggle_4_ui(lv_obj_t* toggle_box, int active_index) {
    if (!toggle_box) return;
    for(int i = 0; i < 4; i++) {
        lv_obj_t* btn = lv_obj_get_child(toggle_box, i);
        if(!btn) continue;
        if(i == active_index) {
            lv_obj_set_style_bg_color(btn, lv_palette_main(LV_PALETTE_BLUE), 0);
            lv_obj_set_style_text_color(lv_obj_get_child(btn, 0), lv_color_white(), 0);
        } else {
             // Цвет для неактивных кнопок будет задан в функции темы
        }
    }
}

static void settings_screen_event_cb(lv_event_t * e) {
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t* target = lv_event_get_target(e);
    const char* user_data_str = (const char*)lv_event_get_user_data(e);

    if (code == LV_EVENT_VALUE_CHANGED) {
        if (user_data_str && strcmp(user_data_str, "nitro_sys") == 0) {
            current_global_settings.nitrogen_system_enabled = lv_obj_has_state(target, LV_STATE_CHECKED);
            Serial.printf("Settings: Global Nitrogen System %s\n", current_global_settings.nitrogen_system_enabled ? "Enabled" : "Disabled");
            
            // <<< ИЗМЕНЕНИЕ: Добавляем логику для кнопки "Тест" >>>
            if (current_global_settings.nitrogen_system_enabled) {
                lv_obj_clear_state(btn_settings_test_nitro, LV_STATE_DISABLED);
            } else {
                lv_obj_add_state(btn_settings_test_nitro, LV_STATE_DISABLED);
            }

        } else if (user_data_str && strcmp(user_data_str, "air_sys") == 0) {
            current_global_settings.compressed_air_system_enabled = lv_obj_has_state(target, LV_STATE_CHECKED);
            Serial.printf("Settings: Global Compressed Air System %s\n", current_global_settings.compressed_air_system_enabled ? "Enabled" : "Disabled");
            
            // <<< ИЗМЕНЕНИЕ: Добавляем логику для кнопки "Тест" >>>
            if (current_global_settings.compressed_air_system_enabled) {
                lv_obj_clear_state(btn_settings_test_air, LV_STATE_DISABLED);
            } else {
                lv_obj_add_state(btn_settings_test_air, LV_STATE_DISABLED);
            }
        }
    }
    
    if (code == LV_EVENT_CLICKED) {
        if (!user_data_str) return;

        if (strcmp(user_data_str, "back_save") == 0) {
            Serial.println("Settings: Save and Back button clicked.");
            saveConfiguration();
            needs_list_refresh = true;
            if (screen_main_app) {
                load_screen(screen_main_app); 
            }
        }
        else if (strcmp(user_data_str, "test_nitro") == 0) {
            Serial.println("Nitrogen test button clicked.");
            if (screen_test_nitrogen) {
                 is_nitrogen_test_active = false; // Сбрасываем состояние
                 lv_label_set_text(label_btn_nitrogen_test_press, tr("Start Supply"));
                 lv_label_set_text(label_nitrogen_test_header, tr("Nitrogen Supply Test"));
                 lv_label_set_text(label_btn_nitrogen_test_back, tr("Back"));
                 apply_theme_to_test_nitrogen_screen();
                 load_screen(screen_test_nitrogen);
            }
        }
        else if (strcmp(user_data_str, "test_air") == 0) {
            Serial.println("Air test button clicked.");
            if (screen_test_air) {
                 is_air_test_active = false; // Сбрасываем состояние
                 lv_label_set_text(label_btn_air_test_press, tr("Start Supply"));
                 lv_label_set_text(label_air_test_header, tr("Compressed Air Test"));
                 lv_label_set_text(label_btn_air_test_back, tr("Back"));
                 apply_theme_to_test_air_screen();
                 load_screen(screen_test_air);
            }
        }
        // --- Обработка клика по языку и теме ---
        else if (strcmp(user_data_str, "lang1") == 0) {
            current_global_settings.language = 0; // ENG
            update_custom_toggle_ui(lang_toggle_box, current_global_settings.language);
        }
        else if (strcmp(user_data_str, "lang2") == 0) {
            current_global_settings.language = 1; // RUS
            update_custom_toggle_ui(lang_toggle_box, current_global_settings.language);
        }
        else if (strcmp(user_data_str, "theme1") == 0) {
            current_global_settings.theme = 0; // Light
            update_custom_toggle_ui(theme_toggle_box, current_global_settings.theme);
            update_timeout_toggle_ui(timeout_toggle_box, current_global_settings.screen_timeout_mode);
            apply_current_theme_to_all_screens();

        }
        else if (strcmp(user_data_str, "theme2") == 0) {
            current_global_settings.theme = 1; // Dark
            update_custom_toggle_ui(theme_toggle_box, current_global_settings.theme);
            update_timeout_toggle_ui(timeout_toggle_box, current_global_settings.screen_timeout_mode);
            apply_current_theme_to_all_screens();
        }
        else if (strcmp(user_data_str, "timeout0") == 0) {
            current_global_settings.screen_timeout_mode = 0;
            update_timeout_toggle_ui(timeout_toggle_box, current_global_settings.screen_timeout_mode);
        }
        else if (strcmp(user_data_str, "timeout1") == 0) {
            current_global_settings.screen_timeout_mode = 1;
            update_timeout_toggle_ui(timeout_toggle_box, current_global_settings.screen_timeout_mode);
        }
        else if (strcmp(user_data_str, "timeout2") == 0) {
            current_global_settings.screen_timeout_mode = 2;
            update_timeout_toggle_ui(timeout_toggle_box, current_global_settings.screen_timeout_mode);
        }
        else if (strcmp(user_data_str, "timeout3") == 0) {
            current_global_settings.screen_timeout_mode = 3;
            update_timeout_toggle_ui(timeout_toggle_box, current_global_settings.screen_timeout_mode);
        }
    }
}
static void btn_goto_settings_event_cb(lv_event_t* e) {
    if (lv_event_get_code(e) == LV_EVENT_CLICKED) {
        Serial.println("Settings button on main screen clicked. Updating UI from memory...");
        if (screen_settings) {
            lvgl_port_lock(-1); 

            // Обновляем UI на основе данных, которые уже есть в памяти
            if (sw_settings_global_nitrogen_enabled) {
                if (current_global_settings.nitrogen_system_enabled) lv_obj_add_state(sw_settings_global_nitrogen_enabled, LV_STATE_CHECKED);
                else lv_obj_clear_state(sw_settings_global_nitrogen_enabled, LV_STATE_CHECKED);
            }
            if (sw_settings_global_air_enabled) {
                if (current_global_settings.compressed_air_system_enabled) lv_obj_add_state(sw_settings_global_air_enabled, LV_STATE_CHECKED);
                else lv_obj_clear_state(sw_settings_global_air_enabled, LV_STATE_CHECKED);
            }

            if (current_global_settings.nitrogen_system_enabled) {
                lv_obj_clear_state(btn_settings_test_nitro, LV_STATE_DISABLED);
            } else {
                lv_obj_add_state(btn_settings_test_nitro, LV_STATE_DISABLED);
            }
            if (current_global_settings.compressed_air_system_enabled) {
                lv_obj_clear_state(btn_settings_test_air, LV_STATE_DISABLED);
            } else {
                lv_obj_add_state(btn_settings_test_air, LV_STATE_DISABLED);
            }
            
            update_custom_toggle_ui(lang_toggle_box, current_global_settings.language);
            update_custom_toggle_ui(theme_toggle_box, current_global_settings.theme);
            update_timeout_toggle_ui(timeout_toggle_box, current_global_settings.screen_timeout_mode);
            
            if (current_global_settings.language == 1) { // 1 = RUS
                if (label_settings_title) { lv_label_set_text(label_settings_title, "Общие Настройки"); lv_obj_add_style(label_settings_title, &style_my_text_22, 0); }
                if (label_settings_nitrogen) { lv_label_set_text(label_settings_nitrogen, "Система Азота:"); lv_obj_add_style(label_settings_nitrogen, &style_my_text_18, 0); }
                if (label_settings_air) { lv_label_set_text(label_settings_air, "Сжатый Воздух:"); lv_obj_add_style(label_settings_air, &style_my_text_18, 0); }
                if (label_settings_language) { lv_label_set_text(label_settings_language, "Язык:"); lv_obj_add_style(label_settings_language, &style_my_text_18, 0); }
                if (label_settings_theme) { lv_label_set_text(label_settings_theme, "Тема:"); lv_obj_add_style(label_settings_theme, &style_my_text_18, 0); }
                lv_label_set_text(label_settings_timeout, "Автоотключение экрана:");
                lv_obj_add_style(label_settings_timeout, &style_my_text_18, 0);
                lv_label_set_text(lv_obj_get_child(lv_obj_get_child(timeout_toggle_box, 0), 0), "5 мин");
                lv_label_set_text(lv_obj_get_child(lv_obj_get_child(timeout_toggle_box, 1), 0), "15 мин");
                lv_label_set_text(lv_obj_get_child(lv_obj_get_child(timeout_toggle_box, 2), 0), "30 мин");
                lv_label_set_text(lv_obj_get_child(lv_obj_get_child(timeout_toggle_box, 3), 0), "60 мин");
                lv_label_set_text(label_settings_lang_opt1, "АНГЛ");
                lv_obj_add_style(label_settings_lang_opt1, &style_my_text_16, 0);
                lv_label_set_text(label_settings_lang_opt2, "РУС");
                lv_obj_add_style(label_settings_lang_opt2, &style_my_text_16, 0);
                lv_label_set_text(label_settings_theme_opt1, "Светлая");
                lv_obj_add_style(label_settings_theme_opt1, &style_my_text_16, 0);
                lv_label_set_text(label_settings_theme_opt2, "Темная");
                lv_obj_add_style(label_settings_theme_opt2, &style_my_text_16, 0);
                lv_obj_t* lbl_test_nitro = lv_obj_get_child(lv_obj_get_child(lv_obj_get_child(lv_obj_get_child(content_grid_settings, 0), 0), 2), 0);
                lv_obj_t* lbl_test_air = lv_obj_get_child(lv_obj_get_child(lv_obj_get_child(lv_obj_get_child(content_grid_settings, 0), 1), 2), 0);
                lv_label_set_text(lbl_test_nitro, tr("Test"));
                lv_label_set_text(lbl_test_air, tr("Test"));
                if (btn_settings_save_and_back) {
                    lv_obj_t* label = lv_obj_get_child(btn_settings_save_and_back, 0);
                    if (label) {
                        lv_label_set_text(label, "Сохранить и Выйти");
                        lv_obj_add_style(label, &style_my_text_18_white, 0);
                    }
                }
            } else { // 0 = ENG
                if (label_settings_title) { lv_label_set_text(label_settings_title, "Global Settings"); lv_obj_add_style(label_settings_title, &style_my_text_22, 0); }
                if (label_settings_nitrogen) { lv_label_set_text(label_settings_nitrogen, "Nitrogen System:"); lv_obj_add_style(label_settings_nitrogen, &style_my_text_18, 0); }
                if (label_settings_air) { lv_label_set_text(label_settings_air, "Compressed Air:"); lv_obj_add_style(label_settings_air, &style_my_text_18, 0); }
                if (label_settings_language) { lv_label_set_text(label_settings_language, "Language:"); lv_obj_add_style(label_settings_language, &style_my_text_18, 0); }
                if (label_settings_theme) { lv_label_set_text(label_settings_theme, "Theme:"); lv_obj_add_style(label_settings_theme, &style_my_text_18, 0); }
                lv_label_set_text(label_settings_timeout, "Screen auto-off:");
                lv_obj_add_style(label_settings_timeout, &style_my_text_18, 0);
                lv_label_set_text(lv_obj_get_child(lv_obj_get_child(timeout_toggle_box, 0), 0), "5 min");
                lv_label_set_text(lv_obj_get_child(lv_obj_get_child(timeout_toggle_box, 1), 0), "15 min");
                lv_label_set_text(lv_obj_get_child(lv_obj_get_child(timeout_toggle_box, 2), 0), "30 min");
                lv_label_set_text(lv_obj_get_child(lv_obj_get_child(timeout_toggle_box, 3), 0), "60 min");
                lv_label_set_text(label_settings_lang_opt1, "ENG");
                lv_obj_add_style(label_settings_lang_opt1, &style_my_text_16, 0);
                lv_label_set_text(label_settings_lang_opt2, "RUS");
                lv_obj_add_style(label_settings_lang_opt2, &style_my_text_16, 0);
                lv_label_set_text(label_settings_theme_opt1, "Light");
                lv_obj_add_style(label_settings_theme_opt1, &style_my_text_16, 0);
                lv_label_set_text(label_settings_theme_opt2, "Dark");
                lv_obj_add_style(label_settings_theme_opt2, &style_my_text_16, 0);
                lv_obj_t* lbl_test_nitro = lv_obj_get_child(lv_obj_get_child(lv_obj_get_child(lv_obj_get_child(content_grid_settings, 0), 0), 2), 0);
                lv_obj_t* lbl_test_air = lv_obj_get_child(lv_obj_get_child(lv_obj_get_child(lv_obj_get_child(content_grid_settings, 0), 1), 2), 0);
                lv_label_set_text(lbl_test_nitro, "Test");
                lv_label_set_text(lbl_test_air, "Test");
                if (btn_settings_save_and_back) {
                    lv_obj_t* label = lv_obj_get_child(btn_settings_save_and_back, 0);
                    if (label) {
                        lv_label_set_text(label, "Save and Back");
                        lv_obj_add_style(label, &style_my_text_18_white, 0);
                    }
                }
            }

            if(label_settings_lang_opt1) lv_obj_add_style(label_settings_lang_opt1, &style_my_text_16, 0);
            if(label_settings_lang_opt2) lv_obj_add_style(label_settings_lang_opt2, &style_my_text_16, 0);
            if(label_settings_theme_opt1) lv_obj_add_style(label_settings_theme_opt1, &style_my_text_16, 0);
            if(label_settings_theme_opt2) lv_obj_add_style(label_settings_theme_opt2, &style_my_text_16, 0);
            
            apply_theme_to_settings_screen();

            load_screen(screen_settings);
            lvgl_port_unlock();
        }
    }
}

static void secret_button_event_cb(lv_event_t* e) {
    if (lv_event_get_code(e) == LV_EVENT_CLICKED) {
        Serial.println("Secret button clicked! Loading secret game screen...");
        if (screen_secret_game) {
            apply_theme_to_casino_screen();
            // Здесь можно передать какие-то параметры в игру, если нужно
            load_screen(screen_secret_game);
        }
    }
}

static void info_dialog_ok_event_cb(lv_event_t* e) {
    if (lv_event_get_code(e) == LV_EVENT_CLICKED) {
        if (screen_info_dialog) {
            lv_obj_add_flag(screen_info_dialog, LV_OBJ_FLAG_HIDDEN);
        }
    }
}

static void choice_dialog_event_cb(lv_event_t* e) {
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;

    lv_obj_t* btn = lv_event_get_target(e);
    
    // Сначала прячем диалоговое окно в любом случае
    if (screen_choice_dialog) {
        lv_obj_add_flag(screen_choice_dialog, LV_OBJ_FLAG_HIDDEN);
    }

    if (btn == btn_choice_dialog_skip) {
        Serial.println("Choice Dialog: User selected SKIP NITROGEN.");
        command_json_doc.clear();
        command_json_doc["command"] = "USER_CHOSE_TO_SKIP_NITROGEN";
        String output;
        serializeJson(command_json_doc, output);
        MySerial1.println(output);

        // Обновляем UI, чтобы пользователь видел, что что-то происходит
        lv_label_set_text(label_process_status_title, tr("Continuing process..."));
        lv_label_set_text(label_process_status_detail, tr("Nitrogen purge stage skipped."));

    } else if (btn == btn_choice_dialog_cancel) {
        Serial.println("Choice Dialog: User selected CANCEL PROCESS.");
        command_json_doc.clear();
        command_json_doc["command"] = "EMERGENCY_STOP";
        String output;
        serializeJson(command_json_doc, output);
        MySerial1.println(output);
        main_process_running = false; 
        is_lab_mode_running = false;
        current_process_stage = "";
        if (screen_to_return_after_process) {
            load_screen(screen_to_return_after_process);
        } else {
            load_screen(screen_main_app);
        }
        flush_serial_buffer();
    }
}

void build_info_dialog(lv_obj_t* parent_layer) {
    if (screen_info_dialog) return; 

    screen_info_dialog = lv_obj_create(parent_layer);
    lv_obj_add_flag(screen_info_dialog, LV_OBJ_FLAG_HIDDEN); 
    lv_obj_set_size(screen_info_dialog, lv_pct(75), LV_SIZE_CONTENT);
    lv_obj_center(screen_info_dialog); 
    lv_obj_set_style_bg_color(screen_info_dialog, lv_color_white(), 0);
    lv_obj_set_style_border_width(screen_info_dialog, 1, 0);
    lv_obj_set_style_border_color(screen_info_dialog, lv_palette_main(LV_PALETTE_GREY), 0);
    lv_obj_set_style_shadow_width(screen_info_dialog, 8, 0);
    lv_obj_set_style_shadow_opa(screen_info_dialog, LV_OPA_50, 0);
    lv_obj_set_style_radius(screen_info_dialog, 5, 0);
    lv_obj_set_style_pad_all(screen_info_dialog, 15, 0);
    lv_obj_set_flex_flow(screen_info_dialog, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(screen_info_dialog, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_gap(screen_info_dialog, 10, 0);

    label_info_dialog_text = lv_label_create(screen_info_dialog);
    lv_label_set_long_mode(label_info_dialog_text, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(label_info_dialog_text, lv_pct(100));
    lv_obj_set_style_text_align(label_info_dialog_text, LV_TEXT_ALIGN_CENTER, 0);
    
    // <-- ЗАДАЧА 3: ПРИМЕНЯЕМ СТИЛЬ ДЛЯ ПОДДЕРЖКИ РУССКОГО ЯЗЫКА
    lv_obj_add_style(label_info_dialog_text, &style_my_text_18, 0);

    btn_info_dialog_ok = lv_btn_create(screen_info_dialog);
    lv_obj_add_event_cb(btn_info_dialog_ok, info_dialog_ok_event_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_set_width(btn_info_dialog_ok, lv_pct(50));
    lv_obj_t* label_ok_txt = lv_label_create(btn_info_dialog_ok); 
    lv_label_set_text(label_ok_txt, "OK");
    // <-- ЗАДАЧА 3 (бонус): Делаем кнопку консистентной с другими
    lv_obj_add_style(label_ok_txt, &style_my_text_18_white, 0); 
    lv_obj_center(label_ok_txt);

    Serial.println("Universal Info Dialog UI built.");
}

static void service_lock_screen_event_cb(lv_event_t* e) {
    lv_event_code_t code = lv_event_get_code(e);
    if (code != LV_EVENT_CLICKED) return;

    const char* user_data = (const char*)lv_event_get_user_data(e);
    
    // Показать клавиатуру
    if (user_data && strcmp(user_data, "show_modal") == 0) {
        Serial.println("Service code input clicked. Switching to NUMERIC keyboard screen.");
        lvgl_port_lock(-1);
        screen_to_return_to = screen_service_lock;
        current_target_ta = ta_service_code_input;
        lv_textarea_set_text(ta_keyboard_proxy, lv_textarea_get_text(ta_service_code_input));
        lv_textarea_set_placeholder_text(ta_keyboard_proxy, tr("Enter code here..."));
        lv_textarea_set_max_length(ta_keyboard_proxy, 4);
        
        // Показываем ТОЛЬКО цифровую клавиатуру
        lv_obj_add_flag(kb_edit_alpha, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(kb_edit_numeric, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(kb_service_code, LV_OBJ_FLAG_HIDDEN);

        load_screen(screen_keyboard);
        lvgl_port_unlock();
    }
    // Проверить код
    else if (user_data && strcmp(user_data, "check_code") == 0) {
        
        const char* entered_code_c = lv_textarea_get_text(ta_service_code_input);
        bool key_is_valid = false;
        String valid_key_found = "";

        // Используем простое и надежное сравнение C-строк
        for (const auto& key : service_keys) {
            if (strcmp(key.c_str(), entered_code_c) == 0) { // Используем strcmp
                key_is_valid = true;
                valid_key_found = key;
                break;
            }
        }

        if (key_is_valid) {
            Serial.printf("Valid service key '%s' entered. Unlocking device.\n", valid_key_found.c_str());
            
            // 1. Снимаем флаг ошибки в памяти
            current_global_settings.is_heater_error = false;
            // 2. Удаляем ключ из памяти и вызываем saveConfiguration() для записи изменений
            remove_and_save_service_keys(valid_key_found);
            
            // 3. Переключаемся на главный экран
            load_screen(screen_main_app);
            
            // 4. Перезагружаем все данные из только что сохраненного файла и обновляем UI
            lvgl_port_lock(-1);
            if (sd_card_initialized) {
                Serial.println("Device unlocked. Reloading configuration and updating UI...");
                loadConfiguration(); // <<< ИСПРАВЛЕНО
                displayProfileListPage();
            } else {
                if (list_profiles_main) lv_list_add_text(list_profiles_main, "SD Card Error!");
            }
            lvgl_port_unlock();
        } else {
            Serial.printf("Invalid service key '%s' entered.\n", entered_code_c);
            lv_textarea_set_text(ta_service_code_input, "");
            lv_label_set_text(label_service_lock_msg, tr("Invalid code! Please try again."));
        }
    }
}
static void build_service_lock_screen(lv_obj_t* parent_screen) {
    screen_service_lock = parent_screen;

    lv_obj_set_style_bg_color(screen_service_lock, lv_palette_main(LV_PALETTE_RED), 0);
    lv_obj_set_style_bg_grad_color(screen_service_lock, lv_palette_darken(LV_PALETTE_RED, 4), 0);
    lv_obj_set_style_bg_grad_dir(screen_service_lock, LV_GRAD_DIR_VER, 0);

    lv_obj_t* content_cont = lv_obj_create(screen_service_lock);
    lv_obj_remove_style_all(content_cont);
    lv_obj_set_size(content_cont, lv_pct(100), lv_pct(100));
    lv_obj_set_flex_flow(content_cont, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(content_cont, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_gap(content_cont, 20, 0);

    // Заголовок
    lv_obj_t* title = lv_label_create(content_cont);
    lv_obj_set_style_text_color(title, lv_color_white(), 0);
    lv_obj_set_style_text_font(title, &montserrat_rus_22, 0);

    // Сообщение об ошибке
    label_service_lock_msg = lv_label_create(content_cont);
    lv_obj_set_style_text_color(label_service_lock_msg, lv_color_white(), 0);
    lv_obj_set_style_text_align(label_service_lock_msg, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_long_mode(label_service_lock_msg, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(label_service_lock_msg, lv_pct(90));
    lv_obj_set_style_text_font(label_service_lock_msg, &montserrat_rus_18, 0);

    // Поле для ввода кода
    ta_service_code_input = lv_textarea_create(content_cont);
    lv_textarea_set_one_line(ta_service_code_input, true);
    lv_textarea_set_max_length(ta_service_code_input, 4);
    lv_obj_set_width(ta_service_code_input, 180);
    lv_textarea_set_align(ta_service_code_input, LV_TEXT_ALIGN_CENTER);
    lv_obj_set_style_text_font(ta_service_code_input, &montserrat_rus_18, 0);
    lv_obj_add_event_cb(ta_service_code_input, service_lock_screen_event_cb, LV_EVENT_CLICKED, (void*)"show_modal");

    // --- Кнопка "Enter" для проверки кода ---
    btn_service_lock_enter = lv_btn_create(content_cont);
    lv_obj_set_width(btn_service_lock_enter, 180);
    lv_obj_add_event_cb(btn_service_lock_enter, service_lock_screen_event_cb, LV_EVENT_CLICKED, (void*)"check_code");
    lv_obj_t* label_enter = lv_label_create(btn_service_lock_enter);
    lv_obj_center(label_enter);
    lv_obj_add_style(label_enter, &style_my_text_18_white, 0);

    // Устанавливаем текст через tr()
    lv_label_set_text(title, tr("DEVICE LOCKED"));
    lv_label_set_text(label_service_lock_msg, tr("Critical Error!\nHeating element failure.\nDevice is locked."));
    lv_textarea_set_placeholder_text(ta_service_code_input, tr("Enter code here..."));
    lv_label_set_text(label_enter, tr("ENTER"));

    // Весь код создания kb_service_code, service_lock_modal_overlay и ta_service_lock_modal_input УДАЛЕН.
}

static void service_code_keyboard_event_cb(lv_event_t* e) {
    lv_event_code_t code = lv_event_get_code(e);

    if (code != LV_EVENT_READY && code != LV_EVENT_CANCEL) {
        return;
    }

    // Запоминаем целевое поле
    lv_obj_t* target_to_defocus = current_target_ta;

    if (screen_to_return_to) {
        if (code == LV_EVENT_READY && current_target_ta) {
            // Копируем текст из прокси в оригинальное поле
            lv_textarea_set_text(current_target_ta, lv_textarea_get_text(ta_keyboard_proxy));
            
            // Можно сразу же инициировать проверку кода
            lv_event_send(btn_service_lock_enter, LV_EVENT_CLICKED, (void*)"check_code");
        }
        
        // Возвращаемся на экран блокировки
        load_screen(screen_to_return_to);

        // Сбрасываем указатели
        screen_to_return_to = NULL;
        current_target_ta = NULL;

        if (target_to_defocus) {
            lv_obj_clear_state(target_to_defocus, LV_STATE_FOCUSED);
            lv_indev_reset(NULL, target_to_defocus);
        }
    }
}

void show_info_dialog(const char* title_text, const char* message_text) {
    if (!screen_info_dialog) {
        Serial.println("show_info_dialog: screen_info_dialog is NULL! Cannot show.");
        return;
    }
    
    lvgl_port_lock(-1); // Блокируем LVGL

    apply_theme_to_info_dialog();

    if (label_info_dialog_text) {
        // Можно добавить заголовок в начало message_text, если нужно
        String full_message = "";
        if (title_text && strlen(title_text) > 0) {
            full_message += String(title_text) + "\n\n"; // Добавляем заголовок и перенос строки
        }
        full_message += message_text;
        lv_label_set_text(label_info_dialog_text, full_message.c_str());
    }

    lv_obj_clear_flag(screen_info_dialog, LV_OBJ_FLAG_HIDDEN);
    lv_obj_move_foreground(screen_info_dialog); // Поверх других элементов текущего экрана
                                                // (но под модальной клавиатурой, если она активна)
    
    lvgl_port_unlock();
    Serial.printf("Info dialog shown: Title='%s', Msg='%s'\n", title_text ? title_text : "N/A", message_text);
}

static void create_custom_toggle(lv_obj_t* parent, lv_obj_t** p_label, lv_obj_t** p_toggle_box, lv_obj_t** p_lbl1, lv_obj_t** p_lbl2) {
    lv_obj_t* row_container = lv_obj_create(parent);
    lv_obj_remove_style_all(row_container);
    lv_obj_set_width(row_container, lv_pct(90));
    lv_obj_set_height(row_container, LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(row_container, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_gap(row_container, 5, 0);
    
    *p_label = lv_label_create(row_container); // Используем указатель на label заголовка
    lv_obj_set_width(*p_label, lv_pct(100));
    lv_obj_set_style_text_align(*p_label, LV_TEXT_ALIGN_LEFT, 0);

    *p_toggle_box = lv_obj_create(row_container);
    lv_obj_remove_style_all(*p_toggle_box);
    lv_obj_set_size(*p_toggle_box, lv_pct(100), 40);
    lv_obj_set_flex_flow(*p_toggle_box, LV_FLEX_FLOW_ROW);
    lv_obj_set_style_radius(*p_toggle_box, 5, 0);
    lv_obj_set_style_clip_corner(*p_toggle_box, true, 0);
    lv_obj_set_style_border_width(*p_toggle_box, 1, 0);
    lv_obj_set_style_border_color(*p_toggle_box, lv_palette_main(LV_PALETTE_BLUE), 0);
    
    lv_obj_t* btn1 = lv_btn_create(*p_toggle_box);
    lv_obj_set_flex_grow(btn1, 1);
    lv_obj_set_height(btn1, lv_pct(100));
    lv_obj_set_style_radius(btn1, 0, 0);
    *p_lbl1 = lv_label_create(btn1); // Используем указатель на первую метку опции
    lv_obj_center(*p_lbl1);
    
    lv_obj_t* btn2 = lv_btn_create(*p_toggle_box);
    lv_obj_set_flex_grow(btn2, 1);
    lv_obj_set_height(btn2, lv_pct(100));
    lv_obj_set_style_radius(btn2, 0, 0);
    *p_lbl2 = lv_label_create(btn2); // Используем указатель на вторую метку опции
    lv_obj_center(*p_lbl2);
}


// Функция для отображения диалога
// Функция для отображения диалога
static void show_choice_dialog(const char* title, const char* message_text) {
    if (!screen_choice_dialog) return;
    
    lvgl_port_lock(-1);
    
    // Находим все нужные элементы
    lv_obj_t* title_label = lv_obj_get_child(screen_choice_dialog, 0);
    lv_obj_t* msg_label = lv_obj_get_child(screen_choice_dialog, 1);
    lv_obj_t* label_cancel = lv_obj_get_child(btn_choice_dialog_cancel, 0);
    lv_obj_t* label_skip = lv_obj_get_child(btn_choice_dialog_skip, 0);

    // --- ПРИМЕНЯЕМ ТЕМУ ---
    bool is_dark_theme = (current_global_settings.theme == 1);
    if (is_dark_theme) {
        lv_obj_set_style_bg_color(screen_choice_dialog, lv_color_hex(0x2C2C2C), 0);
        lv_obj_add_style(title_label, &style_dark_text, 0);
        lv_obj_add_style(msg_label, &style_dark_text, 0);
        // Красная кнопка остается красной, а вот кнопку "Пропустить" стилизуем
        lv_obj_add_style(btn_choice_dialog_skip, &style_dark_btn, 0);
    } else {
        lv_obj_set_style_bg_color(screen_choice_dialog, lv_color_white(), 0);
        lv_obj_remove_style(title_label, &style_dark_text, 0);
        lv_obj_remove_style(msg_label, &style_dark_text, 0);
        lv_obj_remove_style(btn_choice_dialog_skip, &style_dark_btn, 0);
    }

    // --- ПРИМЕНЯЕМ ЯЗЫК ---
    if (title_label) lv_label_set_text(title_label, tr(title));
    if (msg_label) lv_label_set_text(msg_label, tr(message_text));
    if (label_cancel) lv_label_set_text(label_cancel, tr("Cancel Process"));
    if (label_skip) lv_label_set_text(label_skip, tr("Continue without N2"));

    choice_dialog_result = 0; // Сбрасываем результат перед показом
    lv_obj_clear_flag(screen_choice_dialog, LV_OBJ_FLAG_HIDDEN);
    lv_obj_move_foreground(screen_choice_dialog);
    lvgl_port_unlock();
}

// Функция для построения UI диалога
static void build_choice_dialog(lv_obj_t* parent_layer) {
    if (screen_choice_dialog) return;

    screen_choice_dialog = lv_obj_create(parent_layer);
    lv_obj_add_flag(screen_choice_dialog, LV_OBJ_FLAG_HIDDEN);
    lv_obj_set_size(screen_choice_dialog, lv_pct(80), LV_SIZE_CONTENT);
    lv_obj_center(screen_choice_dialog);
    // ... (все ваши стили остаются) ...
    lv_obj_set_style_bg_color(screen_choice_dialog, lv_color_white(), 0);
    lv_obj_set_style_border_width(screen_choice_dialog, 1, 0);
    lv_obj_set_style_shadow_width(screen_choice_dialog, 8, 0);
    lv_obj_set_flex_flow(screen_choice_dialog, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_all(screen_choice_dialog, 15, 0);
    lv_obj_set_style_pad_gap(screen_choice_dialog, 10, 0);

    // Заголовок
    lv_obj_t* title = lv_label_create(screen_choice_dialog);
    lv_label_set_text(title, "Warning");
    // <<< ДОБАВЛЯЕМ СТИЛЬ ДЛЯ ПОДДЕРЖКИ РУССКОГО >>>
    lv_obj_add_style(title, &style_my_text_18, 0);


    // Текст сообщения
    lv_obj_t* msg = lv_label_create(screen_choice_dialog);
    lv_label_set_long_mode(msg, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(msg, lv_pct(100));
    // <<< ДОБАВЛЯЕМ СТИЛЬ ДЛЯ ПОДДЕРЖКИ РУССКОГО >>>
    lv_obj_add_style(msg, &style_my_text_18, 0);


    // Контейнер для кнопок
    lv_obj_t* btn_area = lv_obj_create(screen_choice_dialog);
    lv_obj_remove_style_all(btn_area);
    lv_obj_set_width(btn_area, lv_pct(100));
    lv_obj_set_height(btn_area, LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(btn_area, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(btn_area, LV_FLEX_ALIGN_SPACE_EVENLY, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

    // <<< ИЗМЕНЕНИЯ ЗДЕСЬ >>>
    // Кнопка "Cancel Process"
    btn_choice_dialog_cancel = lv_btn_create(btn_area);
    lv_obj_add_event_cb(btn_choice_dialog_cancel, choice_dialog_event_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t* label_cancel = lv_label_create(btn_choice_dialog_cancel);
    lv_label_set_text(label_cancel, "Cancel Process"); // Текст по умолчанию
    lv_obj_add_style(label_cancel, &style_my_text_18_white, 0);
    lv_obj_center(label_cancel);
    lv_obj_set_style_bg_color(btn_choice_dialog_cancel, lv_palette_main(LV_PALETTE_RED), 0);
    
    // Кнопка "Continue without N2"
    btn_choice_dialog_skip = lv_btn_create(btn_area);
    lv_obj_add_event_cb(btn_choice_dialog_skip, choice_dialog_event_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t* label_skip = lv_label_create(btn_choice_dialog_skip);
    lv_label_set_text(label_skip, "Continue without N2"); // Текст по умолчанию
    lv_obj_add_style(label_skip, &style_my_text_18_white, 0);
    lv_obj_center(label_skip);
    // Цвет по умолчанию будет из темы
}
static void profile_switch_value_changed_event_cb(lv_event_t * e) {
    lv_obj_t* sw = lv_event_get_target(e);
    const char* switch_id = (const char*)lv_event_get_user_data(e);
    bool attempted_to_enable = lv_obj_has_state(sw, LV_STATE_CHECKED);

    if (strcmp(switch_id, "nitrogen_profile") == 0) {
        bool is_enabled_by_user = lv_obj_has_state(sw, LV_STATE_CHECKED);

        if (!current_global_settings.nitrogen_system_enabled && is_enabled_by_user) {
            lv_obj_clear_state(sw, LV_STATE_CHECKED); 
            show_info_dialog(
                tr("Setting Disabled"), 
                tr("Nitrogen system is disabled in Global Settings.\nPlease connect the valve and enable it in Settings.")
            );
            is_enabled_by_user = false; 
        }
        
        if (is_enabled_by_user) {
            // Делаем контейнер видимым
            lv_obj_set_style_opa(nitrogen_elements_container, LV_OPA_COVER, 0);
            lv_obj_clear_flag(nitrogen_elements_container, LV_OBJ_FLAG_CLICKABLE);
        } else {
            // Делаем контейнер прозрачным
            lv_obj_set_style_opa(nitrogen_elements_container, LV_OPA_TRANSP, 0);
            lv_obj_add_flag(nitrogen_elements_container, LV_OBJ_FLAG_CLICKABLE);
        }
        Serial.printf("Profile Nitrogen switch new state: %s\n", is_enabled_by_user ? "ON" : "OFF");

    } else if (strcmp(switch_id, "cooling_profile") == 0) {
        if (!current_global_settings.compressed_air_system_enabled && attempted_to_enable) {
            lv_obj_clear_state(sw, LV_STATE_CHECKED); 
            show_info_dialog(
                tr("Setting Disabled"), 
                tr("Compressed Air (Chamber Cooling) is disabled in Global Settings.\nPlease connect it and enable in Settings.")
            );
        } else {
            Serial.printf("Profile Chamber Cooling switch new state: %s\n", attempted_to_enable ? "ON" : "OFF");
            if (attempted_to_enable) {
                lv_obj_clear_flag(block_post_cooling_purge, LV_OBJ_FLAG_HIDDEN);

                if (current_active_profile_data.post_cooling_air_purge_sec == 0) {
                    current_active_profile_data.post_cooling_air_purge_sec = 20;
                    // Также нужно обновить UI кнопок, чтобы они соответствовали
                    lv_btnmatrix_set_selected_btn(btnm_post_cooling_purge, 0); // 0 - это ID кнопки "20"
                    Serial.println("Purge time was 0, setting default to 20 seconds in memory.");
                }
            } else {
                lv_obj_add_flag(block_post_cooling_purge, LV_OBJ_FLAG_HIDDEN);
            }
        }
    }
}

static void post_cooling_purge_event_cb(lv_event_t * e) {
    lv_obj_t* btnm = lv_event_get_target(e);
    uint16_t selected_id = lv_btnmatrix_get_selected_btn(btnm);

    int new_value = 20;
    switch(selected_id) {
        case 0: new_value = 20; break;
        case 1: new_value = 40; break;
        case 2: new_value = 60; break;
    }
    
    // СРАЗУ ЖЕ обновляем значение в активной структуре данных
    current_active_profile_data.post_cooling_air_purge_sec = new_value;
    Serial.printf("Purge time selection changed and stored in memory: %d seconds.\n", new_value);
}

// Обработчик событий для глобальных клавиатур, КОГДА ОНИ ИСПОЛЬЗУЮТСЯ В МОДАЛЬНОМ РЕЖИМЕ
static void modal_input_keyboard_event_cb(lv_event_t* e) {
    lv_event_code_t code = lv_event_get_code(e);
    
    if (code != LV_EVENT_READY && code != LV_EVENT_CANCEL) {
        return;
    }

    lv_obj_t* target_to_defocus = current_target_ta;

    if (screen_to_return_to) {
        if (code == LV_EVENT_READY) {
            const char* new_text = lv_textarea_get_text(ta_keyboard_proxy);

            if (current_target_ta == ta_dummy_for_new_profile) {
                handle_save_new_profile_logic(new_text);
            } 
            else if (current_target_ta) {
                // 1. Копируем текст в оригинальное поле
                lv_textarea_set_text(current_target_ta, new_text);

                // 2. <<<--- ГЛАВНОЕ ИСПРАВЛЕНИЕ ---<<<
                // Программно отправляем событие "потери фокуса" оригинальному полю.
                // LVGL сам вызовет ВСЕХ подписчиков этого события: и валидатор, и СОХРАНЯТОР.
                Serial.printf("Keyboard OK: Programmatically sending DEFOCUSED event to target TA.\n");
                lv_event_send(current_target_ta, LV_EVENT_DEFOCUSED, NULL);
                // ------------------------------------
            }
        }

        // 3. Возвращаемся на предыдущий экран
        load_screen(screen_to_return_to);
        screen_to_return_to = NULL;
        current_target_ta = NULL;

        // 4. Снимаем фокус
        if (target_to_defocus) {
            lv_obj_clear_state(target_to_defocus, LV_STATE_FOCUSED);
            lv_indev_reset(NULL, target_to_defocus);
        }
    }
}
// Обработчик клика по затемняющему фону (для отмены ввода)
static void modal_input_overlay_click_event_cb(lv_event_t* e) {
    lv_event_code_t code = lv_event_get_code(e);
    if (code == LV_EVENT_CLICKED) {
        Serial.println("Modal input overlay clicked, cancelling input.");
        lv_obj_t* active_kb = nullptr;
        
        // Ищем активную клавиатуру
        if (kb_edit_numeric && !lv_obj_has_flag(kb_edit_numeric, LV_OBJ_FLAG_HIDDEN)) active_kb = kb_edit_numeric;
        else if (kb_edit_alpha && !lv_obj_has_flag(kb_edit_alpha, LV_OBJ_FLAG_HIDDEN)) active_kb = kb_edit_alpha;
        
        if (active_kb) {
            lv_event_send(active_kb, LV_EVENT_CANCEL, NULL);
        } else {
            // Если клавиатура не найдена, скрываем все модальные элементы вручную
            if (overlay_modal_input_bg) lv_obj_add_flag(overlay_modal_input_bg, LV_OBJ_FLAG_HIDDEN);
            if (ta_modal_input) lv_obj_add_flag(ta_modal_input, LV_OBJ_FLAG_HIDDEN);
            // <<<--- ДОБАВЛЕНО ---<<<
            if (modal_input_container) lv_obj_add_flag(modal_input_container, LV_OBJ_FLAG_HIDDEN); 
            
            if (current_target_ta) {
                lv_obj_clear_state(current_target_ta, LV_STATE_FOCUSED);
                lv_indev_reset(NULL, current_target_ta);
            }
            current_target_ta = nullptr;
        }
    }
}

static void show_modal_input(lv_obj_t* target_ta_original, lv_keyboard_mode_t kb_mode_to_set) {
    if (!target_ta_original || !screen_keyboard) {
        Serial.println("show_modal_input: Target TA or Keyboard Screen is NULL!");
        return;
    }

    lvgl_port_lock(-1);

    // 1. Запоминаем, куда вернуться и с каким полем работать
    screen_to_return_to = lv_scr_act();
    current_target_ta = target_ta_original;

    // 2. Копируем исходный текст и настройки в прокси-поле
    // Для нового профиля исходный текст пустой
    if (current_target_ta == ta_dummy_for_new_profile) {
        lv_textarea_set_text(ta_keyboard_proxy, "");
        lv_textarea_set_placeholder_text(ta_keyboard_proxy, "Enter Profile Name...");
        lv_textarea_set_max_length(ta_keyboard_proxy, 200);
    } else {
        lv_textarea_set_text(ta_keyboard_proxy, lv_textarea_get_text(current_target_ta));
        lv_textarea_set_placeholder_text(ta_keyboard_proxy, lv_textarea_get_placeholder_text(current_target_ta));
        lv_textarea_set_max_length(ta_keyboard_proxy, lv_textarea_get_max_length(current_target_ta));
    }
    
    // 3. Выбираем, какую клавиатуру показать
    lv_obj_add_flag(kb_edit_alpha, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(kb_edit_numeric, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(kb_service_code, LV_OBJ_FLAG_HIDDEN);

    if (kb_mode_to_set == LV_KEYBOARD_MODE_NUMBER) {
        lv_obj_clear_flag(kb_edit_numeric, LV_OBJ_FLAG_HIDDEN);
    } else {
        // По умолчанию используем алфавитную
        lv_obj_clear_flag(kb_edit_alpha, LV_OBJ_FLAG_HIDDEN);
    }

    apply_theme_to_keyboard_screen();
    
    // 4. Переключаем экран
    Serial.println("Switching to dedicated keyboard screen.");
    load_screen(screen_keyboard);
    
    lvgl_port_unlock();
}


// Раздел 3: Функции построения экранов LVGL
// ==========================================================================

static void lab_mode_param_changed_event_cb(lv_event_t * e) {
    lv_obj_t* target = lv_event_get_target(e);
    lv_event_code_t code = lv_event_get_code(e);
    const char* user_data = (const char*)lv_event_get_user_data(e);

    // Убедимся, что есть что обрабатывать
    if (!user_data) return;
    
    // --- Фаза 1: Обновление глобальной структуры current_lab_settings ---
    
    // Для текстовых полей - ждем окончания редактирования (defocus)
    if (code == LV_EVENT_DEFOCUSED) {
        const char* text_value = lv_textarea_get_text(target);
        if (strcmp(user_data, "glaze_uv_on") == 0) current_lab_settings.glaze.uv_on_sec = atof(text_value);
        else if (strcmp(user_data, "glaze_uv_off") == 0) current_lab_settings.glaze.uv_off_sec = atof(text_value);
        else if (strcmp(user_data, "glaze_monomer_blow") == 0) current_lab_settings.glaze.monomer_blow_min = atoi(text_value);
        else if (strcmp(user_data, "glaze_uv_exposure") == 0) current_lab_settings.glaze.uv_exposure_sec = atoi(text_value);
        else if (strcmp(user_data, "glaze_nitrogen_target") == 0) current_lab_settings.glaze.nitrogen_target_percent = atoi(text_value);
        else if (strcmp(user_data, "glaze_nitrogen_boost") == 0) current_lab_settings.glaze.nitrogen_boost_sec = atoi(text_value);
        else if (strcmp(user_data, "repair_countdown") == 0) current_lab_settings.repair.countdown_sec = atoi(text_value);
        else if (strcmp(user_data, "repair_uv_exposure") == 0) current_lab_settings.repair.uv_exposure_sec = atoi(text_value);
        else if (strcmp(user_data, "strength_temp") == 0) current_lab_settings.strength.chamber_temp_c = atoi(text_value);
        else if (strcmp(user_data, "strength_hold") == 0) current_lab_settings.strength.hold_time_min = atoi(text_value);
        else if (strcmp(user_data, "strength_pulse_dur") == 0) current_lab_settings.strength.uv_pulse_duration_sec = atoi(text_value);
        else if (strcmp(user_data, "strength_pulse_int") == 0) current_lab_settings.strength.uv_pulse_interval_min = atoi(text_value);
        else if (strcmp(user_data, "thermal_temp") == 0) current_lab_settings.thermal.chamber_temp_c = atoi(text_value);
        else if (strcmp(user_data, "thermal_hold") == 0) current_lab_settings.thermal.hold_time_min = atoi(text_value);
        else if (strcmp(user_data, "lighten_hold") == 0) current_lab_settings.lighten.hold_time_min = atoi(text_value);
        else if (strcmp(user_data, "darken_exposure") == 0) current_lab_settings.darken.uv_exposure_min = atoi(text_value);
    }
    // Для переключателей - реагируем на изменение значения
    else if (code == LV_EVENT_VALUE_CHANGED) {
        bool is_checked = lv_obj_has_state(target, LV_STATE_CHECKED);
        if (strcmp(user_data, "glaze_cooling") == 0) current_lab_settings.glaze.use_cooling = is_checked;
        else if (strcmp(user_data, "glaze_nitrogen") == 0) current_lab_settings.glaze.use_nitrogen = is_checked;
        else if (strcmp(user_data, "glaze_monomer_blow") == 0) current_lab_settings.glaze.use_monomer_blow = is_checked;
        else if (strcmp(user_data, "strength_cooling") == 0) current_lab_settings.strength.use_cooling = is_checked;
        else if (strcmp(user_data, "lighten_cooling") == 0) current_lab_settings.lighten.use_cooling = is_checked;
        else if (strcmp(user_data, "darken_cooling") == 0) current_lab_settings.darken.use_cooling = is_checked;
    } 
    else {
        return; // Если это не defocus и не value_changed, выходим
    }

    // --- Фаза 2: Сохранение на SD-карту ---
    Serial.printf("Lab parameter '%s' changed. Value updated in RAM.\n", user_data);
}

// Обработчик событий для переключателя
static void lab_mode_switch_event_cb(lv_event_t* e) {
    lv_obj_t* sw = lv_event_get_target(e);
    const char* user_data = (const char*)lv_event_get_user_data(e);
    bool is_checked = lv_obj_has_state(sw, LV_STATE_CHECKED);

    if (!user_data) return;

    // <-- Проверка для всех свитчей охлаждения -->
    if (strstr(user_data, "_cooling") != NULL) {
        if (is_checked && !current_global_settings.compressed_air_system_enabled) {
            lv_obj_clear_state(sw, LV_STATE_CHECKED); // Принудительно выключаем
            show_info_dialog(
                tr("Setting Disabled"), 
                tr("Compressed Air system is disabled in Global Settings.")
            );
            return; // ВАЖНО: выходим из функции после показа окна
        }
        Serial.printf("Lab Mode Cooling Switch (%s) toggled: %s\n", user_data, is_checked ? "ON" : "OFF");
    }
    // --- Проверка для переключателя АЗОТА (только на экране Глазурь) ---
    else if (strcmp(user_data, "glaze_nitrogen") == 0) {
        if (is_checked && !current_global_settings.nitrogen_system_enabled) {
            lv_obj_clear_state(sw, LV_STATE_CHECKED);
            show_info_dialog(
                tr("Setting Disabled"), 
                tr("Nitrogen system is disabled in Global Settings.")
            );
            is_checked = false; // Важно: обновляем состояние, так как мы его изменили
        }
        // Управляем видимостью поля для цели азота
        if (is_checked) {
            lv_obj_set_style_opa(nitrogen_glaze_container, LV_OPA_COVER, 0);
            lv_obj_clear_flag(nitrogen_glaze_container, LV_OBJ_FLAG_CLICKABLE);
            // --- НОВАЯ ЛОГИКА ---
            lv_obj_add_flag(label_glaze_nitrogen_warning, LV_OBJ_FLAG_HIDDEN); // Скрываем предупреждение
        } else {
            lv_obj_set_style_opa(nitrogen_glaze_container, LV_OPA_TRANSP, 0);
            lv_obj_add_flag(nitrogen_glaze_container, LV_OBJ_FLAG_CLICKABLE);
            // --- НОВАЯ ЛОГИКА ---
            lv_obj_clear_flag(label_glaze_nitrogen_warning, LV_OBJ_FLAG_HIDDEN); // Показываем предупреждение
        }
        Serial.printf("Glaze Nitrogen Switch toggled: %s\n", is_checked ? "ON" : "OFF");
    } 
    else if (strcmp(user_data, "glaze_monomer_blow_switch") == 0) {
        // Управляем видимостью поля для времени обдува
        if (is_checked) {
            lv_obj_set_style_opa(monomer_blow_container, LV_OPA_COVER, 0);
            lv_obj_clear_flag(monomer_blow_container, LV_OBJ_FLAG_CLICKABLE);
        } else {
            lv_obj_set_style_opa(monomer_blow_container, LV_OPA_TRANSP, 0);
            lv_obj_add_flag(monomer_blow_container, LV_OBJ_FLAG_CLICKABLE);
        }
        Serial.printf("Glaze Monomer Blow Switch toggled: %s\n", is_checked ? "ON" : "OFF");
    }
}


static void build_lab_glaze_screen(lv_obj_t* parent_screen) {
    screen_lab_glaze = parent_screen;
    Serial.println("Building THEME-AGNOSTIC lab_glaze_screen UI...");

    const lv_coord_t TEXT_INPUT_HEIGHT = 36;
    
    static lv_style_t style_edit_textarea;
    lv_style_init(&style_edit_textarea);
    lv_style_set_text_align(&style_edit_textarea, LV_TEXT_ALIGN_CENTER);
    lv_style_set_text_font(&style_edit_textarea, &montserrat_rus_18);
    lv_style_set_pad_ver(&style_edit_textarea, 2); 

    static lv_style_t style_param_block;
    lv_style_init(&style_param_block);
    lv_style_set_border_width(&style_param_block, 2);
    lv_style_set_radius(&style_param_block, 8);
    lv_style_set_pad_all(&style_param_block, 10);
    lv_style_set_flex_flow(&style_param_block, LV_FLEX_FLOW_COLUMN);
    lv_style_set_pad_row(&style_param_block, 5);

    lv_obj_t* main_container = lv_obj_create(screen_lab_glaze);
    lv_obj_set_size(main_container, lv_pct(100), lv_pct(100));
    lv_obj_center(main_container);
    lv_obj_set_flex_flow(main_container, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_all(main_container, 5, 0);
    lv_obj_set_style_border_width(main_container, 0, 0);
    lv_obj_set_style_radius(main_container, 0, 0);

    lv_obj_t* header = lv_obj_create(main_container);
    lv_obj_remove_style_all(header);
    lv_obj_set_size(header, lv_pct(100), LV_SIZE_CONTENT);
    lv_obj_set_style_pad_all(header, 5, 0);
    lv_obj_set_layout(header, LV_LAYOUT_FLEX);
    lv_obj_set_flex_flow(header, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(header, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

    label_glaze_header = lv_label_create(header);
    lv_obj_add_style(label_glaze_header, &style_my_text_22, 0);
    lv_obj_set_flex_grow(label_glaze_header, 1); 
    lv_obj_set_style_text_align(label_glaze_header, LV_TEXT_ALIGN_LEFT, 0);
    btn_glaze_help = lv_btn_create(header);
    lv_obj_set_size(btn_glaze_help, LV_SIZE_CONTENT, 30);
    lv_obj_add_event_cb(btn_glaze_help, glaze_screen_event_cb, LV_EVENT_CLICKED, (void*)"show_help");
    lv_obj_t* label_help_btn = lv_label_create(btn_glaze_help);
    lv_obj_add_style(label_help_btn, &style_my_text_18, 0);
    lv_obj_center(label_help_btn);
    
    lv_obj_t* content_area = lv_obj_create(main_container);
    lv_obj_remove_style_all(content_area);
    lv_obj_set_width(content_area, lv_pct(100));
    lv_obj_set_flex_grow(content_area, 1);
    lv_obj_set_layout(content_area, LV_LAYOUT_GRID);
    static lv_coord_t col_dsc[] = {LV_GRID_FR(1), 10, LV_GRID_FR(1), LV_GRID_TEMPLATE_LAST};
    static lv_coord_t row_dsc[] = {LV_GRID_FR(1), LV_GRID_TEMPLATE_LAST};
    lv_obj_set_grid_dsc_array(content_area, col_dsc, row_dsc);
    lv_obj_t* left_col = lv_obj_create(content_area);
    lv_obj_remove_style_all(left_col);
    lv_obj_set_grid_cell(left_col, LV_GRID_ALIGN_STRETCH, 0, 1, LV_GRID_ALIGN_STRETCH, 0, 1);
    lv_obj_set_flex_flow(left_col, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_gap(left_col, 10, 0);
    lv_obj_t* right_col = lv_obj_create(content_area);
    lv_obj_remove_style_all(right_col);
    lv_obj_set_grid_cell(right_col, LV_GRID_ALIGN_STRETCH, 2, 1, LV_GRID_ALIGN_STRETCH, 0, 1);
    
    lv_obj_t* block_flicker = lv_obj_create(left_col);
    lv_obj_add_style(block_flicker, &style_param_block, 0);
    lv_obj_set_width(block_flicker, lv_pct(100));
    lv_obj_set_flex_grow(block_flicker, 1);
    lv_obj_set_layout(block_flicker, LV_LAYOUT_FLEX);
    label_glaze_title_flicker = lv_label_create(block_flicker);
    lv_obj_add_style(label_glaze_title_flicker, &style_my_text_18, 0);
    lv_obj_add_style(label_glaze_title_flicker, &style_block_header, 0);
    lv_obj_set_width(label_glaze_title_flicker, lv_pct(100));
    lv_obj_set_style_text_align(label_glaze_title_flicker, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_t* flicker_params_cont = lv_obj_create(block_flicker);
    lv_obj_remove_style_all(flicker_params_cont);
    lv_obj_set_width(flicker_params_cont, lv_pct(100));
    lv_obj_set_flex_grow(flicker_params_cont, 1);
    lv_obj_set_flex_flow(flicker_params_cont, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(flicker_params_cont, LV_FLEX_ALIGN_SPACE_EVENLY, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    { lv_obj_t* row = lv_obj_create(flicker_params_cont); lv_obj_remove_style_all(row); lv_obj_set_width(row, lv_pct(100)); lv_obj_set_height(row, LV_SIZE_CONTENT); lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW); lv_obj_set_flex_align(row, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER); label_glaze_uv_on = lv_label_create(row); lv_obj_add_style(label_glaze_uv_on, &style_my_text_18, 0); ta_glaze_uv_on = lv_textarea_create(row); lv_textarea_set_one_line(ta_glaze_uv_on, true); lv_obj_set_size(ta_glaze_uv_on, 80, TEXT_INPUT_HEIGHT); lv_obj_add_style(ta_glaze_uv_on, &style_edit_textarea, 0); lv_obj_set_scrollbar_mode(ta_glaze_uv_on, LV_SCROLLBAR_MODE_OFF); lv_obj_add_event_cb(ta_glaze_uv_on, numeric_textarea_focus_event_cb, LV_EVENT_FOCUSED, NULL); lv_obj_add_event_cb(ta_glaze_uv_on, generic_textarea_defocus_event_cb, LV_EVENT_DEFOCUSED, NULL); lv_obj_add_event_cb(ta_glaze_uv_on, lab_mode_param_changed_event_cb, LV_EVENT_DEFOCUSED, (void*)"glaze_uv_on");}
    { lv_obj_t* row = lv_obj_create(flicker_params_cont); lv_obj_remove_style_all(row); lv_obj_set_width(row, lv_pct(100)); lv_obj_set_height(row, LV_SIZE_CONTENT); lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW); lv_obj_set_flex_align(row, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER); label_glaze_uv_off = lv_label_create(row); lv_obj_add_style(label_glaze_uv_off, &style_my_text_18, 0); ta_glaze_uv_off = lv_textarea_create(row); lv_textarea_set_one_line(ta_glaze_uv_off, true); lv_obj_set_size(ta_glaze_uv_off, 80, TEXT_INPUT_HEIGHT); lv_obj_add_style(ta_glaze_uv_off, &style_edit_textarea, 0); lv_obj_set_scrollbar_mode(ta_glaze_uv_off, LV_SCROLLBAR_MODE_OFF);lv_obj_add_event_cb(ta_glaze_uv_off, numeric_textarea_focus_event_cb, LV_EVENT_FOCUSED, NULL); lv_obj_add_event_cb(ta_glaze_uv_off, generic_textarea_defocus_event_cb, LV_EVENT_DEFOCUSED, NULL);lv_obj_add_event_cb(ta_glaze_uv_off, lab_mode_param_changed_event_cb, LV_EVENT_DEFOCUSED, (void*)"glaze_uv_off");}
    
    lv_obj_t* block_timers = lv_obj_create(left_col);
    lv_obj_add_style(block_timers, &style_param_block, 0);
    lv_obj_set_width(block_timers, lv_pct(100));
    lv_obj_set_flex_grow(block_timers, 1);
    lv_obj_set_layout(block_timers, LV_LAYOUT_FLEX); // Используем flex для внутреннего расположения
    
    label_glaze_title_timers = lv_label_create(block_timers);
    lv_obj_add_style(label_glaze_title_timers, &style_my_text_18, 0);
    lv_obj_add_style(label_glaze_title_timers, &style_block_header, 0);
    lv_obj_set_width(label_glaze_title_timers, lv_pct(100));
    lv_obj_set_style_text_align(label_glaze_title_timers, LV_TEXT_ALIGN_CENTER, 0);
    
    lv_obj_t* timers_params_cont = lv_obj_create(block_timers);
    lv_obj_remove_style_all(timers_params_cont);
    lv_obj_set_width(timers_params_cont, lv_pct(100));
    lv_obj_set_flex_grow(timers_params_cont, 1);
    lv_obj_set_flex_flow(timers_params_cont, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(timers_params_cont, LV_FLEX_ALIGN_SPACE_EVENLY, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

    // СТРОКА С ТРОЙНЫМ ПЕРЕКЛЮЧАТЕЛЕМ UV >>>
    { 
        // Создаем строку-контейнер для переключателя
        lv_obj_t* row_uv_glaze = lv_obj_create(timers_params_cont);
        lv_obj_remove_style_all(row_uv_glaze);
        // Применяем стиль, как в профилях, для единообразия (но пока без цвета)
        static lv_style_t style_switch_row; // Локальный стиль, если нужно
        lv_style_init(&style_switch_row);
        lv_style_set_radius(&style_switch_row, 5);
        // lv_style_set_pad_hor(&style_switch_row, 5);
        lv_style_set_pad_ver(&style_switch_row, 3);
        lv_obj_add_style(row_uv_glaze, &style_switch_row, 0);
        
        lv_obj_set_width(row_uv_glaze, lv_pct(100)); 
        lv_obj_set_height(row_uv_glaze, 45); 
        lv_obj_set_flex_flow(row_uv_glaze, LV_FLEX_FLOW_ROW); 
        lv_obj_set_flex_align(row_uv_glaze, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER); 
        
        label_glaze_uv_mode_title = lv_label_create(row_uv_glaze); 
        lv_obj_add_style(label_glaze_uv_mode_title, &style_my_text_18, 0);
        label_glaze_uv_mode_status = lv_label_create(row_uv_glaze); 
        lv_obj_add_style(label_glaze_uv_mode_status, &style_my_text_18, 0); 
        
        // Контейнер для трех кнопок
        lv_obj_t* btn_container = lv_obj_create(row_uv_glaze);
        lv_obj_remove_style_all(btn_container);
        lv_obj_set_size(btn_container, 150, 40);
        lv_obj_set_flex_flow(btn_container, LV_FLEX_FLOW_ROW);
        lv_obj_set_flex_align(btn_container, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
        lv_obj_set_style_pad_gap(btn_container, 5, 0);

        btn_glaze_uv_1 = lv_btn_create(btn_container);
        btn_glaze_uv_2 = lv_btn_create(btn_container);
        btn_glaze_uv_3 = lv_btn_create(btn_container);
        
        lv_obj_t* btns[] = {btn_glaze_uv_1, btn_glaze_uv_2, btn_glaze_uv_3};
        for(int i = 0; i < 3; i++) {
            lv_obj_set_size(btns[i], 40, 40);
            lv_obj_add_flag(btns[i], LV_OBJ_FLAG_CHECKABLE);
            lv_obj_set_style_radius(btns[i], LV_RADIUS_CIRCLE, 0);
            lv_obj_t* lbl = lv_label_create(btns[i]);
            lv_label_set_text_fmt(lbl, "%d", i + 1);
            lv_obj_add_style(lbl, &style_my_text_18_white, 0);
            lv_obj_center(lbl);
            lv_obj_add_event_cb(btns[i], uv_mode_btn_group_event_cb, LV_EVENT_CLICKED, label_glaze_uv_mode_status);
            lv_obj_add_event_cb(btns[i], glaze_uv_mode_save_cb, LV_EVENT_CLICKED, NULL);
        }
    }
    
    { lv_obj_t* row = lv_obj_create(timers_params_cont); lv_obj_remove_style_all(row); lv_obj_set_width(row, lv_pct(100)); lv_obj_set_height(row, LV_SIZE_CONTENT); lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW); lv_obj_set_flex_align(row, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER); label_glaze_uv_exposure = lv_label_create(row); lv_obj_add_style(label_glaze_uv_exposure, &style_my_text_18, 0); ta_glaze_uv_exposure = lv_textarea_create(row); lv_textarea_set_one_line(ta_glaze_uv_exposure, true); lv_obj_set_size(ta_glaze_uv_exposure, 80, TEXT_INPUT_HEIGHT); lv_obj_add_style(ta_glaze_uv_exposure, &style_edit_textarea, 0); lv_obj_set_scrollbar_mode(ta_glaze_uv_exposure, LV_SCROLLBAR_MODE_OFF); lv_obj_add_event_cb(ta_glaze_uv_exposure, numeric_textarea_focus_event_cb, LV_EVENT_FOCUSED, NULL); lv_obj_add_event_cb(ta_glaze_uv_exposure, generic_textarea_defocus_event_cb, LV_EVENT_DEFOCUSED, NULL);lv_obj_add_event_cb(ta_glaze_uv_exposure, lab_mode_param_changed_event_cb, LV_EVENT_DEFOCUSED, (void*)"glaze_uv_exposure");}

    lv_obj_t* block_aux = lv_obj_create(right_col);
    lv_obj_add_style(block_aux, &style_param_block, 0);
    lv_obj_set_size(block_aux, lv_pct(100), lv_pct(100));
    lv_obj_set_flex_flow(block_aux, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_gap(block_aux, 10, 0);

    label_glaze_title_aux = lv_label_create(block_aux);
    lv_obj_add_style(label_glaze_title_aux, &style_my_text_18, 0);
    lv_obj_add_style(label_glaze_title_aux, &style_block_header, 0);
    lv_obj_set_width(label_glaze_title_aux, lv_pct(100));
    lv_obj_set_style_text_align(label_glaze_title_aux, LV_TEXT_ALIGN_CENTER, 0);

    // --- Блок с охлаждением (без изменений) ---
    { 
        lv_obj_t* row = lv_obj_create(block_aux);
        lv_obj_remove_style_all(row);
        lv_obj_set_width(row, lv_pct(100));
        lv_obj_set_height(row, LV_SIZE_CONTENT);
        lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW);
        lv_obj_set_flex_align(row, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
        label_glaze_cooling = lv_label_create(row);
        lv_obj_add_style(label_glaze_cooling, &style_my_text_18, 0);
        sw_glaze_cooling = lv_switch_create(row);
        lv_obj_add_event_cb(sw_glaze_cooling, lab_mode_switch_event_cb, LV_EVENT_VALUE_CHANGED, (void*)"glaze_cooling");
        lv_obj_add_event_cb(sw_glaze_cooling, lab_mode_param_changed_event_cb, LV_EVENT_VALUE_CHANGED, (void*)"glaze_cooling");
    }

    // <<< ДОБАВЛЕНА СТРОКА С ПЕРЕКЛЮЧАТЕЛЕМ "ОБДУВ МОНОМЕРА" >>>
    { 
        lv_obj_t* row = lv_obj_create(block_aux);
        lv_obj_remove_style_all(row);
        lv_obj_set_width(row, lv_pct(100));
        lv_obj_set_height(row, LV_SIZE_CONTENT);
        lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW);
        lv_obj_set_flex_align(row, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
        
        label_glaze_monomer_blow_switch = lv_label_create(row);
        lv_obj_add_style(label_glaze_monomer_blow_switch, &style_my_text_18, 0);
        
        sw_glaze_monomer_blow = lv_switch_create(row);
        lv_obj_add_event_cb(sw_glaze_monomer_blow, lab_mode_switch_event_cb, LV_EVENT_VALUE_CHANGED, (void*)"glaze_monomer_blow_switch");
        lv_obj_add_event_cb(sw_glaze_monomer_blow, lab_mode_param_changed_event_cb, LV_EVENT_VALUE_CHANGED, (void*)"glaze_monomer_blow");
    }

    // <<< ДОБАВЛЕН КОНТЕЙНЕР ДЛЯ НАСТРОЙКИ ВРЕМЕНИ ОБДУВА >>>
    monomer_blow_container = lv_obj_create(block_aux);
    lv_obj_remove_style_all(monomer_blow_container);
    lv_obj_set_width(monomer_blow_container, lv_pct(100));
    lv_obj_set_height(monomer_blow_container, LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(monomer_blow_container, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(monomer_blow_container, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    {
        label_glaze_monomer_blow = lv_label_create(monomer_blow_container);
        lv_obj_add_style(label_glaze_monomer_blow, &style_my_text_18, 0);
        ta_glaze_monomer_blow = lv_textarea_create(monomer_blow_container);
        lv_textarea_set_one_line(ta_glaze_monomer_blow, true);
        lv_obj_set_size(ta_glaze_monomer_blow, 80, TEXT_INPUT_HEIGHT);
        lv_obj_add_style(ta_glaze_monomer_blow, &style_edit_textarea, 0);
        lv_obj_set_scrollbar_mode(ta_glaze_monomer_blow, LV_SCROLLBAR_MODE_OFF);
        lv_obj_add_event_cb(ta_glaze_monomer_blow, numeric_textarea_focus_event_cb, LV_EVENT_FOCUSED, NULL); 
        lv_obj_add_event_cb(ta_glaze_monomer_blow, generic_textarea_defocus_event_cb, LV_EVENT_DEFOCUSED, NULL);
        lv_obj_add_event_cb(ta_glaze_monomer_blow, lab_mode_param_changed_event_cb, LV_EVENT_DEFOCUSED, (void*)"glaze_monomer_blow");
    }

    // --- Новый контейнер ТОЛЬКО для настроек азота ---
    lv_obj_t* nitrogen_block = lv_obj_create(block_aux);
    lv_obj_remove_style_all(nitrogen_block);
    lv_obj_set_width(nitrogen_block, lv_pct(100));
    lv_obj_set_height(nitrogen_block, LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(nitrogen_block, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_gap(nitrogen_block, 5, 0);

    // Строка с переключателем азота
    {
        lv_obj_t* row = lv_obj_create(nitrogen_block);
        lv_obj_remove_style_all(row);
        lv_obj_set_width(row, lv_pct(100));
        lv_obj_set_height(row, LV_SIZE_CONTENT);
        lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW);
        lv_obj_set_flex_align(row, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
        label_glaze_nitrogen = lv_label_create(row);
        lv_obj_add_style(label_glaze_nitrogen, &style_my_text_18, 0);
        sw_glaze_nitrogen = lv_switch_create(row);
        lv_obj_add_event_cb(sw_glaze_nitrogen, lab_mode_switch_event_cb, LV_EVENT_VALUE_CHANGED, (void*)"glaze_nitrogen");
        lv_obj_add_event_cb(sw_glaze_nitrogen, lab_mode_param_changed_event_cb, LV_EVENT_VALUE_CHANGED, (void*)"glaze_nitrogen");
    }

    // Строка с вводом цели по азоту (бывший nitrogen_glaze_container)
    nitrogen_glaze_container = lv_obj_create(nitrogen_block);
    lv_obj_remove_style_all(nitrogen_glaze_container);
    lv_obj_set_width(nitrogen_glaze_container, lv_pct(100));
    lv_obj_set_height(nitrogen_glaze_container, LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(nitrogen_glaze_container, LV_FLEX_FLOW_COLUMN); // <<< ВАЖНО: теперь это колонка
    lv_obj_set_style_pad_gap(nitrogen_glaze_container, 5, 0); // Отступ между строками внутри

    // Строка с целью по N2
    {
        lv_obj_t* row = lv_obj_create(nitrogen_glaze_container);
        lv_obj_remove_style_all(row);
        lv_obj_set_width(row, lv_pct(100));
        lv_obj_set_height(row, LV_SIZE_CONTENT);
        lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW);
        lv_obj_set_flex_align(row, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
        
        label_glaze_nitrogen_target = lv_label_create(row);
        lv_obj_add_style(label_glaze_nitrogen_target, &style_my_text_18, 0);
        
        ta_glaze_nitrogen_target = lv_textarea_create(row);
        lv_textarea_set_one_line(ta_glaze_nitrogen_target, true);
        lv_obj_set_size(ta_glaze_nitrogen_target, 80, TEXT_INPUT_HEIGHT);
        lv_obj_add_style(ta_glaze_nitrogen_target, &style_edit_textarea, 0);
        lv_obj_set_scrollbar_mode(ta_glaze_nitrogen_target, LV_SCROLLBAR_MODE_OFF);
        lv_obj_add_event_cb(ta_glaze_nitrogen_target, numeric_textarea_focus_event_cb, LV_EVENT_FOCUSED, NULL); 
        lv_obj_add_event_cb(ta_glaze_nitrogen_target, generic_textarea_defocus_event_cb, LV_EVENT_DEFOCUSED, NULL);
        lv_obj_add_event_cb(ta_glaze_nitrogen_target, lab_mode_param_changed_event_cb, LV_EVENT_DEFOCUSED, (void*)"glaze_nitrogen_target");
    }

    // Строка с доп. подачей
    {
        lv_obj_t* row = lv_obj_create(nitrogen_glaze_container);
        lv_obj_remove_style_all(row);
        lv_obj_set_width(row, lv_pct(100));
        lv_obj_set_height(row, LV_SIZE_CONTENT);
        lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW);
        lv_obj_set_flex_align(row, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

        label_glaze_nitrogen_boost = lv_label_create(row);
        lv_obj_add_style(label_glaze_nitrogen_boost, &style_my_text_18, 0);

        ta_glaze_nitrogen_boost = lv_textarea_create(row);
        lv_textarea_set_one_line(ta_glaze_nitrogen_boost, true);
        lv_obj_set_size(ta_glaze_nitrogen_boost, 80, TEXT_INPUT_HEIGHT);
        lv_obj_add_style(ta_glaze_nitrogen_boost, &style_edit_textarea, 0); 
        lv_obj_set_scrollbar_mode(ta_glaze_nitrogen_boost, LV_SCROLLBAR_MODE_OFF); 
        lv_obj_add_event_cb(ta_glaze_nitrogen_boost, numeric_textarea_focus_event_cb, LV_EVENT_FOCUSED, NULL); 
        lv_obj_add_event_cb(ta_glaze_nitrogen_boost, generic_textarea_defocus_event_cb, LV_EVENT_DEFOCUSED, NULL);
        lv_obj_add_event_cb(ta_glaze_nitrogen_boost, lab_mode_param_changed_event_cb, LV_EVENT_DEFOCUSED, (void*)"glaze_nitrogen_boost");
    }

    // НОВЫЙ ЛЕЙБЛ-ПРЕДУПРЕЖДЕНИЕ
    label_glaze_nitrogen_warning = lv_label_create(nitrogen_block);
    lv_obj_add_style(label_glaze_nitrogen_warning, &style_my_text_16, 0);
    lv_obj_set_style_text_color(label_glaze_nitrogen_warning, lv_palette_main(LV_PALETTE_AMBER), 0);
    lv_obj_set_width(label_glaze_nitrogen_warning, lv_pct(100));
    lv_label_set_long_mode(label_glaze_nitrogen_warning, LV_LABEL_LONG_WRAP);
    lv_obj_set_style_text_align(label_glaze_nitrogen_warning, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_add_flag(label_glaze_nitrogen_warning, LV_OBJ_FLAG_HIDDEN); // Скрываем по умолчанию
    lv_label_set_text(label_glaze_nitrogen_warning, tr("Without nitrogen supply, the surface will be sticky!"));

    // --- Футер (без изменений) ---
    lv_obj_t* footer = lv_obj_create(main_container);
    lv_obj_remove_style_all(footer);
    lv_obj_set_size(footer, lv_pct(100), LV_SIZE_CONTENT);
    lv_obj_set_style_pad_all(footer, 5, 0);
    lv_obj_set_flex_flow(footer, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(footer, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_gap(footer, 20, 0);
    btn_glaze_start = lv_btn_create(footer);
    lv_obj_set_flex_grow(btn_glaze_start, 1);
    lv_obj_set_height(btn_glaze_start, 45);
    lv_obj_add_event_cb(btn_glaze_start, glaze_screen_event_cb, LV_EVENT_CLICKED, (void*)"start");
    label_btn_glaze_start = lv_label_create(btn_glaze_start);
    lv_obj_add_style(label_btn_glaze_start, &style_my_text_18_white, 0);
    lv_obj_center(label_btn_glaze_start);
    btn_glaze_back = lv_btn_create(footer);
    lv_obj_set_flex_grow(btn_glaze_back, 1);
    lv_obj_set_height(btn_glaze_back, 45);
    lv_obj_add_event_cb(btn_glaze_back, glaze_screen_event_cb, LV_EVENT_CLICKED, (void*)"back");
    label_btn_glaze_back = lv_label_create(btn_glaze_back);
    lv_obj_add_style(label_btn_glaze_back, &style_my_text_18_white, 0);
    lv_obj_center(label_btn_glaze_back);
}

static void repair_screen_event_cb(lv_event_t* e) {
    const char* user_data = (const char*)lv_event_get_user_data(e);
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;

    if (strcmp(user_data, "repair_show_help") == 0) {
        if (current_global_settings.language == 1) { // RUS
            show_info_dialog("Справка: Ремонт Модели", "Этот режим предназначен для локального ремонта и отверждения композита. Установите таймер для подготовки и время засветки.");
        } else { // ENG
            show_info_dialog("Help: Model Repair", "This mode is designed for local repair and curing of the composite. Set a timer for preparation and the exposure time.");
        }
    } else if (strcmp(user_data, "repair_back") == 0) {
        if (help_blink_timer) { lv_timer_del(help_blink_timer); help_blink_timer = nullptr; }
        trigger_lab_action(1); // 1 = Назад
    } else if (strcmp(user_data, "repair_start") == 0) {
        trigger_lab_action(3);
    }
}

static void build_lab_repair_screen(lv_obj_t* parent_screen) {
    screen_lab_repair = parent_screen;
    Serial.println("Building THEME-AGNOSTIC lab_repair_screen UI (FINAL CORRECTED)...");

    const lv_coord_t TEXT_INPUT_HEIGHT = 36;
    static lv_style_t style_edit_textarea;
    lv_style_init(&style_edit_textarea);
    lv_style_set_text_align(&style_edit_textarea, LV_TEXT_ALIGN_CENTER);
    lv_style_set_text_font(&style_edit_textarea, &montserrat_rus_18);
    lv_style_set_pad_ver(&style_edit_textarea, 2);

    static lv_style_t style_param_block;
    lv_style_init(&style_param_block);
    lv_style_set_border_width(&style_param_block, 2);
    lv_style_set_radius(&style_param_block, 8);
    lv_style_set_pad_all(&style_param_block, 10);

    lv_obj_t* main_container = lv_obj_create(screen_lab_repair);
    lv_obj_set_size(main_container, lv_pct(100), lv_pct(100));
    lv_obj_center(main_container);
    lv_obj_set_flex_flow(main_container, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_all(main_container, 5, 0);
    lv_obj_set_style_border_width(main_container, 0, 0);
    lv_obj_set_style_radius(main_container, 0, 0);
    
    lv_obj_t* header = lv_obj_create(main_container);
    lv_obj_remove_style_all(header);
    lv_obj_set_size(header, lv_pct(100), LV_SIZE_CONTENT);
    lv_obj_set_style_pad_all(header, 5, 0);
    lv_obj_set_layout(header, LV_LAYOUT_FLEX);
    lv_obj_set_flex_flow(header, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(header, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    label_repair_header = lv_label_create(header);
    lv_obj_add_style(label_repair_header, &style_my_text_22, 0);
    lv_obj_set_flex_grow(label_repair_header, 1);
    lv_obj_set_style_text_align(label_repair_header, LV_TEXT_ALIGN_LEFT, 0);
    btn_repair_help = lv_btn_create(header);
    lv_obj_set_size(btn_repair_help, LV_SIZE_CONTENT, 30);
    lv_obj_add_event_cb(btn_repair_help, repair_screen_event_cb, LV_EVENT_CLICKED, (void*)"repair_show_help");
    lv_obj_t* label_help_btn = lv_label_create(btn_repair_help);
    lv_obj_add_style(label_help_btn, &style_my_text_18, 0);
    lv_obj_center(label_help_btn);

    lv_obj_t* content_area = lv_obj_create(main_container);
    lv_obj_remove_style_all(content_area);
    lv_obj_set_width(content_area, lv_pct(100));
    lv_obj_set_flex_grow(content_area, 1);
    lv_obj_set_flex_flow(content_area, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_gap(content_area, 10, 0);

    lv_obj_t* block_timers = lv_obj_create(content_area);
    lv_obj_add_style(block_timers, &style_param_block, 0);
    lv_obj_set_width(block_timers, lv_pct(100));
    lv_obj_set_height(block_timers, LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(block_timers, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_gap(block_timers, 5, 0);
    label_repair_title_timers = lv_label_create(block_timers);
    lv_obj_add_style(label_repair_title_timers, &style_my_text_18, 0);
    lv_obj_add_style(label_repair_title_timers, &style_block_header, 0);
    lv_obj_set_width(label_repair_title_timers, lv_pct(100));
    lv_obj_set_style_text_align(label_repair_title_timers, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_t* timers_params_cont = lv_obj_create(block_timers);
    lv_obj_remove_style_all(timers_params_cont);
    lv_obj_set_width(timers_params_cont, lv_pct(100));
    lv_obj_set_flex_grow(timers_params_cont, 1);
    lv_obj_set_flex_flow(timers_params_cont, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(timers_params_cont, LV_FLEX_ALIGN_SPACE_EVENLY, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);
    lv_obj_set_style_pad_hor(timers_params_cont, 10, 0);
    { lv_obj_t* row = lv_obj_create(timers_params_cont); lv_obj_remove_style_all(row); lv_obj_set_size(row, lv_pct(100), LV_SIZE_CONTENT); lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW); lv_obj_set_flex_align(row, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER); label_repair_countdown = lv_label_create(row); lv_obj_add_style(label_repair_countdown, &style_my_text_18, 0); ta_repair_countdown = lv_textarea_create(row); lv_textarea_set_one_line(ta_repair_countdown, true); lv_obj_set_size(ta_repair_countdown, 80, TEXT_INPUT_HEIGHT); lv_obj_add_style(ta_repair_countdown, &style_edit_textarea, 0); lv_obj_set_scrollbar_mode(ta_repair_countdown, LV_SCROLLBAR_MODE_OFF); lv_obj_add_event_cb(ta_repair_countdown, numeric_textarea_focus_event_cb, LV_EVENT_FOCUSED, NULL); lv_obj_add_event_cb(ta_repair_countdown, generic_textarea_defocus_event_cb, LV_EVENT_DEFOCUSED, NULL); lv_obj_add_event_cb(ta_repair_countdown, lab_mode_param_changed_event_cb, LV_EVENT_DEFOCUSED, (void*)"repair_countdown");}
    { lv_obj_t* row = lv_obj_create(timers_params_cont); lv_obj_remove_style_all(row); lv_obj_set_size(row, lv_pct(100), LV_SIZE_CONTENT); lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW); lv_obj_set_flex_align(row, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER); label_repair_uv_exposure = lv_label_create(row); lv_obj_add_style(label_repair_uv_exposure, &style_my_text_18, 0); ta_repair_uv_exposure = lv_textarea_create(row); lv_textarea_set_one_line(ta_repair_uv_exposure, true); lv_obj_set_size(ta_repair_uv_exposure, 80, TEXT_INPUT_HEIGHT); lv_obj_add_style(ta_repair_uv_exposure, &style_edit_textarea, 0); lv_obj_set_scrollbar_mode(ta_repair_uv_exposure, LV_SCROLLBAR_MODE_OFF); lv_obj_add_event_cb(ta_repair_uv_exposure, numeric_textarea_focus_event_cb, LV_EVENT_FOCUSED, NULL); lv_obj_add_event_cb(ta_repair_uv_exposure, generic_textarea_defocus_event_cb, LV_EVENT_DEFOCUSED, NULL); lv_obj_add_event_cb(ta_repair_uv_exposure, lab_mode_param_changed_event_cb, LV_EVENT_DEFOCUSED, (void*)"repair_uv_exposure");}

    lv_obj_t* block_warning = lv_obj_create(content_area);
    lv_obj_add_style(block_warning, &style_param_block, 0);   
    lv_obj_set_width(block_warning, lv_pct(100));
    lv_obj_set_flex_grow(block_warning, 1);
    lv_obj_set_flex_flow(block_warning, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(block_warning, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);
    lv_obj_set_style_pad_gap(block_warning, 10, 0);

    lv_obj_t* warning_header = lv_label_create(block_warning);
    lv_obj_add_style(warning_header, &style_my_text_18, 0);
    lv_obj_add_style(warning_header, &style_block_header, 0);
    lv_obj_set_style_bg_color(warning_header, lv_palette_main(LV_PALETTE_AMBER), 0);
    lv_obj_set_style_text_color(warning_header, lv_color_black(), 0);
    lv_label_set_text(warning_header, "ВНИМАНИЕ!");
    lv_obj_set_width(warning_header, lv_pct(100));
    lv_obj_set_style_text_align(warning_header, LV_TEXT_ALIGN_CENTER, 0);

    label_repair_info_text = lv_label_create(block_warning);
    lv_obj_add_style(label_repair_info_text, &style_my_text_18, 0);
    lv_label_set_long_mode(label_repair_info_text, LV_LABEL_LONG_WRAP);
    lv_obj_set_style_text_align(label_repair_info_text, LV_TEXT_ALIGN_LEFT, 0);
    lv_obj_set_width(label_repair_info_text, lv_pct(100));

    label_repair_arrow_text = lv_label_create(block_warning);
    lv_obj_add_style(label_repair_arrow_text, &style_my_text_18, 0);
    lv_label_set_long_mode(label_repair_arrow_text, LV_LABEL_LONG_WRAP);
    lv_obj_set_style_text_align(label_repair_arrow_text, LV_TEXT_ALIGN_LEFT, 0);
    lv_obj_set_width(label_repair_arrow_text, lv_pct(100));
    
    lv_obj_t* footer = lv_obj_create(main_container);
    lv_obj_remove_style_all(footer);
    lv_obj_set_size(footer, lv_pct(100), LV_SIZE_CONTENT);
    lv_obj_set_style_pad_all(footer, 5, 0);
    lv_obj_set_flex_flow(footer, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(footer, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_gap(footer, 20, 0);
    btn_repair_start = lv_btn_create(footer);
    lv_obj_set_flex_grow(btn_repair_start, 1);
    lv_obj_set_height(btn_repair_start, 45);
    lv_obj_add_event_cb(btn_repair_start, repair_screen_event_cb, LV_EVENT_CLICKED, (void*)"repair_start");
    label_btn_repair_start = lv_label_create(btn_repair_start);
    lv_obj_add_style(label_btn_repair_start, &style_my_text_18_white, 0);
    lv_obj_center(label_btn_repair_start);
    btn_repair_back = lv_btn_create(footer);
    lv_obj_set_flex_grow(btn_repair_back, 1);
    lv_obj_set_height(btn_repair_back, 45);
    lv_obj_add_event_cb(btn_repair_back, repair_screen_event_cb, LV_EVENT_CLICKED, (void*)"repair_back");
    label_btn_repair_back = lv_label_create(btn_repair_back);
    lv_obj_add_style(label_btn_repair_back, &style_my_text_18_white, 0);
    lv_obj_center(label_btn_repair_back);
}

static void strength_screen_event_cb(lv_event_t* e) {
    const char* user_data = (const char*)lv_event_get_user_data(e);
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;

    if (strcmp(user_data, "strength_show_help") == 0) {
        if (current_global_settings.language == 1) { // RUS
            show_info_dialog("Справка: Повышение прочности", "Термическая постобработка с УФ-импульсами для создания дополнительных поперечных связей в структуре полимера, что делает его более твердым и износостойким.");
        } else { // ENG
            show_info_dialog("Help: Strength Boost", "Thermal post-processing with UV pulses to create additional cross-links in the polymer structure, making it harder and more wear-resistant.");
        }
    } else if (strcmp(user_data, "strength_back") == 0) {
        if (help_blink_timer) { lv_timer_del(help_blink_timer); help_blink_timer = nullptr; }
        trigger_lab_action(1); // 1 = Назад
    } else if (strcmp(user_data, "strength_start") == 0) {
        trigger_lab_action(4);
    }
}

// --- НОВАЯ ФУНКЦИЯ ДЛЯ ПОСТРОЕНИЯ ЭКРАНА "ПОВЫШЕНИЕ ПРОЧНОСТИ" ---
static void build_lab_strength_screen(lv_obj_t* parent_screen) {
    screen_lab_strength = parent_screen;
    Serial.println("Building THEME-AGNOSTIC lab_strength_screen UI...");

    const lv_coord_t TEXT_INPUT_HEIGHT = 36;
    static lv_style_t style_edit_textarea;
    lv_style_init(&style_edit_textarea);
    lv_style_set_text_align(&style_edit_textarea, LV_TEXT_ALIGN_CENTER);
    lv_style_set_text_font(&style_edit_textarea, &montserrat_rus_18);
    lv_style_set_pad_ver(&style_edit_textarea, 2);

    static lv_style_t style_param_block;
    lv_style_init(&style_param_block);
    lv_style_set_border_width(&style_param_block, 2);
    lv_style_set_radius(&style_param_block, 8);
    lv_style_set_pad_all(&style_param_block, 10);

    lv_obj_t* main_container = lv_obj_create(screen_lab_strength);
    lv_obj_set_size(main_container, lv_pct(100), lv_pct(100));
    lv_obj_center(main_container);
    lv_obj_set_flex_flow(main_container, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_all(main_container, 5, 0);
    lv_obj_set_style_border_width(main_container, 0, 0);
    lv_obj_set_style_radius(main_container, 0, 0);

    lv_obj_t* header = lv_obj_create(main_container);
    lv_obj_remove_style_all(header);
    lv_obj_set_size(header, lv_pct(100), LV_SIZE_CONTENT);
    lv_obj_set_style_pad_all(header, 5, 0);
    lv_obj_set_layout(header, LV_LAYOUT_FLEX);
    lv_obj_set_flex_flow(header, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(header, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    label_strength_header = lv_label_create(header);
    lv_obj_add_style(label_strength_header, &style_my_text_22, 0);
    lv_obj_set_flex_grow(label_strength_header, 1);
    lv_obj_set_style_text_align(label_strength_header, LV_TEXT_ALIGN_LEFT, 0);
    btn_strength_help = lv_btn_create(header);
    lv_obj_set_size(btn_strength_help, LV_SIZE_CONTENT, 30);
    lv_obj_add_event_cb(btn_strength_help, strength_screen_event_cb, LV_EVENT_CLICKED, (void*)"strength_show_help");
    lv_obj_t* label_help_btn = lv_label_create(btn_strength_help);
    lv_obj_add_style(label_help_btn, &style_my_text_18, 0);
    lv_obj_center(label_help_btn);

    lv_obj_t* content_area = lv_obj_create(main_container);
    lv_obj_remove_style_all(content_area);
    lv_obj_set_width(content_area, lv_pct(100));
    lv_obj_set_flex_grow(content_area, 1);
    lv_obj_set_layout(content_area, LV_LAYOUT_GRID);
    static lv_coord_t col_dsc[] = {LV_GRID_FR(1), 10, LV_GRID_FR(1), LV_GRID_TEMPLATE_LAST};
    static lv_coord_t row_dsc[] = {LV_GRID_FR(1), LV_GRID_TEMPLATE_LAST};
    lv_obj_set_grid_dsc_array(content_area, col_dsc, row_dsc);

    lv_obj_t* left_col = lv_obj_create(content_area);
    lv_obj_add_style(left_col, &style_param_block, 0);
    lv_obj_set_grid_cell(left_col, LV_GRID_ALIGN_STRETCH, 0, 1, LV_GRID_ALIGN_STRETCH, 0, 1);
    lv_obj_set_flex_flow(left_col, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_gap(left_col, 5, 0);
    
    label_strength_title_thermo = lv_label_create(left_col);
    lv_obj_add_style(label_strength_title_thermo, &style_my_text_18, 0);
    lv_obj_add_style(label_strength_title_thermo, &style_block_header, 0);
    lv_obj_set_width(label_strength_title_thermo, lv_pct(100));
    lv_obj_set_style_text_align(label_strength_title_thermo, LV_TEXT_ALIGN_CENTER, 0);

    lv_obj_t* thermo_params_cont = lv_obj_create(left_col);
    lv_obj_remove_style_all(thermo_params_cont);
    lv_obj_set_width(thermo_params_cont, lv_pct(100));
    lv_obj_set_flex_grow(thermo_params_cont, 1);
    lv_obj_set_flex_flow(thermo_params_cont, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(thermo_params_cont, LV_FLEX_ALIGN_SPACE_AROUND, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);
    { lv_obj_t* row = lv_obj_create(thermo_params_cont); lv_obj_remove_style_all(row); lv_obj_set_width(row, lv_pct(100)); lv_obj_set_height(row, LV_SIZE_CONTENT); lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW); lv_obj_set_flex_align(row, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER); label_strength_temp = lv_label_create(row); lv_obj_add_style(label_strength_temp, &style_my_text_18, 0); ta_strength_temp = lv_textarea_create(row); lv_textarea_set_one_line(ta_strength_temp, true); lv_obj_set_size(ta_strength_temp, 80, TEXT_INPUT_HEIGHT); lv_obj_add_style(ta_strength_temp, &style_edit_textarea, 0); lv_obj_set_scrollbar_mode(ta_strength_temp, LV_SCROLLBAR_MODE_OFF); lv_obj_add_event_cb(ta_strength_temp, numeric_textarea_focus_event_cb, LV_EVENT_FOCUSED, NULL); lv_obj_add_event_cb(ta_strength_temp, generic_textarea_defocus_event_cb, LV_EVENT_DEFOCUSED, NULL); lv_obj_add_event_cb(ta_strength_temp, lab_mode_param_changed_event_cb, LV_EVENT_DEFOCUSED, (void*)"strength_temp"); }
    { lv_obj_t* row = lv_obj_create(thermo_params_cont); lv_obj_remove_style_all(row); lv_obj_set_width(row, lv_pct(100)); lv_obj_set_height(row, LV_SIZE_CONTENT); lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW); lv_obj_set_flex_align(row, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER); label_strength_hold_time = lv_label_create(row); lv_obj_add_style(label_strength_hold_time, &style_my_text_18, 0); ta_strength_hold_time = lv_textarea_create(row); lv_textarea_set_one_line(ta_strength_hold_time, true); lv_obj_set_size(ta_strength_hold_time, 80, TEXT_INPUT_HEIGHT); lv_obj_add_style(ta_strength_hold_time, &style_edit_textarea, 0); lv_obj_set_scrollbar_mode(ta_strength_hold_time, LV_SCROLLBAR_MODE_OFF); lv_obj_add_event_cb(ta_strength_hold_time, numeric_textarea_focus_event_cb, LV_EVENT_FOCUSED, NULL); lv_obj_add_event_cb(ta_strength_hold_time, generic_textarea_defocus_event_cb, LV_EVENT_DEFOCUSED, NULL); lv_obj_add_event_cb(ta_strength_hold_time, lab_mode_param_changed_event_cb, LV_EVENT_DEFOCUSED, (void*)"strength_hold"); }
    { lv_obj_t* row = lv_obj_create(thermo_params_cont); lv_obj_remove_style_all(row); lv_obj_set_width(row, lv_pct(100)); lv_obj_set_height(row, LV_SIZE_CONTENT); lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW); lv_obj_set_flex_align(row, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER); label_strength_cooling = lv_label_create(row); lv_obj_add_style(label_strength_cooling, &style_my_text_18, 0); sw_strength_cooling = lv_switch_create(row); lv_obj_add_event_cb(sw_strength_cooling, lab_mode_switch_event_cb, LV_EVENT_VALUE_CHANGED, (void*)"strength_cooling"); lv_obj_add_event_cb(sw_strength_cooling, lab_mode_param_changed_event_cb, LV_EVENT_VALUE_CHANGED, (void*)"strength_cooling"); }

    lv_obj_t* right_col = lv_obj_create(content_area);
    lv_obj_add_style(right_col, &style_param_block, 0);
    lv_obj_set_grid_cell(right_col, LV_GRID_ALIGN_STRETCH, 2, 1, LV_GRID_ALIGN_STRETCH, 0, 1);
    lv_obj_set_flex_flow(right_col, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_gap(right_col, 5, 0);

    label_strength_title_uv = lv_label_create(right_col);
    lv_obj_add_style(label_strength_title_uv, &style_my_text_18, 0);
    lv_obj_add_style(label_strength_title_uv, &style_block_header, 0);
    lv_obj_set_width(label_strength_title_uv, lv_pct(100));
    lv_obj_set_style_text_align(label_strength_title_uv, LV_TEXT_ALIGN_CENTER, 0);

    lv_obj_t* uv_params_cont = lv_obj_create(right_col);
    lv_obj_remove_style_all(uv_params_cont);
    lv_obj_set_width(uv_params_cont, lv_pct(100));
    lv_obj_set_flex_grow(uv_params_cont, 1);
    lv_obj_set_flex_flow(uv_params_cont, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(uv_params_cont, LV_FLEX_ALIGN_SPACE_AROUND, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);
    { lv_obj_t* row = lv_obj_create(uv_params_cont); lv_obj_remove_style_all(row); lv_obj_set_width(row, lv_pct(100)); lv_obj_set_height(row, LV_SIZE_CONTENT); lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW); lv_obj_set_flex_align(row, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER); label_strength_uv_pulse_duration = lv_label_create(row); lv_obj_add_style(label_strength_uv_pulse_duration, &style_my_text_18, 0); ta_strength_uv_pulse_duration = lv_textarea_create(row); lv_textarea_set_one_line(ta_strength_uv_pulse_duration, true); lv_obj_set_size(ta_strength_uv_pulse_duration, 80, TEXT_INPUT_HEIGHT); lv_obj_add_style(ta_strength_uv_pulse_duration, &style_edit_textarea, 0); lv_obj_set_scrollbar_mode(ta_strength_uv_pulse_duration, LV_SCROLLBAR_MODE_OFF); lv_obj_add_event_cb(ta_strength_uv_pulse_duration, numeric_textarea_focus_event_cb, LV_EVENT_FOCUSED, NULL); lv_obj_add_event_cb(ta_strength_uv_pulse_duration, generic_textarea_defocus_event_cb, LV_EVENT_DEFOCUSED, NULL); lv_obj_add_event_cb(ta_strength_uv_pulse_duration, lab_mode_param_changed_event_cb, LV_EVENT_DEFOCUSED, (void*)"strength_pulse_dur"); }
    { lv_obj_t* row = lv_obj_create(uv_params_cont); lv_obj_remove_style_all(row); lv_obj_set_width(row, lv_pct(100)); lv_obj_set_height(row, LV_SIZE_CONTENT); lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW); lv_obj_set_flex_align(row, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER); label_strength_uv_pulse_interval = lv_label_create(row); lv_obj_add_style(label_strength_uv_pulse_interval, &style_my_text_18, 0); ta_strength_uv_pulse_interval = lv_textarea_create(row); lv_textarea_set_one_line(ta_strength_uv_pulse_interval, true); lv_obj_set_size(ta_strength_uv_pulse_interval, 80, TEXT_INPUT_HEIGHT); lv_obj_add_style(ta_strength_uv_pulse_interval, &style_edit_textarea, 0); lv_obj_set_scrollbar_mode(ta_strength_uv_pulse_interval, LV_SCROLLBAR_MODE_OFF); lv_obj_add_event_cb(ta_strength_uv_pulse_interval, numeric_textarea_focus_event_cb, LV_EVENT_FOCUSED, NULL); lv_obj_add_event_cb(ta_strength_uv_pulse_interval, generic_textarea_defocus_event_cb, LV_EVENT_DEFOCUSED, NULL); lv_obj_add_event_cb(ta_strength_uv_pulse_interval, lab_mode_param_changed_event_cb, LV_EVENT_DEFOCUSED, (void*)"strength_pulse_int"); }
    label_strength_info_uv_text = lv_label_create(uv_params_cont);
    lv_obj_add_style(label_strength_info_uv_text, &style_my_text_16, 0);
    lv_obj_set_align(label_strength_info_uv_text, LV_ALIGN_BOTTOM_LEFT);
    lv_label_set_long_mode(label_strength_info_uv_text, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(label_strength_info_uv_text, lv_pct(100));

    lv_obj_t* footer = lv_obj_create(main_container);
    lv_obj_remove_style_all(footer);
    lv_obj_set_size(footer, lv_pct(100), LV_SIZE_CONTENT);
    lv_obj_set_style_pad_all(footer, 5, 0);
    lv_obj_set_flex_flow(footer, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(footer, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_gap(footer, 20, 0);
    btn_strength_start = lv_btn_create(footer);
    lv_obj_set_flex_grow(btn_strength_start, 1);
    lv_obj_set_height(btn_strength_start, 45);
    lv_obj_add_event_cb(btn_strength_start, strength_screen_event_cb, LV_EVENT_CLICKED, (void*)"strength_start");
    label_btn_strength_start = lv_label_create(btn_strength_start);
    lv_obj_add_style(label_btn_strength_start, &style_my_text_18_white, 0);
    lv_obj_center(label_btn_strength_start);
    btn_strength_back = lv_btn_create(footer);
    lv_obj_set_flex_grow(btn_strength_back, 1);
    lv_obj_set_height(btn_strength_back, 45);
    lv_obj_add_event_cb(btn_strength_back, strength_screen_event_cb, LV_EVENT_CLICKED, (void*)"strength_back");
    label_btn_strength_back = lv_label_create(btn_strength_back);
    lv_obj_add_style(label_btn_strength_back, &style_my_text_18_white, 0);
    lv_obj_center(label_btn_strength_back);
}

static void thermal_screen_event_cb(lv_event_t* e) {
    const char* user_data = (const char*)lv_event_get_user_data(e);
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;

    if (strcmp(user_data, "thermal_show_help") == 0) {
        if (current_global_settings.language == 1) { // RUS
            show_info_dialog("Справка: Термокамера", "Режим простого нагрева камеры до заданной температуры и удержания в течение указанного времени. Используется для различных лабораторных нужд.");
        } else { // ENG
            show_info_dialog("Help: Thermal Chamber", "A simple mode for heating the chamber to a set temperature and holding it for a specified time. Used for various laboratory needs.");
        }
    } else if (strcmp(user_data, "thermal_back") == 0) {
        if (help_blink_timer) { lv_timer_del(help_blink_timer); help_blink_timer = nullptr; }
        trigger_lab_action(1); // 1 = Назад
    } else if (strcmp(user_data, "thermal_start") == 0) {
        trigger_lab_action(5);
    }
}

static void build_lab_thermal_screen(lv_obj_t* parent_screen) {
    screen_lab_thermal = parent_screen;
    Serial.println("Building THEME-AGNOSTIC lab_thermal_screen UI...");

    const lv_coord_t TEXT_INPUT_HEIGHT = 36;
    static lv_style_t style_edit_textarea;
    lv_style_init(&style_edit_textarea);
    lv_style_set_text_align(&style_edit_textarea, LV_TEXT_ALIGN_CENTER);
    lv_style_set_text_font(&style_edit_textarea, &montserrat_rus_18);
    lv_style_set_pad_ver(&style_edit_textarea, 6);

    static lv_style_t style_param_block;
    lv_style_init(&style_param_block);
    lv_style_set_border_width(&style_param_block, 2);
    lv_style_set_radius(&style_param_block, 8);
    lv_style_set_pad_all(&style_param_block, 10);
    
    static lv_style_t style_info_text;
    lv_style_init(&style_info_text);
    lv_style_set_text_align(&style_info_text, LV_TEXT_ALIGN_LEFT);

    lv_obj_t* main_container = lv_obj_create(screen_lab_thermal);
    lv_obj_set_size(main_container, lv_pct(100), lv_pct(100));
    lv_obj_center(main_container);
    lv_obj_set_flex_flow(main_container, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_all(main_container, 5, 0);
    lv_obj_set_style_border_width(main_container, 0, 0);
    lv_obj_set_style_radius(main_container, 0, 0);

    lv_obj_t* header = lv_obj_create(main_container);
    lv_obj_remove_style_all(header);
    lv_obj_set_size(header, lv_pct(100), LV_SIZE_CONTENT);
    lv_obj_set_style_pad_all(header, 5, 0);
    lv_obj_set_layout(header, LV_LAYOUT_FLEX);
    lv_obj_set_flex_flow(header, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(header, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

    label_thermal_header = lv_label_create(header);
    lv_obj_add_style(label_thermal_header, &style_my_text_22, 0);
    lv_obj_set_flex_grow(label_thermal_header, 1);
    lv_obj_set_style_text_align(label_thermal_header, LV_TEXT_ALIGN_LEFT, 0);
    
    btn_thermal_help = lv_btn_create(header);
    lv_obj_set_size(btn_thermal_help, LV_SIZE_CONTENT, 30);
    lv_obj_add_event_cb(btn_thermal_help, thermal_screen_event_cb, LV_EVENT_CLICKED, (void*)"thermal_show_help");
    lv_obj_t* label_help_btn = lv_label_create(btn_thermal_help);
    lv_obj_add_style(label_help_btn, &style_my_text_18, 0);
    lv_obj_center(label_help_btn);

    lv_obj_t* content_area = lv_obj_create(main_container);
    lv_obj_remove_style_all(content_area);
    lv_obj_set_width(content_area, lv_pct(100));
    lv_obj_set_flex_grow(content_area, 1);
    lv_obj_set_flex_flow(content_area, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(content_area, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);

    lv_obj_t* block_thermo = lv_obj_create(content_area);
    lv_obj_add_style(block_thermo, &style_param_block, 0);
    lv_obj_set_size(block_thermo, lv_pct(100), lv_pct(100));
    lv_obj_set_flex_flow(block_thermo, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_gap(block_thermo, 5, 0);

    label_thermal_title_thermo = lv_label_create(block_thermo);
    lv_obj_add_style(label_thermal_title_thermo, &style_my_text_18, 0);
    lv_obj_add_style(label_thermal_title_thermo, &style_block_header, 0);
    lv_obj_set_width(label_thermal_title_thermo, lv_pct(100));
    lv_obj_set_style_text_align(label_thermal_title_thermo, LV_TEXT_ALIGN_CENTER, 0);

    lv_obj_t* thermo_params_cont = lv_obj_create(block_thermo);
    lv_obj_remove_style_all(thermo_params_cont);
    lv_obj_set_width(thermo_params_cont, lv_pct(100));
    lv_obj_set_flex_grow(thermo_params_cont, 1);
    lv_obj_set_flex_flow(thermo_params_cont, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(thermo_params_cont, LV_FLEX_ALIGN_SPACE_EVENLY, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);
    lv_obj_set_style_pad_hor(thermo_params_cont, 10, 0);
    
    lv_obj_t* temp_group = lv_obj_create(thermo_params_cont);
    lv_obj_remove_style_all(temp_group);
    lv_obj_set_width(temp_group, lv_pct(100));
    lv_obj_set_height(temp_group, LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(temp_group, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_gap(temp_group, 2, 0);

    { 
        lv_obj_t* row = lv_obj_create(temp_group); 
        lv_obj_remove_style_all(row); 
        lv_obj_set_width(row, lv_pct(100)); 
        lv_obj_set_height(row, LV_SIZE_CONTENT); 
        lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW); 
        lv_obj_set_flex_align(row, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER); 
        label_thermal_temp = lv_label_create(row); 
        lv_obj_add_style(label_thermal_temp, &style_my_text_18, 0); 
        ta_thermal_temp = lv_textarea_create(row); 
        lv_textarea_set_one_line(ta_thermal_temp, true); 
        lv_obj_set_size(ta_thermal_temp, 80, TEXT_INPUT_HEIGHT); 
        lv_obj_add_style(ta_thermal_temp, &style_edit_textarea, 0);
        lv_obj_set_scrollbar_mode(ta_thermal_temp, LV_SCROLLBAR_MODE_OFF);
        lv_obj_add_event_cb(ta_thermal_temp, numeric_textarea_focus_event_cb, LV_EVENT_FOCUSED, NULL);
        lv_obj_add_event_cb(ta_thermal_temp, generic_textarea_defocus_event_cb, LV_EVENT_DEFOCUSED, NULL);
        lv_obj_add_event_cb(ta_thermal_temp, lab_mode_param_changed_event_cb, LV_EVENT_DEFOCUSED, (void*)"thermal_temp");
    }
    
    label_thermal_info = lv_label_create(temp_group);
    lv_obj_add_style(label_thermal_info, &style_info_text, 0);
    lv_obj_add_style(label_thermal_info, &style_my_text_16, 0);
    lv_obj_set_width(label_thermal_info, lv_pct(100));

    { 
        lv_obj_t* row = lv_obj_create(thermo_params_cont); 
        lv_obj_remove_style_all(row); 
        lv_obj_set_width(row, lv_pct(100)); 
        lv_obj_set_height(row, LV_SIZE_CONTENT); 
        lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW); 
        lv_obj_set_flex_align(row, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER); 
        label_thermal_hold_time = lv_label_create(row); 
        lv_obj_add_style(label_thermal_hold_time, &style_my_text_18, 0); 
        ta_thermal_hold_time = lv_textarea_create(row); 
        lv_textarea_set_one_line(ta_thermal_hold_time, true); 
        lv_obj_set_size(ta_thermal_hold_time, 80, TEXT_INPUT_HEIGHT); 
        lv_obj_add_style(ta_thermal_hold_time, &style_edit_textarea, 0);
        lv_obj_set_scrollbar_mode(ta_thermal_hold_time, LV_SCROLLBAR_MODE_OFF);
        lv_obj_add_event_cb(ta_thermal_hold_time, numeric_textarea_focus_event_cb, LV_EVENT_FOCUSED, NULL);
        lv_obj_add_event_cb(ta_thermal_hold_time, generic_textarea_defocus_event_cb, LV_EVENT_DEFOCUSED, NULL);
        lv_obj_add_event_cb(ta_thermal_hold_time, lab_mode_param_changed_event_cb, LV_EVENT_DEFOCUSED, (void*)"thermal_hold");
    }

    lv_obj_t* footer = lv_obj_create(main_container);
    lv_obj_remove_style_all(footer);
    lv_obj_set_size(footer, lv_pct(100), LV_SIZE_CONTENT);
    lv_obj_set_style_pad_all(footer, 5, 0);
    lv_obj_set_flex_flow(footer, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(footer, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_gap(footer, 20, 0);
    btn_thermal_start = lv_btn_create(footer);
    lv_obj_set_flex_grow(btn_thermal_start, 1);
    lv_obj_set_height(btn_thermal_start, 45);
    lv_obj_add_event_cb(btn_thermal_start, thermal_screen_event_cb, LV_EVENT_CLICKED, (void*)"thermal_start");
    label_btn_thermal_start = lv_label_create(btn_thermal_start);
    lv_obj_add_style(label_btn_thermal_start, &style_my_text_18_white, 0);
    lv_obj_center(label_btn_thermal_start);
    btn_thermal_back = lv_btn_create(footer);
    lv_obj_set_flex_grow(btn_thermal_back, 1);
    lv_obj_set_height(btn_thermal_back, 45);
    lv_obj_add_event_cb(btn_thermal_back, thermal_screen_event_cb, LV_EVENT_CLICKED, (void*)"thermal_back");
    label_btn_thermal_back = lv_label_create(btn_thermal_back);
    lv_obj_add_style(label_btn_thermal_back, &style_my_text_18_white, 0);
    lv_obj_center(label_btn_thermal_back);
}

static void lighten_screen_event_cb(lv_event_t* e) {
    const char* user_data = (const char*)lv_event_get_user_data(e);
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;

    if (strcmp(user_data, "lighten_show_help") == 0) {
        if (current_global_settings.language == 1) { // RUS
            show_info_dialog("Справка: Осветление", "Этот режим осветляет композитные материалы путем выдержки при фиксированной высокой температуре.");
        } else { // ENG
            show_info_dialog("Help: Composite Lightening", "This mode lightens composite materials by holding them at a fixed high temperature.");
        }
    } else if (strcmp(user_data, "lighten_back") == 0) {
        if (help_blink_timer) { lv_timer_del(help_blink_timer); help_blink_timer = nullptr; }
        trigger_lab_action(1); // 1 = Назад
    } else if (strcmp(user_data, "lighten_start") == 0) {
        trigger_lab_action(6);
    }
}

static void build_lab_lighten_screen(lv_obj_t* parent_screen) {
    screen_lab_lighten = parent_screen;
    Serial.println("Building THEME-AGNOSTIC lab_lighten_screen UI...");

    const lv_coord_t TEXT_INPUT_HEIGHT = 36;
    static lv_style_t style_edit_textarea;
    lv_style_init(&style_edit_textarea);
    lv_style_set_text_align(&style_edit_textarea, LV_TEXT_ALIGN_CENTER);
    lv_style_set_text_font(&style_edit_textarea, &montserrat_rus_18);
    lv_style_set_pad_ver(&style_edit_textarea, 6);

    static lv_style_t style_param_block;
    lv_style_init(&style_param_block);
    lv_style_set_border_width(&style_param_block, 2);
    lv_style_set_radius(&style_param_block, 8);
    lv_style_set_pad_all(&style_param_block, 10);
    
    lv_obj_t* main_container = lv_obj_create(screen_lab_lighten);
    lv_obj_set_size(main_container, lv_pct(100), lv_pct(100));
    lv_obj_center(main_container);
    lv_obj_set_flex_flow(main_container, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_all(main_container, 5, 0);
    lv_obj_set_style_border_width(main_container, 0, 0);
    lv_obj_set_style_radius(main_container, 0, 0);

    lv_obj_t* header = lv_obj_create(main_container);
    lv_obj_remove_style_all(header);
    lv_obj_set_size(header, lv_pct(100), LV_SIZE_CONTENT);
    lv_obj_set_style_pad_all(header, 5, 0);
    lv_obj_set_layout(header, LV_LAYOUT_FLEX);
    lv_obj_set_flex_flow(header, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(header, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    label_lighten_header = lv_label_create(header);
    lv_obj_add_style(label_lighten_header, &style_my_text_22, 0);
    lv_obj_set_flex_grow(label_lighten_header, 1);
    lv_obj_set_style_text_align(label_lighten_header, LV_TEXT_ALIGN_LEFT, 0);
    btn_lighten_help = lv_btn_create(header);
    lv_obj_set_size(btn_lighten_help, LV_SIZE_CONTENT, 30);
    lv_obj_add_event_cb(btn_lighten_help, lighten_screen_event_cb, LV_EVENT_CLICKED, (void*)"lighten_show_help");
    lv_obj_t* label_help_btn = lv_label_create(btn_lighten_help);
    lv_obj_add_style(label_help_btn, &style_my_text_18, 0);
    lv_obj_center(label_help_btn);

    lv_obj_t* content_area = lv_obj_create(main_container);
    lv_obj_remove_style_all(content_area);
    lv_obj_set_width(content_area, lv_pct(100));
    lv_obj_set_flex_grow(content_area, 1);
    lv_obj_set_flex_flow(content_area, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(content_area, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);

    lv_obj_t* block_thermo = lv_obj_create(content_area);
    lv_obj_add_style(block_thermo, &style_param_block, 0);
    lv_obj_set_size(block_thermo, lv_pct(100), lv_pct(100));
    lv_obj_set_flex_flow(block_thermo, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_gap(block_thermo, 5, 0);
    label_lighten_title_thermo = lv_label_create(block_thermo);
    lv_obj_add_style(label_lighten_title_thermo, &style_my_text_18, 0);
    lv_obj_add_style(label_lighten_title_thermo, &style_block_header, 0);
    lv_obj_set_width(label_lighten_title_thermo, lv_pct(100));
    lv_obj_set_style_text_align(label_lighten_title_thermo, LV_TEXT_ALIGN_CENTER, 0);

    lv_obj_t* params_cont = lv_obj_create(block_thermo);
    lv_obj_remove_style_all(params_cont);
    lv_obj_set_width(params_cont, lv_pct(100));
    lv_obj_set_flex_grow(params_cont, 1);
    lv_obj_set_flex_flow(params_cont, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(params_cont, LV_FLEX_ALIGN_SPACE_EVENLY, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);
    lv_obj_set_style_pad_hor(params_cont, 10, 0);

    { 
        lv_obj_t* row = lv_obj_create(params_cont); 
        lv_obj_remove_style_all(row); 
        lv_obj_set_width(row, lv_pct(100)); 
        lv_obj_set_height(row, LV_SIZE_CONTENT); 
        label_lighten_temp_fixed = lv_label_create(row); 
        lv_obj_add_style(label_lighten_temp_fixed, &style_my_text_18, 0); 
    }

    { 
        lv_obj_t* row = lv_obj_create(params_cont);
        lv_obj_remove_style_all(row);
        lv_obj_set_width(row, lv_pct(100));
        lv_obj_set_height(row, LV_SIZE_CONTENT);
        lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW);
        lv_obj_set_flex_align(row, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
        label_lighten_hold_time = lv_label_create(row);
        lv_obj_add_style(label_lighten_hold_time, &style_my_text_18, 0);
        ta_lighten_hold_time = lv_textarea_create(row);
        lv_textarea_set_one_line(ta_lighten_hold_time, true);
        lv_obj_set_size(ta_lighten_hold_time, 80, TEXT_INPUT_HEIGHT);
        lv_obj_add_style(ta_lighten_hold_time, &style_edit_textarea, 0);
        lv_obj_set_scrollbar_mode(ta_lighten_hold_time, LV_SCROLLBAR_MODE_OFF);
        lv_obj_add_event_cb(ta_lighten_hold_time, numeric_textarea_focus_event_cb, LV_EVENT_FOCUSED, NULL);
        lv_obj_add_event_cb(ta_lighten_hold_time, generic_textarea_defocus_event_cb, LV_EVENT_DEFOCUSED, NULL);
        lv_obj_add_event_cb(ta_lighten_hold_time, lab_mode_param_changed_event_cb, LV_EVENT_DEFOCUSED, (void*)"lighten_hold");
    }

    {
        lv_obj_t* row = lv_obj_create(params_cont);
        lv_obj_remove_style_all(row);
        lv_obj_set_width(row, lv_pct(100));
        lv_obj_set_height(row, LV_SIZE_CONTENT);
        lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW);
        lv_obj_set_flex_align(row, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
        label_lighten_cooling = lv_label_create(row);
        lv_obj_add_style(label_lighten_cooling, &style_my_text_18, 0);
        sw_lighten_cooling = lv_switch_create(row);
        lv_obj_add_event_cb(sw_lighten_cooling, lab_mode_switch_event_cb, LV_EVENT_VALUE_CHANGED, (void*)"lighten_cooling");
        lv_obj_add_event_cb(sw_lighten_cooling, lab_mode_param_changed_event_cb, LV_EVENT_VALUE_CHANGED, (void*)"lighten_cooling");
    }

    lv_obj_t* footer = lv_obj_create(main_container);
    lv_obj_remove_style_all(footer);
    lv_obj_set_size(footer, lv_pct(100), LV_SIZE_CONTENT);
    lv_obj_set_style_pad_all(footer, 5, 0);
    lv_obj_set_flex_flow(footer, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(footer, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_gap(footer, 20, 0);
    btn_lighten_start = lv_btn_create(footer);
    lv_obj_set_flex_grow(btn_lighten_start, 1);
    lv_obj_set_height(btn_lighten_start, 45);
    lv_obj_add_event_cb(btn_lighten_start, lighten_screen_event_cb, LV_EVENT_CLICKED, (void*)"lighten_start");
    label_btn_lighten_start = lv_label_create(btn_lighten_start);
    lv_obj_add_style(label_btn_lighten_start, &style_my_text_18_white, 0);
    lv_obj_center(label_btn_lighten_start);
    btn_lighten_back = lv_btn_create(footer);
    lv_obj_set_flex_grow(btn_lighten_back, 1);
    lv_obj_set_height(btn_lighten_back, 45);
    lv_obj_add_event_cb(btn_lighten_back, lighten_screen_event_cb, LV_EVENT_CLICKED, (void*)"lighten_back");
    label_btn_lighten_back = lv_label_create(btn_lighten_back);
    lv_obj_add_style(label_btn_lighten_back, &style_my_text_18_white, 0);
    lv_obj_center(label_btn_lighten_back);
}

static void darken_screen_event_cb(lv_event_t* e) {
    const char* user_data = (const char*)lv_event_get_user_data(e);
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;

    if (strcmp(user_data, "darken_show_help") == 0) {
        if (current_global_settings.language == 1) { // RUS
            show_info_dialog("Справка: Затемнение", "Этот режим затемняет композитные материалы с помощью УФ-облучения.");
        } else { // ENG
            show_info_dialog("Help: Composite Darkening", "This mode darkens composite materials using UV exposure.");
        }
    } else if (strcmp(user_data, "darken_back") == 0) {
        if (help_blink_timer) { lv_timer_del(help_blink_timer); help_blink_timer = nullptr; }
        trigger_lab_action(1); // 1 = Назад
    } else if (strcmp(user_data, "darken_start") == 0) {
        trigger_lab_action(7);
    }
}

// --- Функция для построения экрана "Затемнение" ---
static void build_lab_darken_screen(lv_obj_t* parent_screen) {
    screen_lab_darken = parent_screen;
    Serial.println("Building THEME-AGNOSTIC lab_darken_screen UI...");

    const lv_coord_t TEXT_INPUT_HEIGHT = 36;
    static lv_style_t style_edit_textarea;
    lv_style_init(&style_edit_textarea);
    lv_style_set_text_align(&style_edit_textarea, LV_TEXT_ALIGN_CENTER);
    lv_style_set_text_font(&style_edit_textarea, &montserrat_rus_18);
    lv_style_set_pad_ver(&style_edit_textarea, 6);

    static lv_style_t style_param_block;
    lv_style_init(&style_param_block);
    lv_style_set_border_width(&style_param_block, 2);
    lv_style_set_radius(&style_param_block, 8);
    lv_style_set_pad_all(&style_param_block, 10);
    
    lv_obj_t* main_container = lv_obj_create(screen_lab_darken);
    lv_obj_set_size(main_container, lv_pct(100), lv_pct(100));
    lv_obj_center(main_container);
    lv_obj_set_flex_flow(main_container, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_all(main_container, 5, 0);
    lv_obj_set_style_border_width(main_container, 0, 0);
    lv_obj_set_style_radius(main_container, 0, 0);

    lv_obj_t* header = lv_obj_create(main_container);
    lv_obj_remove_style_all(header);
    lv_obj_set_size(header, lv_pct(100), LV_SIZE_CONTENT);
    lv_obj_set_style_pad_all(header, 5, 0);
    lv_obj_set_layout(header, LV_LAYOUT_FLEX);
    lv_obj_set_flex_flow(header, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(header, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    label_darken_header = lv_label_create(header);
    lv_obj_add_style(label_darken_header, &style_my_text_22, 0);
    lv_obj_set_flex_grow(label_darken_header, 1);
    lv_obj_set_style_text_align(label_darken_header, LV_TEXT_ALIGN_LEFT, 0);
    btn_darken_help = lv_btn_create(header);
    lv_obj_set_size(btn_darken_help, LV_SIZE_CONTENT, 30);
    lv_obj_add_event_cb(btn_darken_help, darken_screen_event_cb, LV_EVENT_CLICKED, (void*)"darken_show_help");
    lv_obj_t* label_help_btn = lv_label_create(btn_darken_help);
    lv_obj_add_style(label_help_btn, &style_my_text_18, 0);
    lv_obj_center(label_help_btn);

    lv_obj_t* content_area = lv_obj_create(main_container);
    lv_obj_remove_style_all(content_area);
    lv_obj_set_width(content_area, lv_pct(100));
    lv_obj_set_flex_grow(content_area, 1);
    lv_obj_set_flex_flow(content_area, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(content_area, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);

    lv_obj_t* block_params = lv_obj_create(content_area);
    lv_obj_add_style(block_params, &style_param_block, 0);
    lv_obj_set_size(block_params, lv_pct(100), lv_pct(100));
    lv_obj_set_flex_flow(block_params, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_gap(block_params, 5, 0);
    label_darken_title_params = lv_label_create(block_params);
    lv_obj_add_style(label_darken_title_params, &style_my_text_18, 0);
    lv_obj_add_style(label_darken_title_params, &style_block_header, 0);
    lv_obj_set_width(label_darken_title_params, lv_pct(100));
    lv_obj_set_style_text_align(label_darken_title_params, LV_TEXT_ALIGN_CENTER, 0);

    lv_obj_t* params_cont = lv_obj_create(block_params);
    lv_obj_remove_style_all(params_cont);
    lv_obj_set_width(params_cont, lv_pct(100));
    lv_obj_set_flex_grow(params_cont, 1);
    lv_obj_set_flex_flow(params_cont, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(params_cont, LV_FLEX_ALIGN_SPACE_EVENLY, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);
    lv_obj_set_style_pad_hor(params_cont, 10, 0);

    {
        lv_obj_t* row = lv_obj_create(params_cont);
        lv_obj_remove_style_all(row);
        lv_obj_set_width(row, lv_pct(100));
        lv_obj_set_height(row, LV_SIZE_CONTENT);
        lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW);
        lv_obj_set_flex_align(row, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
        label_darken_uv_exposure = lv_label_create(row);
        lv_obj_add_style(label_darken_uv_exposure, &style_my_text_18, 0);
        ta_darken_uv_exposure = lv_textarea_create(row);
        lv_textarea_set_one_line(ta_darken_uv_exposure, true);
        lv_obj_set_size(ta_darken_uv_exposure, 80, TEXT_INPUT_HEIGHT);
        lv_obj_add_style(ta_darken_uv_exposure, &style_edit_textarea, 0);
        lv_obj_set_scrollbar_mode(ta_darken_uv_exposure, LV_SCROLLBAR_MODE_OFF);
        lv_obj_add_event_cb(ta_darken_uv_exposure, numeric_textarea_focus_event_cb, LV_EVENT_FOCUSED, NULL);
        lv_obj_add_event_cb(ta_darken_uv_exposure, generic_textarea_defocus_event_cb, LV_EVENT_DEFOCUSED, NULL);
        lv_obj_add_event_cb(ta_darken_uv_exposure, lab_mode_param_changed_event_cb, LV_EVENT_DEFOCUSED, (void*)"darken_exposure");
    }

    {
        lv_obj_t* row = lv_obj_create(params_cont);
        lv_obj_remove_style_all(row);
        lv_obj_set_width(row, lv_pct(100));
        lv_obj_set_height(row, LV_SIZE_CONTENT);
        lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW);
        lv_obj_set_flex_align(row, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
        label_darken_cooling = lv_label_create(row);
        lv_obj_add_style(label_darken_cooling, &style_my_text_18, 0);
        sw_darken_cooling = lv_switch_create(row);
        lv_obj_add_event_cb(sw_darken_cooling, lab_mode_switch_event_cb, LV_EVENT_VALUE_CHANGED, (void*)"darken_cooling");
        lv_obj_add_event_cb(sw_darken_cooling, lab_mode_param_changed_event_cb, LV_EVENT_VALUE_CHANGED, (void*)"darken_cooling");
    }
    
    lv_obj_t* footer = lv_obj_create(main_container);
    lv_obj_remove_style_all(footer);
    lv_obj_set_size(footer, lv_pct(100), LV_SIZE_CONTENT);
    lv_obj_set_style_pad_all(footer, 5, 0);
    lv_obj_set_flex_flow(footer, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(footer, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_gap(footer, 20, 0);
    btn_darken_start = lv_btn_create(footer);
    lv_obj_set_flex_grow(btn_darken_start, 1);
    lv_obj_set_height(btn_darken_start, 45);
    lv_obj_add_event_cb(btn_darken_start, darken_screen_event_cb, LV_EVENT_CLICKED, (void*)"darken_start");
    label_btn_darken_start = lv_label_create(btn_darken_start);
    lv_obj_add_style(label_btn_darken_start, &style_my_text_18_white, 0);
    lv_obj_center(label_btn_darken_start);
    btn_darken_back = lv_btn_create(footer);
    lv_obj_set_flex_grow(btn_darken_back, 1);
    lv_obj_set_height(btn_darken_back, 45);
    lv_obj_add_event_cb(btn_darken_back, darken_screen_event_cb, LV_EVENT_CLICKED, (void*)"darken_back");
    label_btn_darken_back = lv_label_create(btn_darken_back);
    lv_obj_add_style(label_btn_darken_back, &style_my_text_18_white, 0);
    lv_obj_center(label_btn_darken_back);
}

static void build_help_screen(lv_obj_t* parent_screen) {
    screen_help = parent_screen;
    Serial.println("Building help_screen UI...");

    // Главный контейнер
    lv_obj_t* main_container = lv_obj_create(screen_help);
    lv_obj_set_size(main_container, lv_pct(100), lv_pct(100));
    lv_obj_center(main_container);
    lv_obj_set_flex_flow(main_container, LV_FLEX_FLOW_COLUMN);
    
    // <<< НАЧАЛО ИЗМЕНЕНИЙ (БЛОК 1) >>>
    // Убираем отступы, рамку и скругление у главного контейнера
    lv_obj_set_style_pad_all(main_container, 0, 0);
    lv_obj_set_style_border_width(main_container, 0, 0);
    lv_obj_set_style_radius(main_container, 0, 0);
    // Добавим внешние отступы, чтобы контент не прилипал к краям экрана
    lv_obj_set_style_pad_top(main_container, 10, 0);
    lv_obj_set_style_pad_bottom(main_container, 10, 0);
    lv_obj_set_style_pad_hor(main_container, 15, 0);
    // <<< КОНЕЦ ИЗМЕНЕНИЙ (БЛОК 1) >>>
    
    lv_obj_set_style_pad_gap(main_container, 10, 0);

    // 1. Заголовок (без изменений)
    label_help_title = lv_label_create(main_container);
    lv_label_set_text(label_help_title, "Help Section");
    lv_obj_add_style(label_help_title, &style_my_text_22, 0);
    lv_obj_set_width(label_help_title, lv_pct(100));
    lv_obj_set_style_text_align(label_help_title, LV_TEXT_ALIGN_CENTER, 0);

    // 2. Контейнер для текста с прокруткой
    lv_obj_t* text_container = lv_obj_create(main_container);
    
    // <<< НАЧАЛО ИЗМЕНЕНИЙ (БЛОК 2) >>>
    lv_obj_remove_style_all(text_container); // Убираем все стили, включая фон и рамку
    // <<< КОНЕЦ ИЗМЕНЕНИЙ (БЛОК 2) >>>

    lv_obj_set_flex_grow(text_container, 1);
    lv_obj_set_width(text_container, lv_pct(100));

    // <<< НАЧАЛО ИЗМЕНЕНИЙ (БЛОК 3) >>>
    // Было: lv_obj_set_scrollbar_mode(text_container, LV_SCROLLBAR_MODE_AUTO);
    // Стало:
    lv_obj_set_scrollbar_mode(text_container, LV_SCROLLBAR_MODE_OFF); // Выключаем полосу прокрутки
    lv_obj_add_flag(text_container, LV_OBJ_FLAG_SCROLLABLE); // Но саму возможность скроллинга пальцем оставляем
    // <<< КОНЕЦ ИЗМЕНЕНИЙ (БЛОК 3) >>>

    // 3. Текстовое поле (без изменений)
    label_help_content = lv_label_create(text_container);
    lv_obj_add_style(label_help_content, &style_my_text_18, 0);
    lv_label_set_long_mode(label_help_content, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(label_help_content, lv_pct(100));
    lv_label_set_text(label_help_content, "Loading help content...");

    // 4. Кнопка "Понятно" (без изменений)
    btn_help_close = lv_btn_create(main_container);
    lv_obj_add_event_cb(btn_help_close, help_button_event_cb, LV_EVENT_CLICKED, (void*)"close_help");
    lv_obj_set_width(btn_help_close, lv_pct(50));
    lv_obj_align(btn_help_close, LV_ALIGN_CENTER, 0, 0);

    label_btn_help_close = lv_label_create(btn_help_close);
    lv_label_set_text(label_btn_help_close, "Got it!");
    lv_obj_add_style(label_btn_help_close, &style_my_text_18_white, 0);
    lv_obj_center(label_btn_help_close);
}
static void build_input_shield(void) {
    // Создаем щит на самом верхнем слое (lv_layer_top()), чтобы он был поверх всего
    input_shield = lv_obj_create(lv_layer_top());
    
    // Делаем его полностью прозрачным
    lv_obj_remove_style_all(input_shield);
    
    // Растягиваем на весь экран
    lv_obj_set_size(input_shield, LV_PCT(100), LV_PCT(100));
    
    // По умолчанию он скрыт и неактивен для касаний
    lv_obj_add_flag(input_shield, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(input_shield, LV_OBJ_FLAG_CLICKABLE);
    
    Serial.println("Input shield object created and is hidden/passive.");
}

static void start_splash_inversion_anim(lv_timer_t* timer) {
    // 1. Получаем указатель на нашу структуру
    SplashScreenData* data = (SplashScreenData*)timer->user_data;
    lv_obj_t* screen = data->screen;
    lv_obj_t* title_label = data->title;

    // Удаляем таймер, так как он больше не нужен
    lv_timer_del(timer);
    
    Serial.println("Starting splash screen inversion animation (STABLE version)...");

    // 2. Анимация ФОНА (код без изменений)
    lv_anim_t bg_anim;
    lv_anim_init(&bg_anim);
    lv_anim_set_var(&bg_anim, screen);
    lv_anim_set_exec_cb(&bg_anim, [](void* obj, int32_t v) {
        lv_color_t color = lv_color_mix(lv_color_white(), lv_color_black(), v);
        lv_obj_set_style_bg_color((lv_obj_t*)obj, color, 0);
    });
    lv_anim_set_values(&bg_anim, 255, 0);
    lv_anim_set_time(&bg_anim, 1500);
    lv_anim_start(&bg_anim);

    // 3. Анимация ТЕКСТА (код без изменений, но теперь с надежным указателем)
    if (title_label) {
        lv_anim_t text_anim;
        lv_anim_init(&text_anim);
        lv_anim_set_var(&text_anim, title_label);
        lv_anim_set_exec_cb(&text_anim, [](void* obj, int32_t v) {
            lv_color_t color = lv_color_mix(lv_color_black(), lv_color_white(), v);
            lv_obj_set_style_text_color((lv_obj_t*)obj, color, 0);
        });
        lv_anim_set_values(&text_anim, 255, 0);
        lv_anim_set_time(&text_anim, 1500);
        lv_anim_start(&text_anim);
    } else {
        Serial.println("   ERROR: title_label was not found for animation!");
    }
}

static void build_loading_screen(lv_obj_t* parent_screen) {
    screen_loading = parent_screen;
    lv_obj_clear_flag(screen_loading, LV_OBJ_FLAG_SCROLLABLE);

    // 1. Создаем главный контейнер, как на всех других экранах.
    // Именно этот контейнер будет получать цвет фона от темы.
    lv_obj_t* main_container = lv_obj_create(screen_loading);
    lv_obj_set_size(main_container, lv_pct(100), lv_pct(100));
    lv_obj_center(main_container);
    lv_obj_set_style_border_width(main_container, 0, 0);
    lv_obj_set_style_radius(main_container, 0, 0);

    // 2. Помещаем текст ВНУТРЬ этого контейнера, а не прямо на экран.
    label_loading_text = lv_label_create(main_container);
    lv_obj_set_style_text_font(label_loading_text, &montserrat_rus_22, 0);
    lv_obj_center(label_loading_text);
    lv_label_set_text(label_loading_text, "..."); // Начальный текст-заглушка
}

static void build_splash_screen(lv_obj_t* parent_screen) {
    Serial.println("Building splash screen with error label support (STABLE)...");

    // --- Основной заголовок (снова локальная переменная) ---
    lv_obj_t* title_label = lv_label_create(parent_screen); 
    lv_label_set_text(title_label, "SpectraMaster N2");
    lv_obj_set_style_text_color(title_label, lv_color_black(), 0);
    lv_obj_set_style_text_font(title_label, &lv_font_montserrat_48, 0);
    lv_obj_align(title_label, LV_ALIGN_CENTER, 0, -20);

    // --- Метка для сообщений об ошибках ---
    label_splash_error = lv_label_create(parent_screen);
    lv_label_set_text(label_splash_error, "");
    lv_obj_set_style_text_color(label_splash_error, lv_palette_main(LV_PALETTE_RED), 0);
    lv_obj_add_style(label_splash_error, &style_my_text_18, 0);
    lv_obj_set_width(label_splash_error, lv_pct(80));
    lv_obj_set_style_text_align(label_splash_error, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_long_mode(label_splash_error, LV_LABEL_LONG_WRAP);
    lv_obj_align_to(label_splash_error, title_label, LV_ALIGN_OUT_BOTTOM_MID, 0, 15);
    lv_obj_add_flag(label_splash_error, LV_OBJ_FLAG_HIDDEN);
}

void build_main_app_screen(lv_obj_t* parent_screen) {
    Serial.println("Building THEME-AGNOSTIC main_app_screen UI...");
    
    lv_obj_set_scrollbar_mode(parent_screen, LV_SCROLLBAR_MODE_OFF);
    lv_obj_clear_flag(parent_screen, LV_OBJ_FLAG_SCROLLABLE);

    ta_dummy_for_new_profile = lv_textarea_create(parent_screen);
    lv_textarea_set_max_length(ta_dummy_for_new_profile, 200);
    lv_obj_add_flag(ta_dummy_for_new_profile, LV_OBJ_FLAG_HIDDEN);

    main_screen_content_container = lv_obj_create(parent_screen);
    lv_obj_set_size(main_screen_content_container, lv_pct(100), lv_pct(100));
    lv_obj_set_flex_flow(main_screen_content_container, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_all(main_screen_content_container, 5, 0);
    lv_obj_set_style_pad_gap(main_screen_content_container, 10, 0);
    lv_obj_set_style_border_width(main_screen_content_container, 0, 0);
    lv_obj_set_style_radius(main_screen_content_container, 0, 0);

    list_header_label_main = lv_label_create(main_screen_content_container);
    lv_obj_add_style(list_header_label_main, &style_my_text_22, 0);
    lv_obj_set_width(list_header_label_main, lv_pct(100));
    lv_obj_set_style_text_align(list_header_label_main, LV_TEXT_ALIGN_CENTER, 0);
    
    lv_obj_t* center_panel_container = lv_obj_create(main_screen_content_container);
    lv_obj_remove_style_all(center_panel_container);
    lv_obj_set_width(center_panel_container, lv_pct(100));
    lv_obj_set_flex_grow(center_panel_container, 1);
    lv_obj_set_layout(center_panel_container, LV_LAYOUT_FLEX);
    lv_obj_set_flex_flow(center_panel_container, LV_FLEX_FLOW_ROW);

    btn_profiles_prev = lv_btn_create(center_panel_container);
    lv_obj_remove_style_all(btn_profiles_prev);
    lv_obj_set_size(btn_profiles_prev, 35, lv_pct(100));
    lv_obj_add_event_cb(btn_profiles_prev, profile_list_prev_btn_event_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t* lbl_left = lv_label_create(btn_profiles_prev);
    lv_label_set_text(lbl_left, LV_SYMBOL_LEFT);
    lv_obj_center(lbl_left);

    list_profiles_main = lv_obj_create(center_panel_container);
    lv_obj_remove_style_all(list_profiles_main);
    lv_obj_set_flex_grow(list_profiles_main, 1);
    lv_obj_set_height(list_profiles_main, lv_pct(100));
    lv_obj_clear_flag(list_profiles_main, LV_OBJ_FLAG_SCROLLABLE); 
    lv_obj_set_layout(list_profiles_main, LV_LAYOUT_FLEX);
    lv_obj_set_flex_flow(list_profiles_main, LV_FLEX_FLOW_ROW_WRAP);
    lv_obj_set_flex_align(list_profiles_main, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_START);
    lv_obj_set_style_pad_all(list_profiles_main, 5, 0);
    lv_obj_set_style_pad_column(list_profiles_main, 8, 0);
    lv_obj_set_style_pad_row(list_profiles_main, 8, 0);

    btn_profiles_next = lv_btn_create(center_panel_container);
    lv_obj_remove_style_all(btn_profiles_next);
    lv_obj_set_size(btn_profiles_next, 35, lv_pct(100));
    lv_obj_add_event_cb(btn_profiles_next, profile_list_next_btn_event_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t* lbl_right = lv_label_create(btn_profiles_next);
    lv_label_set_text(lbl_right, LV_SYMBOL_RIGHT);
    lv_obj_center(lbl_right);

    lv_obj_t* actions_btn_container = lv_obj_create(main_screen_content_container);
    lv_obj_remove_style_all(actions_btn_container);
    lv_obj_set_style_pad_all(actions_btn_container, 5, 0);
    lv_obj_set_width(actions_btn_container, lv_pct(100));
    lv_obj_set_height(actions_btn_container, LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(actions_btn_container, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(actions_btn_container, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_gap(actions_btn_container, 10, 0);

    btn_add_main = lv_btn_create(actions_btn_container);
    lv_obj_add_event_cb(btn_add_main, add_new_profile_btn_event_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_set_flex_grow(btn_add_main, 1);
    label_btn_add_main = lv_label_create(btn_add_main);
    lv_label_set_text(label_btn_add_main, LV_SYMBOL_PLUS "Add Profile"); 
    lv_obj_center(label_btn_add_main); 
    
    btn_lab_main = lv_btn_create(actions_btn_container);
    lv_obj_add_event_cb(btn_lab_main, laboratory_mode_btn_event_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_set_flex_grow(btn_lab_main, 1);
    label_btn_lab_main = lv_label_create(btn_lab_main); 
    lv_label_set_text(label_btn_lab_main, LV_SYMBOL_SETTINGS "Lab Mode"); 
    lv_obj_center(label_btn_lab_main); 

    btn_settings_main = lv_btn_create(actions_btn_container);
    lv_obj_add_event_cb(btn_settings_main, btn_goto_settings_event_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_set_flex_grow(btn_settings_main, 1);
    label_btn_settings_main = lv_label_create(btn_settings_main); 
    lv_label_set_text(label_btn_settings_main, "Settings"); 
    lv_obj_center(label_btn_settings_main); 
}

static void build_laboratory_screen(lv_obj_t* parent_screen) {
    screen_laboratory = parent_screen;
    Serial.println("Building THEME-AGNOSTIC laboratory_screen UI...");
    
    // Этот стиль больше не нужен, его заменит логика тем
    // static lv_style_t style_lab_tile;
    // lv_style_init(&style_lab_tile);
    // lv_style_set_bg_color(&style_lab_tile, lv_color_hex(0xE0E0E0)); 
    // lv_style_set_radius(&style_lab_tile, 5);
    // lv_style_set_border_width(&style_lab_tile, 0);

    lv_obj_t* content_container = lv_obj_create(parent_screen);
    lv_obj_set_size(content_container, lv_pct(100), lv_pct(100));
    lv_obj_center(content_container);
    lv_obj_set_flex_flow(content_container, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_all(content_container, 5, 0);
    lv_obj_set_style_pad_gap(content_container, 10, 0);
    lv_obj_set_style_border_width(content_container, 0, 0);
    lv_obj_set_style_radius(content_container, 0, 0);

    label_lab_header = lv_label_create(content_container);
    lv_obj_add_style(label_lab_header, &style_my_text_22, 0);
    lv_obj_set_width(label_lab_header, lv_pct(100));
    lv_obj_set_style_text_align(label_lab_header, LV_TEXT_ALIGN_CENTER, 0);

    lv_obj_t* tiles_container = lv_obj_create(content_container);
    lv_obj_remove_style_all(tiles_container);
    lv_obj_set_width(tiles_container, lv_pct(100));
    lv_obj_set_flex_grow(tiles_container, 1);
    
    static lv_coord_t col_dsc[] = {LV_GRID_FR(1), LV_GRID_FR(1), LV_GRID_TEMPLATE_LAST};
    static lv_coord_t row_dsc[] = {LV_GRID_FR(1), LV_GRID_FR(1), LV_GRID_FR(1), LV_GRID_TEMPLATE_LAST};
    lv_obj_set_layout(tiles_container, LV_LAYOUT_GRID);
    lv_obj_set_grid_dsc_array(tiles_container, col_dsc, row_dsc);
    lv_obj_set_style_pad_all(tiles_container, 10, 0);
    lv_obj_set_style_pad_gap(tiles_container, 15, 0);

    lv_obj_t** tile_labels[] = {
        &label_lab_tile_1, &label_lab_tile_2, &label_lab_tile_3,
        &label_lab_tile_4, &label_lab_tile_5, &label_lab_tile_6
    };

    for (int i = 0; i < 6; i++) {
        uint8_t row = i / 2;
        uint8_t col = i % 2;

        lv_obj_t* tile = lv_btn_create(tiles_container);
        lv_obj_set_grid_cell(tile, LV_GRID_ALIGN_STRETCH, col, 1, LV_GRID_ALIGN_STRETCH, row, 1);
        lv_obj_add_event_cb(tile, lab_mode_tile_event_cb, LV_EVENT_CLICKED, (void*)(intptr_t)(i + 1));
        
        lv_obj_set_style_border_width(tile, 1, 0);
        lv_obj_set_style_border_color(tile, lv_palette_main(LV_PALETTE_GREY), 0);
        
        *(tile_labels[i]) = lv_label_create(tile);
        lv_label_set_text(*(tile_labels[i]), "...");
        lv_obj_center(*(tile_labels[i]));
    }

    lv_obj_t* footer_container = lv_obj_create(content_container);
    lv_obj_remove_style_all(footer_container);
    lv_obj_set_style_pad_all(footer_container, 5, 0);
    lv_obj_set_height(footer_container, LV_SIZE_CONTENT);
    lv_obj_set_width(footer_container, lv_pct(100));
    lv_obj_set_flex_align(footer_container, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

    btn_lab_back = lv_btn_create(footer_container);
    lv_obj_set_width(btn_lab_back, lv_pct(50));
    lv_obj_add_event_cb(btn_lab_back, lab_screen_back_event_cb, LV_EVENT_CLICKED, NULL);
    label_btn_lab_back = lv_label_create(btn_lab_back);
    lv_obj_add_style(label_btn_lab_back, &style_my_text_18_white, 0);
    lv_obj_center(label_btn_lab_back);
}

void build_profile_details_screen(lv_obj_t* parent_screen) {
    Serial.println("Building THEME-AGNOSTIC profile_details_screen UI...");

    lv_obj_t* content_container = lv_obj_create(parent_screen);
    lv_obj_set_size(content_container, lv_pct(100), lv_pct(100));
    lv_obj_align(content_container, LV_ALIGN_TOP_LEFT, 0, 0);
    lv_obj_set_flex_flow(content_container, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_all(content_container, 5, 0);
    lv_obj_set_style_pad_gap(content_container, 10, 0);
    lv_obj_set_scrollbar_mode(content_container, LV_SCROLLBAR_MODE_OFF);
    lv_obj_clear_flag(content_container, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_radius(content_container, 0, 0);
    lv_obj_set_style_border_width(content_container, 0, 0);

    label_detail_header = lv_label_create(content_container); 
    lv_label_set_text(label_detail_header, "Profile name and details:");
    lv_obj_set_width(label_detail_header, lv_pct(100));                          
    lv_obj_set_style_text_align(label_detail_header, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_add_style(label_detail_header, &style_my_text_18, 0);

    label_detail_view_profile_name = lv_label_create(content_container);
    lv_label_set_text(label_detail_view_profile_name, "Loading...");
    lv_obj_set_style_text_font(label_detail_view_profile_name, &lv_font_montserrat_32, 0);
    lv_obj_set_width(label_detail_view_profile_name, lv_pct(100));
    lv_obj_set_style_text_align(label_detail_view_profile_name, LV_TEXT_ALIGN_CENTER, 0);

    lv_obj_t* details_content_block = lv_obj_create(content_container);
    lv_obj_remove_style_all(details_content_block);
    lv_obj_set_width(details_content_block, lv_pct(100));
    lv_obj_set_flex_grow(details_content_block, 1);
    lv_obj_set_flex_flow(details_content_block, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(details_content_block, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);
    lv_obj_set_style_pad_gap(details_content_block, 15, 0);
    lv_obj_set_style_pad_hor(details_content_block, 15, 0);
    lv_obj_set_style_pad_ver(details_content_block, 10, 0);

    label_detail_view_id = lv_label_create(details_content_block);
    lv_obj_add_style(label_detail_view_id, &style_my_text_22, 0); 
    label_detail_view_thermal_chamber_enabled = lv_label_create(details_content_block);
    lv_obj_add_style(label_detail_view_thermal_chamber_enabled, &style_my_text_22, 0); 
    label_detail_view_thermal_chamber = lv_label_create(details_content_block);
    lv_obj_add_style(label_detail_view_thermal_chamber, &style_my_text_22, 0); 
    label_detail_view_chamber_cooling = lv_label_create(details_content_block);
    lv_obj_add_style(label_detail_view_chamber_cooling, &style_my_text_22, 0);
    label_detail_view_nitrogen = lv_label_create(details_content_block);
    lv_obj_add_style(label_detail_view_nitrogen, &style_my_text_22, 0); 
    label_detail_view_primary_uv = lv_label_create(details_content_block);
    lv_obj_add_style(label_detail_view_primary_uv, &style_my_text_22, 0); 
    label_detail_view_secondary_uv = lv_label_create(details_content_block);
    lv_obj_add_style(label_detail_view_secondary_uv, &style_my_text_22, 0); 
    label_detail_view_tertiary_uv = lv_label_create(details_content_block);
    lv_obj_add_style(label_detail_view_tertiary_uv, &style_my_text_22, 0);

    lv_obj_t* actions_btn_container = lv_obj_create(content_container);
    lv_obj_remove_style_all(actions_btn_container);
    lv_obj_set_style_pad_all(actions_btn_container, 5, 0);
    lv_obj_set_width(actions_btn_container, lv_pct(100));
    lv_obj_set_height(actions_btn_container, LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(actions_btn_container, LV_FLEX_FLOW_ROW);
    lv_obj_set_style_pad_gap(actions_btn_container, 10, 0);
    
    lv_obj_t* btn_start = lv_btn_create(actions_btn_container); 
    lv_obj_add_event_cb(btn_start, profile_detail_start_event_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_set_flex_grow(btn_start, 1);
    label_detail_btn_start = lv_label_create(btn_start); 
    lv_obj_add_style(label_detail_btn_start, &style_my_text_18_white, 0);
    lv_obj_center(label_detail_btn_start); 

    lv_obj_t* btn_edit = lv_btn_create(actions_btn_container);
    lv_obj_add_event_cb(btn_edit, profile_detail_edit_btn_event_cb, LV_EVENT_CLICKED, NULL); 
    lv_obj_set_flex_grow(btn_edit, 1);
    label_detail_btn_edit = lv_label_create(btn_edit); 
    lv_obj_add_style(label_detail_btn_edit, &style_my_text_18_white, 0);
    lv_obj_center(label_detail_btn_edit); 
    
    lv_obj_t* btn_delete = lv_btn_create(actions_btn_container);
    lv_obj_set_style_bg_color(btn_delete, lv_palette_main(LV_PALETTE_RED), 0);
    lv_obj_add_event_cb(btn_delete, profile_detail_delete_btn_event_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_set_flex_grow(btn_delete, 1);
    label_detail_btn_delete = lv_label_create(btn_delete); 
    lv_obj_add_style(label_detail_btn_delete, &style_my_text_18_white, 0);
    lv_obj_center(label_detail_btn_delete); 
    
    lv_obj_t* btn_close = lv_btn_create(actions_btn_container);
    lv_obj_add_event_cb(btn_close, profile_detail_close_event_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_set_flex_grow(btn_close, 1);
    label_detail_btn_close = lv_label_create(btn_close);
    lv_obj_add_style(label_detail_btn_close, &style_my_text_18_white, 0);
    lv_obj_center(label_detail_btn_close); 
}
static void build_profile_edit_screen(lv_obj_t* parent_screen) {
    screen_profile_edit = parent_screen;
    lv_obj_clear_flag(parent_screen, LV_OBJ_FLAG_SCROLLABLE);
    Serial.println("Building THEME-AGNOSTIC profile_edit_screen UI (Corrected)...");

    const lv_coord_t TEXT_INPUT_HEIGHT = 34;

    static lv_style_t style_edit_textarea;
    lv_style_init(&style_edit_textarea);
    lv_style_set_text_align(&style_edit_textarea, LV_TEXT_ALIGN_CENTER);
    lv_style_set_text_font(&style_edit_textarea, &montserrat_rus_18);
    lv_style_set_pad_ver(&style_edit_textarea, 2);

    static lv_style_t style_column_header;
    lv_style_init(&style_column_header);
    lv_style_set_pad_all(&style_column_header, 5);
    lv_style_set_radius(&style_column_header, 3);
    lv_style_set_width(&style_column_header, lv_pct(100));
    lv_style_set_text_align(&style_column_header, LV_TEXT_ALIGN_CENTER);

    static lv_style_t style_param_block;
    lv_style_init(&style_param_block);
    lv_style_set_border_width(&style_param_block, 2);
    lv_style_set_radius(&style_param_block, 8);
    lv_style_set_pad_all(&style_param_block, 10);
    
    static lv_style_t style_switch_row;
    lv_style_init(&style_switch_row);
    lv_style_set_radius(&style_switch_row, 5);
    lv_style_set_pad_hor(&style_switch_row, 5);
    lv_style_set_pad_ver(&style_switch_row, 3);

    lv_obj_t* main_container = lv_obj_create(parent_screen);
    lv_obj_set_size(main_container, lv_pct(100), lv_pct(100));
    lv_obj_center(main_container);
    lv_obj_set_style_pad_all(main_container, 5, 0);
    lv_obj_set_layout(main_container, LV_LAYOUT_GRID);
    static lv_coord_t main_col_dsc[] = {LV_GRID_FR(1), LV_GRID_TEMPLATE_LAST};
    static lv_coord_t main_row_dsc[] = {LV_GRID_CONTENT, LV_GRID_FR(1), LV_GRID_CONTENT, LV_GRID_TEMPLATE_LAST};
    lv_obj_set_grid_dsc_array(main_container, main_col_dsc, main_row_dsc);
    lv_obj_set_style_border_width(main_container, 0, 0);
    lv_obj_set_style_radius(main_container, 0, 0);
    lv_obj_set_style_pad_row(main_container, 2, 0);

    header_container = lv_obj_create(main_container);
    lv_obj_remove_style_all(header_container);
    lv_obj_set_size(header_container, lv_pct(100), LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(header_container, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(header_container, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_gap(header_container, 10, 0);
    lv_obj_set_grid_cell(header_container, LV_GRID_ALIGN_STRETCH, 0, 1, LV_GRID_ALIGN_START, 0, 1);
    label_edit_name_title = lv_label_create(header_container);
    lv_obj_add_style(label_edit_name_title, &style_my_text_18, 0);
    ta_edit_profile_name = lv_textarea_create(header_container);
    lv_obj_set_flex_grow(ta_edit_profile_name, 1);
    lv_textarea_set_one_line(ta_edit_profile_name, true);
    lv_textarea_set_max_length(ta_edit_profile_name, 200);
    lv_obj_add_event_cb(ta_edit_profile_name, alpha_textarea_focus_event_cb, LV_EVENT_FOCUSED, NULL);
    lv_obj_set_height(ta_edit_profile_name, 40);
    lv_obj_set_scrollbar_mode(ta_edit_profile_name, LV_SCROLLBAR_MODE_OFF);
    btn_help_section = lv_btn_create(header_container);
    lv_obj_set_width(btn_help_section, LV_SIZE_CONTENT);
    lv_obj_add_event_cb(btn_help_section, help_button_event_cb, LV_EVENT_CLICKED, (void*)"open_help");
    lv_obj_set_height(btn_help_section, 36);
    label_btn_help_section = lv_label_create(btn_help_section);
    lv_obj_add_style(label_btn_help_section, &style_my_text_18, 0);
    lv_obj_center(label_btn_help_section);

    main_content_container = lv_obj_create(main_container);
    lv_obj_remove_style_all(main_content_container);
    lv_obj_set_width(main_content_container, lv_pct(100));
    lv_obj_set_flex_grow(main_content_container, 1);
    lv_obj_set_layout(main_content_container, LV_LAYOUT_GRID);
    static lv_coord_t col_dsc[] = {LV_GRID_FR(1), LV_GRID_FR(1), LV_GRID_TEMPLATE_LAST};
    static lv_coord_t row_dsc[] = {LV_GRID_CONTENT, LV_GRID_FR(1), LV_GRID_TEMPLATE_LAST};
    lv_obj_set_grid_dsc_array(main_content_container, col_dsc, row_dsc);
    lv_obj_set_style_pad_column(main_content_container, 10, 0);
    lv_obj_set_grid_cell(main_content_container, LV_GRID_ALIGN_STRETCH, 0, 1, LV_GRID_ALIGN_STRETCH, 1, 1);
    left_column = lv_obj_create(main_content_container);
    lv_obj_remove_style_all(left_column);
    lv_obj_set_grid_cell(left_column, LV_GRID_ALIGN_STRETCH, 0, 1, LV_GRID_ALIGN_STRETCH, 0, 2);
    lv_obj_set_flex_flow(left_column, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_gap(left_column, 5, 0);
    right_column = lv_obj_create(main_content_container);
    lv_obj_remove_style_all(right_column);
    lv_obj_set_grid_cell(right_column, LV_GRID_ALIGN_STRETCH, 1, 1, LV_GRID_ALIGN_STRETCH, 0, 2);
    lv_obj_set_flex_flow(right_column, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_gap(right_column, 5, 0);
    
    header_uv_params = lv_label_create(left_column);
    lv_obj_add_style(header_uv_params, &style_my_text_18, 0);
    lv_obj_add_style(header_uv_params, &style_column_header, 0);

    block_uv_all_stages = lv_obj_create(left_column);
    lv_obj_add_style(block_uv_all_stages, &style_param_block, 0);
    lv_obj_set_flex_flow(block_uv_all_stages, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(block_uv_all_stages, LV_FLEX_ALIGN_SPACE_AROUND, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);
    lv_obj_set_flex_grow(block_uv_all_stages, 1);
    lv_obj_set_width(block_uv_all_stages, lv_pct(100));
    lv_obj_set_scrollbar_mode(block_uv_all_stages, LV_SCROLLBAR_MODE_OFF);
    lv_obj_clear_flag(block_uv_all_stages, LV_OBJ_FLAG_SCROLLABLE);
    
    { // --- ПЕРВЫЙ ЭТАП УФ ---
        row_uv_primary = lv_obj_create(block_uv_all_stages);
        lv_obj_remove_style_all(row_uv_primary);
        lv_obj_add_style(row_uv_primary, &style_switch_row, 0);
        lv_obj_set_size(row_uv_primary, lv_pct(100), LV_SIZE_CONTENT);
        lv_obj_set_flex_flow(row_uv_primary, LV_FLEX_FLOW_ROW);
        lv_obj_set_flex_align(row_uv_primary, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
        
        label_uv_primary_title = lv_label_create(row_uv_primary);
        lv_obj_add_style(label_uv_primary_title, &style_my_text_18, 0);
        label_primary_uv_mode_status = lv_label_create(row_uv_primary);
        lv_obj_add_style(label_primary_uv_mode_status, &style_my_text_18, 0);

        lv_obj_t* btn_container = lv_obj_create(row_uv_primary);
        lv_obj_remove_style_all(btn_container);
        lv_obj_set_size(btn_container, 150, 40);
        lv_obj_set_flex_flow(btn_container, LV_FLEX_FLOW_ROW);
        lv_obj_set_flex_align(btn_container, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
        lv_obj_set_style_pad_gap(btn_container, 5, 0);

        btn_primary_1 = lv_btn_create(btn_container);
        btn_primary_2 = lv_btn_create(btn_container);
        btn_primary_3 = lv_btn_create(btn_container);
        
        lv_obj_t* btns[] = {btn_primary_1, btn_primary_2, btn_primary_3};
        for(int i = 0; i < 3; i++) {
            lv_obj_set_size(btns[i], 40, 40);
            lv_obj_add_flag(btns[i], LV_OBJ_FLAG_CHECKABLE);
            lv_obj_set_style_radius(btns[i], LV_RADIUS_CIRCLE, 0);
            lv_obj_t* lbl = lv_label_create(btns[i]);
            lv_label_set_text_fmt(lbl, "%d", i + 1);
            lv_obj_add_style(lbl, &style_my_text_18_white, 0);
            lv_obj_center(lbl);
            lv_obj_add_event_cb(btns[i], uv_mode_btn_group_event_cb, LV_EVENT_CLICKED, label_primary_uv_mode_status);
        }
    }
    {
        lv_obj_t* row = lv_obj_create(block_uv_all_stages);
        lv_obj_remove_style_all(row);
        lv_obj_set_size(row, lv_pct(100), LV_SIZE_CONTENT);
        lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW);
        lv_obj_set_flex_align(row, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
        label_edit_flicker_rate_title = lv_label_create(row);
        lv_obj_add_style(label_edit_flicker_rate_title, &style_my_text_18, 0);
        ta_edit_flicker_rate = lv_textarea_create(row);
        lv_textarea_set_one_line(ta_edit_flicker_rate, true);
        lv_obj_set_size(ta_edit_flicker_rate, 80, TEXT_INPUT_HEIGHT);
        lv_obj_add_style(ta_edit_flicker_rate, &style_edit_textarea, 0);
        lv_obj_add_event_cb(ta_edit_flicker_rate, numeric_textarea_focus_event_cb, LV_EVENT_FOCUSED, NULL);
        lv_obj_add_event_cb(ta_edit_flicker_rate, generic_textarea_defocus_event_cb, LV_EVENT_DEFOCUSED, NULL);
    }
    {
        lv_obj_t* row = lv_obj_create(block_uv_all_stages);
        lv_obj_remove_style_all(row);
        lv_obj_set_size(row, lv_pct(100), LV_SIZE_CONTENT);
        lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW);
        lv_obj_set_flex_align(row, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
        label_edit_primary_uv_time_title = lv_label_create(row);
        lv_obj_add_style(label_edit_primary_uv_time_title, &style_my_text_18, 0);
        ta_edit_primary_uv = lv_textarea_create(row);
        lv_textarea_set_one_line(ta_edit_primary_uv, true);
        lv_obj_set_size(ta_edit_primary_uv, 80, TEXT_INPUT_HEIGHT);
        lv_obj_add_style(ta_edit_primary_uv, &style_edit_textarea, 0);
        lv_obj_add_event_cb(ta_edit_primary_uv, numeric_textarea_focus_event_cb, LV_EVENT_FOCUSED, NULL);
        lv_obj_add_event_cb(ta_edit_primary_uv, generic_textarea_defocus_event_cb, LV_EVENT_DEFOCUSED, NULL);
    }
    
    { // --- ВТОРОЙ ЭТАП УФ ---
        row_uv_secondary = lv_obj_create(block_uv_all_stages);
        lv_obj_remove_style_all(row_uv_secondary);
        lv_obj_add_style(row_uv_secondary, &style_switch_row, 0);
        lv_obj_set_size(row_uv_secondary, lv_pct(100), LV_SIZE_CONTENT);
        lv_obj_set_flex_flow(row_uv_secondary, LV_FLEX_FLOW_ROW);
        lv_obj_set_flex_align(row_uv_secondary, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
        
        label_uv_secondary_title = lv_label_create(row_uv_secondary);
        lv_obj_add_style(label_uv_secondary_title, &style_my_text_18, 0);
        label_secondary_uv_mode_status = lv_label_create(row_uv_secondary);
        lv_obj_add_style(label_secondary_uv_mode_status, &style_my_text_18, 0);
        
        lv_obj_t* btn_container = lv_obj_create(row_uv_secondary);
        lv_obj_remove_style_all(btn_container);
        lv_obj_set_size(btn_container, 150, 40);
        lv_obj_set_flex_flow(btn_container, LV_FLEX_FLOW_ROW);
        lv_obj_set_flex_align(btn_container, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
        lv_obj_set_style_pad_gap(btn_container, 5, 0);

        btn_secondary_1 = lv_btn_create(btn_container);
        btn_secondary_2 = lv_btn_create(btn_container);
        btn_secondary_3 = lv_btn_create(btn_container);
        
        lv_obj_t* btns[] = {btn_secondary_1, btn_secondary_2, btn_secondary_3};
        for(int i = 0; i < 3; i++) {
            lv_obj_set_size(btns[i], 40, 40);
            lv_obj_add_flag(btns[i], LV_OBJ_FLAG_CHECKABLE);
            lv_obj_set_style_radius(btns[i], LV_RADIUS_CIRCLE, 0);
            lv_obj_t* lbl = lv_label_create(btns[i]);
            lv_label_set_text_fmt(lbl, "%d", i + 1);
            lv_obj_add_style(lbl, &style_my_text_18_white, 0);
            lv_obj_center(lbl);
            lv_obj_add_event_cb(btns[i], uv_mode_btn_group_event_cb, LV_EVENT_CLICKED, label_secondary_uv_mode_status);
        }
    }
    {
        lv_obj_t* row = lv_obj_create(block_uv_all_stages);
        lv_obj_remove_style_all(row);
        lv_obj_set_size(row, lv_pct(100), LV_SIZE_CONTENT);
        lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW);
        lv_obj_set_flex_align(row, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
        label_edit_secondary_uv_time_title = lv_label_create(row);
        lv_obj_add_style(label_edit_secondary_uv_time_title, &style_my_text_18, 0);
        ta_edit_secondary_uv = lv_textarea_create(row);
        lv_textarea_set_one_line(ta_edit_secondary_uv, true);
        lv_obj_set_size(ta_edit_secondary_uv, 80, TEXT_INPUT_HEIGHT);
        lv_obj_add_style(ta_edit_secondary_uv, &style_edit_textarea, 0);
        lv_obj_add_event_cb(ta_edit_secondary_uv, numeric_textarea_focus_event_cb, LV_EVENT_FOCUSED, NULL);
        lv_obj_add_event_cb(ta_edit_secondary_uv, generic_textarea_defocus_event_cb, LV_EVENT_DEFOCUSED, NULL);
    }
    
    { // --- ТРЕТИЙ ЭТАП УФ ---
        row_uv_tertiary = lv_obj_create(block_uv_all_stages);
        lv_obj_remove_style_all(row_uv_tertiary);
        lv_obj_add_style(row_uv_tertiary, &style_switch_row, 0);
        lv_obj_set_size(row_uv_tertiary, lv_pct(100), LV_SIZE_CONTENT);
        lv_obj_set_flex_flow(row_uv_tertiary, LV_FLEX_FLOW_ROW);
        lv_obj_set_flex_align(row_uv_tertiary, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
        
        label_uv_tertiary_title = lv_label_create(row_uv_tertiary);
        lv_obj_add_style(label_uv_tertiary_title, &style_my_text_18, 0);
        label_tertiary_uv_mode_status = lv_label_create(row_uv_tertiary);
        lv_obj_add_style(label_tertiary_uv_mode_status, &style_my_text_18, 0);
        
        lv_obj_t* btn_container = lv_obj_create(row_uv_tertiary);
        lv_obj_remove_style_all(btn_container);
        lv_obj_set_size(btn_container, 150, 40);
        lv_obj_set_flex_flow(btn_container, LV_FLEX_FLOW_ROW);
        lv_obj_set_flex_align(btn_container, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
        lv_obj_set_style_pad_gap(btn_container, 5, 0);

        btn_tertiary_1 = lv_btn_create(btn_container);
        btn_tertiary_2 = lv_btn_create(btn_container);
        btn_tertiary_3 = lv_btn_create(btn_container);
        
        lv_obj_t* btns[] = {btn_tertiary_1, btn_tertiary_2, btn_tertiary_3};
        for(int i = 0; i < 3; i++) {
            lv_obj_set_size(btns[i], 40, 40);
            lv_obj_add_flag(btns[i], LV_OBJ_FLAG_CHECKABLE);
            lv_obj_set_style_radius(btns[i], LV_RADIUS_CIRCLE, 0);
            lv_obj_t* lbl = lv_label_create(btns[i]);
            lv_label_set_text_fmt(lbl, "%d", i + 1);
            lv_obj_add_style(lbl, &style_my_text_18_white, 0);
            lv_obj_center(lbl);
            lv_obj_add_event_cb(btns[i], uv_mode_btn_group_event_cb, LV_EVENT_CLICKED, label_tertiary_uv_mode_status);
        }
    }
    {
        lv_obj_t* row = lv_obj_create(block_uv_all_stages);
        lv_obj_remove_style_all(row);
        lv_obj_set_size(row, lv_pct(100), LV_SIZE_CONTENT);
        lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW);
        lv_obj_set_flex_align(row, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
        label_edit_tertiary_uv_time_title = lv_label_create(row);
        lv_obj_add_style(label_edit_tertiary_uv_time_title, &style_my_text_18, 0);
        ta_edit_tertiary_uv = lv_textarea_create(row);
        lv_textarea_set_one_line(ta_edit_tertiary_uv, true);
        lv_obj_set_size(ta_edit_tertiary_uv, 80, TEXT_INPUT_HEIGHT);
        lv_obj_add_style(ta_edit_tertiary_uv, &style_edit_textarea, 0);
        lv_obj_add_event_cb(ta_edit_tertiary_uv, numeric_textarea_focus_event_cb, LV_EVENT_FOCUSED, NULL);
        lv_obj_add_event_cb(ta_edit_tertiary_uv, generic_textarea_defocus_event_cb, LV_EVENT_DEFOCUSED, NULL);
    }

    // --- ПРАВАЯ КОЛОНКА И ФУТЕР ---
    header_poly_params = lv_label_create(right_column);
    lv_obj_add_style(header_poly_params, &style_my_text_18, 0);
    lv_obj_add_style(header_poly_params, &style_column_header, 0);
    block_gases = lv_obj_create(right_column);
    lv_obj_add_style(block_gases, &style_param_block, 0);
    lv_obj_clear_flag(block_gases, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_flex_flow(block_gases, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(block_gases, LV_FLEX_ALIGN_SPACE_EVENLY, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_width(block_gases, lv_pct(100));
    lv_obj_set_flex_grow(block_gases, 31);
    {
        lv_obj_t* row = lv_obj_create(block_gases);
        lv_obj_remove_style_all(row);
        lv_obj_set_size(row, lv_pct(100), LV_SIZE_CONTENT);
        lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW);
        lv_obj_set_flex_align(row, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
        label_edit_nitrogen_title = lv_label_create(row);
        lv_obj_add_style(label_edit_nitrogen_title, &style_my_text_18, 0);
        sw_edit_nitrogen = lv_switch_create(row);
        lv_obj_add_event_cb(sw_edit_nitrogen, profile_switch_value_changed_event_cb, LV_EVENT_VALUE_CHANGED, (void*)"nitrogen_profile");
    }
    nitrogen_elements_container = lv_obj_create(block_gases);
    lv_obj_remove_style_all(nitrogen_elements_container);
    lv_obj_set_width(nitrogen_elements_container, lv_pct(100));
    lv_obj_set_height(nitrogen_elements_container, LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(nitrogen_elements_container, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_gap(nitrogen_elements_container, 10, 0);
    lv_obj_set_flex_align(nitrogen_elements_container, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);
    {
        lv_obj_t* row = lv_obj_create(nitrogen_elements_container);
        lv_obj_remove_style_all(row);
        lv_obj_set_size(row, lv_pct(100), LV_SIZE_CONTENT);
        lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW);
        lv_obj_set_flex_align(row, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
        label_edit_nitrogen_target_title = lv_label_create(row);
        lv_obj_add_style(label_edit_nitrogen_target_title, &style_my_text_18, 0);
        ta_edit_nitrogen_target = lv_textarea_create(row);
        lv_textarea_set_one_line(ta_edit_nitrogen_target, true);
        lv_obj_set_size(ta_edit_nitrogen_target, 80, TEXT_INPUT_HEIGHT);
        lv_obj_add_style(ta_edit_nitrogen_target, &style_edit_textarea, 0);
        lv_obj_add_event_cb(ta_edit_nitrogen_target, numeric_textarea_focus_event_cb, LV_EVENT_FOCUSED, NULL);
        lv_obj_add_event_cb(ta_edit_nitrogen_target, generic_textarea_defocus_event_cb, LV_EVENT_DEFOCUSED, NULL);
    }
    {
        lv_obj_t* row = lv_obj_create(nitrogen_elements_container);
        lv_obj_remove_style_all(row);
        lv_obj_set_size(row, lv_pct(100), LV_SIZE_CONTENT);
        lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW);
        lv_obj_set_flex_align(row, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
        label_edit_nitrogen_boost_title = lv_label_create(row);
        lv_obj_add_style(label_edit_nitrogen_boost_title, &style_my_text_18, 0);
        ta_edit_nitrogen_boost = lv_textarea_create(row);
        lv_textarea_set_one_line(ta_edit_nitrogen_boost, true);
        lv_obj_set_size(ta_edit_nitrogen_boost, 80, TEXT_INPUT_HEIGHT);
        lv_obj_add_style(ta_edit_nitrogen_boost, &style_edit_textarea, 0);
        lv_obj_add_event_cb(ta_edit_nitrogen_boost, numeric_textarea_focus_event_cb, LV_EVENT_FOCUSED, NULL);
        lv_obj_add_event_cb(ta_edit_nitrogen_boost, generic_textarea_defocus_event_cb, LV_EVENT_DEFOCUSED, NULL);
    }
    {
        lv_obj_t* row = lv_obj_create(block_gases);
        lv_obj_remove_style_all(row);
        lv_obj_set_size(row, lv_pct(100), LV_SIZE_CONTENT);
        lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW);
        lv_obj_set_flex_align(row, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER); 
        label_edit_cooling_title = lv_label_create(row);
        lv_obj_add_style(label_edit_cooling_title, &style_my_text_18, 0);
        sw_edit_chamber_cooling = lv_switch_create(row);
        lv_obj_add_event_cb(sw_edit_chamber_cooling, profile_switch_value_changed_event_cb, LV_EVENT_VALUE_CHANGED, (void*)"cooling_profile");
    }
    block_post_cooling_purge = lv_obj_create(block_gases);
    lv_obj_remove_style_all(block_post_cooling_purge);
    lv_obj_set_width(block_post_cooling_purge, lv_pct(100));
    lv_obj_set_height(block_post_cooling_purge, LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(block_post_cooling_purge, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(block_post_cooling_purge, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_add_flag(block_post_cooling_purge, LV_OBJ_FLAG_HIDDEN);

    label_post_cooling_title = lv_label_create(block_post_cooling_purge);
    lv_obj_add_style(label_post_cooling_title, &style_my_text_18, 0);
    lv_label_set_text(label_post_cooling_title, "Purge time (sec):");

    static const char* purge_time_map[] = {"20", "40", "60", ""};
    btnm_post_cooling_purge = lv_btnmatrix_create(block_post_cooling_purge);
    lv_obj_set_size(btnm_post_cooling_purge, 150, 40);
    lv_btnmatrix_set_map(btnm_post_cooling_purge, purge_time_map);
    lv_btnmatrix_set_btn_ctrl_all(btnm_post_cooling_purge, LV_BTNMATRIX_CTRL_CHECKABLE);
    lv_btnmatrix_set_one_checked(btnm_post_cooling_purge, true);
    lv_obj_add_event_cb(btnm_post_cooling_purge, post_cooling_purge_event_cb, LV_EVENT_VALUE_CHANGED, NULL);
    lv_obj_set_style_bg_opa(btnm_post_cooling_purge, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(btnm_post_cooling_purge, 0, 0);
    lv_obj_set_style_pad_all(btnm_post_cooling_purge, 3, 0);
    lv_obj_set_style_radius(btnm_post_cooling_purge, LV_RADIUS_CIRCLE, LV_PART_ITEMS);
    lv_obj_set_style_border_width(btnm_post_cooling_purge, 0, LV_PART_ITEMS);
    lv_obj_set_style_text_font(btnm_post_cooling_purge, &montserrat_rus_18, LV_PART_ITEMS);

    block_thermal = lv_obj_create(right_column);
    lv_obj_add_style(block_thermal, &style_param_block, 0);
    lv_obj_clear_flag(block_thermal, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_flex_flow(block_thermal, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(block_thermal, LV_FLEX_ALIGN_SPACE_EVENLY, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_width(block_thermal, lv_pct(100));
    lv_obj_set_flex_grow(block_thermal, 17);
    {
        lv_obj_t* row = lv_obj_create(block_thermal);
        lv_obj_remove_style_all(row);
        lv_obj_set_size(row, lv_pct(100), LV_SIZE_CONTENT);
        lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW);
        lv_obj_set_flex_align(row, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
        label_edit_thermal_chamber_title = lv_label_create(row);
        lv_obj_add_style(label_edit_thermal_chamber_title, &style_my_text_18, 0);
        sw_edit_thermal_chamber_enable = lv_switch_create(row);
        lv_obj_add_event_cb(sw_edit_thermal_chamber_enable, thermal_chamber_enable_switch_event_cb, LV_EVENT_VALUE_CHANGED, NULL);
    }
    thermal_elements_container = lv_obj_create(block_thermal);
    lv_obj_remove_style_all(thermal_elements_container);
    lv_obj_set_width(thermal_elements_container, lv_pct(100));
    lv_obj_set_height(thermal_elements_container, LV_SIZE_CONTENT); 
    lv_obj_set_flex_flow(thermal_elements_container, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_gap(thermal_elements_container, 10, 0);
    lv_obj_set_flex_align(thermal_elements_container, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);
    {
        lv_obj_t* row = lv_obj_create(thermal_elements_container);
        lv_obj_remove_style_all(row);
        lv_obj_set_size(row, lv_pct(100), LV_SIZE_CONTENT);
        lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW);
        lv_obj_set_flex_align(row, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
        label_edit_thermal_temp_title = lv_label_create(row);
        lv_obj_add_style(label_edit_thermal_temp_title, &style_my_text_18, 0);
        ta_edit_thermal_temp = lv_textarea_create(row);
        lv_textarea_set_one_line(ta_edit_thermal_temp, true);
        lv_obj_set_size(ta_edit_thermal_temp, 80, TEXT_INPUT_HEIGHT);
        lv_obj_add_style(ta_edit_thermal_temp, &style_edit_textarea, 0);
        lv_obj_add_event_cb(ta_edit_thermal_temp, numeric_textarea_focus_event_cb, LV_EVENT_FOCUSED, NULL);
        lv_obj_add_event_cb(ta_edit_thermal_temp, generic_textarea_defocus_event_cb, LV_EVENT_DEFOCUSED, NULL);
    }
    {
        lv_obj_t* row = lv_obj_create(thermal_elements_container);
        lv_obj_remove_style_all(row);
        lv_obj_set_size(row, lv_pct(100), LV_SIZE_CONTENT);
        lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW);
        lv_obj_set_flex_align(row, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
        label_edit_heat_hold_title = lv_label_create(row);
        lv_obj_add_style(label_edit_heat_hold_title, &style_my_text_18, 0);
        ta_edit_heat_hold = lv_textarea_create(row);
        lv_textarea_set_one_line(ta_edit_heat_hold, true);
        lv_obj_set_size(ta_edit_heat_hold, 80, TEXT_INPUT_HEIGHT);
        lv_obj_add_style(ta_edit_heat_hold, &style_edit_textarea, 0);
        lv_obj_add_event_cb(ta_edit_heat_hold, numeric_textarea_focus_event_cb, LV_EVENT_FOCUSED, NULL);
        lv_obj_add_event_cb(ta_edit_heat_hold, generic_textarea_defocus_event_cb, LV_EVENT_DEFOCUSED, NULL);
    }
    footer_container = lv_obj_create(main_container);
    lv_obj_remove_style_all(footer_container);
    lv_obj_set_width(footer_container, lv_pct(100));
    lv_obj_set_height(footer_container, LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(footer_container, LV_FLEX_FLOW_ROW);
    lv_obj_set_style_pad_gap(footer_container, 20, 0);
    lv_obj_set_style_pad_all(footer_container, 5, 0);
    lv_obj_set_grid_cell(footer_container, LV_GRID_ALIGN_STRETCH, 0, 1, LV_GRID_ALIGN_END, 2, 1);
    btn_save_changes = lv_btn_create(footer_container);
    lv_obj_set_flex_grow(btn_save_changes, 1);
    lv_obj_set_height(btn_save_changes, 45);
    lv_obj_add_event_cb(btn_save_changes, profile_edit_save_changes_btn_event_cb, LV_EVENT_CLICKED, NULL);
    label_btn_save = lv_label_create(btn_save_changes);
    lv_obj_add_style(label_btn_save, &style_my_text_18_white, 0);
    lv_obj_center(label_btn_save);
    btn_cancel_edit = lv_btn_create(footer_container);
    lv_obj_set_flex_grow(btn_cancel_edit, 1);
    lv_obj_set_height(btn_cancel_edit, 45);
    lv_obj_add_event_cb(btn_cancel_edit, profile_edit_cancel_btn_event_cb, LV_EVENT_CLICKED, NULL);
    label_btn_cancel = lv_label_create(btn_cancel_edit);
    lv_obj_add_style(label_btn_cancel, &style_my_text_18_white, 0);
    lv_obj_center(label_btn_cancel);
}

void build_confirm_delete_dialog(lv_obj_t* parent_for_dialog) {
    screen_confirm_delete_dialog = lv_obj_create(parent_for_dialog); 
    lv_obj_add_flag(screen_confirm_delete_dialog, LV_OBJ_FLAG_HIDDEN); 
    lv_obj_set_size(screen_confirm_delete_dialog, lv_pct(70), LV_SIZE_CONTENT); 
    lv_obj_center(screen_confirm_delete_dialog); 
    lv_obj_set_style_bg_color(screen_confirm_delete_dialog, lv_color_white(), 0);
    lv_obj_set_style_border_width(screen_confirm_delete_dialog, 1, 0);
    lv_obj_set_style_border_color(screen_confirm_delete_dialog, lv_palette_main(LV_PALETTE_GREY), 0);
    lv_obj_set_style_shadow_width(screen_confirm_delete_dialog, 8, 0);
    lv_obj_set_style_shadow_opa(screen_confirm_delete_dialog, LV_OPA_50, 0);
    lv_obj_set_style_radius(screen_confirm_delete_dialog, 5, 0);
    lv_obj_set_style_pad_all(screen_confirm_delete_dialog, 15, 0);
    lv_obj_set_flex_flow(screen_confirm_delete_dialog, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(screen_confirm_delete_dialog, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_gap(screen_confirm_delete_dialog, 10, 0);
    label_confirm_delete_title = lv_label_create(screen_confirm_delete_dialog); // ИЗМЕНЕНИЕ
    lv_label_set_text(label_confirm_delete_title, "Confirm Deletion"); // ИЗМЕНЕНИЕ
    lv_obj_add_style(label_confirm_delete_title, &style_my_text_18, 0); // ДОБАВЛЕНО
    label_confirm_delete_text = lv_label_create(screen_confirm_delete_dialog);
    lv_label_set_long_mode(label_confirm_delete_text, LV_LABEL_LONG_WRAP);
    lv_label_set_text(label_confirm_delete_text, "Really delete 'Profile X'?"); 
    lv_obj_add_style(label_confirm_delete_text, &style_my_text_18, 0); // ДОБАВЛЕНО
    lv_obj_set_width(label_confirm_delete_text, lv_pct(100));
    lv_obj_set_style_text_align(label_confirm_delete_text, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_t* btn_area = lv_obj_create(screen_confirm_delete_dialog);
    lv_obj_remove_style_all(btn_area); 
    lv_obj_set_style_pad_all(btn_area, 5, 0);
    lv_obj_set_width(btn_area, lv_pct(100));
    lv_obj_set_height(btn_area, LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(btn_area, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(btn_area, LV_FLEX_ALIGN_SPACE_EVENLY, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_t* btn_cancel = lv_btn_create(btn_area);
    lv_obj_add_event_cb(btn_cancel, confirm_dialog_cancel_btn_event_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_set_width(btn_cancel, 100);
    label_confirm_btn_cancel = lv_label_create(btn_cancel); // ИЗМЕНЕНИЕ
    lv_label_set_text(label_confirm_btn_cancel, "Cancel"); // ИЗМЕНЕНИЕ
    lv_obj_add_style(label_confirm_btn_cancel, &style_my_text_18_white, 0); // ДОБАВЛЕНО
    lv_obj_center(label_confirm_btn_cancel); // ИЗМЕНЕНИЕ
    lv_obj_t* btn_del = lv_btn_create(btn_area);
    lv_obj_set_style_bg_color(btn_del, lv_palette_main(LV_PALETTE_RED), 0);
    lv_obj_add_event_cb(btn_del, confirm_dialog_delete_btn_event_cb, LV_EVENT_CLICKED, NULL); 
    lv_obj_set_width(btn_del, 100);
    label_confirm_btn_delete = lv_label_create(btn_del); // ИЗМЕНЕНИЕ
    lv_label_set_text(label_confirm_btn_delete, "Delete"); // ИЗМЕНЕНИЕ
    lv_obj_add_style(label_confirm_btn_delete, &style_my_text_18_white, 0); // ДОБАВЛЕНО (белый цвет)
    lv_obj_center(label_confirm_btn_delete); // ИЗМЕНЕНИЕ
    Serial.println("Confirm delete dialog UI built.");
}

void build_process_execution_screen(lv_obj_t* parent_screen) {
    Serial.println("Building THEME-AGNOSTIC process_execution_screen...");

    // --- Параметры для нашего треугольника ---
    const lv_coord_t INDICATOR_WIDTH = 240;
    const lv_coord_t INDICATOR_HEIGHT = 100;

    // --- Буфер памяти для холста. Должен быть static! ---
    static lv_color_t cbuf[LV_CANVAS_BUF_SIZE_TRUE_COLOR(INDICATOR_WIDTH, INDICATOR_HEIGHT)];

    // --- Главный контейнер ---
    lv_obj_t* content_container = lv_obj_create(parent_screen);
    // <<< ИЗМЕНЕНИЕ: УДАЛЕНА ПРОЗРАЧНОСТЬ, КОНТЕЙНЕР ТЕПЕРЬ НЕПРОЗРАЧНЫЙ ПО УМОЛЧАНИЮ >>>
    lv_obj_set_style_border_width(content_container, 0, 0);
    lv_obj_set_size(content_container, lv_pct(100), lv_pct(100));
    lv_obj_set_flex_flow(content_container, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_top(content_container, 20, 0);
    lv_obj_set_style_pad_hor(content_container, 20, 0);
    lv_obj_set_style_pad_bottom(content_container, 0, 0);
    lv_obj_set_style_pad_gap(content_container, 15, 0);
    lv_obj_set_scrollbar_mode(content_container, LV_SCROLLBAR_MODE_OFF);
    lv_obj_set_flex_align(content_container, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

    // --- Растягиваемая область для основного контента ---
    lv_obj_t* main_process_content_area = lv_obj_create(content_container);
    lv_obj_remove_style_all(main_process_content_area);
    lv_obj_set_width(main_process_content_area, lv_pct(100));
    lv_obj_set_flex_grow(main_process_content_area, 1);
    lv_obj_set_flex_flow(main_process_content_area, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(main_process_content_area, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_gap(main_process_content_area, 20, 0);

    label_process_profile_name = lv_label_create(main_process_content_area);
    lv_obj_add_style(label_process_profile_name, &style_my_text_22, 0); // Используем тот же стиль
    lv_label_set_text(label_process_profile_name, ""); // По умолчанию пустая
    lv_obj_set_width(label_process_profile_name, LV_PCT(100));
    lv_obj_set_style_text_align(label_process_profile_name, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_long_mode(label_process_profile_name, LV_LABEL_LONG_DOT);
    
    // Стандартные элементы
    label_process_status_title = lv_label_create(main_process_content_area);
    lv_obj_add_style(label_process_status_title, &style_my_text_22, 0);
    lv_label_set_text(label_process_status_title, "Initializing Process...");
    lv_obj_set_width(label_process_status_title, LV_PCT(100));
    lv_obj_set_style_text_align(label_process_status_title, LV_TEXT_ALIGN_CENTER, 0);
    
    // spinner_process_execution = lv_spinner_create(main_process_content_area, 1000, 60);
    // lv_obj_set_size(spinner_process_execution, 100, 100);
    
    label_process_status_detail = lv_label_create(main_process_content_area);
    lv_obj_add_style(label_process_status_detail, &style_my_text_18, 0);
    lv_label_set_text(label_process_status_detail, "Please wait...");
    lv_obj_set_width(label_process_status_detail, LV_PCT(100));
    lv_obj_set_style_text_align(label_process_status_detail, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_long_mode(label_process_status_detail, LV_LABEL_LONG_WRAP);

    // Кнопка отмены
    btn_process_cancel = lv_btn_create(content_container);
    lv_obj_add_event_cb(btn_process_cancel, process_execution_cancel_btn_event_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_set_width(btn_process_cancel, lv_pct(50));
    lv_obj_set_style_pad_bottom(content_container, 10, 0);
    label_btn_process_cancel = lv_label_create(btn_process_cancel);
    lv_label_set_text(label_btn_process_cancel, "Cancel Process");
    lv_obj_add_style(label_btn_process_cancel, &style_my_text_18_white, 0);
    lv_obj_center(label_btn_process_cancel);
    lv_obj_add_flag(btn_process_cancel, LV_OBJ_FLAG_HIDDEN);
    
    // --- СОЗДАЕМ ПУСТОЙ ХОЛСТ ---
    repair_mode_indicator_obj = lv_canvas_create(content_container);
    lv_canvas_set_buffer(repair_mode_indicator_obj, cbuf, INDICATOR_WIDTH, INDICATOR_HEIGHT, LV_IMG_CF_TRUE_COLOR);
    lv_obj_add_flag(repair_mode_indicator_obj, LV_OBJ_FLAG_HIDDEN);
    
    Serial.println("Process execution screen UI built.");
}


static void build_settings_screen(lv_obj_t* parent_screen) {
    Serial.println("Building settings_screen UI (Blocks Layout)...");

    static lv_style_t style_settings_block;
    lv_style_init(&style_settings_block);
    lv_style_set_radius(&style_settings_block, 8);
    lv_style_set_pad_all(&style_settings_block, 15);
    lv_style_set_flex_flow(&style_settings_block, LV_FLEX_FLOW_COLUMN);
    lv_style_set_pad_gap(&style_settings_block, 25);

    lv_obj_t* main_container = lv_obj_create(parent_screen);
    lv_obj_set_size(main_container, lv_pct(100), lv_pct(100));
    lv_obj_center(main_container);
    lv_obj_set_flex_flow(main_container, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_all(main_container, 0, 0);
    lv_obj_set_style_border_width(main_container, 0, 0);
    lv_obj_set_style_radius(main_container, 0, 0);
    lv_obj_set_style_pad_ver(main_container, 10, 0);
    lv_obj_set_style_pad_hor(main_container, 15, 0);
    lv_obj_set_style_pad_gap(main_container, 10, 0);
    lv_obj_clear_flag(main_container, LV_OBJ_FLAG_SCROLLABLE);

    label_settings_title = lv_label_create(main_container);
    lv_obj_add_style(label_settings_title, &style_my_text_22, 0);
    lv_obj_add_flag(label_settings_title, LV_OBJ_FLAG_CLICKABLE); // 1. Делаем метку кликабельной
    lv_obj_add_event_cb(label_settings_title, secret_button_event_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_set_width(label_settings_title, lv_pct(100));
    lv_obj_set_style_text_align(label_settings_title, LV_TEXT_ALIGN_CENTER, 0);

    content_grid_settings = lv_obj_create(main_container);
    lv_obj_remove_style_all(content_grid_settings);
    lv_obj_set_flex_grow(content_grid_settings, 1);
    lv_obj_set_width(content_grid_settings, lv_pct(100));
    lv_obj_set_layout(content_grid_settings, LV_LAYOUT_GRID);
    static lv_coord_t col_dsc[] = {LV_GRID_FR(1), 15, LV_GRID_FR(1), LV_GRID_TEMPLATE_LAST};
    static lv_coord_t row_dsc[] = {LV_GRID_FR(1), LV_GRID_TEMPLATE_LAST};
    lv_obj_set_grid_dsc_array(content_grid_settings, col_dsc, row_dsc);

    // --- ЛЕВЫЙ БЛОК (Системы) ---
    left_block_settings = lv_obj_create(content_grid_settings); // <<< ИСПОЛЬЗУЕМ ГЛОБАЛЬНЫЙ УКАЗАТЕЛЬ
    lv_obj_add_style(left_block_settings, &style_settings_block, 0);
    lv_obj_set_grid_cell(left_block_settings, LV_GRID_ALIGN_STRETCH, 0, 1, LV_GRID_ALIGN_STRETCH, 0, 1);
    lv_obj_set_flex_align(left_block_settings, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_START);
    
    // Строка Азота
    lv_obj_t* row_nitro = lv_obj_create(left_block_settings); // <<< ИЗМЕНЕНИЕ
    lv_obj_remove_style_all(row_nitro);
    lv_obj_set_width(row_nitro, lv_pct(100));
    lv_obj_set_height(row_nitro, 38);
    lv_obj_set_flex_flow(row_nitro, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(row_nitro, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    label_settings_nitrogen = lv_label_create(row_nitro);
    sw_settings_global_nitrogen_enabled = lv_switch_create(row_nitro);
    lv_obj_add_event_cb(sw_settings_global_nitrogen_enabled, settings_screen_event_cb, LV_EVENT_VALUE_CHANGED, (void*)"nitro_sys");
    btn_settings_test_nitro = lv_btn_create(row_nitro);
    lv_obj_set_size(btn_settings_test_nitro, 80, 35);
    lv_obj_add_event_cb(btn_settings_test_nitro, settings_screen_event_cb, LV_EVENT_CLICKED, (void*)"test_nitro");
    lv_obj_t* lbl_test_nitro = lv_label_create(btn_settings_test_nitro);
    lv_label_set_text(lbl_test_nitro, "Test");
    lv_obj_add_style(lbl_test_nitro, &style_my_text_18_white, 0);
    lv_obj_center(lbl_test_nitro);

    // Строка Воздуха
    lv_obj_t* row_air = lv_obj_create(left_block_settings); // <<< ИЗМЕНЕНИЕ
    lv_obj_remove_style_all(row_air);
    lv_obj_set_width(row_air, lv_pct(100));
    lv_obj_set_height(row_air, 38);
    lv_obj_set_flex_flow(row_air, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(row_air, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    label_settings_air = lv_label_create(row_air);
    sw_settings_global_air_enabled = lv_switch_create(row_air);
    lv_obj_add_event_cb(sw_settings_global_air_enabled, settings_screen_event_cb, LV_EVENT_VALUE_CHANGED, (void*)"air_sys");
    btn_settings_test_air = lv_btn_create(row_air);
    lv_obj_set_size(btn_settings_test_air, 80, 35);
    lv_obj_add_event_cb(btn_settings_test_air, settings_screen_event_cb, LV_EVENT_CLICKED, (void*)"test_air");
    lv_obj_t* lbl_test_air = lv_label_create(btn_settings_test_air);
    lv_label_set_text(lbl_test_air, "Test");
    lv_obj_add_style(lbl_test_air, &style_my_text_18_white, 0);
    lv_obj_center(lbl_test_air);

    // --- ПРАВЫЙ БЛОК (Интерфейс) ---
    right_block_settings = lv_obj_create(content_grid_settings); // <<< ИСПОЛЬЗУЕМ ГЛОБАЛЬНЫЙ УКАЗАТЕЛЬ
    lv_obj_add_style(right_block_settings, &style_settings_block, 0);
    lv_obj_set_grid_cell(right_block_settings, LV_GRID_ALIGN_STRETCH, 2, 1, LV_GRID_ALIGN_STRETCH, 0, 1);
    lv_obj_set_flex_align(right_block_settings, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_START);
    
    // Переключатель языка
    create_custom_toggle(right_block_settings, &label_settings_language, &lang_toggle_box, &label_settings_lang_opt1, &label_settings_lang_opt2); // <<< ИЗМЕНЕНИЕ
    lv_label_set_text(label_settings_language, "Language:");
    lv_label_set_text(label_settings_lang_opt1, "ENG");
    lv_label_set_text(label_settings_lang_opt2, "RUS");
    lv_obj_add_event_cb(lv_obj_get_child(lang_toggle_box, 0), settings_screen_event_cb, LV_EVENT_CLICKED, (void*)"lang1");
    lv_obj_add_event_cb(lv_obj_get_child(lang_toggle_box, 1), settings_screen_event_cb, LV_EVENT_CLICKED, (void*)"lang2");
    
    // Переключатель темы
    create_custom_toggle(right_block_settings, &label_settings_theme, &theme_toggle_box, &label_settings_theme_opt1, &label_settings_theme_opt2); // <<< ИЗМЕНЕНИЕ
    lv_label_set_text(label_settings_theme, "Theme:");
    lv_label_set_text(label_settings_theme_opt1, "Light");
    lv_label_set_text(label_settings_theme_opt2, "Dark");
    lv_obj_add_event_cb(lv_obj_get_child(theme_toggle_box, 0), settings_screen_event_cb, LV_EVENT_CLICKED, (void*)"theme1");
    lv_obj_add_event_cb(lv_obj_get_child(theme_toggle_box, 1), settings_screen_event_cb, LV_EVENT_CLICKED, (void*)"theme2");
    
    // НОВЫЙ 4-позиционный переключатель
    lv_obj_t* timeout_container = lv_obj_create(right_block_settings); // <<< ИЗМЕНЕНИЕ
    lv_obj_remove_style_all(timeout_container);
    lv_obj_set_width(timeout_container, lv_pct(100));
    lv_obj_set_height(timeout_container, LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(timeout_container, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_gap(timeout_container, 5, 0);
    label_settings_timeout = lv_label_create(timeout_container);
    timeout_toggle_box = lv_obj_create(timeout_container);
    lv_obj_remove_style_all(timeout_toggle_box);
    lv_obj_set_size(timeout_toggle_box, lv_pct(100), 40);
    lv_obj_set_flex_flow(timeout_toggle_box, LV_FLEX_FLOW_ROW);
    lv_obj_set_style_radius(timeout_toggle_box, 5, 0);
    lv_obj_set_style_clip_corner(timeout_toggle_box, true, 0);
    lv_obj_set_style_border_width(timeout_toggle_box, 1, 0);
    
    const char* timeout_opts[] = {"5 min", "15 min", "30 min", "60 min"};
    lv_obj_t** timeout_labels[] = {&label_timeout_opt1, &label_timeout_opt2, &label_timeout_opt3, &label_timeout_opt4};
    for(int i = 0; i < 4; i++) {
        lv_obj_t* btn = lv_btn_create(timeout_toggle_box);
        lv_obj_set_flex_grow(btn, 1);
        lv_obj_set_height(btn, lv_pct(100));
        lv_obj_set_style_radius(btn, 0, 0);
        *(timeout_labels[i]) = lv_label_create(btn);
        lv_label_set_text(*(timeout_labels[i]), timeout_opts[i]);
        lv_obj_add_style(*(timeout_labels[i]), &style_my_text_16, 0);
        lv_obj_center(*(timeout_labels[i]));
    }
     lv_obj_add_event_cb(lv_obj_get_child(timeout_toggle_box, 0), settings_screen_event_cb, LV_EVENT_CLICKED, (void*)"timeout0");
     lv_obj_add_event_cb(lv_obj_get_child(timeout_toggle_box, 1), settings_screen_event_cb, LV_EVENT_CLICKED, (void*)"timeout1");
     lv_obj_add_event_cb(lv_obj_get_child(timeout_toggle_box, 2), settings_screen_event_cb, LV_EVENT_CLICKED, (void*)"timeout2");
     lv_obj_add_event_cb(lv_obj_get_child(timeout_toggle_box, 3), settings_screen_event_cb, LV_EVENT_CLICKED, (void*)"timeout3");

    // --- Футер ---
    lv_obj_t* footer_container = lv_obj_create(main_container);
    lv_obj_remove_style_all(footer_container);
    lv_obj_set_style_pad_all(footer_container, 5, 0);
    lv_obj_set_width(footer_container, lv_pct(100));
    lv_obj_set_height(footer_container, LV_SIZE_CONTENT);
    lv_obj_set_flex_align(footer_container, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    btn_settings_save_and_back = lv_btn_create(footer_container);
    lv_obj_add_event_cb(btn_settings_save_and_back, settings_screen_event_cb, LV_EVENT_CLICKED, (void*)"back_save");
    lv_obj_set_width(btn_settings_save_and_back, lv_pct(60));
    lv_obj_t* label_btn_back = lv_label_create(btn_settings_save_and_back);
    lv_obj_center(label_btn_back);
}

// Обработчик событий для всех кнопок на экране казино
static void casino_event_cb(lv_event_t* e) {
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
    if (game_state == ANIMATING) return;

    const char* user_data = (const char*)lv_event_get_user_data(e);

    if (strcmp(user_data, "BACK") == 0) {
        if (screen_settings) load_screen(screen_settings);
    } else if (strcmp(user_data, "BET_PLUS") == 0) {
        if (current_bet < 10) { current_bet++; update_casino_ui_values(); }
    } else if (strcmp(user_data, "BET_MINUS") == 0) {
        if (current_bet > 1) { current_bet--; update_casino_ui_values(); }
    } else if (strcmp(user_data, "SPIN") == 0) {
        start_spin();
    }
}

static void apply_theme_to_casino_screen() {
    if (!screen_secret_game) return;

    lv_obj_t* main_container = lv_obj_get_child(screen_secret_game, 0);
    if (!main_container) return;

    // Находим элементы в их правильных местах
    lv_obj_t* top_panel = lv_obj_get_child(main_container, 0);
    lv_obj_t* balance_cont = lv_obj_get_child(top_panel, 0);
    lv_obj_t* label_balance_title = lv_obj_get_child(balance_cont, 0);
    lv_obj_t* btn_close = lv_obj_get_child(top_panel, 1);

    lv_obj_t* grid_container = lv_obj_get_child(main_container, 1);

    lv_obj_t* control_panel = lv_obj_get_child(main_container, 2);
    
    lv_obj_t* bet_cont = lv_obj_get_child(control_panel, 0);
    lv_obj_t* label_bet_title = lv_obj_get_child(bet_cont, 0);
    
    lv_obj_t* btn_cont = lv_obj_get_child(control_panel, 1);
    lv_obj_t* btn_bet_minus = lv_obj_get_child(btn_cont, 0);
    lv_obj_t* btn_bet_plus = lv_obj_get_child(btn_cont, 2);
    
    lv_obj_t* win_cont = lv_obj_get_child(control_panel, 2);
    lv_obj_t* label_win_title = lv_obj_get_child(win_cont, 0);
    
    if (current_global_settings.theme == 1) { // --- ТЕМНАЯ ТЕМА ---
        lv_obj_add_style(main_container, &style_dark_bg, 0); 
        lv_obj_set_style_bg_color(grid_container, lv_color_hex(0x2C2C4C), 0);
        lv_obj_set_style_border_color(grid_container, lv_palette_darken(LV_PALETTE_GREY, 1), 0);
        lv_obj_set_style_border_width(grid_container, 2, 0);
        lv_style_set_border_color(&style_casino_cell, lv_palette_main(LV_PALETTE_GREY));
        
        lv_obj_set_style_text_color(label_balance_title, lv_color_hex(0xFFD700), 0);
        lv_obj_add_style(label_balance_value, &style_dark_text, 0);
        lv_obj_set_style_text_color(label_bet_title, lv_color_hex(0xAAAAAA), 0);
        lv_obj_add_style(label_bet_value, &style_dark_text, 0);
        lv_obj_set_style_text_color(label_win_title, lv_color_hex(0xAAAAAA), 0);
        lv_obj_add_style(label_win_value, &style_dark_text, 0);

        lv_obj_add_style(btn_close, &style_dark_btn, 0);
        lv_obj_add_style(btn_bet_minus, &style_dark_btn, 0);
        lv_obj_add_style(btn_bet_plus, &style_dark_btn, 0);

    } else { // --- СВЕТЛАЯ ТЕМА ---
        lv_obj_add_style(main_container, &style_light_bg, 0);
        lv_obj_set_style_bg_color(grid_container, lv_color_white(), 0);

        lv_obj_set_style_border_color(grid_container, lv_palette_lighten(LV_PALETTE_GREY, 2), 0);
        lv_obj_set_style_border_width(grid_container, 2, 0);
        lv_style_set_border_color(&style_casino_cell, lv_palette_lighten(LV_PALETTE_GREY, 3));

        lv_obj_set_style_text_color(label_balance_title, lv_color_hex(0x444444), 0);
        lv_obj_remove_style(label_balance_value, &style_dark_text, 0); 
        lv_obj_add_style(label_balance_value, &style_light_text, 0);
        lv_obj_set_style_text_color(label_bet_title, lv_color_hex(0x666666), 0);
        lv_obj_remove_style(label_bet_value, &style_dark_text, 0);
        lv_obj_add_style(label_bet_value, &style_light_text, 0);
        lv_obj_set_style_text_color(label_win_title, lv_color_hex(0x666666), 0);
        lv_obj_remove_style(label_win_value, &style_dark_text, 0);
        lv_obj_add_style(label_win_value, &style_light_text, 0);

        lv_obj_remove_style(btn_close, &style_dark_btn, 0);
        lv_obj_remove_style(btn_bet_minus, &style_dark_btn, 0);
        lv_obj_remove_style(btn_bet_plus, &style_dark_btn, 0);
        lv_obj_add_style(btn_close, &style_light_btn, 0);
        lv_obj_add_style(btn_bet_minus, &style_light_btn, 0);
        lv_obj_add_style(btn_bet_plus, &style_light_btn, 0);
    }
    
    lv_obj_report_style_change(&style_casino_cell);
}


// Функция ищет комбинации 3+ и помечает их флагом is_winning
bool find_and_mark_wins() {
    bool win_found = false;

    // Сначала сбрасываем флаги
    for (int r = 0; r < GRID_ROWS; r++) {
        for (int c = 0; c < GRID_COLS; c++) {
            game_grid[c][r].is_winning = false;
        }
    }

    // Проверка по горизонтали
    for (int r = 0; r < GRID_ROWS; r++) {
        for (int c = 0; c <= GRID_COLS - 3; c++) {
            int type = game_grid[c][r].type;
            if (type == game_grid[c+1][r].type && type == game_grid[c+2][r].type) {
                game_grid[c][r].is_winning = true;
                game_grid[c+1][r].is_winning = true;
                game_grid[c+2][r].is_winning = true;
                win_found = true;
            }
        }
    }
    
    // Проверка по вертикали
    for (int c = 0; c < GRID_COLS; c++) {
        for (int r = 0; r <= GRID_ROWS - 3; r++) {
            int type = game_grid[c][r].type;
            if (type == game_grid[c][r+1].type && type == game_grid[c][r+2].type) {
                game_grid[c][r].is_winning = true;
                game_grid[c][r+1].is_winning = true;
                game_grid[c][r+2].is_winning = true;
                win_found = true;
            }
        }
    }
    return win_found;
}


// Коллбэк, который вызывается, когда все символы упали
void cascade_fall_anim_finish_cb(lv_anim_t* a) {
    // Анимация падения завершена, снова ищем выигрыши
    if (find_and_mark_wins()) {
        start_cascade_animation(); // Если есть - запускаем новый каскад
    } else {
        // Выигрышей больше нет, завершаем спин
        is_cascade_active = false;
        game_state = READY_TO_SPIN;
        lv_obj_clear_state(btn_spin, LV_STATE_DISABLED);
        Serial.println("Cascade finished.");
    }
}

// Эта функция будет вызвана таймером после анимации "пульсации"
static void process_cascade_after_delay(lv_timer_t* timer) {
    const lv_coord_t CELL_SIZE = 68;
    
    // Проходим по каждой колонке снизу вверх
    for (int c = 0; c < GRID_COLS; c++) {
        int empty_slot = -1;
        for (int r = GRID_ROWS - 1; r >= 0; r--) {
            if (game_grid[c][r].to_be_removed && empty_slot == -1) {
                empty_slot = r; // Нашли первое пустое место снизу
            }
            
            if (!game_grid[c][r].to_be_removed && empty_slot != -1) {
                // Этот символ нужно сдвинуть вниз
                game_grid[c][empty_slot].type = game_grid[c][r].type;
                game_grid[c][r].to_be_removed = true; 

                // Запускаем анимацию падения
                lv_anim_t a;
                lv_anim_init(&a);
                lv_anim_set_var(&a, game_grid[c][r].img_obj);
                lv_anim_set_exec_cb(&a, (lv_anim_exec_xcb_t)lv_obj_set_y);
                lv_anim_set_values(&a, lv_obj_get_y(game_grid[c][r].img_obj), r * CELL_SIZE + (empty_slot - r) * CELL_SIZE);
                lv_anim_set_time(&a, 500);
                lv_anim_set_path_cb(&a, lv_anim_path_bounce);
                if (c == GRID_COLS -1) { // Привязываем коллбэк только к последней анимации в цикле
                    lv_anim_set_ready_cb(&a, cascade_fall_anim_finish_cb);
                }
                lv_anim_start(&a);
                
                lv_obj_t* temp_obj = game_grid[c][empty_slot].img_obj;
                game_grid[c][empty_slot].img_obj = game_grid[c][r].img_obj;
                game_grid[c][r].img_obj = temp_obj;
                
                empty_slot--;
            }
        }
    }
    
    // Шаг 3: Генерируем новые символы
    const void* symbol_images[SYMBOL_COUNT] = {
        &symbol_flask, &symbol_pipette, &symbol_molecule, &symbol_tooth,
        &symbol_drill, &symbol_uv_lamp, &symbol_wild, &symbol_scatter
    };
    for (int c = 0; c < GRID_COLS; c++) {
        for (int r = 0; r < GRID_ROWS; r++) {
            if (game_grid[c][r].to_be_removed) {
                int new_type = rand() % SYMBOL_COUNT;
                game_grid[c][r].type = new_type;
                lv_img_set_src(game_grid[c][r].img_obj, symbol_images[new_type]);
                
                lv_obj_set_y(game_grid[c][r].img_obj, -(CELL_SIZE * (GRID_ROWS - r)));
                lv_anim_t a;
                lv_anim_init(&a);
                lv_anim_set_var(&a, game_grid[c][r].img_obj);
                lv_anim_set_exec_cb(&a, (lv_anim_exec_xcb_t)lv_obj_set_y);
                lv_anim_set_values(&a, lv_obj_get_y(game_grid[c][r].img_obj), r * CELL_SIZE);
                lv_anim_set_time(&a, 500);
                lv_anim_set_delay(&a, 100 + (c * 50));
                lv_anim_set_path_cb(&a, lv_anim_path_bounce);
                if (c == GRID_COLS - 1) { // Привязываем коллбэк только к последней анимации
                     lv_anim_set_ready_cb(&a, cascade_fall_anim_finish_cb);
                }
                lv_anim_start(&a);
            }
        }
    }
}   

// Главная функция, управляющая анимацией каскада
void start_cascade_animation() {
    int winning_symbol_count = 0;
    const lv_coord_t CELL_SIZE = 68;

    // Шаг 1: Подсчитываем выигрыш и помечаем символы для удаления
    for (int r = 0; r < GRID_ROWS; r++) {
        for (int c = 0; c < GRID_COLS; c++) {
            if (game_grid[c][r].is_winning) {
                winning_symbol_count++;
                game_grid[c][r].to_be_removed = true;
                
                // Анимация "пульсации" для выигрышных символов
                lv_anim_t a;
                lv_anim_init(&a);
                lv_anim_set_var(&a, game_grid[c][r].img_obj);
                lv_anim_set_values(&a, 255, 150); // Уменьшаем прозрачность
                lv_anim_set_playback_time(&a, 300);
                lv_anim_set_repeat_count(&a, 1);
                lv_anim_set_time(&a, 300);
                lv_anim_start(&a);
            } else {
                game_grid[c][r].to_be_removed = false;
            }
        }
    }

    current_spin_win += winning_symbol_count * current_bet; // Простое начисление: 1 символ = 1 ставка
    player_balance += winning_symbol_count * current_bet;
    update_casino_ui_values();

    // Шаг 2: После пульсации, сдвигаем символы вниз
    lv_timer_t* cascade_timer = lv_timer_create(process_cascade_after_delay, 600, NULL);
    lv_timer_set_repeat_count(cascade_timer, 1);
}

// Коллбэк, который вызывается по завершении ВСЕЙ анимации
static void spin_anim_finish_cb(lv_anim_t *a) {
    // 1. Отображаем финальные символы из game_grid
    generate_and_display_grid(); 

    // 3. Возвращаем игру в состояние готовности
    game_state = READY_TO_SPIN;
    lv_obj_clear_state(btn_spin, LV_STATE_DISABLED);
    Serial.println("Spin finished.");
}
// Коллбэк, который вызывается на каждом "тике" анимации-таймера
static void spin_anim_exec_cb(void * var, int32_t v) {
    const void* symbol_images[SYMBOL_COUNT] = {
        &symbol_flask, &symbol_pipette, &symbol_molecule, &symbol_tooth,
        &symbol_drill, &symbol_uv_lamp, &symbol_wild, &symbol_scatter
    };
    for (int col = 0; col < GRID_COLS; col++) {
        int stop_time = 1000 + col * 150;
        if (v < stop_time) {
            for (int row = 0; row < GRID_ROWS; row++) {
                int random_type = rand() % SYMBOL_COUNT;
                lv_img_set_src(game_grid[col][row].img_obj, symbol_images[random_type]);
            }
        } else {
            for (int row = 0; row < GRID_ROWS; row++) {
                int final_type = game_grid[col][row].type;
                lv_img_set_src(game_grid[col][row].img_obj, symbol_images[final_type]);
            }
        }
    }
}

// Новая функция, которая делает всю работу по подсчету
static void check_wins_and_payout() {
    // Таблица выплат (остается без изменений)
    const int low_tier_payout[] =  {0, 1, 2, 3, 5, 10};
    const int mid_tier_payout[] =  {1, 2, 3, 5, 8, 15};
    const int high_tier_payout[] = {2, 3, 5, 8, 12, 25};
    const int scatter_payout[] =   {5, 10, 20, 50, 100, 250};

    // Массив цветов для каждой группы символов
    const lv_color_t win_colors[] = {
        lv_palette_main(LV_PALETTE_BLUE),   // Колба (0)
        lv_palette_main(LV_PALETTE_BLUE),   // Пипетка (1)
        lv_palette_main(LV_PALETTE_BLUE),   // Молекула (2)
        lv_palette_main(LV_PALETTE_GREEN),  // Зуб (3)
        lv_palette_main(LV_PALETTE_GREEN),  // Бор (4)
        lv_palette_main(LV_PALETTE_PURPLE), // УФ-лампа (5)
        lv_color_white(),                   // Wild (6) - просто белый
        lv_palette_main(LV_PALETTE_YELLOW)  // Scatter (7)
    };

    int symbol_counts[SYMBOL_COUNT] = {0};
    int total_win_this_spin = 0;

    // Шаг 1: Считаем символы и сбрасываем флаги
    for (int c = 0; c < GRID_COLS; c++) {
        for (int r = 0; r < GRID_ROWS; r++) {
            symbol_counts[game_grid[c][r].type]++;
            game_grid[c][r].is_winning = false;
        }
    }

    int wild_count = symbol_counts[6]; 

    // Шаг 2: Проверяем выигрыши для обычных символов
    for (int type_id = 0; type_id < 6; type_id++) {
        int count_with_wilds = symbol_counts[type_id] + wild_count;
        if (count_with_wilds >= 3) {
            int payout_index = count_with_wilds - 3;
            if (payout_index > 5) payout_index = 5;

            int multiplier = 0;
            if (type_id <= 2) multiplier = low_tier_payout[payout_index];
            else if (type_id <= 4) multiplier = mid_tier_payout[payout_index];
            else if (type_id == 5) multiplier = high_tier_payout[payout_index];

            if (multiplier > 0) {
                total_win_this_spin += current_bet * multiplier;
                lv_color_t color_to_set = win_colors[type_id];
                // Помечаем все символы этого типа и Wild'ы как выигрышные и задаем им цвет
                for (int c = 0; c < GRID_COLS; c++) {
                    for (int r = 0; r < GRID_ROWS; r++) {
                        if (game_grid[c][r].type == type_id || game_grid[c][r].type == 6) {
                            game_grid[c][r].is_winning = true;
                            game_grid[c][r].win_color = color_to_set;
                        }
                    }
                }
            }
        }
    }

    // Шаг 3: Проверяем выигрыш по Scatter
    int scatter_count = symbol_counts[7];
    if (scatter_count >= 3) {
        int payout_index = scatter_count - 3;
        if (payout_index > 5) payout_index = 5;
        total_win_this_spin += current_bet * scatter_payout[payout_index];
        for (int c = 0; c < GRID_COLS; c++) {
            for (int r = 0; r < GRID_ROWS; r++) {
                if (game_grid[c][r].type == 7) {
                    game_grid[c][r].is_winning = true;
                    game_grid[c][r].win_color = win_colors[7];
                }
            }
        }
    }
    
    // Шаг 4: Обновляем UI и применяем стили (цвет рамки ИЛИ затемнение)
    if (total_win_this_spin > 0) {
        current_spin_win = total_win_this_spin;
        player_balance += current_spin_win;
        update_casino_ui_values();

        for (int c = 0; c < GRID_COLS; c++) {
            for (int r = 0; r < GRID_ROWS; r++) {
                lv_obj_t* img_obj = game_grid[c][r].img_obj;
                if (game_grid[c][r].is_winning) {
                    lv_obj_set_style_border_color(img_obj, game_grid[c][r].win_color, 0);
                    lv_obj_set_style_img_opa(img_obj, LV_OPA_COVER, 0);
                } else {
                    lv_obj_set_style_img_opa(img_obj, LV_OPA_50, 0);
                }
            }
        }
    }
}

// Коллбэк, который вызывается один раз по завершении анимации-таймера
static void spin_anim_ready_cb(lv_anim_t *a) {
    check_wins_and_payout(); // Вызываем новую функцию

    game_state = READY_TO_SPIN;
    lv_obj_clear_state(btn_spin, LV_STATE_DISABLED);
    Serial.println("Spin finished.");
}

// Главная функция, запускающая анимацию
void start_spin() {
    if (player_balance < current_bet) return;

    game_state = ANIMATING;
    lv_obj_add_state(btn_spin, LV_STATE_DISABLED);
    player_balance -= current_bet;
    current_spin_win = 0;
    update_casino_ui_values();

    // Сбрасываем прозрачность всех иконок перед новым спином
    for (int c = 0; c < GRID_COLS; c++) {
        for (int r = 0; r < GRID_ROWS; r++) {
            lv_obj_t* img_obj = game_grid[c][r].img_obj;
            lv_obj_set_style_img_opa(img_obj, LV_OPA_COVER, 0);
            // Получаем цвет рамки из глобального стиля и применяем его
            lv_style_value_t v;
            lv_style_get_prop(&style_casino_cell, LV_STYLE_BORDER_COLOR, &v);
            lv_obj_set_style_border_color(img_obj, v.color, 0);
        }
    }

    
    for (int col = 0; col < GRID_COLS; col++) {
        for (int row = 0; row < GRID_ROWS; row++) {
            game_grid[col][row].type = rand() % SYMBOL_COUNT;
        }
    }
    
    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_var(&a, NULL);
    lv_anim_set_exec_cb(&a, spin_anim_exec_cb);
    int total_anim_time = 1000 + (GRID_COLS * 150) + 200;
    lv_anim_set_values(&a, 0, total_anim_time); 
    lv_anim_set_time(&a, total_anim_time);
    lv_anim_set_ready_cb(&a, spin_anim_ready_cb);
    lv_anim_start(&a);
}

void update_casino_ui_values() {
    lv_label_set_text_fmt(label_balance_value, "%d", player_balance);
    lv_label_set_text_fmt(label_bet_value, "%d", current_bet);
    lv_label_set_text_fmt(label_win_value, "%d", current_spin_win);
}

// Эта функция теперь просто обновляет данные в массиве game_grid
void generate_and_display_grid() {
    const void* symbol_images[SYMBOL_COUNT] = {
        &symbol_flask, &symbol_pipette, &symbol_molecule, &symbol_tooth,
        &symbol_drill, &symbol_uv_lamp, &symbol_wild, &symbol_scatter
    };
    for (int col = 0; col < GRID_COLS; col++) {
        for (int row = 0; row < GRID_ROWS; row++) {
            game_grid[col][row].type = rand() % SYMBOL_COUNT;
            int type = game_grid[col][row].type;
            if (game_grid[col][row].img_obj) {
                lv_img_set_src(game_grid[col][row].img_obj, symbol_images[type]);
            }
        }
    }
}


static void build_secret_game_screen(lv_obj_t* parent_screen) {
    screen_secret_game = parent_screen;
    lv_obj_clear_flag(parent_screen, LV_OBJ_FLAG_SCROLLABLE);
    
    lv_obj_t* main_container = lv_obj_create(parent_screen);
    lv_obj_set_size(main_container, lv_pct(100), lv_pct(100));
    lv_obj_center(main_container);
    lv_obj_set_style_border_width(main_container, 0, 0);
    lv_obj_set_style_radius(main_container, 0, 0);
    lv_obj_clear_flag(main_container, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_flex_flow(main_container, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(main_container, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_all(main_container, 10, 0);
    lv_obj_set_style_pad_row(main_container, 5, 0);

    // --- Верхняя панель ---
    lv_obj_t* top_panel = lv_obj_create(main_container);
    lv_obj_remove_style_all(top_panel);
    lv_obj_set_width(top_panel, lv_pct(100));
    lv_obj_set_height(top_panel, LV_SIZE_CONTENT);
    lv_obj_set_style_pad_top(top_panel, 3, 0);
    lv_obj_set_flex_flow(top_panel, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(top_panel, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_t* balance_cont = lv_obj_create(top_panel);
    lv_obj_remove_style_all(balance_cont);
    lv_obj_set_size(balance_cont, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(balance_cont, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(balance_cont, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_t* label_balance_title = lv_label_create(balance_cont);
    lv_label_set_text(label_balance_title, "БАЛАНС:");
    lv_obj_add_style(label_balance_title, &style_my_text_18, 0);
    label_balance_value = lv_label_create(balance_cont);
    lv_obj_add_style(label_balance_value, &style_my_text_18, 0);
    lv_obj_set_style_pad_left(label_balance_value, 10, 0);
    lv_obj_t* btn_close = lv_btn_create(top_panel);
    lv_obj_set_size(btn_close, 40, 40);
    lv_obj_set_style_radius(btn_close, LV_RADIUS_CIRCLE, 0);
    lv_obj_add_event_cb(btn_close, casino_event_cb, LV_EVENT_CLICKED, (void*)"BACK");
    lv_obj_t* lbl_close = lv_label_create(btn_close);
    lv_label_set_text(lbl_close, LV_SYMBOL_CLOSE);
    lv_obj_center(lbl_close);

    // --- Игровое поле ---
    lv_obj_t* grid_container = lv_obj_create(main_container);
    lv_obj_remove_style_all(grid_container);
    lv_obj_set_layout(grid_container, LV_LAYOUT_GRID);
    lv_obj_set_style_radius(grid_container, 10, 0);
    const lv_coord_t CELL_SIZE = 68;
    const lv_coord_t GAP_SIZE = 5;
    const lv_coord_t PADDING_SIZE = 5;
    lv_coord_t grid_width = (GRID_COLS * CELL_SIZE) + ((GRID_COLS - 1) * GAP_SIZE) + (PADDING_SIZE * 2);
    lv_coord_t grid_height = (GRID_ROWS * CELL_SIZE) + ((GRID_ROWS - 1) * GAP_SIZE) + (PADDING_SIZE * 2);
    lv_obj_set_size(grid_container, grid_width, grid_height);
    lv_obj_set_scrollbar_mode(grid_container, LV_SCROLLBAR_MODE_OFF);
    lv_obj_set_style_pad_all(grid_container, PADDING_SIZE, 0);
    lv_obj_set_style_pad_gap(grid_container, GAP_SIZE, 0);
    static lv_coord_t col_dsc[] = {CELL_SIZE, CELL_SIZE, CELL_SIZE, CELL_SIZE, CELL_SIZE, LV_GRID_TEMPLATE_LAST};
    static lv_coord_t row_dsc[] = {CELL_SIZE, CELL_SIZE, CELL_SIZE, CELL_SIZE, LV_GRID_TEMPLATE_LAST};
    lv_obj_set_grid_dsc_array(grid_container, col_dsc, row_dsc);
    for (int col = 0; col < GRID_COLS; col++) {
        for (int row = 0; row < GRID_ROWS; row++) {
            lv_obj_t* img = lv_img_create(grid_container);
            lv_img_set_src(img, &symbol_flask); 
            lv_obj_add_style(img, &style_casino_cell, 0);
            lv_obj_set_grid_cell(img, LV_GRID_ALIGN_STRETCH, col, 1, LV_GRID_ALIGN_STRETCH, row, 1);
            game_grid[col][row].img_obj = img;
        }
    }
    
    // --- ПАНЕЛЬ УПРАВЛЕНИЯ ---
    lv_obj_t* control_panel = lv_obj_create(main_container);
    lv_obj_remove_style_all(control_panel);
    lv_obj_set_width(control_panel, lv_pct(100));
    lv_obj_set_height(control_panel, LV_SIZE_CONTENT);
    lv_obj_set_layout(control_panel, LV_LAYOUT_GRID);
    static lv_coord_t ctrl_col_dsc[] = {LV_GRID_FR(1), LV_GRID_FR(2), LV_GRID_FR(1), LV_GRID_TEMPLATE_LAST};
    static lv_coord_t ctrl_row_dsc[] = {LV_GRID_CONTENT, LV_GRID_TEMPLATE_LAST};
    lv_obj_set_grid_dsc_array(control_panel, ctrl_col_dsc, ctrl_row_dsc);

    // Левая колонка: Ставка
    lv_obj_t* bet_cont = lv_obj_create(control_panel);
    lv_obj_remove_style_all(bet_cont);
    lv_obj_set_grid_cell(bet_cont, LV_GRID_ALIGN_STRETCH, 0, 1, LV_GRID_ALIGN_STRETCH, 0, 1);
    lv_obj_set_flex_flow(bet_cont, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(bet_cont, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_gap(bet_cont, 5, 0);
    
    lv_obj_t* label_bet_title = lv_label_create(bet_cont);
    lv_label_set_text(label_bet_title, "СТАВКА");
    lv_obj_add_style(label_bet_title, &style_my_text_16, 0);
    label_bet_value = lv_label_create(bet_cont);
    lv_obj_add_style(label_bet_value, &style_my_text_18, 0);

    // Центральная колонка: Кнопки
    lv_obj_t* btn_cont = lv_obj_create(control_panel);
    lv_obj_remove_style_all(btn_cont);
    lv_obj_set_grid_cell(btn_cont, LV_GRID_ALIGN_STRETCH, 1, 1, LV_GRID_ALIGN_STRETCH, 0, 1);
    lv_obj_set_flex_flow(btn_cont, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(btn_cont, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_gap(btn_cont, 20, 0);

    lv_obj_t* btn_bet_minus_local = lv_btn_create(btn_cont);
    lv_obj_set_size(btn_bet_minus_local, 60, 60);
    lv_obj_add_event_cb(btn_bet_minus_local, casino_event_cb, LV_EVENT_CLICKED, (void*)"BET_MINUS");
    lv_obj_t* lbl_minus = lv_label_create(btn_bet_minus_local);
    lv_label_set_text(lbl_minus, LV_SYMBOL_MINUS);
    lv_obj_center(lbl_minus);

    btn_spin = lv_btn_create(btn_cont);
    lv_obj_set_size(btn_spin, 80, 80);
    lv_obj_set_style_bg_color(btn_spin, lv_palette_main(LV_PALETTE_GREEN), 0);
    lv_obj_set_style_radius(btn_spin, LV_RADIUS_CIRCLE, 0);
    lv_obj_add_event_cb(btn_spin, casino_event_cb, LV_EVENT_CLICKED, (void*)"SPIN");
    lv_obj_t* lbl_spin = lv_label_create(btn_spin);
    lv_label_set_text(lbl_spin, LV_SYMBOL_PLAY);
    lv_obj_set_style_text_font(lbl_spin, &lv_font_montserrat_24, 0);
    lv_obj_center(lbl_spin);

    lv_obj_t* btn_bet_plus_local = lv_btn_create(btn_cont);
    lv_obj_set_size(btn_bet_plus_local, 60, 60);
    lv_obj_add_event_cb(btn_bet_plus_local, casino_event_cb, LV_EVENT_CLICKED, (void*)"BET_PLUS");
    lv_obj_t* lbl_plus = lv_label_create(btn_bet_plus_local);
    lv_label_set_text(lbl_plus, LV_SYMBOL_PLUS);
    lv_obj_center(lbl_plus);
    
    // Правая колонка: Выигрыш
    lv_obj_t* win_cont = lv_obj_create(control_panel);
    lv_obj_remove_style_all(win_cont);
    lv_obj_set_grid_cell(win_cont, LV_GRID_ALIGN_STRETCH, 2, 1, LV_GRID_ALIGN_STRETCH, 0, 1);
    lv_obj_set_flex_flow(win_cont, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(win_cont, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_gap(win_cont, 5, 0);
    lv_obj_t* label_win_title = lv_label_create(win_cont);
    lv_label_set_text(label_win_title, "ВЫИГРЫШ");
    lv_obj_add_style(label_win_title, &style_my_text_16, 0);
    label_win_value = lv_label_create(win_cont);
    lv_obj_add_style(label_win_value, &style_my_text_18, 0);
    
    // Инициализация
    update_casino_ui_values();
    generate_and_display_grid();
}
//==========================================================================

// Раздел 4: Функции инициализации оборудования
// ==========================================================================
bool initializeSDCard() { 
    Serial.println("Step 3: Initializing SD Card...");
    if (!ch422g) { handleFatalError("CH422G missing for SD init."); return false; }
    SPI.end(); delay(10); 
    SPI.setHwCs(false); 
    SPI.begin(SD_CLK, SD_MISO, SD_MOSI);
    Serial.println("SPI for SD card initialized.");
    Serial.println("   Activating SD_CS (LOW) via expander BEFORE SD.begin(SD_SS=-1)...");
    ch422g->digitalWrite(SD_CS, LOW); 
    delayMicroseconds(150);
    if (!SD.begin(SD_SS)) {
        Serial.println("ERROR: Card Mount Failed (SD.begin() returned false).");
        ch422g->digitalWrite(SD_CS, HIGH); 
        return false;
    }
    Serial.println("SUCCESS: SD Card initialized. CS is now managed by SD.h library for operations.");
    uint8_t cardType = SD.cardType(); 
    Serial.print("         Card Type: ");
    if (cardType == CARD_NONE) Serial.println("None");
    else if (cardType == CARD_MMC) Serial.println("MMC");
    else if (cardType == CARD_SD)  Serial.println("SDSC");
    else if (cardType == CARD_SDHC) Serial.println("SDHC");
    else Serial.println("UNKNOWN");
    if (cardType != CARD_NONE) {
        uint64_t cardSize = SD.cardSize() / (1024 * 1024);
        Serial.printf("         Card Size: %lluMB\n", cardSize);
        Serial.println("         Listing root directory (using your listDir from waveshare_sd_card.h)...");
        listDir(SD, "/", 0); 
    }
    Serial.println("Step 3: SD Card initialized successfully.");
    return true;
}


// Раздел 5: Главные функции Arduino (setup, loop)
// ==========================================================================
void setup() {
    file_content_buffer = (char*)heap_caps_malloc(FILE_CONTENT_BUFFER_SIZE, MALLOC_CAP_SPIRAM);
    if (file_content_buffer == NULL) {
        Serial.println("!!! FATAL ERROR: Failed to allocate file buffer in PSRAM. Halting. !!!");
        while(1);
    }

    Serial.begin(115200);
    Serial.println("\n\n--- System Setup Starting ---");
    
    // 1. ИНИЦИАЛИЗАЦИЯ ПЛАТЫ И ПЕРИФЕРИИ
    board = new Board();
    board->init();
    ch422g = static_cast<esp_expander::CH422G*>(board->getExpander());
    #if defined(LVGL_PORT_AVOID_TEARING_MODE) && LVGL_PORT_AVOID_TEARING_MODE
        auto lcd = board->getLCD();
        if (lcd) {
            #ifndef LVGL_PORT_DISP_BUFFER_NUM
                #define LVGL_PORT_DISP_BUFFER_NUM 2
            #endif
            lcd->configFrameBufferNumber(LVGL_PORT_DISP_BUFFER_NUM);
        #if ESP_PANEL_DRIVERS_BUS_ENABLE_RGB && CONFIG_IDF_TARGET_ESP32S3
            auto lcd_bus = lcd->getBus();
            if (lcd_bus && lcd_bus->getBasicAttributes().type == ESP_PANEL_BUS_TYPE_RGB) {
                int bounce_buffer_size_px = lcd->getFrameWidth() * 10;
                static_cast<BusRGB *>(lcd_bus)->configRGB_BounceBufferSize(bounce_buffer_size_px);
            }
        #endif
        }
    #endif
    board->begin();
    
    // 2. НАСТРОЙКА ПИНОВ РАСШИРИТЕЛЯ
    ch422g->multiPinMode(TP_RST | LCD_BL | LCD_RST, OUTPUT);
    ch422g->multiDigitalWrite(TP_RST | LCD_RST, HIGH);
    ch422g->digitalWrite(LCD_BL, HIGH);
    ch422g->multiPinMode(SD_CS | USB_SEL, OUTPUT);
    ch422g->digitalWrite(USB_SEL, LOW);
    ch422g->digitalWrite(SD_CS, HIGH);
    
    // 3. РАННЯЯ ИНИЦИАЛИЗАЦИЯ LVGL
    lvgl_port_init(board->getLCD(), board->getTouch());
    EVENT_REFRESH_PROFILES = lv_event_register_id();

    // Устанавливаем начальное время, чтобы экран не погас сразу
    last_interaction_time = millis();
    Serial.println("Global inactivity timer handler has been registered.");

    // 3.1 НАСТРОЙКА СТИЛЕЙ И ТЕМЫ UI
    Serial.println("Step 3.1: Initializing UI styles...");
    lv_style_init(&style_my_text_16);
    lv_style_set_text_font(&style_my_text_16, &montserrat_rus_16);
    lv_style_set_text_color(&style_my_text_16, lv_color_black());
    lv_style_init(&style_my_text_18);
    lv_style_set_text_font(&style_my_text_18, &montserrat_rus_18);
    lv_style_set_text_color(&style_my_text_18, lv_color_black());
    font_18_with_fallback = montserrat_rus_18; 
    font_18_with_fallback.fallback = &lv_font_montserrat_14;
    lv_style_init(&style_just_font_18);
    lv_style_set_text_font(&style_just_font_18, &font_18_with_fallback);
    lv_style_init(&style_my_text_18_white);
    lv_style_set_text_font(&style_my_text_18_white, &font_18_with_fallback);
    lv_style_set_text_color(&style_my_text_18_white, lv_color_white());
    lv_style_init(&style_just_font_18);
    lv_style_set_text_font(&style_just_font_18, &montserrat_rus_18);
    lv_style_init(&style_my_text_22);
    lv_style_set_text_font(&style_my_text_22, &montserrat_rus_22);
    lv_style_set_text_color(&style_my_text_22, lv_color_black());

    lv_style_init(&style_block_header);
    lv_style_set_bg_color(&style_block_header, lv_palette_lighten(LV_PALETTE_GREY, 1));
    lv_style_set_bg_opa(&style_block_header, LV_OPA_COVER);
    lv_style_set_text_color(&style_block_header, lv_color_white()); 
    lv_style_set_radius(&style_block_header, 5);
    lv_style_set_pad_ver(&style_block_header, 4);
    
    // --- Инициализация стилей для СВЕТЛОЙ ТЕМЫ ---
    lv_style_init(&style_light_bg);
    lv_style_set_bg_color(&style_light_bg, lv_color_hex(0xF0F0F0));
    lv_style_init(&style_light_text);
    lv_style_set_text_color(&style_light_text, lv_color_black());
    lv_style_init(&style_light_spinner_bg);
    lv_style_set_arc_color(&style_light_spinner_bg, lv_palette_lighten(LV_PALETTE_GREY, 1));
    lv_style_init(&style_light_spinner_indic);
    lv_style_set_arc_color(&style_light_spinner_indic, lv_palette_main(LV_PALETTE_BLUE));
    lv_style_init(&style_light_btn);
    lv_style_set_bg_color(&style_light_btn, lv_palette_main(LV_PALETTE_BLUE));

    lv_style_init(&style_light_block_border);
    lv_style_set_border_color(&style_light_block_border, lv_palette_lighten(LV_PALETTE_GREY, 1));

    lv_style_init(&style_light_block_bg);
    lv_style_set_bg_color(&style_light_block_bg, lv_color_hex(0xFFFFFF)); 
    lv_style_init(&style_light_block_header);
    lv_style_set_bg_color(&style_light_block_header, lv_palette_lighten(LV_PALETTE_GREY, 1));
    lv_style_set_text_color(&style_light_block_header, lv_color_white());

    lv_style_init(&style_light_tile_bg);
    lv_style_set_bg_color(&style_light_tile_bg, lv_color_hex(0xE0E0E0));

    lv_style_init(&style_light_tile_border);
    lv_style_set_border_color(&style_light_tile_border, lv_palette_main(LV_PALETTE_GREY));
    lv_style_init(&style_light_arrow);
    lv_style_set_text_color(&style_light_arrow, lv_palette_main(LV_PALETTE_BLUE));

    lv_style_init(&style_light_subheader);
    lv_style_set_text_color(&style_light_subheader, lv_color_hex(0x888888));

    lv_style_init(&style_light_textarea);
    lv_style_set_bg_color(&style_light_textarea, lv_color_white());
    lv_style_set_border_color(&style_light_textarea, lv_palette_main(LV_PALETTE_GREY));
    lv_style_set_text_color(&style_light_textarea, lv_color_black());

    lv_style_init(&style_light_column_header);
    lv_style_set_bg_color(&style_light_column_header, lv_palette_lighten(LV_PALETTE_GREY, 2));
    lv_style_set_text_color(&style_light_column_header, lv_color_black());

    light_uv_btn1_active = lv_palette_main(LV_PALETTE_RED);
    light_uv_btn1_inactive = lv_palette_lighten(LV_PALETTE_GREY, 2);
    light_uv_btn2_active = lv_palette_main(LV_PALETTE_GREEN);
    light_uv_btn2_inactive = lv_palette_lighten(LV_PALETTE_GREY, 2);
    light_uv_btn3_active = lv_palette_main(LV_PALETTE_BLUE);
    light_uv_btn3_inactive = lv_palette_lighten(LV_PALETTE_GREY, 2);


    // --- Инициализация стилей для ТЕМНОЙ ТЕМЫ ---
    lv_style_init(&style_dark_bg);
    lv_style_set_bg_color(&style_dark_bg, lv_color_black());
    lv_style_init(&style_dark_text);
    lv_style_set_text_color(&style_dark_text, lv_color_white());
    lv_style_init(&style_dark_spinner_bg);
    lv_style_set_arc_color(&style_dark_spinner_bg, lv_palette_darken(LV_PALETTE_GREY, 2));
    lv_style_init(&style_dark_spinner_indic);
    lv_style_set_arc_color(&style_dark_spinner_indic, lv_color_hex(0xff05b8));
    lv_style_init(&style_dark_btn);
    lv_style_set_bg_color(&style_dark_btn, lv_color_hex(0x424242));

    lv_style_init(&style_dark_block_border);
    lv_style_set_border_color(&style_dark_block_border, lv_palette_darken(LV_PALETTE_GREY, 2));

    lv_style_init(&style_dark_block_bg);
    // lv_style_set_bg_color(&style_dark_block_bg, lv_color_hex(0x2C2C2C)); //1
    lv_style_set_bg_color(&style_dark_block_bg, lv_color_black()); //2
    lv_style_init(&style_dark_block_header);
    // lv_style_set_bg_color(&style_dark_block_header, lv_palette_darken(LV_PALETTE_GREY, 2)); //1
    lv_style_set_bg_color(&style_dark_block_header, lv_color_hex(0x2C2C2C)); //2
    // lv_style_set_text_color(&style_dark_block_header, lv_color_black()); //1
    lv_style_set_text_color(&style_dark_block_header, lv_color_white());//2

    lv_style_init(&style_dark_tile_bg);
    lv_style_set_bg_color(&style_dark_tile_bg, lv_color_black());


    lv_style_init(&style_dark_tile_border);
    lv_style_set_border_color(&style_dark_tile_border, lv_palette_darken(LV_PALETTE_GREY, 2));
    lv_style_init(&style_dark_arrow);
    lv_style_set_text_color(&style_dark_arrow, lv_palette_main(LV_PALETTE_GREY));

    lv_style_init(&style_dark_subheader);
    lv_style_set_text_color(&style_dark_subheader, lv_palette_main(LV_PALETTE_GREY));

    lv_style_init(&style_dark_textarea);
    lv_style_set_bg_color(&style_dark_textarea, lv_color_hex(0x333333));
    lv_style_set_border_color(&style_dark_textarea, lv_palette_darken(LV_PALETTE_GREY, 1));
    lv_style_set_text_color(&style_dark_textarea, lv_color_white());

    lv_style_init(&style_dark_column_header);
    lv_style_set_bg_color(&style_dark_column_header, lv_color_hex(0x2C2C2C));
    lv_style_set_text_color(&style_dark_column_header, lv_color_white());
    
    lv_style_init(&style_casino_cell);
    lv_style_set_radius(&style_casino_cell, 5); // Сделаем ячейки слегка скругленными
    lv_style_set_border_width(&style_casino_cell, 2); // Зададим толщину рамки

    lv_style_init(&style_info_box);
    lv_style_set_radius(&style_info_box, 8);
    lv_style_set_border_width(&style_info_box, 2);
    lv_style_set_bg_opa(&style_info_box, LV_OPA_TRANSP);
    lv_style_set_pad_ver(&style_info_box, 5);
    lv_style_set_pad_hor(&style_info_box, 10);

    dark_uv_btn1_active = lv_palette_main(LV_PALETTE_RED);
    dark_uv_btn1_inactive = lv_color_hex(0x424242);
    dark_uv_btn2_active = lv_palette_main(LV_PALETTE_GREEN);
    dark_uv_btn2_inactive = lv_color_hex(0x424242);
    dark_uv_btn3_active = lv_color_hex(0xff05b8); // Оставим ваш фирменный цвет для кнопки "3"
    dark_uv_btn3_inactive = lv_color_hex(0x424242);

    Serial.println("Step 3.1: UI styles initialized.");

    // 4. НЕМЕДЛЕННО СОЗДАЕМ И ЗАГРУЖАЕМ ЗАСТАВКУ (БЕЗ ЗАПУСКА ТАЙМЕРА!)
    if (ENABLE_SPLASH_SCREEN) {
        Serial.println("Step 4: Building and loading SPLASH SCREEN...");
        delay(500);
        screen_splash = lv_obj_create(NULL);
        delay(500);
        build_splash_screen(screen_splash);
        delay(500);
        load_screen(screen_splash);
        delay(500);
        is_on_splash_screen = true;
        
        // Сразу же запускаем таймеры, связанные с заставкой
        splash_screen_start_time = millis();
        Serial.println("Splash screen timer started.");
        
        static SplashScreenData splash_data;
        splash_data.screen = screen_splash;
        splash_data.title = lv_obj_get_child(screen_splash, 0);
        
        if (strcmp(lv_label_get_text(splash_data.title), "SpectraMaster N2") != 0) {
            for (int i = 0; i < lv_obj_get_child_cnt(screen_splash); i++) {
                lv_obj_t* child = lv_obj_get_child(screen_splash, i);
                if (lv_obj_check_type(child, &lv_label_class)) {
                    if (strcmp(lv_label_get_text(child), "SpectraMaster N2") == 0) {
                        splash_data.title = child;
                        break;
                    }
                }
            }
        }
        
        lv_timer_create(start_splash_inversion_anim, 3000, &splash_data);
    } else {
        is_on_splash_screen = false;
        Serial.println("Step 4: Splash screen is DISABLED. Skipping.");
    }
    
    // 5. ИНИЦИАЛИЗАЦИЯ SD-КАРТЫ И НАСТРОЕК
    delay(100); // Небольшая пауза
    sd_card_initialized = initializeSDCard();
    delay(100);
    if (!sd_card_initialized) {
        Serial.println("!!! FATAL: SD Card initialization failed. Halting setup. !!!");
        // Показываем сообщение об ошибке на заставке
        if (label_splash_error) {
            lv_obj_clear_flag(label_splash_error, LV_OBJ_FLAG_HIDDEN);
            lv_label_set_text(label_splash_error, "SD Card Error!\nPlease insert card and restart device.");
        }
        // Бесконечный цикл, чтобы остановить дальнейшую загрузку
        while(true) {
            lv_timer_handler();
            delay(5);
        }
    }
    delay(100);
    loadConfiguration();

    // 6. СОЗДАНИЕ ЧИСТЫХ ЭКРАНОВ (БЕЗ СТИЛЕЙ ПО УМОЛЧАНИЮ)
    Serial.println("Step 6: Creating black-background screens...");
    
    screen_main_app = lv_obj_create(NULL);
    lv_obj_remove_style_all(screen_main_app);
    lv_obj_set_style_bg_color(screen_main_app, lv_color_black(), 0);

    screen_profile_details = lv_obj_create(NULL);
    lv_obj_remove_style_all(screen_profile_details);
    lv_obj_set_style_bg_color(screen_profile_details, lv_color_black(), 0);

    screen_profile_edit = lv_obj_create(NULL);
    lv_obj_remove_style_all(screen_profile_edit);
    lv_obj_set_style_bg_color(screen_profile_edit, lv_color_black(), 0);

    screen_process_execution = lv_obj_create(NULL);
    lv_obj_remove_style_all(screen_process_execution);
    lv_obj_set_style_bg_color(screen_process_execution, lv_color_black(), 0);

    screen_settings = lv_obj_create(NULL);
    lv_obj_remove_style_all(screen_settings);
    lv_obj_set_style_bg_color(screen_settings, lv_color_black(), 0);

    screen_test_nitrogen = lv_obj_create(NULL);
    lv_obj_remove_style_all(screen_test_nitrogen);
    lv_obj_set_style_bg_color(screen_test_nitrogen, lv_color_black(), 0);

    screen_test_air = lv_obj_create(NULL);
    lv_obj_remove_style_all(screen_test_air);
    lv_obj_set_style_bg_color(screen_test_air, lv_color_black(), 0);

    screen_secret_game = lv_obj_create(NULL);
    lv_obj_remove_style_all(screen_secret_game);
    lv_obj_set_style_bg_color(screen_secret_game, lv_color_black(), 0);

    screen_laboratory = lv_obj_create(NULL);
    lv_obj_remove_style_all(screen_laboratory);
    lv_obj_set_style_bg_color(screen_laboratory, lv_color_black(), 0);

    screen_lab_glaze = lv_obj_create(NULL);
    lv_obj_remove_style_all(screen_lab_glaze);
    lv_obj_set_style_bg_color(screen_lab_glaze, lv_color_black(), 0);

    screen_lab_repair = lv_obj_create(NULL);
    lv_obj_remove_style_all(screen_lab_repair);
    lv_obj_set_style_bg_color(screen_lab_repair, lv_color_black(), 0);
    
    screen_lab_strength = lv_obj_create(NULL);
    lv_obj_remove_style_all(screen_lab_strength);
    lv_obj_set_style_bg_color(screen_lab_strength, lv_color_black(), 0);

    screen_lab_thermal = lv_obj_create(NULL);
    lv_obj_remove_style_all(screen_lab_thermal);
    lv_obj_set_style_bg_color(screen_lab_thermal, lv_color_black(), 0);

    screen_lab_lighten = lv_obj_create(NULL);
    lv_obj_remove_style_all(screen_lab_lighten);
    lv_obj_set_style_bg_color(screen_lab_lighten, lv_color_black(), 0);
    
    screen_lab_darken = lv_obj_create(NULL);
    lv_obj_remove_style_all(screen_lab_darken);
    lv_obj_set_style_bg_color(screen_lab_darken, lv_color_black(), 0);
    
    screen_service_lock = lv_obj_create(NULL); // Этот экран сам себя красит
    
    screen_help = lv_obj_create(NULL);
    lv_obj_remove_style_all(screen_help);
    lv_obj_set_style_bg_color(screen_help, lv_color_black(), 0);

    screen_keyboard = lv_obj_create(NULL);
    lv_obj_remove_style_all(screen_keyboard);
    lv_obj_set_style_bg_color(screen_keyboard, lv_color_black(), 0);

    screen_loading = lv_obj_create(NULL);
    lv_obj_remove_style_all(screen_loading);
    lv_obj_set_style_bg_color(screen_loading, lv_color_black(), 0);


    // Строим все остальные экраны
    build_main_app_screen(screen_main_app);
    build_profile_list_pool();
    build_profile_details_screen(screen_profile_details);
    build_profile_edit_screen(screen_profile_edit);
    build_confirm_delete_dialog(screen_profile_details);
    build_process_execution_screen(screen_process_execution);
    build_settings_screen(screen_settings);
    build_secret_game_screen(screen_secret_game);
    build_laboratory_screen(screen_laboratory);
    build_lab_glaze_screen(screen_lab_glaze);
    build_lab_repair_screen(screen_lab_repair);
    build_lab_strength_screen(screen_lab_strength);
    build_lab_thermal_screen(screen_lab_thermal);
    build_lab_lighten_screen(screen_lab_lighten); 
    build_lab_darken_screen(screen_lab_darken); 
    build_service_lock_screen(screen_service_lock);
    build_test_nitrogen_screen(screen_test_nitrogen);
    build_test_air_screen(screen_test_air);
    build_help_screen(screen_help);
    build_keyboard_screen(screen_keyboard); 
    build_loading_screen(screen_loading);

    Serial.println("Building universal info dialogs...");
    build_info_dialog(lv_layer_top());
    build_choice_dialog(lv_layer_top());
    build_input_shield();
    apply_current_theme_to_all_screens();
    apply_theme_to_loading_screen();

    // 7. ПОДГОТОВКА ДАННЫХ ДЛЯ ГЛАВНОГО ЭКРАНА
    if (sd_card_initialized) {
        Serial.println("Configuration loaded, displaying profiles...");
        displayProfileListPage();
    } else {
        Serial.println("SD card not initialized. Main UI will reflect this.");
        current_profile_next_id = 1;
        if (list_profiles_main) { lv_list_add_text(list_profiles_main, "SD Card not detected."); }
        if (list_header_label_main) { lv_label_set_text(list_header_label_main, "Saved Profiles (SD Error)"); }
        if (btn_profiles_prev) { lv_obj_add_state(btn_profiles_prev, LV_STATE_DISABLED); lv_obj_add_flag(btn_profiles_prev, LV_OBJ_FLAG_HIDDEN); }
        if (btn_profiles_next) { lv_obj_add_state(btn_profiles_next, LV_STATE_DISABLED); lv_obj_add_flag(btn_profiles_next, LV_OBJ_FLAG_HIDDEN); }
    }

    if (!ENABLE_SPLASH_SCREEN) {
        Serial.println("Loading main app screen directly from setup()...");
        if (current_global_settings.is_heater_error) {
            Serial.println("!!! DEVICE IS IN LOCKED STATE. Loading service screen. !!!");
            lv_label_set_text(label_service_lock_msg, "Critical Error!\nHeating element failure.\nDevice is locked.");
            load_screen(screen_service_lock);
        } else {
            Serial.println("Device is OK. Loading main app screen...");
            load_screen(screen_main_app);
        }
    }

    // Инициализация UART последним шагом, чтобы не мешать другим устройствам
    Serial.println("Step 8: Finalizing setup with UART initialization...");
    MySerial1.begin(115200, SERIAL_8N1, RX1_PIN, TX1_PIN);
    Serial.println("UART to receiver is now active on pins RX=15, TX=44.");

    Serial.println("--- System Setup Complete ---");
}

void loop() {

    // --- БЛОК 1: УПРАВЛЕНИЕ ЭКРАНОМ (СОН/ПРОБУЖДЕНИЕ) ---
    lv_indev_t* indev = lv_indev_get_act();
    if (indev && indev->proc.state == LV_INDEV_STATE_PRESSED) {
        if (!is_screen_on) {
            ch422g->digitalWrite(LCD_BL, HIGH);
            is_screen_on = true;
            if (input_shield) {
                lv_obj_add_flag(input_shield, LV_OBJ_FLAG_HIDDEN);
                lv_obj_clear_flag(input_shield, LV_OBJ_FLAG_CLICKABLE);
            }
            Serial.println("Screen WAKE UP. Input shield immediately deactivated.");
        }
        last_interaction_time = millis();
    }

    if (is_screen_on && (millis() - last_interaction_time > get_current_screen_timeout_ms())) {
        ch422g->digitalWrite(LCD_BL, LOW);
        is_screen_on = false;
        if (input_shield) {
            lv_obj_clear_flag(input_shield, LV_OBJ_FLAG_HIDDEN);
            lv_obj_add_flag(input_shield, LV_OBJ_FLAG_CLICKABLE);
            lv_obj_move_foreground(input_shield);
        }
        Serial.println("Screen OFF due to inactivity. Input shield is now active.");
    }

    // --- БЛОК 2: ОТЛОЖЕННЫЕ ЗАДАЧИ И ОБНОВЛЕНИЯ UI ---
    if (ENABLE_SPLASH_SCREEN && is_on_splash_screen && (millis() - splash_screen_start_time >= 5000)) {
        is_on_splash_screen = false; 
        if (current_global_settings.is_heater_error) {
            enter_service_lock_mode("Critical Error!\nHeating element failure.\nDevice is locked.", "Критическая ошибка!\nОтказ нагревательного элемента.\nУстройство заблокировано.");
        } else {
            load_screen(screen_main_app);
        }
    }
    
    if (needs_list_refresh && lv_scr_act() == screen_main_app) {
        needs_list_refresh = false; 
        lvgl_port_lock(-1);
        displayProfileListPage();
        lvgl_port_unlock();
    }
    
    if (lab_settings_action_pending) {
        lab_settings_action_pending = false;
        handle_lab_save_and_action();
    }
    
    if (profile_save_action_pending) {
        profile_save_action_pending = false;
        handle_profile_save_action();
    }

    // --- БЛОК 3: ОБРАБОТКА UART ---

    // 3.1. Ожидание подтверждения отмены (STOP_ACK)
    if (waiting_for_stop_ack) {
        if (millis() - stop_ack_timeout_start > STOP_ACK_TIMEOUT_MS) {
            Serial.println("!!! WARNING: Timeout waiting for STOP_ACK. Retrying...");
            command_json_doc.clear();
            command_json_doc["command"] = "EMERGENCY_STOP";
            String output;
            serializeJson(command_json_doc, output);
            MySerial1.println(output);
            stop_ack_timeout_start = millis();
        }
    }

    // 3.2. Чтение и обработка всех входящих сообщений от ПП
    // Используем `if`, а не `while`, чтобы не блокировать loop надолго при большом потоке данных
    if (MySerial1.available() > 0) {
        String response = MySerial1.readStringUntil('\n');
        response.trim();
        
        if (response.length() > 0) {
            Serial.printf("<-- Rcvd: %s\n", response.c_str());

            // Сначала обрабатываем критически важные текстовые ответы
            if (response == "ACK:STOP_COMMAND_RECEIVED") {
                if (waiting_for_stop_ack) {
                    Serial.println("Successfully received STOP_ACK from controller.");
                    lvgl_port_lock(-1);
                    completeCancellationSequence();
                    lvgl_port_unlock();
                }
                return; // Выходим из loop, чтобы не обрабатывать другие сообщения в этой итерации
            }

            // Пытаемся распарсить как JSON
            command_json_doc.clear();
            DeserializationError error = deserializeJson(command_json_doc, response);

            if (!error) { // Успешно распарсили JSON - это телеметрия
                const char* type = command_json_doc["type"];
                if (type && strcmp(type, "TELEMETRY") == 0) {
                    if (main_process_running && lv_scr_act() == screen_process_execution) {
                        lvgl_port_lock(-1);
                        update_telemetry_display(command_json_doc["temp"], command_json_doc["o2"], command_json_doc["timer_rem"]);
                        lvgl_port_unlock();
                    }
                }
            } 
            else { // Это не JSON, значит это текстовый статус
                if (main_process_running || waiting_for_stop_ack) {
                    if (response == "FATAL_ERROR:HEATER_FAILURE" || response == "PROCESS_COMPLETE" || response.startsWith("ERROR:") || response == "PROCESS_COMPLETE_WITH_COOLING_WARNING") {
                        main_process_running = false; 
                        is_lab_mode_running = false;

                        if (response == "FATAL_ERROR:HEATER_FAILURE") {
                            enter_service_lock_mode("...", "...");
                            return;
                        }

                        if (lv_scr_act() == screen_process_execution) {
                            lvgl_port_lock(-1);
                            // if(spinner_process_execution) lv_obj_add_flag(spinner_process_execution, LV_OBJ_FLAG_HIDDEN);
                            if(repair_mode_indicator_obj) lv_obj_add_flag(repair_mode_indicator_obj, LV_OBJ_FLAG_HIDDEN);
                            
                            lv_label_set_text(label_process_profile_name, "");
                            lv_label_set_text(label_process_status_title, translateSystemStatus(response.c_str()));
                            
                            if (response == "PROCESS_COMPLETE_WITH_COOLING_WARNING") {
                                lv_label_set_text(label_process_status_detail, tr("Warning: The compressed air supply may have failed during the process. Please check the system in Settings -> Test."));
                            } else {
                                lv_label_set_text(label_process_status_detail, "");
                            }

                            if (response == "PROCESS_COMPLETE" || response == "PROCESS_COMPLETE_WITH_COOLING_WARNING") {
                                lv_obj_t* container = lv_obj_get_child(screen_process_execution, 0);
                                if(container) {
                                    lv_obj_set_style_bg_color(container, lv_palette_main(LV_PALETTE_GREEN), 0);
                                    lv_obj_set_style_bg_grad_color(container, lv_color_black(), 0); 
                                    lv_obj_set_style_bg_main_stop(container, 128, 0);
                                    lv_obj_set_style_bg_grad_stop(container, 255, 0);
                                    lv_obj_set_style_bg_grad_dir(container, LV_GRAD_DIR_VER, 0);
                                }
                            }

                            if(label_btn_process_cancel) lv_label_set_text(label_btn_process_cancel, tr("Back"));
                            lv_obj_clear_state(btn_process_cancel, LV_STATE_DISABLED);
                            lvgl_port_unlock();
                        }
                        return; // Выходим после обработки завершающей команды
                    } 
                    else if (response == "EVENT:NITROGEN_ERROR_CHOICE_REQUIRED") {
                        nitrogen_error_dialog_pending = true;
                    } 
                    else if (lv_scr_act() == screen_process_execution) {
                        lvgl_port_lock(-1);
                        if (response.startsWith("EVENT:START_COUNTDOWN:")) {
                            lv_label_set_text(label_process_status_title, tr("Bring the model to the indicator"));
                        } else {
                            lv_label_set_text(label_process_status_title, translateSystemStatus(response.c_str()));
                        }
                        lvgl_port_unlock();
                    }
                }
            }
        }
    }

    // 3.5. Обработка отложенных UI-событий
    if (nitrogen_error_dialog_pending) {
        nitrogen_error_dialog_pending = false;
        show_choice_dialog("Nitrogen System Error", "The oxygen level is not decreasing. Continue the process without nitrogen?");
    }
}