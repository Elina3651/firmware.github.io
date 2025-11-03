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
// Global Definitions & Fonts
//==========================================================================
using namespace esp_panel::drivers;
using namespace esp_panel::board;

const bool ENABLE_SPLASH_SCREEN = true; // true - показывать заствку, false - НЕ показывать заствку
#define LVGL_HEAP_SIZE (96 * 1024)
#define FILE_CONTENT_BUFFER_SIZE 1024
const char* settings_file_path = "/settings.txt";
const char* lab_settings_file_path = "/lab_settings.txt";
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
    int monomer_blow_min;           // 1 - 10, default 1
    int uv_exposure_sec;            // 10 - 200, default 10
    bool use_cooling;
    bool use_nitrogen;              
    int nitrogen_target_percent;
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
    int primary_uv_exposure_sec;
    int secondary_uv_exposure_sec;
    bool chamber_cooling_enabled;
    int  primary_uv_mode; // 0=Type1, 1=Type2, 2=Both
    int  primary_uv_flicker_rate;
    int  secondary_uv_mode; // 0=Type1, 1=Type2, 2=Both
    int  tertiary_uv_exposure_sec;
    int  tertiary_uv_mode; // 0=Type1, 1=Type2, 2=Both
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

struct ProfileCacheEntry {
    char display_name[100];
    char filename[64];
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

//==========================================================================
// Global Variables
//==========================================================================

// --- System & Core Objects ---
Board* board = nullptr;
esp_expander::CH422G* ch422g = nullptr;
#define RX1_PIN 15 // желтый
#define TX1_PIN 44 // зелёный
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

// --- Application State & Data ---
bool sd_card_initialized = false;
bool main_process_running = false;
bool needs_list_refresh = false;
bool is_on_splash_screen = false;
int current_profile_next_id = 1;
int current_profile_list_page = 0;
int total_profile_pages = 0;
GlobalSettingsData current_global_settings;
LaboratorySettingsData current_lab_settings;
ProfileData current_active_profile_data;
std::vector<ProfileCacheEntry> all_profile_entries_cache;
std::vector<String> service_keys;
char file_content_buffer[FILE_CONTENT_BUFFER_SIZE];
char current_selected_profile_filename[64];
char decision_text[50];
char full_status_text[100];
bool heater_decision = false;
bool cooling_decision = false;
bool is_lab_mode_running = false;

bool is_nitrogen_test_active = false;
bool is_air_test_active = false;
volatile bool nitrogen_error_dialog_pending = false; // Флаг для показа окна

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

// Переключатели
lv_obj_t* sw_glaze_cooling;
lv_obj_t* sw_glaze_nitrogen;

// Контейнеры и метки (для доступа и перевода)
lv_obj_t* nitrogen_glaze_container; // Контейнер для настроек азота
lv_obj_t* label_glaze_title_flicker;
lv_obj_t* label_glaze_uv_on;
lv_obj_t* label_glaze_uv_off;
lv_obj_t* label_glaze_title_timers;
lv_obj_t* label_glaze_monomer_blow;
lv_obj_t* label_glaze_uv_exposure;
lv_obj_t* label_glaze_title_aux;
lv_obj_t* label_glaze_cooling;
lv_obj_t* label_glaze_nitrogen;
lv_obj_t* label_glaze_nitrogen_target;

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
lv_obj_t* sw_edit_chamber_cooling;

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
lv_obj_t* label_edit_cooling_title;
lv_obj_t* label_edit_thermal_chamber_title;
lv_obj_t* label_edit_thermal_temp_title;
lv_obj_t* label_edit_heat_hold_title;

// --- Help Screen Widgets ---
lv_obj_t* label_help_title;
lv_obj_t* label_help_content;
lv_obj_t* btn_help_close;
lv_obj_t* label_btn_help_close;

// --- Process Execution Screen Widgets ---
lv_obj_t * label_process_status_title, *label_process_status_detail;
lv_obj_t * spinner_process_execution, *btn_process_cancel;
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


//==========================================================================
// Function Prototypes
//==========================================================================

// --- Initialization & System ---
bool initializeSDCard();
void loadGlobalSettings();
void saveGlobalSettings();
void loadLaboratorySettings();
void saveLaboratorySettings();

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
static void build_confirm_delete_dialog(lv_obj_t* parent_for_dialog);
static void build_info_dialog(lv_obj_t* parent_layer);
static void build_choice_dialog(lv_obj_t* parent_layer);
static void build_help_screen(lv_obj_t* parent_screen);
static void build_input_shield(void);

// --- Custom Widget Functions ---
static void uv_mode_selector_event_cb(lv_event_t * e);
static uint16_t get_checked_btnmatrix_id(lv_obj_t* btnm);
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
static void lab_mode_start_event_cb(lv_event_t* e);
void prepare_and_send_lab_command(int mode_id);
static void lab_mode_param_changed_event_cb(lv_event_t * e);

// --- Core Logic, File System & Helpers ---
void displayProfileListPage();
void enter_service_lock_mode(const char* message_eng, const char* message_rus);
bool handle_save_new_profile_logic(const char* profile_input_name);
int scanAndCacheAllProfiles(fs::FS &fs_ref, std::vector<ProfileCacheEntry>& cache_vector);
bool readFileContentToBuffer_ino(fs::FS &fs_ref, const char * path, char* buffer, size_t buffer_size);
bool parseProfileJson(const char* jsonString, ProfileData& profile);
bool serializeProfileJson(const ProfileData& profile, char* outputBuffer, size_t bufferSize);
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



// Раздел 1: Вспомогательные функции 
// ==========================================================================
void flush_serial_buffer() {
    // Ждем небольшую паузу, чтобы все возможные данные успели прийти
    delay(50); 
    while (MySerial1.available() > 0) {
        MySerial1.read();
    }
    Serial.println("UART RX Buffer Flushed.");
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


// Обработчик для кнопок ВНУТРИ сцены теста азота
static void test_nitrogen_screen_event_cb(lv_event_t* e) {
    lv_event_code_t code = lv_event_get_code(e);
    if (code != LV_EVENT_CLICKED) return;

    const char* user_data = (const char*)lv_event_get_user_data(e);
    StaticJsonDocument<128> doc;
    String output;
    
    if (strcmp(user_data, "TOGGLE_SUPPLY") == 0) {
        is_nitrogen_test_active = !is_nitrogen_test_active; // Инвертируем состояние

        if (is_nitrogen_test_active) {
            // --- Включаем подачу ---
            doc["command"] = "TEST_VALVE_NITROGEN_OPEN";
            lv_label_set_text(label_btn_nitrogen_test_press, tr("Stop Supply"));
            // Меняем цвет кнопки на "активный" (красный)
            lv_obj_set_style_bg_color(btn_nitrogen_test_press, lv_palette_main(LV_PALETTE_RED), 0);
        } else {
            // --- Выключаем подачу ---
            doc["command"] = "TEST_VALVE_NITROGEN_CLOSE";
            lv_label_set_text(label_btn_nitrogen_test_press, tr("Start Supply"));

            // <<< ВОТ ПРАВИЛЬНОЕ ИСПРАВЛЕНИЕ >>>
            // Удаляем ТОЛЬКО локальное свойство цвета фона. Все остальные стили темы остаются!
            lv_obj_remove_local_style_prop(btn_nitrogen_test_press, LV_STYLE_BG_COLOR, 0);
        }
        serializeJson(doc, output);
        flush_serial_buffer();
        MySerial1.println(output);

    } else if (strcmp(user_data, "BACK") == 0) {
        // --- Безопасный выход: всегда выключаем подачу ---
        if (is_nitrogen_test_active) {
            is_nitrogen_test_active = false; // Сбрасываем состояние
            doc["command"] = "TEST_VALVE_NITROGEN_CLOSE";
            serializeJson(doc, output);
            MySerial1.println(output);
        }
        lv_scr_load(screen_settings);
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
    StaticJsonDocument<128> doc;
    String output;

    if (strcmp(user_data, "TOGGLE_SUPPLY") == 0) {
        is_air_test_active = !is_air_test_active; // Инвертируем состояние

        if (is_air_test_active) {
            // --- Включаем подачу ---
            doc["command"] = "TEST_VALVE_AIR_OPEN";
            lv_label_set_text(label_btn_air_test_press, tr("Stop Supply"));
            lv_obj_set_style_bg_color(btn_air_test_press, lv_palette_main(LV_PALETTE_RED), 0);
        } else {
            // --- Выключаем подачу ---
            doc["command"] = "TEST_VALVE_AIR_CLOSE";
            lv_label_set_text(label_btn_air_test_press, tr("Start Supply"));
            
            // <<< И ЗДЕСЬ ТОЖЕ ПРАВИЛЬНОЕ ИСПРАВЛЕНИЕ >>>
            lv_obj_remove_local_style_prop(btn_air_test_press, LV_STYLE_BG_COLOR, 0);
        }
        serializeJson(doc, output);
        MySerial1.println(output);

    } else if (strcmp(user_data, "BACK") == 0) {
        // --- Безопасный выход: всегда выключаем подачу ---
        if (is_air_test_active) {
            is_air_test_active = false; // Сбрасываем состояние
            doc["command"] = "TEST_VALVE_AIR_CLOSE";
            serializeJson(doc, output);
            MySerial1.println(output);
        }
        lv_scr_load(screen_settings);
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
static uint16_t get_checked_btnmatrix_id(lv_obj_t* btnm) {
    if (!btnm) return 1; // Безопасное значение по умолчанию

    // Мы точно знаем, что у нас всегда 3 активные кнопки с ID 0, 1, 2.
    // Просто перебираем их.
    for (uint16_t i = 0; i < 3; i++) {
        if (lv_btnmatrix_has_btn_ctrl(btnm, i, LV_BTNMATRIX_CTRL_CHECKED)) {
            return i; // Нашли! Возвращаем ID.
        }
    }

    // Если по какой-то причине ни одна кнопка не выбрана (чего быть не должно),
    // возвращаем безопасное значение по умолчанию.
    return 1; 
}
static void update_uv_mode_selector_ui(lv_obj_t* btnm, lv_obj_t* label, uint32_t mode) {
    if (!btnm || !label) return;

    // 1. Прямая команда на сброс и установку нужной кнопки
    lv_btnmatrix_clear_btn_ctrl_all(btnm, LV_BTNMATRIX_CTRL_CHECKED);
    lv_btnmatrix_set_btn_ctrl(btnm, mode, LV_BTNMATRIX_CTRL_CHECKED);

    // 2. Прямая команда на обновление текста
    const char* text_eng;
    const char* text_rus;

    switch(mode) {
        case 0: text_eng = "type 1"; text_rus = "выбран тип 1"; break;
        case 1: text_eng = "type 2"; text_rus = "выбран тип 2"; break;
        case 2: text_eng = "both types";         text_rus = "оба типа";         break;
        default: text_eng = ""; text_rus = ""; break;
    }

    if (current_global_settings.language == 1) {
        lv_label_set_text(label, text_rus);
    } else {
        lv_label_set_text(label, text_eng);
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
int scanAndCacheAllProfiles(fs::FS &fs_ref, std::vector<ProfileCacheEntry>& cache_vector) {
    lvgl_port_lock(-1);
    cache_vector.clear();
    if (!sd_card_initialized) {
        Serial.println("scanAndCacheAllProfiles: SD card not initialized.");
        lvgl_port_unlock();
        return 1;
    }
    File root = fs_ref.open("/");
    if (!root) {
        Serial.println("scanAndCacheAllProfiles: Failed to open root directory.");
        lvgl_port_unlock();
        return 1;
    }
    if (!root.isDirectory()) {
        Serial.println("scanAndCacheAllProfiles: Root is not a directory.");
        root.close();
        lvgl_port_unlock();
        return 1;
    }
    Serial.println("Scanning ALL profile files and reading names for cache...");
    int max_id_found = 0;
    File entry = root.openNextFile();

    // --- НАЧАЛО ИЗМЕНЕНИЙ ---
    // Перемещаем большие переменные из стека в статическую память.
    // Это решает проблему "Stack canary".
    static ProfileData temp_profile_data_scan;
    static char temp_file_buffer_scan[FILE_CONTENT_BUFFER_SIZE];
    // --- КОНЕЦ ИЗМЕНЕНИЙ ---

    while (entry) {
        String entryFilenameOnly = entry.name();
        if (entryFilenameOnly.startsWith("/")) {
            entryFilenameOnly = entryFilenameOnly.substring(1);
        }
        if (!entry.isDirectory() && entryFilenameOnly.startsWith("profile_") && entryFilenameOnly.endsWith(".txt")) {
            
            ProfileCacheEntry new_entry;
            String profile_display_name_scan;

            strncpy(new_entry.filename, entryFilenameOnly.c_str(), sizeof(new_entry.filename) - 1);
            new_entry.filename[sizeof(new_entry.filename) - 1] = '\0';

            String full_path_scan = "/" + entryFilenameOnly;
            bool read_ok_scan = readFileContentToBuffer_ino(fs_ref, full_path_scan.c_str(), temp_file_buffer_scan, sizeof(temp_file_buffer_scan));
            
            if (read_ok_scan) {
                if (parseProfileJson(temp_file_buffer_scan, temp_profile_data_scan)) {
                    if (strlen(temp_profile_data_scan.name) > 0) {
                        profile_display_name_scan = String(temp_profile_data_scan.name);
                    } else {
                        profile_display_name_scan = entryFilenameOnly + " (No Name)";
                    }
                } else {
                    profile_display_name_scan = entryFilenameOnly + " (JSON err)";
                }
            } else {
                profile_display_name_scan = entryFilenameOnly + " (Read err)";
            }

            strncpy(new_entry.display_name, profile_display_name_scan.c_str(), sizeof(new_entry.display_name) - 1);
            new_entry.display_name[sizeof(new_entry.display_name) - 1] = '\0';
            
            cache_vector.push_back(new_entry);

            String id_str_scan = entryFilenameOnly.substring(8, entryFilenameOnly.length() - 4);
            int current_id_scan = id_str_scan.toInt();
            if (current_id_scan > 0 && current_id_scan > max_id_found) {
                max_id_found = current_id_scan;
            }
        }
        entry.close();
        entry = root.openNextFile();
    }
    root.close();

    // ... остальная часть функции без изменений ...
    total_profile_pages = (cache_vector.size() + PROFILES_PER_PAGE - 1) / PROFILES_PER_PAGE;
    if (total_profile_pages == 0 && !cache_vector.empty()) total_profile_pages = 1;
    else if (cache_vector.empty()) total_profile_pages = 0;
    if (current_profile_list_page >= total_profile_pages && total_profile_pages > 0) {
        current_profile_list_page = total_profile_pages - 1;
    } else if (total_profile_pages == 0) {
        current_profile_list_page = 0;
    }
    Serial.printf("Total profiles cached: %d, Total pages: %d\n", cache_vector.size(), total_profile_pages);
    Serial.printf("Max profile ID found: %d. Next ID for new profile will be: %d\n", max_id_found, max_id_found + 1);
    lvgl_port_unlock();
    return max_id_found + 1;
}
bool parseProfileJson(const char* jsonString, ProfileData& profile) {
    StaticJsonDocument<FILE_CONTENT_BUFFER_SIZE> doc;
    DeserializationError error = deserializeJson(doc, jsonString);
    if (error) { Serial.print(F("deserializeJson() failed: ")); Serial.println(error.f_str()); return false; }

    profile.id = doc["id"] | -1;
    const char* name_ptr = doc["name"] | "Unnamed_Profile";
    strncpy(profile.name, name_ptr, sizeof(profile.name) - 1);
    profile.name[sizeof(profile.name) - 1] = '\0';

    profile.thermal_chamber_enabled = doc["thermal_chamber_enabled"] | true;
    profile.thermal_chamber_temp = doc["thermal_chamber_temp"] | 45;
    profile.heat_exchange_hold_sec = doc["heat_exchange_hold_sec"] | 60;
    profile.nitrogen_use_enabled = doc["nitrogen_use_enabled"] | false;
    profile.nitrogen_target_percent = doc["nitrogen_target_percent"] | 99;
    profile.primary_uv_exposure_sec = doc["primary_uv_exposure_sec"] | 30;
    profile.secondary_uv_exposure_sec = doc["secondary_uv_exposure_sec"] | 60;

    if (doc.containsKey("primary_uv_flicker_rate")) {
        profile.primary_uv_flicker_rate = doc["primary_uv_flicker_rate"];
    } else {
        profile.primary_uv_flicker_rate = doc["primary_uv_type1_flickers"] | 5;
    }
    profile.tertiary_uv_exposure_sec = doc["tertiary_uv_exposure_sec"] | 60;

    // --- Primary UV Mode ---
    if (doc.containsKey("primary_uv_mode")) {
        // ИСПРАВЛЕНИЕ: Просто читаем значение как есть.
        profile.primary_uv_mode = doc["primary_uv_mode"];
    } else { // Ключа нет, это старый профиль, конвертируем из bool
        bool t1 = doc["primary_uv_type1_enabled"] | false;
        bool t2 = doc["primary_uv_type2_enabled"] | true;
        if (t1 && t2) profile.primary_uv_mode = 2; else if (t1) profile.primary_uv_mode = 0; else profile.primary_uv_mode = 1;
    }

    // --- Secondary UV Mode ---
    if (doc.containsKey("secondary_uv_mode")) {
        // ИСПРАВЛЕНИЕ: Просто читаем значение как есть.
        profile.secondary_uv_mode = doc["secondary_uv_mode"];
    } else {
        bool t1 = doc["secondary_uv_type1_enabled"] | false;
        bool t2 = doc["secondary_uv_type2_enabled"] | true;
        if (t1 && t2) profile.secondary_uv_mode = 2; else if (t1) profile.secondary_uv_mode = 0; else profile.secondary_uv_mode = 1;
    }

    // --- Tertiary UV Mode ---
    if (doc.containsKey("tertiary_uv_mode")) {
        // ИСПРАВЛЕНИЕ: Просто читаем значение как есть.
        profile.tertiary_uv_mode = doc["tertiary_uv_mode"];
    } else {
        bool t1 = doc["tertiary_uv_type1_enabled"] | false;
        bool t2 = doc["tertiary_uv_type2_enabled"] | true;
        if (t1 && t2) profile.tertiary_uv_mode = 2; else if (t1) profile.tertiary_uv_mode = 0; else profile.tertiary_uv_mode = 1;
    }

    profile.chamber_cooling_enabled = doc["chamber_cooling_enabled"] | false;
    return true;
}
bool serializeProfileJson(const ProfileData& profile, char* outputBuffer, size_t bufferSize) {
    StaticJsonDocument<FILE_CONTENT_BUFFER_SIZE> doc; 
    doc["id"] = profile.id; doc["name"] = profile.name;
    doc["thermal_chamber_enabled"] = profile.thermal_chamber_enabled; 
    doc["thermal_chamber_temp"] = profile.thermal_chamber_temp;
    doc["heat_exchange_hold_sec"] = profile.heat_exchange_hold_sec;
    doc["nitrogen_use_enabled"] = profile.nitrogen_use_enabled; 
    doc["nitrogen_target_percent"] = profile.nitrogen_target_percent;
    doc["primary_uv_exposure_sec"] = profile.primary_uv_exposure_sec;
    doc["secondary_uv_exposure_sec"] = profile.secondary_uv_exposure_sec; 
    
    // <<<--- ЗАПИСЬ НОВЫХ ПОЛЕЙ В JSON ---<<<
    // <<<--- ИЗМЕНЕНО: Запись int селекторов вместо bool ---<<<
    doc["primary_uv_mode"] = profile.primary_uv_mode;
    doc["primary_uv_flicker_rate"] = profile.primary_uv_flicker_rate;

    doc["secondary_uv_mode"] = profile.secondary_uv_mode;
    doc["tertiary_uv_exposure_sec"] = profile.tertiary_uv_exposure_sec;
    doc["tertiary_uv_mode"] = profile.tertiary_uv_mode;

    doc["chamber_cooling_enabled"] = profile.chamber_cooling_enabled;
    
    size_t written = serializeJsonPretty(doc, outputBuffer, bufferSize); 
    if (written == 0 || written >= bufferSize -1 ) { Serial.println(F("serializeJsonPretty() failed or buffer too small.")); outputBuffer[bufferSize-1] = '\0'; return false; }
    return true;
}
void loadGlobalSettings() {
    // // <<< НАЧАЛО ВРЕМЕННОГО БЛОКА ДЛЯ ПЕРЕСОЗДАНИЯ ФАЙЛОВ >>>
    // Serial.println("\n\n!!! DEBUG: FORCING REGENERATION OF SETTINGS AND KEYS !!!");
    // if (sd_card_initialized) {
    //     if (SD.exists(settings_file_path)) {
    //         SD.remove(settings_file_path);
    //         Serial.println("!!! DEBUG: Old settings.txt REMOVED.");
    //     }
    //     if (SD.exists("/service_keys.json")) {
    //         SD.remove("/service_keys.json");
    //         Serial.println("!!! DEBUG: Old service_keys.json REMOVED.");
    //     }
    // }
    // Serial.println("!!! DEBUG: PROCEEDING WITH FIRST-RUN LOGIC... !!!\n");
    // // конец

    Serial.println("-> Entering loadGlobalSettings...");
    lvgl_port_lock(-1);
    // Устанавливаем значения по умолчанию на случай, если файл не найден или ошибка
    current_global_settings.is_first_run = true;
    current_global_settings.is_heater_error = false;
    current_global_settings.nitrogen_system_enabled = false;
    current_global_settings.compressed_air_system_enabled = false;
    current_global_settings.language = 1; // 0 = ENG, 1 = RUS
    current_global_settings.theme = 1;    // 0 = Light, 1 = DARK

    if (!sd_card_initialized) {
        Serial.println("loadGlobalSettings: SD card not initialized. Exiting.");
        return;
    }

    Serial.println("   Checking for settings.txt...");
    if (SD.exists(settings_file_path)) {
        Serial.println("   settings.txt exists. Opening for read...");
        File settingsFile = SD.open(settings_file_path, FILE_READ);
        if (settingsFile) {
            Serial.println("   File opened. Deserializing JSON...");
            StaticJsonDocument<512> doc; // Небольшой JSON для настроек
            DeserializationError error = deserializeJson(doc, settingsFile);
            if (!error) {
                Serial.println("   JSON OK. Reading flags...");
                current_global_settings.is_first_run = doc["is_first_run"] | true;
                current_global_settings.is_heater_error = doc["is_heater_error"] | false;
                current_global_settings.nitrogen_system_enabled = doc["nitrogen_system_enabled"] | false;
                current_global_settings.compressed_air_system_enabled = doc["compressed_air_system_enabled"] | false;
                current_global_settings.language = doc["language"] | 0; // Если ключа нет, будет 0 (ENG)
                current_global_settings.theme = doc["theme"] | 0;
                current_global_settings.screen_timeout_mode = doc["screen_timeout_mode"] | 0; 
                Serial.println("Global settings loaded successfully.");
            } else {
                Serial.print("   JSON ERROR: "); Serial.println(error.c_str());
            }
            settingsFile.close();
            Serial.println("   File closed.");
        } else {
            Serial.println("   ERROR: Failed to open settings.txt for reading.");
        }
    } 
    else {
        Serial.println("   settings.txt does not exist.");
    }


    // КЛЮЧЕВАЯ ЛОГИКА ПРОВЕРКИ
    Serial.printf("   Checking is_first_run flag. It is: %s\n", current_global_settings.is_first_run ? "true" : "false");
    if (current_global_settings.is_first_run) {
        Serial.println("   First run detected. Calling generate_and_save_service_keys()...");
        generate_and_save_service_keys();
        
        Serial.println("   Setting is_first_run to false...");
        current_global_settings.is_first_run = false;
        
        Serial.println("   Calling saveGlobalSettings() to persist the new flag...");
        saveGlobalSettings();
    } else {
        Serial.println("   Regular run. Skipping key generation.");
    }

    Serial.println("   Calling load_service_keys_from_sd()...");
    load_service_keys_from_sd();

    lvgl_port_unlock();
    Serial.println("<- Exiting loadGlobalSettings.");
}

void saveGlobalSettings() {
    Serial.println("   -> Entering saveGlobalSettings...");
    lvgl_port_lock(-1);
    if (!sd_card_initialized) {
        Serial.println("saveGlobalSettings: SD card not initialized. Cannot save settings.");
        return;
    }

    StaticJsonDocument<512> doc;
    doc["is_first_run"] = current_global_settings.is_first_run;
    doc["is_heater_error"] = current_global_settings.is_heater_error;
    doc["nitrogen_system_enabled"] = current_global_settings.nitrogen_system_enabled;
    doc["compressed_air_system_enabled"] = current_global_settings.compressed_air_system_enabled;
    doc["language"] = current_global_settings.language;
    doc["theme"] = current_global_settings.theme;
    doc["screen_timeout_mode"] = current_global_settings.screen_timeout_mode;

    File settingsFile = SD.open(settings_file_path, FILE_WRITE);
    if (settingsFile) {
        serializeJsonPretty(doc, settingsFile);
        settingsFile.close();
        Serial.println("      Global settings saved to file.");
    } else {
        Serial.println("      ERROR: Failed to save global settings.");
    }
    lvgl_port_unlock();
    Serial.println("   <- Exiting saveGlobalSettings.");
}
void saveLaboratorySettings() {
    Serial.println("   -> Entering saveLaboratorySettings...");
    lvgl_port_lock(-1);
    if (!sd_card_initialized) {
        Serial.println("saveLaboratorySettings: SD card not initialized. Cannot save.");
        lvgl_port_unlock();
        return;
    }

    StaticJsonDocument<1024> doc;

    // Режим 1: Глазурь
    JsonObject glaze = doc.createNestedObject("glaze");
    glaze["uv_on_sec"] = current_lab_settings.glaze.uv_on_sec;
    glaze["uv_off_sec"] = current_lab_settings.glaze.uv_off_sec;
    glaze["monomer_blow_min"] = current_lab_settings.glaze.monomer_blow_min;
    glaze["uv_exposure_sec"] = current_lab_settings.glaze.uv_exposure_sec;
    glaze["use_cooling"] = current_lab_settings.glaze.use_cooling;
    glaze["use_nitrogen"] = current_lab_settings.glaze.use_nitrogen; 
    glaze["nitrogen_target_percent"] = current_lab_settings.glaze.nitrogen_target_percent;
    
    // Режим 2: Ремонт
    JsonObject repair = doc.createNestedObject("repair");
    repair["countdown_sec"] = current_lab_settings.repair.countdown_sec;
    repair["uv_exposure_sec"] = current_lab_settings.repair.uv_exposure_sec;

    // Режим 3: Прочность
    JsonObject strength = doc.createNestedObject("strength");
    strength["chamber_temp_c"] = current_lab_settings.strength.chamber_temp_c;
    strength["hold_time_min"] = current_lab_settings.strength.hold_time_min;
    strength["use_cooling"] = current_lab_settings.strength.use_cooling;
    strength["uv_pulse_duration_sec"] = current_lab_settings.strength.uv_pulse_duration_sec;
    strength["uv_pulse_interval_min"] = current_lab_settings.strength.uv_pulse_interval_min;

    // Режим 4: Термокамера
    JsonObject thermal = doc.createNestedObject("thermal");
    thermal["chamber_temp_c"] = current_lab_settings.thermal.chamber_temp_c;
    thermal["hold_time_min"] = current_lab_settings.thermal.hold_time_min;
    
    // Режим 5: Осветление
    JsonObject lighten = doc.createNestedObject("lighten");
    lighten["chamber_temp_c"] = current_lab_settings.lighten.chamber_temp_c;
    lighten["hold_time_min"] = current_lab_settings.lighten.hold_time_min;
    lighten["use_cooling"] = current_lab_settings.lighten.use_cooling;

    // Режим 6: Затемнение
    JsonObject darken = doc.createNestedObject("darken");
    darken["uv_exposure_min"] = current_lab_settings.darken.uv_exposure_min;
    darken["use_cooling"] = current_lab_settings.darken.use_cooling;

    File settingsFile = SD.open(lab_settings_file_path, FILE_WRITE);
    if (settingsFile) {
        serializeJsonPretty(doc, settingsFile);
        settingsFile.close();
        Serial.println("      Laboratory settings saved to file.");
    } else {
        Serial.println("      ERROR: Failed to save laboratory settings.");
    }
    lvgl_port_unlock();
    Serial.println("   <- Exiting saveLaboratorySettings.");
}

void loadLaboratorySettings() {
    Serial.println("-> Entering loadLaboratorySettings...");
    lvgl_port_lock(-1);

    // Устанавливаем значения по умолчанию
    current_lab_settings.glaze.uv_on_sec = 2.0f;
    current_lab_settings.glaze.uv_off_sec = 2.0f;
    current_lab_settings.glaze.monomer_blow_min = 1;
    current_lab_settings.glaze.uv_exposure_sec = 10;
    current_lab_settings.glaze.use_cooling = false;
    current_lab_settings.glaze.use_nitrogen = false;
    current_lab_settings.glaze.nitrogen_target_percent = 99;
    
    current_lab_settings.repair.countdown_sec = 5;
    current_lab_settings.repair.uv_exposure_sec = 5;
    
    current_lab_settings.strength.chamber_temp_c = 50;
    current_lab_settings.strength.hold_time_min = 30;
    current_lab_settings.strength.use_cooling = false;
    current_lab_settings.strength.uv_pulse_duration_sec = 1;
    current_lab_settings.strength.uv_pulse_interval_min = 1;
    
    current_lab_settings.thermal.chamber_temp_c = 40;
    current_lab_settings.thermal.hold_time_min = 5;
    
    current_lab_settings.lighten.chamber_temp_c = 80; // Фиксированное значение
    current_lab_settings.lighten.hold_time_min = 10;
    current_lab_settings.lighten.use_cooling = false;
    
    current_lab_settings.darken.uv_exposure_min = 1;
    current_lab_settings.darken.use_cooling = false;
    
    if (!sd_card_initialized) {
        Serial.println("loadLaboratorySettings: SD card not initialized. Using defaults.");
        lvgl_port_unlock();
        return;
    }

    if (!SD.exists(lab_settings_file_path)) {
        Serial.println("   lab_settings.txt not found. Creating with default values...");
        saveLaboratorySettings(); // Сохраняем файл с дефолтными значениями
        lvgl_port_unlock();
        return;
    }

    File settingsFile = SD.open(lab_settings_file_path, FILE_READ);
    if (settingsFile) {
        StaticJsonDocument<1024> doc;
        DeserializationError error = deserializeJson(doc, settingsFile);
        if (!error) {
            // Читаем настройки для каждого режима, используя | для установки дефолтного значения в случае отсутствия ключа
            JsonObject glaze = doc["glaze"];
            current_lab_settings.glaze.uv_on_sec = glaze["uv_on_sec"] | 2.0f;
            current_lab_settings.glaze.uv_off_sec = glaze["uv_off_sec"] | 2.0f;
            current_lab_settings.glaze.monomer_blow_min = glaze["monomer_blow_min"] | 1;
            current_lab_settings.glaze.uv_exposure_sec = glaze["uv_exposure_sec"] | 10;
            current_lab_settings.glaze.use_cooling = glaze["use_cooling"] | false;
            current_lab_settings.glaze.use_nitrogen = glaze["use_nitrogen"] | false;
            current_lab_settings.glaze.nitrogen_target_percent = glaze["nitrogen_target_percent"] | 99; 

            JsonObject repair = doc["repair"];
            current_lab_settings.repair.countdown_sec = repair["countdown_sec"] | 5;
            current_lab_settings.repair.uv_exposure_sec = repair["uv_exposure_sec"] | 5;

            JsonObject strength = doc["strength"];
            current_lab_settings.strength.chamber_temp_c = strength["chamber_temp_c"] | 50;
            current_lab_settings.strength.hold_time_min = strength["hold_time_min"] | 30;
            current_lab_settings.strength.use_cooling = strength["use_cooling"] | false;
            current_lab_settings.strength.uv_pulse_duration_sec = strength["uv_pulse_duration_sec"] | 1;
            current_lab_settings.strength.uv_pulse_interval_min = strength["uv_pulse_interval_min"] | 1;
            
            JsonObject thermal = doc["thermal"];
            current_lab_settings.thermal.chamber_temp_c = thermal["chamber_temp_c"] | 40;
            current_lab_settings.thermal.hold_time_min = thermal["hold_time_min"] | 5;

            JsonObject lighten = doc["lighten"];
            current_lab_settings.lighten.chamber_temp_c = lighten["chamber_temp_c"] | 80;
            current_lab_settings.lighten.hold_time_min = lighten["hold_time_min"] | 10;
            current_lab_settings.lighten.use_cooling = lighten["use_cooling"] | false;

            JsonObject darken = doc["darken"];
            current_lab_settings.darken.uv_exposure_min = darken["uv_exposure_min"] | 1;
            current_lab_settings.darken.use_cooling = darken["use_cooling"] | false;

            Serial.println("Laboratory settings loaded successfully.");
        } else {
            Serial.print("   JSON ERROR in lab_settings.txt: "); Serial.println(error.c_str());
        }
        settingsFile.close();
    } else {
        Serial.println("   ERROR: Failed to open lab_settings.txt for reading.");
    }
    lvgl_port_unlock();
    Serial.println("<- Exiting loadLaboratorySettings.");
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

// Записывает предопределенный список из 100 ключей в service_keys.json
void generate_and_save_service_keys() {
    Serial.println("   -> Entering generate_and_save_service_keys...");
    const char* keys_file_path = "/service_keys.json";

    Serial.println("      Checking for service_keys.json...");
    if (SD.exists(keys_file_path)) {
        Serial.println("Service keys file already exists. Skipping creation.");
        return;
    }

    Serial.println("      Creating predefined keys in JSON object...");

    // Используем StaticJsonDocument, так как размер известен и не меняется
    StaticJsonDocument<4096> doc; 
    JsonArray keys_array = doc.to<JsonArray>();

    // Добавляем твой список ключей
    keys_array.add("8614"); keys_array.add("5483"); keys_array.add("7419"); keys_array.add("2057");
    keys_array.add("1928"); keys_array.add("8370"); keys_array.add("8107"); keys_array.add("9531");
    keys_array.add("4172"); keys_array.add("8425"); keys_array.add("7936"); keys_array.add("8512");
    keys_array.add("3270"); keys_array.add("4829"); keys_array.add("4310"); keys_array.add("2834");
    keys_array.add("5719"); keys_array.add("8542"); keys_array.add("7759"); keys_array.add("1036");
    keys_array.add("1584"); keys_array.add("2471"); keys_array.add("9538"); keys_array.add("4821");
    keys_array.add("7309"); keys_array.add("7056"); keys_array.add("2184"); keys_array.add("4802");
    keys_array.add("8249"); keys_array.add("8591"); keys_array.add("3805"); keys_array.add("1853");
    keys_array.add("9217"); keys_array.add("2830"); keys_array.add("5429"); keys_array.add("4918");
    keys_array.add("7124"); keys_array.add("1937"); keys_array.add("8201"); keys_array.add("6731");
    keys_array.add("7409"); keys_array.add("5082"); keys_array.add("1429"); keys_array.add("5320");
    keys_array.add("8915"); keys_array.add("1504"); keys_array.add("1520"); keys_array.add("3981");
    keys_array.add("1479"); keys_array.add("6210"); keys_array.add("7812"); keys_array.add("7904");
    keys_array.add("5109"); keys_array.add("2971"); keys_array.add("9130"); keys_array.add("6621");
    keys_array.add("1035"); keys_array.add("1872"); keys_array.add("2906"); keys_array.add("5625");
    keys_array.add("4719"); keys_array.add("3184"); keys_array.add("5590"); keys_array.add("2517");
    keys_array.add("1852"); keys_array.add("6520"); keys_array.add("2167"); keys_array.add("5421");
    keys_array.add("5813"); keys_array.add("3712"); keys_array.add("1592"); keys_array.add("4720");
    keys_array.add("4018"); keys_array.add("5921"); keys_array.add("2017"); keys_array.add("2019");
    keys_array.add("3751"); keys_array.add("9148"); keys_array.add("4862"); keys_array.add("1408");
    keys_array.add("8206"); keys_array.add("9210"); keys_array.add("3710"); keys_array.add("5139");
    keys_array.add("9370"); keys_array.add("1305"); keys_array.add("6109"); keys_array.add("2501");
    keys_array.add("8271"); keys_array.add("9714"); keys_array.add("1703"); keys_array.add("3921");
    keys_array.add("1730"); keys_array.add("2805"); keys_array.add("7412"); keys_array.add("2958");
    keys_array.add("6142"); keys_array.add("7316"); keys_array.add("5731"); keys_array.add("1472");

    Serial.println("      Opening service_keys.json for writing...");
    File file = SD.open(keys_file_path, FILE_WRITE);
    if (file) {
        Serial.println("      File opened. Serializing and writing...");
        if (serializeJson(doc, file) == 0) {
            Serial.println("      ERROR: Failed to write keys to file.");
        } else {
            Serial.println("      Keys written successfully.");
        }
        file.close();
        Serial.println("      File closed.");
    } else {
        Serial.println("      ERROR: Failed to open service_keys.json for writing.");
    }
    Serial.println("   <- Exiting generate_and_save_service_keys.");
}

// Загружает ключи из файла в вектор service_keys в памяти
void load_service_keys_from_sd() {
    const char* keys_file_path = "/service_keys.json";
    service_keys.clear();

    File file = SD.open(keys_file_path, FILE_READ);
    if (!file) {
        Serial.println("ERROR: service_keys.json not found! Cannot load keys.");
        return;
    }

    DynamicJsonDocument doc(4096);
    DeserializationError error = deserializeJson(doc, file);
    file.close();

    if (error) {
        Serial.println("ERROR: Failed to parse service_keys.json");
        return;
    }

    JsonArray keys_array = doc.as<JsonArray>();
    for (JsonVariant v : keys_array) {
        service_keys.push_back(v.as<String>());
    }
    Serial.printf("%d service keys loaded into memory.\n", service_keys.size());
}

// Удаляет использованный ключ и перезаписывает файл
void remove_and_save_service_keys(String key_to_remove) {
    const char* keys_file_path = "/service_keys.json";
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

    // Теперь перезаписываем файл на SD карте из обновленного вектора
    DynamicJsonDocument doc(4096);
    JsonArray keys_array = doc.to<JsonArray>();
    for (const auto& key : service_keys) {
        keys_array.add(key);
    }

    File file = SD.open(keys_file_path, FILE_WRITE);
    if (file) {
        serializeJson(doc, file);
        file.close();
        Serial.printf("Key '%s' removed. %d keys remaining.\n", key_to_remove.c_str(), service_keys.size());
    } else {
        Serial.println("ERROR: Failed to open service_keys.json for re-writing.");
    }
}

void enter_service_lock_mode(const char* message_eng, const char* message_rus) {
    // const char* message = (current_global_settings.language == 1) ? message_rus : message_eng; // <<< УДАЛИ ЭТУ СТРОКУ
    Serial.printf("ENTERING SERVICE LOCK MODE. Reason: %s\n", message_eng); // Логируем всегда на английском для удобства
    
    // 1. Устанавливаем флаг ошибки
    current_global_settings.is_heater_error = true;
    
    // 2. НЕМЕДЛЕННО СОХРАНЯЕМ НАСТРОЙКИ, ЧТОБЫ ЗАФИКСИРОВАТЬ БЛОКИРОВКУ
    saveGlobalSettings();
    
    // 3. Останавливаем все процессы
    main_process_running = false;
    // Отправляем команду аварийной остановки по UART
    StaticJsonDocument<128> doc;
    doc["command"] = "EMERGENCY_STOP";
    String output;
    serializeJson(doc, output);
    MySerial1.println(output);
    Serial.println("Sent UART command: EMERGENCY_STOP");
    
    // 4. Переключаемся на экран блокировки
    lvgl_port_lock(-1);
    
    // <<< УДАЛИ ЭТУ СТРОКУ >>>
    // lv_label_set_text(label_service_lock_msg, message);

    // <<< ДОБАВЬ ЭТОТ БЛОК >>>
    // Мы просто загружаем экран. Текст будет установлен на самом экране.
    if(screen_service_lock) {
        lv_scr_load(screen_service_lock);
    }
    
    lvgl_port_unlock();
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
            lv_scr_load(screen_laboratory);
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

    screen_to_return_after_process = screen_profile_details; 

    is_lab_mode_running = false;

    Serial.println("--- START button clicked. Preparing full process command. ---");

    // 1. Переходим на экран процесса и показываем, что мы ждем подтверждения
    if (screen_process_execution) {
        // --- БЛОК ПЕРЕВОДА ---
        if (current_global_settings.language == 1) { // RUS
            lv_label_set_text(label_process_status_title, "Запуск процесса...");
            lv_label_set_text(label_process_status_detail, "Отправка профиля на контроллер...");
            lv_label_set_text(label_btn_process_cancel, "Отмена");
        } else { // ENG
            lv_label_set_text(label_process_status_title, "Starting Process...");
            lv_label_set_text(label_process_status_detail, "Sending profile to controller...");
            lv_label_set_text(label_btn_process_cancel, "Cancel");
        }

        apply_theme_to_process_screen(); 

        // ---------------------
        lv_scr_load(screen_process_execution);
        lv_obj_clear_flag(btn_process_cancel, LV_OBJ_FLAG_HIDDEN);
        if(spinner_process_execution) lv_obj_clear_flag(spinner_process_execution, LV_OBJ_FLAG_HIDDEN);
    }

    // 2. Создаем JSON-документ для команды
    StaticJsonDocument<1024> doc;

    // 3. Формируем команду
    doc["command"] = "START_PROCESS";
    
    // 4. Вкладываем все параметры профиля в объект "params"
    JsonObject params = doc.createNestedObject("params");
    params["thermal_chamber_enabled"] = current_active_profile_data.thermal_chamber_enabled;
    params["thermal_chamber_temp"] = current_active_profile_data.thermal_chamber_temp;
    params["heat_exchange_hold_sec"] = current_active_profile_data.heat_exchange_hold_sec;
    params["nitrogen_use_enabled"] = current_active_profile_data.nitrogen_use_enabled;
    params["nitrogen_target_percent"] = current_active_profile_data.nitrogen_target_percent;
    params["primary_uv_exposure_sec"] = current_active_profile_data.primary_uv_exposure_sec;
    params["primary_uv_mode"] = current_active_profile_data.primary_uv_mode;
    params["primary_uv_flicker_rate"] = current_active_profile_data.primary_uv_flicker_rate;
    params["secondary_uv_exposure_sec"] = current_active_profile_data.secondary_uv_exposure_sec;
    params["secondary_uv_mode"] = current_active_profile_data.secondary_uv_mode;
    params["tertiary_uv_exposure_sec"] = current_active_profile_data.tertiary_uv_exposure_sec;
    params["tertiary_uv_mode"] = current_active_profile_data.tertiary_uv_mode;
    params["chamber_cooling_enabled"] = current_active_profile_data.chamber_cooling_enabled;

    // 5. Сериализуем JSON в строку и отправляем
    String output;
    serializeJson(doc, output);
    MySerial1.println(output);

    Serial.println("Full process command sent:");
    Serial.println(output);

    // 6. Устанавливаем флаг, что процесс запущен.
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

    if (current_global_settings.theme == 1) { // --- ТЕМНАЯ ТЕМА ---
        // 1. Фон всего экрана
        lv_obj_set_style_bg_color(screen_keyboard, lv_color_hex(0x1C1C1C), 0);
        lv_obj_set_style_bg_opa(screen_keyboard, LV_OPA_COVER, 0);
        
        // 2. Поле для ввода текста
        lv_obj_add_style(ta_keyboard_proxy, &style_dark_textarea, 0);
        lv_obj_set_style_bg_color(ta_keyboard_proxy, lv_color_hex(0x424242), 0);
        
        // 3. Стилизуем все клавиатуры
        for (auto kb : keyboards) {
            if (!kb) continue;
            // Фон самой клавиатуры (подложка)
            lv_obj_set_style_bg_color(kb, lv_color_hex(0x2C2C2C), LV_PART_MAIN);
            // Стили для кнопок в обычном состоянии
            lv_obj_set_style_bg_color(kb, lv_color_hex(0x424242), LV_PART_ITEMS); 
            lv_obj_set_style_text_color(kb, lv_color_white(), LV_PART_ITEMS);
            
            // <<< --- ВОТ ЭТА НОВАЯ СТРОЧКА --- >>>
            // Стиль для кнопок В МОМЕНТ НАЖАТИЯ (LV_STATE_PRESSED)
            lv_obj_set_style_bg_color(kb, lv_color_hex(0x757575), LV_PART_ITEMS | LV_STATE_PRESSED);
        }

    } else { // --- СВЕТЛАЯ ТЕМА ---
        // 1. Фон всего экрана
        lv_obj_set_style_bg_color(screen_keyboard, lv_color_hex(0x333333), 0);
        lv_obj_set_style_bg_opa(screen_keyboard, LV_OPA_COVER, 0);

        // 2. Поле для ввода текста
        lv_obj_remove_style(ta_keyboard_proxy, &style_dark_textarea, 0);
        lv_obj_add_style(ta_keyboard_proxy, &style_light_textarea, 0);

        // 3. Стилизуем все клавиатуры для светлой темы
        for (auto kb : keyboards) {
            if (!kb) continue;
            // Фон самой клавиатуры (подложка)
            lv_obj_set_style_bg_color(kb, lv_color_hex(0xD3D3D3), LV_PART_MAIN);
            // Стили для кнопок в обычном состоянии
            lv_obj_set_style_bg_color(kb, lv_color_white(), LV_PART_ITEMS);
            lv_obj_set_style_text_color(kb, lv_color_black(), LV_PART_ITEMS);
            
            // <<< --- И ЗДЕСЬ ТОЖЕ СБРАСЫВАЕМ СТИЛЬ НАЖАТИЯ --- >>>
            // Чтобы при переключении с темной темы на светлую не остался светло-серый цвет нажатия
            lv_obj_remove_style(kb, NULL, LV_PART_ITEMS | LV_STATE_PRESSED);
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
    lv_obj_t* btn_matrices[] = { btnm_primary_uv_mode, btnm_secondary_uv_mode, btnm_tertiary_uv_mode };
    
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
        lv_obj_remove_style(label_edit_cooling_title, &style_light_text, 0);
        lv_obj_remove_style(label_edit_thermal_chamber_title, &style_light_text, 0);
        lv_obj_remove_style(label_edit_thermal_temp_title, &style_light_text, 0);
        lv_obj_remove_style(label_edit_heat_hold_title, &style_light_text, 0);
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
        lv_obj_add_style(label_edit_cooling_title, &style_dark_text, 0);
        lv_obj_add_style(label_edit_thermal_chamber_title, &style_dark_text, 0);
        lv_obj_add_style(label_edit_thermal_temp_title, &style_dark_text, 0);
        lv_obj_add_style(label_edit_heat_hold_title, &style_dark_text, 0);
        lv_obj_add_style(btn_save_changes, &style_dark_btn, 0);
        lv_obj_add_style(btn_cancel_edit, &style_dark_btn, 0);
        
        lv_obj_add_style(ta_edit_primary_uv, &style_dark_textarea, 0);
        lv_obj_add_style(ta_edit_flicker_rate, &style_dark_textarea, 0);
        lv_obj_add_style(ta_edit_secondary_uv, &style_dark_textarea, 0);
        lv_obj_add_style(ta_edit_tertiary_uv, &style_dark_textarea, 0);
        lv_obj_add_style(ta_edit_nitrogen_target, &style_dark_textarea, 0);
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

        for (auto btnm : btn_matrices) {
            if (!btnm) continue;
            // Неактивные кнопки
            lv_obj_set_style_bg_color(btnm, lv_color_hex(0x424242), LV_PART_ITEMS);
            lv_obj_set_style_text_color(btnm, lv_color_white(), LV_PART_ITEMS);
            // Активная кнопка
            lv_obj_set_style_bg_color(btnm, lv_color_hex(0xff05b8), LV_PART_ITEMS | LV_STATE_CHECKED);
            lv_obj_set_style_text_color(btnm, lv_color_white(), LV_PART_ITEMS | LV_STATE_CHECKED);
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
        lv_obj_remove_style(label_edit_cooling_title, &style_dark_text, 0);
        lv_obj_remove_style(label_edit_thermal_chamber_title, &style_dark_text, 0);
        lv_obj_remove_style(label_edit_thermal_temp_title, &style_dark_text, 0);
        lv_obj_remove_style(label_edit_heat_hold_title, &style_dark_text, 0);
        lv_obj_remove_style(btn_save_changes, &style_dark_btn, 0);
        lv_obj_remove_style(btn_cancel_edit, &style_dark_btn, 0);

        lv_obj_remove_style(ta_edit_primary_uv, &style_dark_textarea, 0);
        lv_obj_remove_style(ta_edit_flicker_rate, &style_dark_textarea, 0);
        lv_obj_remove_style(ta_edit_secondary_uv, &style_dark_textarea, 0);
        lv_obj_remove_style(ta_edit_tertiary_uv, &style_dark_textarea, 0);
        lv_obj_remove_style(ta_edit_nitrogen_target, &style_dark_textarea, 0);
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
        lv_obj_add_style(label_edit_cooling_title, &style_light_text, 0);
        lv_obj_add_style(label_edit_thermal_chamber_title, &style_light_text, 0);
        lv_obj_add_style(label_edit_thermal_temp_title, &style_light_text, 0);
        lv_obj_add_style(label_edit_heat_hold_title, &style_light_text, 0);
        lv_obj_add_style(btn_save_changes, &style_light_btn, 0);
        lv_obj_add_style(btn_cancel_edit, &style_light_btn, 0);

        lv_obj_add_style(ta_edit_primary_uv, &style_light_textarea, 0);
        lv_obj_add_style(ta_edit_flicker_rate, &style_light_textarea, 0);
        lv_obj_add_style(ta_edit_secondary_uv, &style_light_textarea, 0);
        lv_obj_add_style(ta_edit_tertiary_uv, &style_light_textarea, 0);
        lv_obj_add_style(ta_edit_nitrogen_target, &style_light_textarea, 0);
        lv_obj_add_style(ta_edit_thermal_temp, &style_light_textarea, 0);
        lv_obj_add_style(ta_edit_heat_hold, &style_light_textarea, 0);

        // <<< Устанавливаем СВЕТЛЫЙ фон для строк с переключателями >>>
        lv_obj_set_style_bg_color(row_uv_primary, lv_color_hex(0xE0E0E0), 0);
        lv_obj_set_style_bg_opa(row_uv_primary, LV_OPA_COVER, 0);
        lv_obj_set_style_bg_color(row_uv_secondary, lv_color_hex(0xE0E0E0), 0);
        lv_obj_set_style_bg_opa(row_uv_secondary, LV_OPA_COVER, 0);
        lv_obj_set_style_bg_color(row_uv_tertiary, lv_color_hex(0xE0E0E0), 0);
        lv_obj_set_style_bg_opa(row_uv_tertiary, LV_OPA_COVER, 0);

        for (auto btnm : btn_matrices) {
            if (!btnm) continue;
            // Неактивные кнопки
            lv_obj_set_style_bg_color(btnm, lv_color_hex(0xE0E0E0), LV_PART_ITEMS);
            lv_obj_set_style_text_color(btnm, lv_color_black(), LV_PART_ITEMS);
            // Активная кнопка
            lv_obj_set_style_bg_color(btnm, lv_palette_main(LV_PALETTE_BLUE), LV_PART_ITEMS | LV_STATE_CHECKED);
            lv_obj_set_style_text_color(btnm, lv_color_white(), LV_PART_ITEMS | LV_STATE_CHECKED);
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

    if (current_global_settings.theme == 1) { // --- ТЕМНАЯ ТЕМА ---
        // Удаляем стили светлой темы
        lv_obj_remove_style(main_screen_content_container, &style_light_bg, 0);
        lv_obj_remove_style(list_header_label_main, &style_light_text, 0);
        lv_obj_remove_style(lv_obj_get_child(btn_profiles_prev, 0), &style_light_arrow, 0);
        lv_obj_remove_style(lv_obj_get_child(btn_profiles_next, 0), &style_light_arrow, 0);
        lv_obj_remove_style(btn_add_main, &style_light_btn, 0);
        lv_obj_remove_style(btn_lab_main, &style_light_btn, 0);
        lv_obj_remove_style(btn_settings_main, &style_light_btn, 0);
        
        // Добавляем стили темной темы
        lv_obj_add_style(main_screen_content_container, &style_dark_bg, 0);
        lv_obj_add_style(list_header_label_main, &style_dark_text, 0);
        lv_obj_add_style(lv_obj_get_child(btn_profiles_prev, 0), &style_dark_arrow, 0);
        lv_obj_add_style(lv_obj_get_child(btn_profiles_next, 0), &style_dark_arrow, 0);
        lv_obj_add_style(btn_add_main, &style_dark_btn, 0);
        lv_obj_add_style(btn_lab_main, &style_dark_btn, 0);
        lv_obj_add_style(btn_settings_main, &style_dark_btn, 0);

    } else { // --- СВЕТЛАЯ ТЕМА ---
        // Удаляем стили темной темы
        lv_obj_remove_style(main_screen_content_container, &style_dark_bg, 0);
        lv_obj_remove_style(list_header_label_main, &style_dark_text, 0);
        lv_obj_remove_style(lv_obj_get_child(btn_profiles_prev, 0), &style_dark_arrow, 0);
        lv_obj_remove_style(lv_obj_get_child(btn_profiles_next, 0), &style_dark_arrow, 0);
        lv_obj_remove_style(btn_add_main, &style_dark_btn, 0);
        lv_obj_remove_style(btn_lab_main, &style_dark_btn, 0);
        lv_obj_remove_style(btn_settings_main, &style_dark_btn, 0);
        
        // Добавляем стили светлой темы
        lv_obj_add_style(main_screen_content_container, &style_light_bg, 0);
        lv_obj_add_style(list_header_label_main, &style_light_text, 0);
        lv_obj_add_style(lv_obj_get_child(btn_profiles_prev, 0), &style_light_arrow, 0);
        lv_obj_add_style(lv_obj_get_child(btn_profiles_next, 0), &style_light_arrow, 0);
        lv_obj_add_style(btn_add_main, &style_light_btn, 0);
        lv_obj_add_style(btn_lab_main, &style_light_btn, 0);
        lv_obj_add_style(btn_settings_main, &style_light_btn, 0);
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
        lv_obj_add_style(btn_glaze_start, &style_dark_btn, 0);
        lv_obj_add_style(btn_glaze_back, &style_dark_btn, 0);
        
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
        lv_obj_remove_style(btn_glaze_start, &style_dark_btn, 0);
        lv_obj_remove_style(btn_glaze_back, &style_dark_btn, 0);

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
        lv_obj_add_style(btn_glaze_start, &style_light_btn, 0);
        lv_obj_add_style(btn_glaze_back, &style_light_btn, 0);
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
        lv_obj_remove_style(label_process_status_title, &style_light_text, 0);
        lv_obj_remove_style(label_process_status_detail, &style_light_text, 0);
        lv_obj_remove_style(spinner_process_execution, &style_light_spinner_bg, LV_PART_MAIN);
        lv_obj_remove_style(spinner_process_execution, &style_light_spinner_indic, LV_PART_INDICATOR);
        lv_obj_remove_style(btn_process_cancel, &style_light_btn, 0);

        // Добавляем стили темной темы
        lv_obj_add_style(content_container, &style_dark_bg, 0);
        lv_obj_set_style_radius(content_container, 0, 0);
        lv_obj_add_style(label_process_status_title, &style_dark_text, 0);
        lv_obj_add_style(label_process_status_detail, &style_dark_text, 0);
        lv_obj_add_style(spinner_process_execution, &style_dark_spinner_bg, LV_PART_MAIN);
        lv_obj_add_style(spinner_process_execution, &style_dark_spinner_indic, LV_PART_INDICATOR);
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
        lv_obj_remove_style(label_process_status_title, &style_dark_text, 0);
        lv_obj_remove_style(label_process_status_detail, &style_dark_text, 0);
        lv_obj_remove_style(spinner_process_execution, &style_dark_spinner_bg, LV_PART_MAIN);
        lv_obj_remove_style(spinner_process_execution, &style_dark_spinner_indic, LV_PART_INDICATOR);
        lv_obj_remove_style(btn_process_cancel, &style_dark_btn, 0);

        // Добавляем стили светлой темы
        lv_obj_add_style(content_container, &style_light_bg, 0);
        lv_obj_add_style(label_process_status_title, &style_light_text, 0);
        lv_obj_add_style(label_process_status_detail, &style_light_text, 0);
        lv_obj_add_style(spinner_process_execution, &style_light_spinner_bg, LV_PART_MAIN);
        lv_obj_add_style(spinner_process_execution, &style_light_spinner_indic, LV_PART_INDICATOR);
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

    // --- Создаем стиль для зеленой кнопки ---
    static lv_style_t style_kb_button_ok;
    lv_style_init(&style_kb_button_ok);
    lv_style_set_bg_color(&style_kb_button_ok, lv_palette_main(LV_PALETTE_GREEN));

    // --- Универсальная функция для стилизации кнопки "OK" ---
    auto style_ok_button = [&](lv_obj_t* kb) {
        lv_obj_t* btnm = lv_obj_get_child(kb, 0);
        if (!btnm) return;

        const char** map = lv_btnmatrix_get_map(btnm);
        if (!map) return;

        // Итерируем по карте кнопок, пока не встретим пустую строку
        for (uint16_t i = 0; strcmp(map[i], "") != 0; i++) {
            if (strcmp(map[i], LV_SYMBOL_OK) == 0) {
                // Нашли! Применяем стиль к кнопке с этим индексом (ID)
                lv_obj_add_style(btnm, &style_kb_button_ok, LV_PART_ITEMS | i);
                break; // Выходим из цикла, так как кнопка найдена
            }
        }
    };

    // --- Создаем и настраиваем все три клавиатуры ---

    // 1. Алфавитная клавиатура
    kb_edit_alpha = lv_keyboard_create(screen_keyboard);
    lv_obj_add_flag(kb_edit_alpha, LV_OBJ_FLAG_HIDDEN);
    lv_keyboard_set_textarea(kb_edit_alpha, ta_keyboard_proxy);
    lv_obj_add_event_cb(kb_edit_alpha, modal_input_keyboard_event_cb, LV_EVENT_ALL, NULL);
    lv_obj_set_height(kb_edit_alpha, lv_pct(70));
    style_ok_button(kb_edit_alpha);

    // 2. Цифровая клавиатура
    kb_edit_numeric = lv_keyboard_create(screen_keyboard);
    lv_obj_add_flag(kb_edit_numeric, LV_OBJ_FLAG_HIDDEN);
    lv_keyboard_set_textarea(kb_edit_numeric, ta_keyboard_proxy);
    lv_obj_add_event_cb(kb_edit_numeric, modal_input_keyboard_event_cb, LV_EVENT_ALL, NULL);
    lv_keyboard_set_mode(kb_edit_numeric, LV_KEYBOARD_MODE_NUMBER);
    lv_obj_set_height(kb_edit_numeric, lv_pct(70));
    style_ok_button(kb_edit_numeric);

    // 3. Сервисная клавиатура
    kb_service_code = lv_keyboard_create(screen_keyboard);
    lv_obj_add_flag(kb_service_code, LV_OBJ_FLAG_HIDDEN);
    lv_keyboard_set_textarea(kb_service_code, ta_keyboard_proxy);
    lv_obj_add_event_cb(kb_service_code, service_code_keyboard_event_cb, LV_EVENT_ALL, NULL);
    lv_keyboard_set_mode(kb_service_code, LV_KEYBOARD_MODE_TEXT_UPPER);
    lv_obj_set_height(kb_service_code, lv_pct(70));
    style_ok_button(kb_service_code);
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
        if (screen_laboratory) lv_scr_load(screen_laboratory);
    } else if (strcmp(user_data, "start") == 0) {
        // Передаем управление универсальному обработчику с ID=1
        lv_event_send(lv_event_get_target(e), LV_EVENT_CLICKED, (void*)1);
    }
}

static void lab_screen_back_event_cb(lv_event_t * e) {
    if (lv_event_get_code(e) == LV_EVENT_CLICKED) {
        Serial.println("Back from Lab Mode to Main Menu.");
        if (screen_main_app) {
            lv_scr_load(screen_main_app);
        }
    }
}

static void lab_mode_start_event_cb(lv_event_t* e) {
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
    int mode_id = (intptr_t)lv_event_get_user_data(e);
    if (mode_id == 0) return;

    Serial.printf("--- START Lab Mode (ID: %d). Applying SAFE start sequence v3.1 ---\n", mode_id);
    is_lab_mode_running = true;
    current_process_stage = "";

    // ФАЗА 1: Немедленная смена UI
    Serial.println("Step 1: Switching to Process Screen...");
    
    // --- ИСПРАВЛЕНИЕ: Объявляем переменные здесь ---
    const char* process_title_rus = "Лабораторный процесс";
    const char* process_title_eng = "Laboratory Process";
    lv_obj_t* screen_to_return = screen_laboratory;

    switch(mode_id) {
        case 1: process_title_rus = "Запуск: Глазурь"; process_title_eng = "Starting: Glaze"; screen_to_return = screen_lab_glaze; break;
        case 2: process_title_rus = "Запуск: Ремонт"; process_title_eng = "Starting: Repair"; screen_to_return = screen_lab_repair; break;
        case 3: process_title_rus = "Запуск: Повышение прочности"; process_title_eng = "Starting: Strength Boost"; screen_to_return = screen_lab_strength; break;
        case 4: process_title_rus = "Запуск: Термокамера"; process_title_eng = "Starting: Thermal Chamber"; screen_to_return = screen_lab_thermal; break;
        case 5: process_title_rus = "Запуск: Осветление"; process_title_eng = "Starting: Lightening"; screen_to_return = screen_lab_lighten; break;
        case 6: process_title_rus = "Запуск: Затемнение"; process_title_eng = "Starting: Darkening"; screen_to_return = screen_lab_darken; break;
    }
    screen_to_return_after_process = screen_to_return;

    // --- ИСПРАВЛЕНИЕ: Выбираем правильный заголовок ПЕРЕД использованием ---
    const char* title = (current_global_settings.language == 1) ? process_title_rus : process_title_eng;

    // Настраиваем и немедленно загружаем экран процесса
    lvgl_port_lock(-1);
    lv_label_set_text(label_process_status_title, title);
    lv_label_set_text(label_process_status_detail, "Sending command to controller...");
    lv_obj_clear_flag(btn_process_cancel, LV_OBJ_FLAG_HIDDEN);

    if (current_global_settings.language == 1) { // RUS
        lv_label_set_text(label_btn_process_cancel, "Отмена");
    } else { // ENG
        lv_label_set_text(label_btn_process_cancel, "Cancel");
    }

    apply_theme_to_process_screen();
    
    if(spinner_process_execution) lv_obj_clear_flag(spinner_process_execution, LV_OBJ_FLAG_HIDDEN);
    lv_scr_load(screen_process_execution);
    lvgl_port_unlock();

    delay(20);

    // ФАЗА 2: Работа с данными
    Serial.println("Step 2: Preparing and sending command...");
    
    StaticJsonDocument<512> doc;
    JsonObject params = doc.createNestedObject("params");
    doc["command"] = "START_LAB_PROCESS";
    doc["mode_id"] = mode_id;

    // Наполняем JSON из уже актуальной глобальной структуры
    switch(mode_id) {
        case 1:
            params["uv_on_sec"] = current_lab_settings.glaze.uv_on_sec;
            params["uv_off_sec"] = current_lab_settings.glaze.uv_off_sec;
            params["monomer_blow_min"] = current_lab_settings.glaze.monomer_blow_min;
            params["uv_exposure_sec"] = current_lab_settings.glaze.uv_exposure_sec;
            params["use_cooling"] = current_lab_settings.glaze.use_cooling;
            params["use_nitrogen"] = current_lab_settings.glaze.use_nitrogen;
            params["nitrogen_target_percent"] = current_lab_settings.glaze.nitrogen_target_percent;
            break;
        case 2:
            params["countdown_sec"] = current_lab_settings.repair.countdown_sec;
            params["uv_exposure_sec"] = current_lab_settings.repair.uv_exposure_sec;
            break;
        case 3:
            params["chamber_temp_c"] = current_lab_settings.strength.chamber_temp_c;
            params["hold_time_min"] = current_lab_settings.strength.hold_time_min;
            params["use_cooling"] = current_lab_settings.strength.use_cooling;
            params["uv_pulse_duration_sec"] = current_lab_settings.strength.uv_pulse_duration_sec;
            params["uv_pulse_interval_min"] = current_lab_settings.strength.uv_pulse_interval_min;
            break;
        case 4:
            params["chamber_temp_c"] = current_lab_settings.thermal.chamber_temp_c;
            params["hold_time_min"] = current_lab_settings.thermal.hold_time_min;
            break;
        case 5:
            params["chamber_temp_c"] = current_lab_settings.lighten.chamber_temp_c;
            params["hold_time_min"] = current_lab_settings.lighten.hold_time_min;
            params["use_cooling"] = current_lab_settings.lighten.use_cooling;
            break;
        case 6:
            params["darken_uv_exposure_min"] = current_lab_settings.darken.uv_exposure_min;
            params["use_cooling"] = current_lab_settings.darken.use_cooling;
            break;
    }
    
    String output;
    serializeJson(doc, output);
    flush_serial_buffer();
    MySerial1.println(output);
    Serial.println("Lab process command sent:");
    Serial.println(output);

    // ФАЗА 3: Смена логического состояния
    main_process_running = true;
    process_start_time_ms = millis();
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
                if (lv_obj_has_state(sw_glaze_nitrogen, LV_STATE_CHECKED)) {
                    lv_obj_set_style_opa(nitrogen_glaze_container, LV_OPA_COVER, 0);
                    lv_obj_clear_flag(nitrogen_glaze_container, LV_OBJ_FLAG_CLICKABLE);
                } else {
                    lv_obj_set_style_opa(nitrogen_glaze_container, LV_OPA_TRANSP, 0);
                    lv_obj_add_flag(nitrogen_glaze_container, LV_OBJ_FLAG_CLICKABLE);
                }

                // 2. Переводим все надписи
                if (current_global_settings.language == 1) { // RUS
                    lv_label_set_text(label_glaze_header, "Режим: Глазурь");
                    lv_label_set_text(lv_obj_get_child(btn_glaze_help, 0), "Справка по разделу ?");
                    lv_label_set_text(label_glaze_title_flicker, "Настройка такта мерцания");
                    lv_label_set_text(label_glaze_uv_on, "Вкл. диода (0.1-3с)");
                    lv_label_set_text(label_glaze_uv_off, "Выкл. диода (0.1-3с)");
                    lv_label_set_text(label_glaze_title_timers, "Настройка таймеров");
                    lv_label_set_text(label_glaze_monomer_blow, "Сушка (1-10мин)");
                    lv_label_set_text(label_glaze_uv_exposure, "Работа УФ (10-200с)");
                    lv_label_set_text(label_glaze_title_aux, "Вспомогательные параметры");
                    lv_label_set_text(label_glaze_cooling, "Быстрое охлаждение");
                    lv_label_set_text(label_glaze_nitrogen, "Использование азота");
                    lv_label_set_text(label_glaze_nitrogen_target, "Цель N2 (%)");
                    lv_label_set_text(label_btn_glaze_start, "Старт");
                    lv_label_set_text(label_btn_glaze_back, "Назад");
                } else { // ENG
                    lv_label_set_text(label_glaze_header, "Mode: Glaze");
                    lv_label_set_text(lv_obj_get_child(btn_glaze_help, 0), "Section Help ?");
                    lv_label_set_text(label_glaze_title_flicker, "Flicker Takt Setup");
                    lv_label_set_text(label_glaze_uv_on, "Diode ON (0.1-3s)");
                    lv_label_set_text(label_glaze_uv_off, "Diode OFF (0.1-3s)");
                    lv_label_set_text(label_glaze_title_timers, "Timers Setup");
                    lv_label_set_text(label_glaze_monomer_blow, "Blow (1-10min)");
                    lv_label_set_text(label_glaze_uv_exposure, "UV Work (10-200s)");
                    lv_label_set_text(label_glaze_title_aux, "Auxiliary Parameters");
                    lv_label_set_text(label_glaze_cooling, "Fast Cooling");
                    lv_label_set_text(label_glaze_nitrogen, "Use Nitrogen");
                    lv_label_set_text(label_glaze_nitrogen_target, "N2 Target (%)");
                    lv_label_set_text(label_btn_glaze_start, "Start");
                    lv_label_set_text(label_btn_glaze_back, "Back");
                }

                help_blink_timer = lv_timer_create(help_blink_timer_cb, 1, btn_glaze_help);

                apply_theme_to_lab_glaze_screen();

                lv_scr_load(screen_lab_glaze);
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

                lv_scr_load(screen_lab_repair);
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
                lv_scr_load(screen_lab_strength);
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
                lv_scr_load(screen_lab_thermal);
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
                lv_scr_load(screen_lab_lighten);
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
                lv_scr_load(screen_lab_darken);
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
    lv_obj_t* btnm = lv_event_get_target(e);
    lv_obj_t* label = (lv_obj_t*)lv_event_get_user_data(e);
    uint32_t id = lv_btnmatrix_get_selected_btn(btnm);

    // Просто вызываем нашу универсальную функцию для обновления UI
    update_uv_mode_selector_ui(btnm, label, id);
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
            lv_scr_load(screen_help);
        }
    } else if (strcmp(user_data, "close_help") == 0) {
        Serial.println("Closing help screen.");
        if (screen_profile_edit) {
            lv_scr_load(screen_profile_edit);
        }
    }
}
static void list_btn_delete_event_cb(lv_event_t * e) {
    char* data_to_free = (char*)lv_event_get_user_data(e);
    if (data_to_free) { Serial.printf("Freeing user_data for list button: %s\n", data_to_free); free(data_to_free); }
}

void displayProfileListPage() {
    if (!list_profiles_main) {
        Serial.println("displayProfileListPage: list_profiles_main is NULL!");
        return;
    }
    lv_obj_clean(list_profiles_main);
    char page_info_buffer[64];

    apply_theme_to_main_app_screen();

    // ==========================================================
    // <<<--- НАЧАЛО ИЗМЕНЕНИЙ (САМЫЙ ПРОСТОЙ СПОСОБ) ---<<<
    // ==========================================================
    if (all_profile_entries_cache.empty()) {
        lv_obj_t* label_empty = lv_label_create(list_profiles_main);
        lv_label_set_text(label_empty, "No saved profiles.");
        lv_obj_center(label_empty);
        if(list_header_label_main) {
            // Просто выбираем одну из двух строк в зависимости от языка
            if (current_global_settings.language == 1) { // 1 = RUS
                lv_label_set_text(list_header_label_main, "Главное Меню (0/0)");
            } else { // 0 = ENG (или любое другое значение)
                lv_label_set_text(list_header_label_main, "Main Menu (0/0)");
            }
        }
        if(btn_profiles_prev) lv_obj_add_state(btn_profiles_prev, LV_STATE_DISABLED);
        if(btn_profiles_next) lv_obj_add_state(btn_profiles_next, LV_STATE_DISABLED);
        return;
    }

    if (current_profile_list_page >= total_profile_pages && total_profile_pages > 0) { current_profile_list_page = total_profile_pages - 1; }
    else if (total_profile_pages == 0) { current_profile_list_page = 0; }
    if (current_profile_list_page < 0) { current_profile_list_page = 0; }

    int start_index = current_profile_list_page * PROFILES_PER_PAGE;
    int end_index = start_index + PROFILES_PER_PAGE;
    if (end_index > all_profile_entries_cache.size()) { end_index = all_profile_entries_cache.size(); }

    for (int i = start_index; i < end_index; ++i) {
        const ProfileCacheEntry& entry = all_profile_entries_cache[i];
        
        lv_obj_t* tile = lv_obj_create(list_profiles_main);
        lv_obj_set_size(tile, 110, 175);
        lv_obj_set_style_border_width(tile, 1, 0);
        lv_obj_set_style_radius(tile, 5, 0);
        lv_obj_set_style_pad_all(tile, 5, 0);
        lv_obj_set_style_pad_gap(tile, 5, 0);
        lv_obj_set_layout(tile, LV_LAYOUT_FLEX);
        lv_obj_set_flex_flow(tile, LV_FLEX_FLOW_COLUMN);
        lv_obj_set_flex_align(tile, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

        lv_obj_t* icon = lv_label_create(tile);
        lv_label_set_text(icon, LV_SYMBOL_FILE);
        lv_obj_set_style_text_font(icon, &lv_font_montserrat_24, 0);

        lv_obj_t* name_label = lv_label_create(tile);
        lv_label_set_text(name_label, entry.display_name);
        lv_obj_set_width(name_label, lv_pct(100));
        lv_label_set_long_mode(name_label, LV_LABEL_LONG_DOT);
        lv_obj_set_style_text_align(name_label, LV_TEXT_ALIGN_CENTER, 0);
        
        lv_obj_add_flag(tile, LV_OBJ_FLAG_CLICKABLE);

        // <<< ПРИМЕНЯЕМ ТЕМУ ДЛЯ ПЛИТОК >>>
        if (current_global_settings.theme == 1) { // Темная тема
            lv_obj_add_style(tile, &style_dark_tile_bg, 0);
            lv_obj_add_style(tile, &style_dark_block_border, 0);
            lv_obj_add_style(icon, &style_dark_text, 0);
            lv_obj_add_style(name_label, &style_dark_text, 0);
        } else { // Светлая тема
            lv_obj_add_style(tile, &style_light_tile_bg, 0);
            lv_obj_add_style(tile, &style_light_tile_border, 0);
            lv_obj_add_style(icon, &style_light_text, 0);
            lv_obj_add_style(name_label, &style_light_text, 0);
        }

        char* filename_data = strdup(entry.filename);
        if(filename_data) {
             lv_obj_add_event_cb(tile, profile_list_event_handler, LV_EVENT_CLICKED, filename_data);
             lv_obj_add_event_cb(tile, list_btn_delete_event_cb, LV_EVENT_DELETE, filename_data);
        }
    }

    // Обновляем заголовок и состояние кнопок
    if (list_header_label_main) {
        // Заранее выбираем нужный текст для заголовка и слова "Страница"
        const char* title_text;
        const char* page_text;

        if (current_global_settings.language == 1) { // 1 = RUS
            title_text = "Главное Меню";
            page_text = "Стр.";
            // --- ДОБАВЛЕННЫЙ БЛОК ПЕРЕВОДА КНОПОК ---
            lv_label_set_text(label_btn_add_main, LV_SYMBOL_PLUS " Добавить Профиль");
            lv_label_set_text(label_btn_lab_main, LV_SYMBOL_SETTINGS " Лаб. режим");
            lv_label_set_text(label_btn_settings_main, "Настройки");
            lv_obj_add_style(label_btn_add_main, &style_my_text_18_white, 0);
            lv_obj_add_style(label_btn_lab_main, &style_my_text_18_white, 0);  
            lv_obj_add_style(label_btn_settings_main, &style_my_text_18_white, 0); 
            // ------------------------------------------
        } else { // 0 = ENG (или любое другое значение)
            title_text = "Main Menu";
            page_text = "Page";
            // --- ДОБАВЛЕННЫЙ БЛОК ПЕРЕВОДА КНОПОК ---
            lv_label_set_text(label_btn_add_main, LV_SYMBOL_PLUS "Add Profile");
            lv_label_set_text(label_btn_lab_main, LV_SYMBOL_SETTINGS "Lab Mode");
            lv_label_set_text(label_btn_settings_main, "Settings");
            // Для английского можно не менять стиль, т.к. стандартный шрифт его поддерживает,
            // но для единообразия лучше тоже его указать.
            lv_obj_add_style(label_btn_add_main, &style_my_text_18_white, 0); 
            lv_obj_add_style(label_btn_lab_main, &style_my_text_18_white, 0);  
            lv_obj_add_style(label_btn_settings_main, &style_my_text_18_white, 0); 
            // ------------------------------------------
        }

        // Формируем финальную строку с уже выбранным текстом
        snprintf(page_info_buffer, sizeof(page_info_buffer), "%s (%s %d/%d)",
            title_text,
            page_text,
            current_profile_list_page + 1,
            total_profile_pages > 0 ? total_profile_pages : 1);
        lv_label_set_text(list_header_label_main, page_info_buffer);
    }
    // ==========================================================
    // <<<--- КОНЕЦ ИЗМЕНЕНИЙ ---<<<
    // ==========================================================
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

        // Перерисовываем страницу
        lvgl_port_lock(-1);
        displayProfileListPage();
        lvgl_port_unlock();
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
        
        // Перерисовываем страницу
        lvgl_port_lock(-1);
        displayProfileListPage();
        lvgl_port_unlock();
    }
}

void prepare_and_send_lab_command(int mode_id) {
    Serial.printf("Preparing and sending command for Lab Mode ID: %d\n", mode_id);
    
    StaticJsonDocument<512> doc;
    JsonObject params = doc.createNestedObject("params");
    doc["command"] = "START_LAB_PROCESS";
    doc["mode_id"] = mode_id;

    // 1. Сбор данных из UI и обновление глобальной структуры
    switch(mode_id) {
        case 1: { // Глазурь
            current_lab_settings.glaze.uv_on_sec = atof(lv_textarea_get_text(ta_glaze_uv_on));
            current_lab_settings.glaze.uv_off_sec = atof(lv_textarea_get_text(ta_glaze_uv_off));
            current_lab_settings.glaze.monomer_blow_min = atoi(lv_textarea_get_text(ta_glaze_monomer_blow));
            current_lab_settings.glaze.uv_exposure_sec = atoi(lv_textarea_get_text(ta_glaze_uv_exposure));
            current_lab_settings.glaze.use_cooling = lv_obj_has_state(sw_glaze_cooling, LV_STATE_CHECKED);
            current_lab_settings.glaze.use_nitrogen = lv_obj_has_state(sw_glaze_nitrogen, LV_STATE_CHECKED);
            current_lab_settings.glaze.nitrogen_target_percent = atoi(lv_textarea_get_text(ta_glaze_nitrogen_target));

            params["uv_on_sec"] = current_lab_settings.glaze.uv_on_sec;
            params["uv_off_sec"] = current_lab_settings.glaze.uv_off_sec;
            params["monomer_blow_min"] = current_lab_settings.glaze.monomer_blow_min;
            params["uv_exposure_sec"] = current_lab_settings.glaze.uv_exposure_sec;
            params["use_cooling"] = current_lab_settings.glaze.use_cooling;
            params["use_nitrogen"] = current_lab_settings.glaze.use_nitrogen;
            params["nitrogen_target_percent"] = current_lab_settings.glaze.nitrogen_target_percent;
            break;
        }
        case 2: { // Ремонт
            current_lab_settings.repair.countdown_sec = atoi(lv_textarea_get_text(ta_repair_countdown));
            current_lab_settings.repair.uv_exposure_sec = atoi(lv_textarea_get_text(ta_repair_uv_exposure));

            params["countdown_sec"] = current_lab_settings.repair.countdown_sec;
            params["uv_exposure_sec"] = current_lab_settings.repair.uv_exposure_sec;
            break;
        }
        case 3: { // Прочность
            current_lab_settings.strength.chamber_temp_c = atoi(lv_textarea_get_text(ta_strength_temp));
            current_lab_settings.strength.hold_time_min = atoi(lv_textarea_get_text(ta_strength_hold_time));
            current_lab_settings.strength.use_cooling = lv_obj_has_state(sw_strength_cooling, LV_STATE_CHECKED);
            current_lab_settings.strength.uv_pulse_duration_sec = atoi(lv_textarea_get_text(ta_strength_uv_pulse_duration));
            current_lab_settings.strength.uv_pulse_interval_min = atoi(lv_textarea_get_text(ta_strength_uv_pulse_interval));

            params["chamber_temp_c"] = current_lab_settings.strength.chamber_temp_c;
            params["hold_time_min"] = current_lab_settings.strength.hold_time_min;
            params["use_cooling"] = current_lab_settings.strength.use_cooling;
            params["uv_pulse_duration_sec"] = current_lab_settings.strength.uv_pulse_duration_sec;
            params["uv_pulse_interval_min"] = current_lab_settings.strength.uv_pulse_interval_min;
            break;
        }
        case 4: { // Термокамера
            current_lab_settings.thermal.chamber_temp_c = atoi(lv_textarea_get_text(ta_thermal_temp));
            current_lab_settings.thermal.hold_time_min = atoi(lv_textarea_get_text(ta_thermal_hold_time));

            params["chamber_temp_c"] = current_lab_settings.thermal.chamber_temp_c;
            params["hold_time_min"] = current_lab_settings.thermal.hold_time_min;
            break;
        }
        case 5: { // Осветление
            current_lab_settings.lighten.hold_time_min = atoi(lv_textarea_get_text(ta_lighten_hold_time));
            current_lab_settings.lighten.use_cooling = lv_obj_has_state(sw_lighten_cooling, LV_STATE_CHECKED);
            
            params["chamber_temp_c"] = current_lab_settings.lighten.chamber_temp_c;
            params["hold_time_min"] = current_lab_settings.lighten.hold_time_min;
            params["use_cooling"] = current_lab_settings.lighten.use_cooling;
            break;
        }
        case 6: { // Затемнение
            current_lab_settings.darken.uv_exposure_min = atoi(lv_textarea_get_text(ta_darken_uv_exposure));
            current_lab_settings.darken.use_cooling = lv_obj_has_state(sw_darken_cooling, LV_STATE_CHECKED);

            params["darken_uv_exposure_min"] = current_lab_settings.darken.uv_exposure_min;
            params["use_cooling"] = current_lab_settings.darken.use_cooling;
            break;
        }
    }

    // 2. Сохраняем обновленные настройки в файл
    saveLaboratorySettings();

    // 3. Сериализуем и отправляем команду
    String output;
    serializeJson(doc, output);
    MySerial1.println(output);
    Serial.println("Lab process command sent:");
    Serial.println(output);
    
}

// Меняем void на bool
bool handle_save_new_profile_logic(const char* profile_input_name) {
    // Проверки
    if (strlen(profile_input_name) == 0) {
        if (current_global_settings.language == 1) { // RUS
            show_info_dialog("Ошибка", "Имя профиля не может быть пустым!");
        } else { // ENG
            show_info_dialog("Error", "Profile name cannot be empty!");
        }
        return false;
    }
    if (strlen(profile_input_name) >= sizeof(ProfileData::name)) {
        if (current_global_settings.language == 1) { // RUS
            show_info_dialog("Ошибка", "Имя профиля слишком длинное!");
        } else { // ENG
            show_info_dialog("Error", "Profile name is too long!");
        }
        return false;
    }
    if (!sd_card_initialized) {
        if (current_global_settings.language == 1) { // RUS
            show_info_dialog("Ошибка", "SD-карта не готова!");
        } else { // ENG
            show_info_dialog("Error", "SD Card not ready!");
        }
        return false;
    }
    lvgl_port_lock(-1);

    // ... (вся твоя логика создания и сохранения профиля без изменений) ...
    ProfileData new_profile_defaults;
    new_profile_defaults.id = current_profile_next_id;
    strncpy(new_profile_defaults.name, profile_input_name, sizeof(new_profile_defaults.name) - 1);
    new_profile_defaults.name[sizeof(new_profile_defaults.name) - 1] = '\0';
    new_profile_defaults.thermal_chamber_enabled = false;
    new_profile_defaults.heat_exchange_hold_sec = 60;
    new_profile_defaults.thermal_chamber_temp = 40;
    new_profile_defaults.nitrogen_use_enabled = false;
    new_profile_defaults.nitrogen_target_percent = 95;
    new_profile_defaults.primary_uv_exposure_sec = 30;
    new_profile_defaults.secondary_uv_exposure_sec = 60;
    new_profile_defaults.chamber_cooling_enabled = false;

    // <<<--- ИЗМЕНЕНО: Установка значений по умолчанию для селекторов ---<<<
    new_profile_defaults.primary_uv_mode = 1; 
    new_profile_defaults.primary_uv_flicker_rate = 5;

    new_profile_defaults.secondary_uv_mode = 1; 

    new_profile_defaults.tertiary_uv_exposure_sec = 60;
    new_profile_defaults.tertiary_uv_mode = 1; 

    String filename_on_sd = "/profile_" + String(new_profile_defaults.id) + ".txt";
    char json_buffer_loc[FILE_CONTENT_BUFFER_SIZE];

    if (!serializeProfileJson(new_profile_defaults, json_buffer_loc, sizeof(json_buffer_loc))) {
        show_info_dialog("Error", "JSON serialization failed!");
        return false; // Возвращаем неудачу
    }

    writeFile(SD, filename_on_sd.c_str(), json_buffer_loc);
    
    // Проверяем, что файл реально создался
    File checkFile = SD.open(filename_on_sd.c_str());
    if (!checkFile) {
        show_info_dialog("Error", "Failed to write profile to SD card.");
        return false;
    }
    checkFile.close();
    
    // Обновляем кэш
    current_profile_next_id = scanAndCacheAllProfiles(SD, all_profile_entries_cache);
    
    // <<<--- ГЛАВНОЕ ИЗМЕНЕНИЕ ---<<<
    needs_list_refresh = true; // Просто взводим флаг
    
    lvgl_port_unlock();

    return true; // Сообщаем, что всё прошло успешно
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
        
        const char* stored_filename_c_str = (const char*)lv_event_get_user_data(e);

        if (!stored_filename_c_str) { 
            Serial.println("Profile click: FATAL - No filename in cache for this index!"); 
            return; 
        }
        strncpy(current_selected_profile_filename, stored_filename_c_str, sizeof(current_selected_profile_filename) - 1);
        current_selected_profile_filename[sizeof(current_selected_profile_filename) - 1] = '\0';
        Serial.printf("Clicked on profile file: %s. Preparing detail screen.\n", current_selected_profile_filename);
        if (!sd_card_initialized) {
            lvgl_port_lock(-1); const char * err_mbox_btns[] = {"OK", ""};
            lv_obj_t * mbox_err_sd = lv_msgbox_create(lv_scr_act(), "SD Error", "SD card not ready.", err_mbox_btns, true);
            lv_obj_center(mbox_err_sd); lvgl_port_unlock(); return;
        }
        String full_path_to_read = "/" + String(current_selected_profile_filename);
        bool read_ok = readFileContentToBuffer_ino(SD, full_path_to_read.c_str(), file_content_buffer, FILE_CONTENT_BUFFER_SIZE);
        lvgl_port_lock(-1);
        if (read_ok && parseProfileJson(file_content_buffer, current_active_profile_data)) {
            // --- ОБНОВЛЕНИЕ ДАННЫХ И ПЕРЕВОД ---
            if (label_detail_view_profile_name) lv_label_set_text(label_detail_view_profile_name, current_active_profile_data.name);
            if (label_detail_view_id) lv_label_set_text_fmt(label_detail_view_id, "ID: %d", current_active_profile_data.id);
            
            // --- Логика перевода в зависимости от языка ---
            if (current_global_settings.language == 1) { // RUS
                const char* on_str = "ВКЛ";
                const char* off_str = "ВЫКЛ";

                if (label_detail_view_thermal_chamber_enabled) lv_label_set_text_fmt(label_detail_view_thermal_chamber_enabled, "Термокамера: %s", current_active_profile_data.thermal_chamber_enabled ? on_str : off_str);
                if (label_detail_view_thermal_chamber) {
                    if (current_active_profile_data.thermal_chamber_enabled) {
                        lv_obj_clear_flag(label_detail_view_thermal_chamber, LV_OBJ_FLAG_HIDDEN);
                        lv_label_set_text_fmt(label_detail_view_thermal_chamber, "Цель: %d C, Удержание: %d с", current_active_profile_data.thermal_chamber_temp, current_active_profile_data.heat_exchange_hold_sec);
                    } else {
                        lv_obj_add_flag(label_detail_view_thermal_chamber, LV_OBJ_FLAG_HIDDEN);
                    }
                }
                if (label_detail_view_nitrogen) {
                    if (current_active_profile_data.nitrogen_use_enabled) {
                        lv_label_set_text_fmt(label_detail_view_nitrogen, "Азот: ВКЛ (Цель: %d%%)", current_active_profile_data.nitrogen_target_percent);
                    } else {
                        lv_label_set_text(label_detail_view_nitrogen, "Азот: ВЫКЛ");
                    }
                }
                if (label_detail_view_chamber_cooling) lv_label_set_text_fmt(label_detail_view_chamber_cooling, "Сжатый воздух: %s", current_active_profile_data.chamber_cooling_enabled ? on_str : off_str);
                if (label_detail_view_primary_uv) lv_label_set_text_fmt(label_detail_view_primary_uv, "Первичный Ультрафиолет (Мерцания): %d с", current_active_profile_data.primary_uv_exposure_sec); // ИЗМЕНЕНО
                if (label_detail_view_secondary_uv) lv_label_set_text_fmt(label_detail_view_secondary_uv, "Вторичный Ультрафиолет (Статичный): %d с", current_active_profile_data.secondary_uv_exposure_sec); // ИЗМЕНЕНО
                if (label_detail_view_tertiary_uv) lv_label_set_text_fmt(label_detail_view_tertiary_uv, "Третичный Ультрафиолет (Статичный): %d с", current_active_profile_data.tertiary_uv_exposure_sec);

            } else { // ENG
                const char* on_str = "ON";
                const char* off_str = "OFF";

                if (label_detail_view_thermal_chamber_enabled) lv_label_set_text_fmt(label_detail_view_thermal_chamber_enabled, "Thermal Chamber: %s", current_active_profile_data.thermal_chamber_enabled ? on_str : off_str);
                 if (label_detail_view_thermal_chamber) {
                    if (current_active_profile_data.thermal_chamber_enabled) {
                        lv_obj_clear_flag(label_detail_view_thermal_chamber, LV_OBJ_FLAG_HIDDEN);
                        lv_label_set_text_fmt(label_detail_view_thermal_chamber, "Target Temp: %d C, Hold: %d s", current_active_profile_data.thermal_chamber_temp, current_active_profile_data.heat_exchange_hold_sec);
                    } else {
                        lv_obj_add_flag(label_detail_view_thermal_chamber, LV_OBJ_FLAG_HIDDEN);
                    }
                }
                if (label_detail_view_nitrogen) {
                    if (current_active_profile_data.nitrogen_use_enabled) {
                        lv_label_set_text_fmt(label_detail_view_nitrogen, "Nitrogen Use: ON (Target: %d%%)", current_active_profile_data.nitrogen_target_percent);
                    } else {
                        lv_label_set_text(label_detail_view_nitrogen, "Nitrogen Use: OFF");
                    }
                }
                if (label_detail_view_chamber_cooling) lv_label_set_text_fmt(label_detail_view_chamber_cooling, "Compressed air: %s", current_active_profile_data.chamber_cooling_enabled ? on_str : off_str); // ПЕРЕМЕЩЕНО
                if (label_detail_view_primary_uv) lv_label_set_text_fmt(label_detail_view_primary_uv, "Primary UV: %d s", current_active_profile_data.primary_uv_exposure_sec);
                if (label_detail_view_secondary_uv) lv_label_set_text_fmt(label_detail_view_secondary_uv, "Secondary UV: %d s", current_active_profile_data.secondary_uv_exposure_sec);
                if (label_detail_view_tertiary_uv) lv_label_set_text_fmt(label_detail_view_tertiary_uv, "Tertiary UV: %d s", current_active_profile_data.tertiary_uv_exposure_sec);

            }
        } else { 
            if (label_detail_view_profile_name) lv_label_set_text(label_detail_view_profile_name, current_selected_profile_filename); 
            if (label_detail_view_id) lv_label_set_text(label_detail_view_id, "ID: N/A (Error)");
            if (label_detail_view_thermal_chamber) {
                lv_label_set_text_fmt(label_detail_view_thermal_chamber, "Target Temp: %d C, Hold: %d s", 
                                    current_active_profile_data.thermal_chamber_temp,
                                    current_active_profile_data.heat_exchange_hold_sec);
            }
            if (label_detail_view_nitrogen) lv_label_set_text(label_detail_view_nitrogen, "");
            if (label_detail_view_chamber_cooling) lv_label_set_text(label_detail_view_chamber_cooling, "");
            if (label_detail_view_primary_uv) lv_label_set_text(label_detail_view_primary_uv, "");
            if (label_detail_view_secondary_uv) lv_label_set_text(label_detail_view_secondary_uv, "");
        }
        // --- ДОБАВЛЕНО: Перевод статичных элементов (заголовка и кнопок) ---
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
        // Применяем белый цвет и кириллический шрифт ко всем кнопкам, независимо от языка
        lv_obj_add_style(label_detail_btn_start, &style_my_text_18_white, 0);
        lv_obj_add_style(label_detail_btn_edit, &style_my_text_18_white, 0);
        lv_obj_add_style(label_detail_btn_delete, &style_my_text_18_white, 0);
        lv_obj_add_style(label_detail_btn_close, &style_my_text_18_white, 0);
        // ----------------------------------------------------------------------
        apply_theme_to_profile_details_screen();
        if (screen_profile_details) { lv_scr_load(screen_profile_details); }
        else { Serial.println("ERROR: screen_profile_details is NULL!"); }
        lvgl_port_unlock();
    }
}

static void profile_detail_edit_btn_event_cb(lv_event_t * e) {
    if (e && lv_event_get_code(e) != LV_EVENT_CLICKED) return;

    // 1. Гарантированно перезагружаем данные из файла перед редактированием
    String full_path_to_read = "/" + String(current_selected_profile_filename);
    if (!readFileContentToBuffer_ino(SD, full_path_to_read.c_str(), file_content_buffer, FILE_CONTENT_BUFFER_SIZE) || !parseProfileJson(file_content_buffer, current_active_profile_data)) {
        if (current_global_settings.language == 1) { // RUS
            show_info_dialog("Ошибка", "Не удалось загрузить данные профиля для редактирования.");
        } else { // ENG
            show_info_dialog("Error", "Could not load profile data for editing.");
        }
        return;
    }
    
    Serial.printf("--- Preparing edit screen for profile: %s ---\n", current_active_profile_data.name);
    lvgl_port_lock(-1);

    if (screen_profile_edit) {
        // ==========================================================
        // БЛОК 1: Заполнение полей данными из current_active_profile_data
        // ==========================================================
        
        char num_buf[10]; // Буфер для конвертации чисел в строки

        // --- Шапка ---
        lv_textarea_set_text(ta_edit_profile_name, current_active_profile_data.name);

        // --- Левая колонка (Параметры УФ) ---
        // Первичный УФ
        snprintf(num_buf, sizeof(num_buf), "%d", current_active_profile_data.primary_uv_exposure_sec);
        lv_textarea_set_text(ta_edit_primary_uv, num_buf);
        snprintf(num_buf, sizeof(num_buf), "%d", current_active_profile_data.primary_uv_flicker_rate);
        lv_textarea_set_text(ta_edit_flicker_rate, num_buf);
        update_uv_mode_selector_ui(btnm_primary_uv_mode, label_primary_uv_mode_status, current_active_profile_data.primary_uv_mode);

        // Вторичный УФ
        snprintf(num_buf, sizeof(num_buf), "%d", current_active_profile_data.secondary_uv_exposure_sec);
        lv_textarea_set_text(ta_edit_secondary_uv, num_buf);
        update_uv_mode_selector_ui(btnm_secondary_uv_mode, label_secondary_uv_mode_status, current_active_profile_data.secondary_uv_mode);

        // Третичный УФ
        snprintf(num_buf, sizeof(num_buf), "%d", current_active_profile_data.tertiary_uv_exposure_sec);
        lv_textarea_set_text(ta_edit_tertiary_uv, num_buf);
        update_uv_mode_selector_ui(btnm_tertiary_uv_mode, label_tertiary_uv_mode_status, current_active_profile_data.tertiary_uv_mode);

        // --- Правая колонка (Дополнительные параметры) ---
        
        // Азот
        snprintf(num_buf, sizeof(num_buf), "%d", current_active_profile_data.nitrogen_target_percent);
        lv_textarea_set_text(ta_edit_nitrogen_target, num_buf); // Устанавливаем значение ДО проверки свитча
        if (current_active_profile_data.nitrogen_use_enabled && current_global_settings.nitrogen_system_enabled) {
            lv_obj_add_state(sw_edit_nitrogen, LV_STATE_CHECKED);
        } else {
            lv_obj_clear_state(sw_edit_nitrogen, LV_STATE_CHECKED);
        }
        
        // Сжатый воздух (Охлаждение)
        if (current_active_profile_data.chamber_cooling_enabled && current_global_settings.compressed_air_system_enabled) {
            lv_obj_add_state(sw_edit_chamber_cooling, LV_STATE_CHECKED);
        } else {
            lv_obj_clear_state(sw_edit_chamber_cooling, LV_STATE_CHECKED);
        }

        // Термокамера
        snprintf(num_buf, sizeof(num_buf), "%d", current_active_profile_data.thermal_chamber_temp);
        lv_textarea_set_text(ta_edit_thermal_temp, num_buf); // Устанавливаем значение ДО проверки свитча
        snprintf(num_buf, sizeof(num_buf), "%d", current_active_profile_data.heat_exchange_hold_sec);
        lv_textarea_set_text(ta_edit_heat_hold, num_buf); // Устанавливаем значение ДО проверки свитча
        if (current_active_profile_data.thermal_chamber_enabled) {
            lv_obj_add_state(sw_edit_thermal_chamber_enable, LV_STATE_CHECKED);
        } else {
            lv_obj_clear_state(sw_edit_thermal_chamber_enable, LV_STATE_CHECKED);
        }

        // Программно вызываем события для свитчей, чтобы UI (скрытие/показ полей) обновился корректно
        lv_event_send(sw_edit_nitrogen, LV_EVENT_VALUE_CHANGED, NULL);
        lv_event_send(sw_edit_thermal_chamber_enable, LV_EVENT_VALUE_CHANGED, NULL);

        // ==========================================================
        // БЛОК 2: Перевод всех надписей на текущий язык
        // ==========================================================
        if (current_global_settings.language == 1) { // RUS
            lv_label_set_text(label_edit_name_title, "Имя:");
            lv_label_set_text(label_btn_help_section, "Справка по разделу ?");
            lv_label_set_text(header_uv_params, "Параметры UV излучения");
            lv_label_set_text(label_uv_primary_title, "1 этап:");
            lv_label_set_text(label_uv_secondary_title, "2 этап:");
            lv_label_set_text(label_uv_tertiary_title, "3 этап:");
            lv_label_set_text(label_edit_flicker_rate_title, "Кол-во вспышек в сек. (1-100)");
            lv_label_set_text(label_edit_primary_uv_time_title, "Время работы (1-60сек)");
            lv_label_set_text(label_edit_secondary_uv_time_title, "Время работы (1-200сек)");
            lv_label_set_text(label_edit_tertiary_uv_time_title, "Время работы (1-200сек)");
            lv_label_set_text(header_poly_params, "Доп. параметры полимеризации");
            lv_label_set_text(label_edit_nitrogen_title, "Использование Азота");
            lv_label_set_text(label_edit_nitrogen_target_title, "% Цель (95-99N2):");
            lv_label_set_text(label_edit_cooling_title, "Подача сжатого воздуха ");
            lv_label_set_text(label_edit_thermal_chamber_title, "Термокамера");
            lv_label_set_text(label_edit_thermal_temp_title, "T нагрева (40-80C):");
            lv_label_set_text(label_edit_heat_hold_title, "Удержание T (30-180сек): ");
            lv_label_set_text(label_btn_save, "Сохранить");
            lv_label_set_text(label_btn_cancel, "Отмена");
        } else { // ENG
            lv_label_set_text(label_edit_name_title, "Name:");
            lv_label_set_text(label_btn_help_section, "Section Help ?");
            lv_label_set_text(header_uv_params, "UV Parameters");
            lv_label_set_text(label_uv_primary_title, "1st Stage:");
            lv_label_set_text(label_uv_secondary_title, "2nd Stage:");
            lv_label_set_text(label_uv_tertiary_title, "3rd Stage:");
            lv_label_set_text(label_edit_flicker_rate_title, "Flashes per sec (1-100)");
            lv_label_set_text(label_edit_primary_uv_time_title, "Exposure time (1-60s)");
            lv_label_set_text(label_edit_secondary_uv_time_title, "Exposure time (1-200s)");
            lv_label_set_text(label_edit_tertiary_uv_time_title, "Exposure time (1-200s)");
            lv_label_set_text(header_poly_params, "Additional Parameters");
            lv_label_set_text(label_edit_nitrogen_title, "Nitrogen Use");
            lv_label_set_text(label_edit_nitrogen_target_title, "% Target (95-99N2):");
            lv_label_set_text(label_edit_cooling_title, "Compressed Air Use");
            lv_label_set_text(label_edit_thermal_chamber_title, "Thermal Chamber");
            lv_label_set_text(label_edit_thermal_temp_title, "Heating t (40-80C):");
            lv_label_set_text(label_edit_heat_hold_title, "Hold Time (30-180sec):");
            lv_label_set_text(label_btn_save, "Save Changes");
            lv_label_set_text(label_btn_cancel, "Cancel");
        }
        
        // ==========================================================
        // БЛОК 3: Запуск таймера мигания и загрузка экрана
        // ==========================================================
        if (help_blink_timer) lv_timer_del(help_blink_timer);
        // Передаем указатель на кнопку в таймер, чтобы он мог ею управлять
        help_blink_timer = lv_timer_create(help_blink_timer_cb, 1, btn_help_section);

        apply_theme_to_profile_edit_screen();
        
        lv_scr_load(screen_profile_edit); 
    }
    
    lvgl_port_unlock();
}
static void profile_detail_delete_btn_event_cb(lv_event_t * e) {
    lv_event_code_t code = lv_event_get_code(e);
    if (code == LV_EVENT_CLICKED) {
        Serial.printf("--- DELETE button clicked for profile: %s. Showing custom confirm dialog ---\n", current_selected_profile_filename);
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
        if (screen_main_app) { lv_scr_load(screen_main_app); }
    }
}
static void confirm_dialog_cancel_btn_event_cb(lv_event_t* e) {
    if (screen_confirm_delete_dialog) {
        lv_obj_add_flag(screen_confirm_delete_dialog, LV_OBJ_FLAG_HIDDEN);
        Serial.println("Delete confirmation cancelled by user.");
    }
}
static void confirm_dialog_delete_btn_event_cb(lv_event_t* e) {
    Serial.printf("Deletion confirmed via custom dialog for: %s\n", current_selected_profile_filename);
    if (screen_confirm_delete_dialog) { lv_obj_add_flag(screen_confirm_delete_dialog, LV_OBJ_FLAG_HIDDEN); }
    String full_path_to_delete = "/" + String(current_selected_profile_filename);
    lvgl_port_lock(-1); 
    deleteFile(SD, full_path_to_delete.c_str()); 
    bool delete_verified = !SD.exists(full_path_to_delete.c_str());
    if (delete_verified) { Serial.println("Verification: File successfully deleted."); }
    else { Serial.println("Verification: File still exists or SD error. Deletion may have failed."); }
    current_profile_next_id = scanAndCacheAllProfiles(SD, all_profile_entries_cache); 
    displayProfileListPage(); 
    if (screen_main_app) { lv_scr_load(screen_main_app); } 
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
    else if (ta == ta_edit_flicker_rate) { validate_numeric_input(e, 1, 100); }
    else if (ta == ta_edit_thermal_temp) { validate_numeric_input(e, 40, 80); }
    else if (ta == ta_edit_nitrogen_target) { validate_numeric_input(e, 90, 99); }

    // --- Поля с экрана "Глазурь" ---
    else if (ta == ta_glaze_uv_on) { validate_float_input(e, 0.1f, 3.0f); }
    else if (ta == ta_glaze_uv_off) { validate_float_input(e, 0.1f, 3.0f); }
    else if (ta == ta_glaze_monomer_blow) { validate_numeric_input(e, 1, 10); }
    else if (ta == ta_glaze_uv_exposure) { validate_numeric_input(e, 10, 200); }
    else if (ta == ta_glaze_nitrogen_target) { validate_numeric_input(e, 95, 99); }

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
    ProfileData edited_data;
    edited_data.id = current_active_profile_data.id;

    // --- Сбор данных с UI ---
    strncpy(edited_data.name, lv_textarea_get_text(ta_edit_profile_name), sizeof(edited_data.name) - 1);
    edited_data.name[sizeof(edited_data.name) - 1] = '\0'; 
    if (strlen(edited_data.name) == 0) { 
        if (current_global_settings.language == 1) { // RUS
            show_info_dialog("Ошибка ввода", "Имя профиля не может быть пустым.");
        } else { // ENG
            show_info_dialog("Input Error", "Profile name cannot be empty.");
        }
        return; 
    }

    edited_data.primary_uv_exposure_sec = constrain(atoi(lv_textarea_get_text(ta_edit_primary_uv)), 1, 60);
    edited_data.primary_uv_flicker_rate = constrain(atoi(lv_textarea_get_text(ta_edit_flicker_rate)), 1, 100);
    
    // ==========================================================
    // <<<--- НАЧАЛО ИСПРАВЛЕНИЙ ---<<<
    // ==========================================================
    edited_data.primary_uv_mode = get_checked_btnmatrix_id(btnm_primary_uv_mode);

    edited_data.secondary_uv_exposure_sec = constrain(atoi(lv_textarea_get_text(ta_edit_secondary_uv)), 1, 200);
    edited_data.secondary_uv_mode = get_checked_btnmatrix_id(btnm_secondary_uv_mode);

    edited_data.tertiary_uv_exposure_sec = constrain(atoi(lv_textarea_get_text(ta_edit_tertiary_uv)), 1, 200);
    edited_data.tertiary_uv_mode = get_checked_btnmatrix_id(btnm_tertiary_uv_mode);
    // ==========================================================
    // <<<--- КОНЕЦ ИСПРАВЛЕНИЙ ---<<<
    // ==========================================================

    edited_data.nitrogen_use_enabled = lv_obj_has_state(sw_edit_nitrogen, LV_STATE_CHECKED);
    if (edited_data.nitrogen_use_enabled) {
        edited_data.nitrogen_target_percent = constrain(atoi(lv_textarea_get_text(ta_edit_nitrogen_target)), 95, 99);
    } else {
        edited_data.nitrogen_target_percent = current_active_profile_data.nitrogen_target_percent;
    }
    
    edited_data.chamber_cooling_enabled = lv_obj_has_state(sw_edit_chamber_cooling, LV_STATE_CHECKED);
    
    edited_data.thermal_chamber_enabled = lv_obj_has_state(sw_edit_thermal_chamber_enable, LV_STATE_CHECKED);
    if (edited_data.thermal_chamber_enabled) {
        edited_data.thermal_chamber_temp = constrain(atoi(lv_textarea_get_text(ta_edit_thermal_temp)), 40, 80);
        edited_data.heat_exchange_hold_sec = constrain(atoi(lv_textarea_get_text(ta_edit_heat_hold)), 30, 180);
    } else {
        edited_data.thermal_chamber_temp = current_active_profile_data.thermal_chamber_temp;
        edited_data.heat_exchange_hold_sec = current_active_profile_data.heat_exchange_hold_sec;
    }

    // --- Сохранение и переход ---
    if (help_blink_timer) {
        lv_timer_del(help_blink_timer);
        help_blink_timer = nullptr;
    }
    
    lvgl_port_lock(-1);

    String filename_on_sd = "/" + String(current_selected_profile_filename); 
    char json_buffer_save[FILE_CONTENT_BUFFER_SIZE];
    if (serializeProfileJson(edited_data, json_buffer_save, sizeof(json_buffer_save))) {
        writeFile(SD, filename_on_sd.c_str(), json_buffer_save);
        Serial.printf("Profile '%s' (File: %s) updated.\n", edited_data.name, filename_on_sd.c_str());
        
        current_active_profile_data = edited_data;
        scanAndCacheAllProfiles(SD, all_profile_entries_cache);
        needs_list_refresh = true;

        lv_event_t fake_event;
        fake_event.code = LV_EVENT_CLICKED;
        fake_event.target = NULL;
        fake_event.user_data = (void*)current_selected_profile_filename;
        profile_list_event_handler(&fake_event);
    } else { 
        if (current_global_settings.language == 1) { // RUS
            show_info_dialog("Ошибка Сохранения", "Не удалось подготовить данные для сохранения.");
        } else { // ENG
            show_info_dialog("Save Error", "Failed to prepare data for saving.");
        }
    }
    lvgl_port_unlock();
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
            if (label_detail_view_chamber_cooling) lv_label_set_text_fmt(label_detail_view_chamber_cooling, "Сжатый воздух: %s", current_active_profile_data.chamber_cooling_enabled ? on_str : off_str);
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
            if (label_detail_view_chamber_cooling) lv_label_set_text_fmt(label_detail_view_chamber_cooling, "Compressed air: %s", current_active_profile_data.chamber_cooling_enabled ? on_str : off_str);
            if (label_detail_view_primary_uv) lv_label_set_text_fmt(label_detail_view_primary_uv, "Primary UV: %d s", current_active_profile_data.primary_uv_exposure_sec);
            if (label_detail_view_secondary_uv) lv_label_set_text_fmt(label_detail_view_secondary_uv, "Secondary UV: %d s", current_active_profile_data.secondary_uv_exposure_sec);
            if (label_detail_view_tertiary_uv) lv_label_set_text_fmt(label_detail_view_tertiary_uv, "Tertiary UV: %d s", current_active_profile_data.tertiary_uv_exposure_sec);

        }

        lvgl_port_unlock();
        lv_scr_load(screen_profile_details);
    }
}

// Обработчик для кнопки "Cancel" на экране выполнения процесса
static void process_execution_cancel_btn_event_cb(lv_event_t * e) {
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;

    Serial.println("--- CANCEL/BACK button clicked by user on process screen. ---");

    // 1. Отправляем команду СТОП, если процесс еще шел
    if (main_process_running) {
        StaticJsonDocument<128> doc;
        doc["command"] = "EMERGENCY_STOP";
        String output;
        serializeJson(doc, output);
        MySerial1.println(output);
        Serial.println("Sent command: EMERGENCY_STOP");
    }
    
    // 2. НЕМЕДЛЕННО сбрасываем все флаги состояния
    main_process_running = false; 
    is_lab_mode_running = false;
    current_process_stage = "";
   
    // 3. НЕМЕДЛЕННО возвращаемся на предыдущий экран
    if (screen_to_return_after_process) {
        Serial.println("Returning to the previous screen.");
        lv_scr_load(screen_to_return_after_process);
    } else {
        Serial.println("Warning: screen_to_return_after_process was NULL. Returning to main screen.");
        lv_scr_load(screen_main_app);
    }

    // 4. И ТОЛЬКО ТЕПЕРЬ вычищаем весь мусор из буфера
    flush_serial_buffer();
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
            saveGlobalSettings(); 
            needs_list_refresh = true;
            if (screen_main_app) {
                lv_scr_load(screen_main_app); 
            }
        }
        else if (strcmp(user_data_str, "test_nitro") == 0) {
            Serial.println("Nitrogen test button clicked.");
            saveGlobalSettings(); 
            needs_list_refresh = true;
            if (screen_test_nitrogen) {
                 // <<< ДОБАВЬТЕ ЭТИ СТРОКИ >>>
                 is_nitrogen_test_active = false; // Сбрасываем состояние
                 lv_label_set_text(label_btn_nitrogen_test_press, tr("Start Supply"));
                 // --------------------------
                 lv_label_set_text(label_nitrogen_test_header, tr("Nitrogen Supply Test"));
                 lv_label_set_text(label_btn_nitrogen_test_back, tr("Back"));
                 apply_theme_to_test_nitrogen_screen();
                 lv_scr_load(screen_test_nitrogen);
            }
        }
        else if (strcmp(user_data_str, "test_air") == 0) {
            Serial.println("Air test button clicked.");
            saveGlobalSettings();
            needs_list_refresh = true;
            if (screen_test_air) {
                 // <<< ДОБАВЬТЕ ЭТИ СТРОКИ >>>
                 is_air_test_active = false; // Сбрасываем состояние
                 lv_label_set_text(label_btn_air_test_press, tr("Start Supply"));
                 // --------------------------
                 lv_label_set_text(label_air_test_header, tr("Compressed Air Test"));
                 lv_label_set_text(label_btn_air_test_back, tr("Back"));
                 apply_theme_to_test_air_screen();
                 lv_scr_load(screen_test_air);
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
            apply_theme_to_settings_screen();
        }
        else if (strcmp(user_data_str, "theme2") == 0) {
            current_global_settings.theme = 1; // Dark
            update_custom_toggle_ui(theme_toggle_box, current_global_settings.theme);
            update_timeout_toggle_ui(timeout_toggle_box, current_global_settings.screen_timeout_mode);
            apply_theme_to_settings_screen();
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
        Serial.println("Settings button on main screen clicked.");
        if (screen_settings) {
            lvgl_port_lock(-1); 
            loadGlobalSettings(); 

            // Обновляем старые свитчи
            if (sw_settings_global_nitrogen_enabled) {
                if (current_global_settings.nitrogen_system_enabled) lv_obj_add_state(sw_settings_global_nitrogen_enabled, LV_STATE_CHECKED);
                else lv_obj_clear_state(sw_settings_global_nitrogen_enabled, LV_STATE_CHECKED);
            }
            if (sw_settings_global_air_enabled) {
                if (current_global_settings.compressed_air_system_enabled) lv_obj_add_state(sw_settings_global_air_enabled, LV_STATE_CHECKED);
                else lv_obj_clear_state(sw_settings_global_air_enabled, LV_STATE_CHECKED);
            }

            // Устанавливаем начальное состояние кнопок "Тест"
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
            
            // Обновляем вид новых кастомных переключателей
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
                lv_label_set_text(label_settings_lang_opt1, "АНГЛ"); // ENG -> АНГЛ
                lv_obj_add_style(label_settings_lang_opt1, &style_my_text_16, 0);
                lv_label_set_text(label_settings_lang_opt2, "РУС");  // RUS -> РУС
                lv_obj_add_style(label_settings_lang_opt2, &style_my_text_16, 0);
                lv_label_set_text(label_settings_theme_opt1, "Светлая"); // Light -> Светлая
                lv_obj_add_style(label_settings_theme_opt1, &style_my_text_16, 0);
                lv_label_set_text(label_settings_theme_opt2, "Темная");  // Dark -> Темная
                lv_obj_add_style(label_settings_theme_opt2, &style_my_text_16, 0);
                lv_obj_t* lbl_test_nitro = lv_obj_get_child(lv_obj_get_child(lv_obj_get_child(lv_obj_get_child(content_grid_settings, 0), 0), 2), 0);
                lv_obj_t* lbl_test_air = lv_obj_get_child(lv_obj_get_child(lv_obj_get_child(lv_obj_get_child(content_grid_settings, 0), 1), 2), 0);
                lv_label_set_text(lbl_test_nitro, tr("Test"));
                lv_label_set_text(lbl_test_air, tr("Test"));
                if (btn_settings_save_and_back) {
                    lv_obj_t* label = lv_obj_get_child(btn_settings_save_and_back, 0);
                    if (label) {
                        lv_label_set_text(label, "Сохранить и Выйти");
                        // <<<--- ИСПОЛЬЗУЕМ НОВЫЙ СТИЛЬ ДЛЯ БЕЛОГО ТЕКСТА ---<<<
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
                         // <<<--- ИСПОЛЬЗУЕМ НОВЫЙ СТИЛЬ ДЛЯ БЕЛОГО ТЕКСТА ---<<<
                        lv_obj_add_style(label, &style_my_text_18_white, 0);
                    }
                }
            }

            if(label_settings_lang_opt1) lv_obj_add_style(label_settings_lang_opt1, &style_my_text_16, 0);
            if(label_settings_lang_opt2) lv_obj_add_style(label_settings_lang_opt2, &style_my_text_16, 0);
            if(label_settings_theme_opt1) lv_obj_add_style(label_settings_theme_opt1, &style_my_text_16, 0);
            if(label_settings_theme_opt2) lv_obj_add_style(label_settings_theme_opt2, &style_my_text_16, 0);
            
            apply_theme_to_settings_screen();

            lv_scr_load(screen_settings);
            lvgl_port_unlock();
        }
    }
}

static void secret_button_event_cb(lv_event_t* e) {
    if (lv_event_get_code(e) == LV_EVENT_CLICKED) {
        Serial.println("Secret button clicked! Loading secret game screen...");
        if (screen_secret_game) {
            // Здесь можно передать какие-то параметры в игру, если нужно
            lv_scr_load(screen_secret_game);
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
        StaticJsonDocument<128> doc;
        doc["command"] = "USER_CHOSE_TO_SKIP_NITROGEN";
        String output;
        serializeJson(doc, output);
        MySerial1.println(output);

        // Обновляем UI, чтобы пользователь видел, что что-то происходит
        lv_label_set_text(label_process_status_title, tr("Continuing process..."));
        lv_label_set_text(label_process_status_detail, tr("Nitrogen purge stage skipped."));

    } else if (btn == btn_choice_dialog_cancel) {
        Serial.println("Choice Dialog: User selected CANCEL PROCESS.");
        
        // 1. Отправляем команду на ПП
        StaticJsonDocument<128> doc;
        doc["command"] = "EMERGENCY_STOP";
        String output;
        serializeJson(doc, output);
        MySerial1.println(output);

        // 2. Немедленно сбрасываем флаги состояния на УП
        main_process_running = false; 
        is_lab_mode_running = false;
        current_process_stage = "";

        // 3. Немедленно возвращаемся на предыдущий экран
        if (screen_to_return_after_process) {
            lv_scr_load(screen_to_return_after_process);
        } else {
            lv_scr_load(screen_main_app);
        }

        // 4. И ТОЛЬКО ТЕПЕРЬ вычищаем весь мусор из буфера
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

        lv_scr_load(screen_keyboard);
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
            
            remove_and_save_service_keys(valid_key_found);
            current_global_settings.is_heater_error = false;
            saveGlobalSettings();
            lv_scr_load(screen_main_app);
            
            lvgl_port_lock(-1);
            if (sd_card_initialized) {
                current_profile_next_id = scanAndCacheAllProfiles(SD, all_profile_entries_cache);
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
        lv_scr_load(screen_to_return_to);

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
        }
    }
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
        lv_scr_load(screen_to_return_to);
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
    lv_scr_load(screen_keyboard);
    
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
        else if (strcmp(user_data, "strength_cooling") == 0) current_lab_settings.strength.use_cooling = is_checked;
        else if (strcmp(user_data, "lighten_cooling") == 0) current_lab_settings.lighten.use_cooling = is_checked;
        else if (strcmp(user_data, "darken_cooling") == 0) current_lab_settings.darken.use_cooling = is_checked;
    } 
    else {
        return; // Если это не defocus и не value_changed, выходим
    }

    // --- Фаза 2: Сохранение на SD-карту ---
    Serial.printf("Lab parameter '%s' changed. Saving settings to SD card...\n", user_data);
    saveLaboratorySettings();
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
        // Управляем видимостью поля для цели азота через ПРОЗРАЧНОСТЬ
        if (is_checked) {
            lv_obj_set_style_opa(nitrogen_glaze_container, LV_OPA_COVER, 0);
            lv_obj_clear_flag(nitrogen_glaze_container, LV_OBJ_FLAG_CLICKABLE);
        } else {
            lv_obj_set_style_opa(nitrogen_glaze_container, LV_OPA_TRANSP, 0);
            lv_obj_add_flag(nitrogen_glaze_container, LV_OBJ_FLAG_CLICKABLE);
        }
        Serial.printf("Glaze Nitrogen Switch toggled: %s\n", is_checked ? "ON" : "OFF");
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
    lv_obj_set_layout(block_timers, LV_LAYOUT_FLEX);
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
    { lv_obj_t* row = lv_obj_create(timers_params_cont); lv_obj_remove_style_all(row); lv_obj_set_width(row, lv_pct(100)); lv_obj_set_height(row, LV_SIZE_CONTENT); lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW); lv_obj_set_flex_align(row, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER); label_glaze_monomer_blow = lv_label_create(row); lv_obj_add_style(label_glaze_monomer_blow, &style_my_text_18, 0); ta_glaze_monomer_blow = lv_textarea_create(row); lv_textarea_set_one_line(ta_glaze_monomer_blow, true); lv_obj_set_size(ta_glaze_monomer_blow, 80, TEXT_INPUT_HEIGHT); lv_obj_add_style(ta_glaze_monomer_blow, &style_edit_textarea, 0); lv_obj_set_scrollbar_mode(ta_glaze_monomer_blow, LV_SCROLLBAR_MODE_OFF); lv_obj_add_event_cb(ta_glaze_monomer_blow, numeric_textarea_focus_event_cb, LV_EVENT_FOCUSED, NULL); lv_obj_add_event_cb(ta_glaze_monomer_blow, generic_textarea_defocus_event_cb, LV_EVENT_DEFOCUSED, NULL); lv_obj_add_event_cb(ta_glaze_monomer_blow, lab_mode_param_changed_event_cb, LV_EVENT_DEFOCUSED, (void*)"glaze_monomer_blow");}
    { lv_obj_t* row = lv_obj_create(timers_params_cont); lv_obj_remove_style_all(row); lv_obj_set_width(row, lv_pct(100)); lv_obj_set_height(row, LV_SIZE_CONTENT); lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW); lv_obj_set_flex_align(row, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER); label_glaze_uv_exposure = lv_label_create(row); lv_obj_add_style(label_glaze_uv_exposure, &style_my_text_18, 0); ta_glaze_uv_exposure = lv_textarea_create(row); lv_textarea_set_one_line(ta_glaze_uv_exposure, true); lv_obj_set_size(ta_glaze_uv_exposure, 80, TEXT_INPUT_HEIGHT); lv_obj_add_style(ta_glaze_uv_exposure, &style_edit_textarea, 0); lv_obj_set_scrollbar_mode(ta_glaze_uv_exposure, LV_SCROLLBAR_MODE_OFF); lv_obj_add_event_cb(ta_glaze_uv_exposure, numeric_textarea_focus_event_cb, LV_EVENT_FOCUSED, NULL); lv_obj_add_event_cb(ta_glaze_uv_exposure, generic_textarea_defocus_event_cb, LV_EVENT_DEFOCUSED, NULL);lv_obj_add_event_cb(ta_glaze_uv_exposure, lab_mode_param_changed_event_cb, LV_EVENT_DEFOCUSED, (void*)"glaze_uv_exposure");}

    lv_obj_t* block_aux = lv_obj_create(right_col);
    lv_obj_add_style(block_aux, &style_param_block, 0);
    lv_obj_set_size(block_aux, lv_pct(100), lv_pct(100));
    lv_obj_set_layout(block_aux, LV_LAYOUT_FLEX);

    label_glaze_title_aux = lv_label_create(block_aux);
    lv_obj_add_style(label_glaze_title_aux, &style_my_text_18, 0);
    lv_obj_add_style(label_glaze_title_aux, &style_block_header, 0);
    lv_obj_set_width(label_glaze_title_aux, lv_pct(100));
    lv_obj_set_style_text_align(label_glaze_title_aux, LV_TEXT_ALIGN_CENTER, 0);

    lv_obj_t* aux_params_cont = lv_obj_create(block_aux);
    lv_obj_remove_style_all(aux_params_cont);
    lv_obj_set_width(aux_params_cont, lv_pct(100));
    lv_obj_set_flex_grow(aux_params_cont, 1);
    
    lv_obj_set_layout(aux_params_cont, LV_LAYOUT_GRID);
    static lv_coord_t aux_col_dsc[] = {LV_GRID_FR(1), LV_GRID_TEMPLATE_LAST};
    static lv_coord_t aux_row_dsc[] = {LV_GRID_CONTENT, LV_GRID_CONTENT, LV_GRID_CONTENT, LV_GRID_TEMPLATE_LAST};
    lv_obj_set_grid_dsc_array(aux_params_cont, aux_col_dsc, aux_row_dsc);
    lv_obj_set_style_grid_row_align(aux_params_cont, LV_GRID_ALIGN_SPACE_EVENLY, 0);
    lv_obj_set_style_pad_top(aux_params_cont, 10, 0);
    
    { lv_obj_t* row = lv_obj_create(aux_params_cont); lv_obj_remove_style_all(row); lv_obj_set_grid_cell(row, LV_GRID_ALIGN_STRETCH, 0, 1, LV_GRID_ALIGN_CENTER, 0, 1); lv_obj_set_width(row, lv_pct(100)); lv_obj_set_height(row, LV_SIZE_CONTENT); lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW); lv_obj_set_flex_align(row, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER); label_glaze_cooling = lv_label_create(row); lv_obj_add_style(label_glaze_cooling, &style_my_text_18, 0); sw_glaze_cooling = lv_switch_create(row); lv_obj_add_event_cb(sw_glaze_cooling, lab_mode_switch_event_cb, LV_EVENT_VALUE_CHANGED, (void*)"glaze_cooling"); lv_obj_add_event_cb(sw_glaze_cooling, lab_mode_param_changed_event_cb, LV_EVENT_VALUE_CHANGED, (void*)"glaze_cooling"); }
    { lv_obj_t* row = lv_obj_create(aux_params_cont); lv_obj_remove_style_all(row); lv_obj_set_grid_cell(row, LV_GRID_ALIGN_STRETCH, 0, 1, LV_GRID_ALIGN_CENTER, 1, 1); lv_obj_set_width(row, lv_pct(100)); lv_obj_set_height(row, LV_SIZE_CONTENT); lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW); lv_obj_set_flex_align(row, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER); label_glaze_nitrogen = lv_label_create(row); lv_obj_add_style(label_glaze_nitrogen, &style_my_text_18, 0); sw_glaze_nitrogen = lv_switch_create(row); lv_obj_add_event_cb(sw_glaze_nitrogen, lab_mode_switch_event_cb, LV_EVENT_VALUE_CHANGED, (void*)"glaze_nitrogen"); lv_obj_add_event_cb(sw_glaze_nitrogen, lab_mode_param_changed_event_cb, LV_EVENT_VALUE_CHANGED, (void*)"glaze_nitrogen"); }
    
    nitrogen_glaze_container = lv_obj_create(aux_params_cont);
    lv_obj_remove_style_all(nitrogen_glaze_container);
    lv_obj_set_grid_cell(nitrogen_glaze_container, LV_GRID_ALIGN_STRETCH, 0, 1, LV_GRID_ALIGN_CENTER, 2, 1);
    lv_obj_set_width(nitrogen_glaze_container, lv_pct(100));
    lv_obj_set_height(nitrogen_glaze_container, LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(nitrogen_glaze_container, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(nitrogen_glaze_container, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    
    label_glaze_nitrogen_target = lv_label_create(nitrogen_glaze_container);
    lv_obj_add_style(label_glaze_nitrogen_target, &style_my_text_18, 0);

    ta_glaze_nitrogen_target = lv_textarea_create(nitrogen_glaze_container);
    lv_textarea_set_one_line(ta_glaze_nitrogen_target, true);
    lv_obj_set_size(ta_glaze_nitrogen_target, 80, TEXT_INPUT_HEIGHT);
    lv_obj_add_style(ta_glaze_nitrogen_target, &style_edit_textarea, 0);
    lv_obj_set_scrollbar_mode(ta_glaze_nitrogen_target, LV_SCROLLBAR_MODE_OFF);
    lv_obj_add_event_cb(ta_glaze_nitrogen_target, numeric_textarea_focus_event_cb, LV_EVENT_FOCUSED, NULL); 
    lv_obj_add_event_cb(ta_glaze_nitrogen_target, generic_textarea_defocus_event_cb, LV_EVENT_DEFOCUSED, NULL);
    lv_obj_add_event_cb(ta_glaze_nitrogen_target, lab_mode_param_changed_event_cb, LV_EVENT_DEFOCUSED, (void*)"glaze_nitrogen_target");

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
    lv_obj_add_event_cb(btn_glaze_start, lab_mode_start_event_cb, LV_EVENT_CLICKED, (void*)1);
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
        if (screen_laboratory) lv_scr_load(screen_laboratory);
    } else if (strcmp(user_data, "repair_start") == 0) {
        lv_event_send(lv_event_get_target(e), LV_EVENT_CLICKED, (void*)2);
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
    lv_obj_add_event_cb(btn_repair_start, lab_mode_start_event_cb, LV_EVENT_CLICKED, (void*)2);
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
        if (screen_laboratory) lv_scr_load(screen_laboratory);
    } else if (strcmp(user_data, "strength_start") == 0) {
        lv_event_send(lv_event_get_target(e), LV_EVENT_CLICKED, (void*)3);
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
    lv_obj_add_event_cb(btn_strength_start, lab_mode_start_event_cb, LV_EVENT_CLICKED, (void*)3);
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
        if (screen_laboratory) lv_scr_load(screen_laboratory);
    } else if (strcmp(user_data, "thermal_start") == 0) {
        lv_event_send(lv_event_get_target(e), LV_EVENT_CLICKED, (void*)4);
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
    lv_obj_add_event_cb(btn_thermal_start, lab_mode_start_event_cb, LV_EVENT_CLICKED, (void*)4);
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
        if (screen_laboratory) lv_scr_load(screen_laboratory);
    } else if (strcmp(user_data, "lighten_start") == 0) {
        lv_event_send(lv_event_get_target(e), LV_EVENT_CLICKED, (void*)5);
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
    lv_obj_add_event_cb(btn_lighten_start, lab_mode_start_event_cb, LV_EVENT_CLICKED, (void*)5);
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
        if (screen_laboratory) lv_scr_load(screen_laboratory);
    } else if (strcmp(user_data, "darken_start") == 0) {
        lv_event_send(lv_event_get_target(e), LV_EVENT_CLICKED, (void*)6);
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
    lv_obj_add_event_cb(btn_darken_start, lab_mode_start_event_cb, LV_EVENT_CLICKED, (void*)6);
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
    // Получаем указатели на наши объекты из данных таймера
    lv_obj_t* screen = (lv_obj_t*)timer->user_data;
    lv_obj_t* title_label = lv_obj_get_child(screen, 0); // Текст - это первый дочерний объект экрана
    
    Serial.println("Starting splash screen inversion animation (Corrected)...");

    // 1. Анимация ФОНА (из белого в черный)
    lv_anim_t bg_anim;
    lv_anim_init(&bg_anim);
    lv_anim_set_var(&bg_anim, screen);
    lv_anim_set_exec_cb(&bg_anim, [](void* obj, int32_t v) {
        // <<< ИСПРАВЛЕНИЕ: Поменяли цвета местами. Теперь из белого в черный. >>>
        lv_color_t color = lv_color_mix(lv_color_white(), lv_color_black(), v);
        lv_obj_set_style_bg_color((lv_obj_t*)obj, color, 0);
    });
    lv_anim_set_values(&bg_anim, 255, 0); // От 255 (белый) до 0 (черный)
    lv_anim_set_time(&bg_anim, 1500); // Длительность 1.5 секунды
    lv_anim_start(&bg_anim);

    // 2. Анимация ТЕКСТА (из черного в белый)
    lv_anim_t text_anim;
    lv_anim_init(&text_anim);
    lv_anim_set_var(&text_anim, title_label);
    lv_anim_set_exec_cb(&text_anim, [](void* obj, int32_t v) {
        // <<< ИСПРАВЛЕНИЕ: Поменяли цвета местами. Теперь из черного в белый. >>>
        lv_color_t color = lv_color_mix(lv_color_black(), lv_color_white(), v);
        lv_obj_set_style_text_color((lv_obj_t*)obj, color, 0);
    });
    lv_anim_set_values(&text_anim, 255, 0); // От 255 (черный) до 0 (белый)
    lv_anim_set_time(&text_anim, 1500); // Та же длительность, что и у фона
    lv_anim_start(&text_anim);
}

static void on_splash_anim_finish(lv_anim_t* anim) {
    Serial.println("Splash animation sequence finished. Loading next screen.");
    
    // Проверяем, есть ли ошибка нагревателя, и загружаем нужный экран
    if (current_global_settings.is_heater_error) {
        enter_service_lock_mode(
            "Critical Error!\nHeating element failure.\nDevice is locked.",
            "Критическая ошибка!\nОтказ нагревательного элемента.\nУстройство заблокировано."
        );
    } else {
        lv_scr_load(screen_main_app);
    }
}

static void build_splash_screen(lv_obj_t* parent_screen) {
    Serial.println("Building simple, robust splash screen...");

    // --- Шаг 1: Устанавливаем БЕЛЫЙ фон ---
    // lv_obj_remove_style_all(parent_screen);
    lv_obj_set_size(parent_screen, lv_pct(100), lv_pct(100)); 
    lv_obj_set_style_bg_color(parent_screen, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_bg_opa(parent_screen, LV_OPA_COVER, 0);

    // --- Шаг 2: Создаем текст ---
    lv_obj_t* title_label = lv_label_create(parent_screen);
    lv_label_set_text(title_label, "SpectraMaster N2");
    lv_obj_center(title_label);
    
    // Стиль текста: ЧЕРНЫЙ цвет, большой шрифт, изначально ПРОЗРАЧНЫЙ
    lv_obj_set_style_text_color(title_label, lv_color_black(), 0);
    lv_obj_set_style_text_font(title_label, &lv_font_montserrat_48, 0);
    lv_obj_set_style_opa(title_label, LV_OPA_TRANSP, 0);

    // --- Шаг 3: Анимация появления текста (Fade-In) ---
    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_var(&a, title_label);
    lv_anim_set_values(&a, LV_OPA_TRANSP, LV_OPA_COVER);
    lv_anim_set_time(&a, 2000); // 2 секунды
    lv_anim_set_delay(&a, 200); 
    lv_anim_set_exec_cb(&a, (lv_anim_exec_xcb_t)lv_obj_set_style_opa);
    lv_anim_start(&a);
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
    Serial.println("Building THEME-AGNOSTIC profile_edit_screen UI... Final version.");

    const lv_coord_t TEXT_INPUT_HEIGHT = 36;

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
    
    // Стиль для строк-контейнеров переключателей. ЦВЕТ НЕ ЗАДАЕТСЯ!
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
    
    // --- ПЕРВЫЙ ПЕРЕКЛЮЧАТЕЛЬ ---
    { 
        row_uv_primary = lv_obj_create(block_uv_all_stages);
        lv_obj_remove_style_all(row_uv_primary);
        lv_obj_add_style(row_uv_primary, &style_switch_row, 0);
        lv_obj_set_width(row_uv_primary, lv_pct(100)); 
        lv_obj_set_height(row_uv_primary, LV_SIZE_CONTENT); 
        lv_obj_set_flex_flow(row_uv_primary, LV_FLEX_FLOW_ROW); 
        lv_obj_set_flex_align(row_uv_primary, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER); 
        
        label_uv_primary_title = lv_label_create(row_uv_primary); 
        lv_obj_add_style(label_uv_primary_title, &style_my_text_18, 0); 
        label_primary_uv_mode_status = lv_label_create(row_uv_primary); 
        lv_obj_add_style(label_primary_uv_mode_status, &style_my_text_18, 0); 
        btnm_primary_uv_mode = lv_btnmatrix_create(row_uv_primary);
        
        lv_obj_set_size(btnm_primary_uv_mode, 150, 40);
        lv_btnmatrix_set_map(btnm_primary_uv_mode, uv_btnm_map);
        lv_btnmatrix_set_btn_ctrl_all(btnm_primary_uv_mode, LV_BTNMATRIX_CTRL_CHECKABLE);
        lv_btnmatrix_set_one_checked(btnm_primary_uv_mode, true);
        lv_obj_set_style_bg_opa(btnm_primary_uv_mode, LV_OPA_TRANSP, 0);
        lv_obj_set_style_border_width(btnm_primary_uv_mode, 0, 0);
        lv_obj_set_style_pad_all(btnm_primary_uv_mode, 3, 0);
        lv_obj_set_style_radius(btnm_primary_uv_mode, LV_RADIUS_CIRCLE, LV_PART_ITEMS);
        lv_obj_set_style_border_width(btnm_primary_uv_mode, 0, LV_PART_ITEMS);
        lv_obj_set_style_text_font(btnm_primary_uv_mode, &montserrat_rus_18, LV_PART_ITEMS);
        
        lv_obj_add_event_cb(btnm_primary_uv_mode, uv_mode_selector_event_cb, LV_EVENT_VALUE_CHANGED, label_primary_uv_mode_status); 
    }
    { lv_obj_t* row = lv_obj_create(block_uv_all_stages); lv_obj_remove_style_all(row); lv_obj_set_width(row, lv_pct(100)); lv_obj_set_height(row, LV_SIZE_CONTENT); lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW); lv_obj_set_flex_align(row, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER); label_edit_flicker_rate_title = lv_label_create(row); lv_obj_add_style(label_edit_flicker_rate_title, &style_my_text_18, 0); ta_edit_flicker_rate = lv_textarea_create(row); lv_textarea_set_one_line(ta_edit_flicker_rate, true); lv_obj_set_size(ta_edit_flicker_rate, 80, TEXT_INPUT_HEIGHT); lv_obj_add_style(ta_edit_flicker_rate, &style_edit_textarea, 0); lv_obj_add_event_cb(ta_edit_flicker_rate, numeric_textarea_focus_event_cb, LV_EVENT_FOCUSED, NULL); lv_obj_add_event_cb(ta_edit_flicker_rate, generic_textarea_defocus_event_cb, LV_EVENT_DEFOCUSED, NULL); }
    { lv_obj_t* row = lv_obj_create(block_uv_all_stages); lv_obj_remove_style_all(row); lv_obj_set_width(row, lv_pct(100)); lv_obj_set_height(row, LV_SIZE_CONTENT); lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW); lv_obj_set_flex_align(row, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER); label_edit_primary_uv_time_title = lv_label_create(row); lv_obj_add_style(label_edit_primary_uv_time_title, &style_my_text_18, 0); ta_edit_primary_uv = lv_textarea_create(row); lv_textarea_set_one_line(ta_edit_primary_uv, true); lv_obj_set_size(ta_edit_primary_uv, 80, TEXT_INPUT_HEIGHT); lv_obj_add_style(ta_edit_primary_uv, &style_edit_textarea, 0); lv_obj_add_event_cb(ta_edit_primary_uv, numeric_textarea_focus_event_cb, LV_EVENT_FOCUSED, NULL); lv_obj_add_event_cb(ta_edit_primary_uv, generic_textarea_defocus_event_cb, LV_EVENT_DEFOCUSED, NULL); }
    
    // --- ВТОРОЙ ПЕРЕКЛЮЧАТЕЛЬ ---
    { 
        row_uv_secondary = lv_obj_create(block_uv_all_stages);
        lv_obj_remove_style_all(row_uv_secondary);
        lv_obj_add_style(row_uv_secondary, &style_switch_row, 0);
        lv_obj_set_width(row_uv_secondary, lv_pct(100)); 
        lv_obj_set_height(row_uv_secondary, LV_SIZE_CONTENT); 
        lv_obj_set_flex_flow(row_uv_secondary, LV_FLEX_FLOW_ROW); 
        lv_obj_set_flex_align(row_uv_secondary, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER); 
        
        label_uv_secondary_title = lv_label_create(row_uv_secondary); 
        lv_obj_add_style(label_uv_secondary_title, &style_my_text_18, 0); 
        label_secondary_uv_mode_status = lv_label_create(row_uv_secondary); 
        lv_obj_add_style(label_secondary_uv_mode_status, &style_my_text_18, 0); 
        btnm_secondary_uv_mode = lv_btnmatrix_create(row_uv_secondary);
        
        lv_obj_set_size(btnm_secondary_uv_mode, 150, 40);
        lv_btnmatrix_set_map(btnm_secondary_uv_mode, uv_btnm_map);
        lv_btnmatrix_set_btn_ctrl_all(btnm_secondary_uv_mode, LV_BTNMATRIX_CTRL_CHECKABLE);
        lv_btnmatrix_set_one_checked(btnm_secondary_uv_mode, true);
        lv_obj_set_style_bg_opa(btnm_secondary_uv_mode, LV_OPA_TRANSP, 0);
        lv_obj_set_style_border_width(btnm_secondary_uv_mode, 0, 0);
        lv_obj_set_style_pad_all(btnm_secondary_uv_mode, 3, 0);
        lv_obj_set_style_radius(btnm_secondary_uv_mode, LV_RADIUS_CIRCLE, LV_PART_ITEMS);
        lv_obj_set_style_border_width(btnm_secondary_uv_mode, 0, LV_PART_ITEMS);
        lv_obj_set_style_text_font(btnm_secondary_uv_mode, &montserrat_rus_18, LV_PART_ITEMS);
        
        lv_obj_add_event_cb(btnm_secondary_uv_mode, uv_mode_selector_event_cb, LV_EVENT_VALUE_CHANGED, label_secondary_uv_mode_status); 
    }
    { lv_obj_t* row = lv_obj_create(block_uv_all_stages); lv_obj_remove_style_all(row); lv_obj_set_width(row, lv_pct(100)); lv_obj_set_height(row, LV_SIZE_CONTENT); lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW); lv_obj_set_flex_align(row, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER); label_edit_secondary_uv_time_title = lv_label_create(row); lv_obj_add_style(label_edit_secondary_uv_time_title, &style_my_text_18, 0); ta_edit_secondary_uv = lv_textarea_create(row); lv_textarea_set_one_line(ta_edit_secondary_uv, true); lv_obj_set_size(ta_edit_secondary_uv, 80, TEXT_INPUT_HEIGHT); lv_obj_add_style(ta_edit_secondary_uv, &style_edit_textarea, 0); lv_obj_add_event_cb(ta_edit_secondary_uv, numeric_textarea_focus_event_cb, LV_EVENT_FOCUSED, NULL); lv_obj_add_event_cb(ta_edit_secondary_uv, generic_textarea_defocus_event_cb, LV_EVENT_DEFOCUSED, NULL); }
    
    // --- ТРЕТИЙ ПЕРЕКЛЮЧАТЕЛЬ ---
    { 
        row_uv_tertiary = lv_obj_create(block_uv_all_stages);
        lv_obj_remove_style_all(row_uv_tertiary);
        lv_obj_add_style(row_uv_tertiary, &style_switch_row, 0);
        lv_obj_set_width(row_uv_tertiary, lv_pct(100)); 
        lv_obj_set_height(row_uv_tertiary, LV_SIZE_CONTENT); 
        lv_obj_set_flex_flow(row_uv_tertiary, LV_FLEX_FLOW_ROW); 
        lv_obj_set_flex_align(row_uv_tertiary, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER); 
        
        label_uv_tertiary_title = lv_label_create(row_uv_tertiary); 
        lv_obj_add_style(label_uv_tertiary_title, &style_my_text_18, 0); 
        label_tertiary_uv_mode_status = lv_label_create(row_uv_tertiary); 
        lv_obj_add_style(label_tertiary_uv_mode_status, &style_my_text_18, 0); 
        btnm_tertiary_uv_mode = lv_btnmatrix_create(row_uv_tertiary);
        
        lv_obj_set_size(btnm_tertiary_uv_mode, 150, 40);
        lv_btnmatrix_set_map(btnm_tertiary_uv_mode, uv_btnm_map);
        lv_btnmatrix_set_btn_ctrl_all(btnm_tertiary_uv_mode, LV_BTNMATRIX_CTRL_CHECKABLE);
        lv_btnmatrix_set_one_checked(btnm_tertiary_uv_mode, true);
        lv_obj_set_style_bg_opa(btnm_tertiary_uv_mode, LV_OPA_TRANSP, 0);
        lv_obj_set_style_border_width(btnm_tertiary_uv_mode, 0, 0);
        lv_obj_set_style_pad_all(btnm_tertiary_uv_mode, 3, 0);
        lv_obj_set_style_radius(btnm_tertiary_uv_mode, LV_RADIUS_CIRCLE, LV_PART_ITEMS);
        lv_obj_set_style_border_width(btnm_tertiary_uv_mode, 0, LV_PART_ITEMS);
        lv_obj_set_style_text_font(btnm_tertiary_uv_mode, &montserrat_rus_18, LV_PART_ITEMS);
        
        lv_obj_add_event_cb(btnm_tertiary_uv_mode, uv_mode_selector_event_cb, LV_EVENT_VALUE_CHANGED, label_tertiary_uv_mode_status); 
    }
    { lv_obj_t* row = lv_obj_create(block_uv_all_stages); lv_obj_remove_style_all(row); lv_obj_set_width(row, lv_pct(100)); lv_obj_set_height(row, LV_SIZE_CONTENT); lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW); lv_obj_set_flex_align(row, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER); label_edit_tertiary_uv_time_title = lv_label_create(row); lv_obj_add_style(label_edit_tertiary_uv_time_title, &style_my_text_18, 0); ta_edit_tertiary_uv = lv_textarea_create(row); lv_textarea_set_one_line(ta_edit_tertiary_uv, true); lv_obj_set_size(ta_edit_tertiary_uv, 80, TEXT_INPUT_HEIGHT); lv_obj_add_style(ta_edit_tertiary_uv, &style_edit_textarea, 0); lv_obj_add_event_cb(ta_edit_tertiary_uv, numeric_textarea_focus_event_cb, LV_EVENT_FOCUSED, NULL); lv_obj_add_event_cb(ta_edit_tertiary_uv, generic_textarea_defocus_event_cb, LV_EVENT_DEFOCUSED, NULL); }

    // --- ПРАВАЯ КОЛОНКА И ФУТЕР (без изменений) ---
    header_poly_params = lv_label_create(right_column);
    lv_obj_add_style(header_poly_params, &style_my_text_18, 0);
    lv_obj_add_style(header_poly_params, &style_column_header, 0);
    block_gases = lv_obj_create(right_column);
    lv_obj_add_style(block_gases, &style_param_block, 0);
    lv_obj_set_flex_flow(block_gases, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(block_gases, LV_FLEX_ALIGN_SPACE_EVENLY, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_width(block_gases, lv_pct(100));
    lv_obj_set_flex_grow(block_gases, 1);
    { lv_obj_t* row = lv_obj_create(block_gases); lv_obj_remove_style_all(row); lv_obj_set_size(row, lv_pct(100), LV_SIZE_CONTENT); lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW); lv_obj_set_flex_align(row, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER); label_edit_nitrogen_title = lv_label_create(row); lv_obj_add_style(label_edit_nitrogen_title, &style_my_text_18, 0); sw_edit_nitrogen = lv_switch_create(row); lv_obj_add_event_cb(sw_edit_nitrogen, profile_switch_value_changed_event_cb, LV_EVENT_VALUE_CHANGED, (void*)"nitrogen_profile"); }
    nitrogen_elements_container = lv_obj_create(block_gases);
    lv_obj_remove_style_all(nitrogen_elements_container);
    lv_obj_set_size(nitrogen_elements_container, lv_pct(100), LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(nitrogen_elements_container, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(nitrogen_elements_container, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    label_edit_nitrogen_target_title = lv_label_create(nitrogen_elements_container);
    lv_obj_add_style(label_edit_nitrogen_target_title, &style_my_text_18, 0);
    ta_edit_nitrogen_target = lv_textarea_create(nitrogen_elements_container);
    lv_textarea_set_one_line(ta_edit_nitrogen_target, true);
    lv_obj_set_size(ta_edit_nitrogen_target, 80, TEXT_INPUT_HEIGHT);
    lv_obj_add_style(ta_edit_nitrogen_target, &style_edit_textarea, 0);
    lv_obj_add_event_cb(ta_edit_nitrogen_target, numeric_textarea_focus_event_cb, LV_EVENT_FOCUSED, NULL);
    lv_obj_add_event_cb(ta_edit_nitrogen_target, generic_textarea_defocus_event_cb, LV_EVENT_DEFOCUSED, NULL);
    { lv_obj_t* row = lv_obj_create(block_gases); lv_obj_remove_style_all(row); lv_obj_set_size(row, lv_pct(100), LV_SIZE_CONTENT); lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW); lv_obj_set_flex_align(row, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER); label_edit_cooling_title = lv_label_create(row); lv_obj_add_style(label_edit_cooling_title, &style_my_text_18, 0); sw_edit_chamber_cooling = lv_switch_create(row); lv_obj_add_event_cb(sw_edit_chamber_cooling, profile_switch_value_changed_event_cb, LV_EVENT_VALUE_CHANGED, (void*)"cooling_profile"); }
    block_thermal = lv_obj_create(right_column);
    lv_obj_add_style(block_thermal, &style_param_block, 0);
    lv_obj_set_flex_flow(block_thermal, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(block_thermal, LV_FLEX_ALIGN_SPACE_EVENLY, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_width(block_thermal, lv_pct(100));
    lv_obj_set_flex_grow(block_thermal, 1);
    { lv_obj_t* row = lv_obj_create(block_thermal); lv_obj_remove_style_all(row); lv_obj_set_size(row, lv_pct(100), LV_SIZE_CONTENT); lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW); lv_obj_set_flex_align(row, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER); label_edit_thermal_chamber_title = lv_label_create(row); lv_obj_add_style(label_edit_thermal_chamber_title, &style_my_text_18, 0); sw_edit_thermal_chamber_enable = lv_switch_create(row); lv_obj_add_event_cb(sw_edit_thermal_chamber_enable, thermal_chamber_enable_switch_event_cb, LV_EVENT_VALUE_CHANGED, NULL); }
    thermal_elements_container = lv_obj_create(block_thermal);
    lv_obj_remove_style_all(thermal_elements_container);
    lv_obj_set_width(thermal_elements_container, lv_pct(100));
    lv_obj_set_height(thermal_elements_container, LV_SIZE_CONTENT); 
    lv_obj_set_flex_flow(thermal_elements_container, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_gap(thermal_elements_container, 10, 0);
    lv_obj_set_flex_align(thermal_elements_container, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);
    { lv_obj_t* row = lv_obj_create(thermal_elements_container); lv_obj_remove_style_all(row); lv_obj_set_size(row, lv_pct(100), LV_SIZE_CONTENT); lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW); lv_obj_set_flex_align(row, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER); label_edit_thermal_temp_title = lv_label_create(row); lv_obj_add_style(label_edit_thermal_temp_title, &style_my_text_18, 0); ta_edit_thermal_temp = lv_textarea_create(row); lv_textarea_set_one_line(ta_edit_thermal_temp, true); lv_obj_set_size(ta_edit_thermal_temp, 80, TEXT_INPUT_HEIGHT); lv_obj_add_style(ta_edit_thermal_temp, &style_edit_textarea, 0); lv_obj_add_event_cb(ta_edit_thermal_temp, numeric_textarea_focus_event_cb, LV_EVENT_FOCUSED, NULL); lv_obj_add_event_cb(ta_edit_thermal_temp, generic_textarea_defocus_event_cb, LV_EVENT_DEFOCUSED, NULL); }
    { lv_obj_t* row = lv_obj_create(thermal_elements_container); lv_obj_remove_style_all(row); lv_obj_set_size(row, lv_pct(100), LV_SIZE_CONTENT); lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW); lv_obj_set_flex_align(row, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER); label_edit_heat_hold_title = lv_label_create(row); lv_obj_add_style(label_edit_heat_hold_title, &style_my_text_18, 0); ta_edit_heat_hold = lv_textarea_create(row); lv_textarea_set_one_line(ta_edit_heat_hold, true); lv_obj_set_size(ta_edit_heat_hold, 80, TEXT_INPUT_HEIGHT); lv_obj_add_style(ta_edit_heat_hold, &style_edit_textarea, 0); lv_obj_add_event_cb(ta_edit_heat_hold, numeric_textarea_focus_event_cb, LV_EVENT_FOCUSED, NULL); lv_obj_add_event_cb(ta_edit_heat_hold, generic_textarea_defocus_event_cb, LV_EVENT_DEFOCUSED, NULL); }
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

    // Стандартные элементы
    label_process_status_title = lv_label_create(main_process_content_area);
    lv_obj_add_style(label_process_status_title, &style_my_text_22, 0);
    lv_label_set_text(label_process_status_title, "Initializing Process...");
    lv_obj_set_width(label_process_status_title, LV_PCT(100));
    lv_obj_set_style_text_align(label_process_status_title, LV_TEXT_ALIGN_CENTER, 0);
    
    spinner_process_execution = lv_spinner_create(main_process_content_area, 1000, 60);
    lv_obj_set_size(spinner_process_execution, 100, 100);
    
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
static void build_secret_game_screen(lv_obj_t* parent_screen) {
    Serial.println("Building secret_game_screen UI...");
    lv_obj_set_style_bg_color(parent_screen, lv_color_black(), LV_STATE_DEFAULT); // Пусть игра будет на черном фоне
    lv_obj_set_style_bg_opa(parent_screen, LV_OPA_COVER, LV_STATE_DEFAULT);
    lv_obj_set_flex_flow(parent_screen, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(parent_screen, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER); 
    lv_obj_set_style_pad_all(parent_screen, 20, 0);

    lv_obj_t* game_title = lv_label_create(parent_screen);
    lv_label_set_text(game_title, "SECRET GAME ZONE!");
    lv_obj_set_style_text_color(game_title, lv_color_white(), 0);
    lv_obj_set_style_text_font(game_title, &lv_font_montserrat_24, 0);

    lv_obj_t* game_placeholder_label = lv_label_create(parent_screen);
    lv_label_set_text(game_placeholder_label, "Game content will be here.\n\n(e.g., Snake, Pong, Clicker...)");
    lv_obj_set_style_text_color(game_placeholder_label, lv_color_hex(0x00ff00), 0); // Зеленый текст
    lv_obj_set_style_text_align(game_placeholder_label, LV_TEXT_ALIGN_CENTER, 0);

    // Кнопка "Назад" из игры
    lv_obj_t* btn_back_from_game = lv_btn_create(parent_screen);
    lv_obj_add_event_cb(btn_back_from_game, secret_game_back_event_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t* label_btn_back_game = lv_label_create(btn_back_from_game);
    lv_label_set_text(label_btn_back_game, "Back to Settings");
}
static void secret_game_back_event_cb(lv_event_t* e) {
    if (lv_event_get_code(e) == LV_EVENT_CLICKED) {
        Serial.println("Back from secret game to settings screen.");
        if (screen_settings) {

            if (sw_settings_global_nitrogen_enabled) {
                if (current_global_settings.nitrogen_system_enabled) lv_obj_add_state(sw_settings_global_nitrogen_enabled, LV_STATE_CHECKED);
                else lv_obj_clear_state(sw_settings_global_nitrogen_enabled, LV_STATE_CHECKED);
            }
            if (sw_settings_global_air_enabled) {
                if (current_global_settings.compressed_air_system_enabled) lv_obj_add_state(sw_settings_global_air_enabled, LV_STATE_CHECKED);
                else lv_obj_clear_state(sw_settings_global_air_enabled, LV_STATE_CHECKED);
            }
            lv_scr_load(screen_settings);
        }
    }
}

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
    
    Serial.println("Step 3.1: UI styles initialized.");

    // 4. НЕМЕДЛЕННО СОЗДАЕМ И ЗАГРУЖАЕМ ЗАСТАВКУ (БЕЗ ЗАПУСКА ТАЙМЕРА!)
    if (ENABLE_SPLASH_SCREEN) {
        Serial.println("Step 4: Building and loading SPLASH SCREEN...");
        screen_splash = lv_obj_create(NULL);
        build_splash_screen(screen_splash); // Вызываем нашу простую функцию
        lv_scr_load(screen_splash);
        is_on_splash_screen = true;
    } else {
        is_on_splash_screen = false;
        Serial.println("Step 4: Splash screen is DISABLED. Skipping.");
    }
    
    // 5. ИНИЦИАЛИЗАЦИЯ SD-КАРТЫ И НАСТРОЕК
    delay(100); // Небольшая пауза
    sd_card_initialized = initializeSDCard();
    delay(100);
    Serial.println("Loading global settings...");
    loadGlobalSettings();
    loadLaboratorySettings();

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


    // Строим все остальные экраны
    build_main_app_screen(screen_main_app);
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

    Serial.println("Building universal info dialogs...");
    build_info_dialog(lv_layer_top());
    build_choice_dialog(lv_layer_top());
    build_input_shield();

    // 7. ПОДГОТОВКА ДАННЫХ ДЛЯ ГЛАВНОГО ЭКРАНА
    if (sd_card_initialized) {
        Serial.println("SD card ready. Caching all profiles and populating UI in background...");
        current_profile_next_id = scanAndCacheAllProfiles(SD, all_profile_entries_cache);
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
            lv_scr_load(screen_service_lock);
        } else {
            Serial.println("Device is OK. Loading main app screen...");
            lv_scr_load(screen_main_app);
        }
    }

    // Инициализация UART последним шагом, чтобы не мешать другим устройствам
    Serial.println("Step 8: Finalizing setup with UART initialization...");
    MySerial1.begin(115200, SERIAL_8N1, RX1_PIN, TX1_PIN);
    Serial.println("UART to receiver is now active on pins RX=15, TX=44.");

    // 9. ЗАПУСКАЕМ ТАЙМЕР ЗАСТАВКИ В САМОМ КОНЦЕ
    if (ENABLE_SPLASH_SCREEN) {
        splash_screen_start_time = millis();
        Serial.println("All systems GO. Splash screen timer started.");
    }

    Serial.println("--- System Setup Complete ---");
}

void loop() {

    // --- БЛОК 2: Проверка касания и пробуждение ---
    lv_indev_t* indev = lv_indev_get_act();
    if (indev && indev->proc.state == LV_INDEV_STATE_PRESSED) {

        // Если экран был выключен - это касание для пробуждения
        if (!is_screen_on) {
            ch422g->digitalWrite(LCD_BL, HIGH);
            is_screen_on = true;

            // --- СТАЛО ---
            // Щит УЖЕ был активен. Теперь, когда экран проснулся, немедленно его деактивируем.
            if (input_shield) {
                lv_obj_add_flag(input_shield, LV_OBJ_FLAG_HIDDEN);
                lv_obj_clear_flag(input_shield, LV_OBJ_FLAG_CLICKABLE);
            }
            Serial.println("Screen WAKE UP. Input shield immediately deactivated.");
        }

        // Обновляем таймер неактивности при любом касании
        last_interaction_time = millis();
    }

    // --- БЛОК 3: Проверка неактивности и засыпание ---
    if (is_screen_on && (millis() - last_interaction_time > get_current_screen_timeout_ms())) {
        ch422g->digitalWrite(LCD_BL, LOW);
        is_screen_on = false;

        // --- СТАЛО ---
        // Немедленно активируем "щит" для поглощения следующего касания
        if (input_shield) {
            lv_obj_clear_flag(input_shield, LV_OBJ_FLAG_HIDDEN);
            lv_obj_add_flag(input_shield, LV_OBJ_FLAG_CLICKABLE);
            lv_obj_move_foreground(input_shield); // На всякий случай, чтобы он был точно сверху
        }
        Serial.println("Screen OFF due to inactivity. Input shield is now active.");
    }


    if (ENABLE_SPLASH_SCREEN && is_on_splash_screen && (millis() - splash_screen_start_time >= 5000)) {
        is_on_splash_screen = false; 
        if (current_global_settings.is_heater_error) {
            enter_service_lock_mode(
                "Critical Error!\nHeating element failure.\nDevice is locked.",
                "Критическая ошибка!\nОтказ нагревательного элемента.\nУстройство заблокировано."
            );
        } else {
            lv_scr_load(screen_main_app);
        }
    }
    
    if (needs_list_refresh && lv_scr_act() == screen_main_app) {
        needs_list_refresh = false; 
        lvgl_port_lock(-1);
        displayProfileListPage();
        lvgl_port_unlock();
    }
    

    // --- ИСПРАВЛЕННАЯ ЛОГИКА ОБРАБОТКИ ПРОЦЕССА ---
    if (main_process_running && MySerial1.available() > 0) {
        String response = MySerial1.readStringUntil('\n');
        response.trim();

        if (response.length() > 0) {
            Serial.printf("Data from controller: '%s'\n", response.c_str());

            // --- Шаг 1: Первым делом ищем КРИТИЧЕСКИЕ ошибки, которые требуют немедленной блокировки ---
            if (response == "FATAL_ERROR:HEATER_FAILURE") {
                enter_service_lock_mode(
                    "Critical Error!\nHeating element failure.\nDevice is locked.",
                    "Критическая ошибка!\nОтказ нагревательного элемента.\nУстройство заблокировано."
                );
                return; // Сразу выходим из дальнейшей обработки
            }

            // --- Шаг 2: Проверяем, не пришло ли событие, требующее действия от пользователя ---
            if (response == "EVENT:NITROGEN_ERROR_CHOICE_REQUIRED") {
                nitrogen_error_dialog_pending = true;
                // Ничего больше не делаем, выйдем и обработаем флаг в основном цикле
            } 
            // --- Шаг 3: Если это не критическая ошибка и не диалог, обрабатываем обычные статусы процесса ---
            else if (main_process_running) {
                
                // --- Шаг 3.1: Обновляем наше ГЛОБАЛЬНОЕ СОСТОЯНИЕ на основе сообщения ---
                if (response.startsWith("EVENT:START_COUNTDOWN:")) {
                    current_process_stage = "REPAIR_COUNTDOWN";
                } else if (response == "STATUS:Repair UV is ON") {
                    current_process_stage = "REPAIR_UV_ON";
                } 
                else if (response.startsWith("PROCESS_") || response.startsWith("ERROR:") || response.startsWith("ACK:")) {
                    current_process_stage = ""; 
                }

                // --- Шаг 3.2: Обновляем весь UI на экране процесса на основе ТЕКУЩЕГО состояния ---
                if (lv_scr_act() == screen_process_execution) {
                    lvgl_port_lock(-1);

                    // Логика для зеленого ИНДИКАТОРА
                    bool show_repair_indicator = (is_lab_mode_running &&
                                            screen_to_return_after_process == screen_lab_repair &&
                                            (current_process_stage == "REPAIR_COUNTDOWN" || current_process_stage == "REPAIR_UV_ON"));
                    
                    if (show_repair_indicator) {
                        if (lv_obj_has_flag(repair_mode_indicator_obj, LV_OBJ_FLAG_HIDDEN)) {
                            lv_obj_clear_flag(repair_mode_indicator_obj, LV_OBJ_FLAG_HIDDEN);
                        }
                    } else {
                        if (!lv_obj_has_flag(repair_mode_indicator_obj, LV_OBJ_FLAG_HIDDEN)) {
                            lv_obj_add_flag(repair_mode_indicator_obj, LV_OBJ_FLAG_HIDDEN);
                        }
                    }
                    
                    // Логика для остального UI (заголовки, телеметрия)
                    StaticJsonDocument<256> doc;
                    DeserializationError error = deserializeJson(doc, response);

                    if (error) { // Не JSON
                        if (response.startsWith("EVENT:START_COUNTDOWN:")) {
                            lv_label_set_text(label_process_status_title, tr("Bring the model to the indicator"));
                        } else {
                            lv_label_set_text(label_process_status_title, translateSystemStatus(response.c_str()));
                        }
                    } else { // JSON
                        const char* type = doc["type"];
                        if (type && strcmp(type, "TELEMETRY") == 0) {
                            float temp = doc["temp"];
                            float o2 = doc["o2"];
                            int timer_rem = doc["timer_rem"];
                            update_telemetry_display(temp, o2, timer_rem);
                        }
                    }
                    
                    lvgl_port_unlock();
                }

                // --- Шаг 3.3: Логика завершения всего процесса ---
                if (response == "PROCESS_COMPLETE" || response.startsWith("ERROR:") || response == "ACK:STOP_COMMAND_RECEIVED" || response == "PROCESS_COMPLETE_WITH_COOLING_WARNING") {
                    main_process_running = false; 
                    is_lab_mode_running = false;
                    
                    if (lv_scr_act() == screen_process_execution) {
                        lvgl_port_lock(-1);
                        if(spinner_process_execution) lv_obj_add_flag(spinner_process_execution, LV_OBJ_FLAG_HIDDEN);
                        if(repair_mode_indicator_obj) lv_obj_add_flag(repair_mode_indicator_obj, LV_OBJ_FLAG_HIDDEN);
                        
                        // Устанавливаем основной статус (заголовок)
                        lv_label_set_text(label_process_status_title, translateSystemStatus(response.c_str()));
                        
                        if (response == "PROCESS_COMPLETE_WITH_COOLING_WARNING") {
                            // Показываем детальное предупреждение
                            lv_label_set_text(label_process_status_detail, tr("Warning: The compressed air supply may have failed during the process. Please check the system in Settings -> Test."));
                        } else {
                            // В остальных случаях просто чистим поле
                            lv_label_set_text(label_process_status_detail, "");
                        }
                        
                        // Обновляем кнопку
                        if(label_btn_process_cancel) lv_label_set_text(label_btn_process_cancel, tr("Back"));
                        lvgl_port_unlock();
                    }
                }
            }
        }
    }
    // --- БЛОК 5: Отложенная обработка UI событий ---
    // Выполняется в "чистом" состоянии цикла, а не внутри обработки UART
    if (nitrogen_error_dialog_pending) {
        nitrogen_error_dialog_pending = false; // Сбрасываем флаг, чтобы не показывать окно 100 раз
        show_choice_dialog(
            "Nitrogen System Error", 
            "The oxygen level is not decreasing. Continue the process without nitrogen?"
        );
    }
}