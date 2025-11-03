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

const bool ENABLE_SPLASH_SCREEN = false;
#define LVGL_HEAP_SIZE (96 * 1024)
#define FILE_CONTENT_BUFFER_SIZE 1024
const char* settings_file_path = "/settings.txt";
const int PROFILES_PER_PAGE = 12;

LV_FONT_DECLARE(montserrat_rus_16);
LV_FONT_DECLARE(montserrat_rus_18);
LV_FONT_DECLARE(montserrat_rus_22);


//==========================================================================
// Data Structures
//==========================================================================
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
#define RX1_PIN 15 
#define TX1_PIN 44 
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

// --- Application State & Data ---
bool sd_card_initialized = false;
bool main_process_running = false;
bool needs_list_refresh = false;
bool is_on_splash_screen = false;
int current_profile_next_id = 1;
int current_profile_list_page = 0;
int total_profile_pages = 0;
GlobalSettingsData current_global_settings;
ProfileData current_active_profile_data;
std::vector<ProfileCacheEntry> all_profile_entries_cache;
std::vector<String> service_keys;
char file_content_buffer[FILE_CONTENT_BUFFER_SIZE];
char current_selected_profile_filename[64];
char decision_text[50];
char full_status_text[100];
bool heater_decision = false;
bool cooling_decision = false;

// --- Timers & Process Control ---
unsigned long splash_screen_start_time, hold_timer_start, heater_error_timer, air_error_timer;
unsigned long nitrogen_error_timer, primary_uv_timer_start, secondary_uv_timer_start;
float temp_at_heater_error_check_start = 0.0f;
float temp_at_air_error_check_start = 0.0f;
float o2_at_error_check_start = 0.0f;
volatile int choice_dialog_result = 0;

//-------------------------------------------------
// LVGL UI Object Pointers
//-------------------------------------------------
// --- Screens & Main Containers ---
lv_obj_t * screen_splash, *screen_main_app, *screen_profile_details, *screen_profile_edit;
lv_obj_t * screen_settings, *screen_process_execution, *screen_service_lock, *screen_secret_game;
lv_obj_t * main_screen_content_container;
lv_obj_t* screen_help;

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

// --- Main Screen Widgets ---
lv_obj_t * list_header_label_main, *list_profiles_main, *ta_profile_input_main;
lv_obj_t * label_status_msg_main, *ta_dummy_for_new_profile;
lv_obj_t * btn_profiles_prev, *btn_profiles_next, *btn_secret_trigger;
lv_obj_t* label_btn_add_main;
lv_obj_t* label_btn_lab_main;
lv_obj_t* label_btn_settings_main;

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
lv_obj_t* label_btn_process_cancel;

// --- Settings Screen Widgets ---
lv_obj_t * sw_settings_global_nitrogen_enabled, *sw_settings_global_air_enabled;
lv_obj_t * lang_toggle_box, *theme_toggle_box;
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

// --- UI Building ---
static void build_splash_screen(lv_obj_t* parent_screen);
static void build_main_app_screen(lv_obj_t* parent_screen);
static void build_profile_details_screen(lv_obj_t* parent_screen);
static void build_profile_edit_screen(lv_obj_t* parent_screen);
static void build_settings_screen(lv_obj_t* parent_screen);
static void build_process_execution_screen(lv_obj_t* parent_screen);
static void build_service_lock_screen(lv_obj_t* parent_screen);
static void build_secret_game_screen(lv_obj_t* parent_screen);
static void build_confirm_delete_dialog(lv_obj_t* parent_for_dialog);
static void build_info_dialog(lv_obj_t* parent_layer);
static void build_choice_dialog(lv_obj_t* parent_layer);
static void build_help_screen(lv_obj_t* parent_screen);

// --- Custom Widget Functions ---
static void uv_mode_selector_event_cb(lv_event_t * e);
static uint16_t get_checked_btnmatrix_id(lv_obj_t* btnm);

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
static void service_lock_screen_event_cb(lv_event_t* e);
static void service_code_keyboard_event_cb(lv_event_t* e);
static void modal_input_keyboard_event_cb(lv_event_t* e);
static void modal_input_overlay_click_event_cb(lv_event_t* e);
static void thermal_temp_slider_event_cb(lv_event_t * e);
static void thermal_chamber_enable_switch_event_cb(lv_event_t * e);
static void profile_switch_value_changed_event_cb(lv_event_t * e);
static void numeric_textarea_focus_event_cb(lv_event_t * e);
static void numeric_textarea_defocus_event_cb(lv_event_t * e);
static void alpha_textarea_focus_event_cb(lv_event_t* e);
static void profile_list_prev_btn_event_cb(lv_event_t* e);
static void profile_list_next_btn_event_cb(lv_event_t* e);
static void secret_button_event_cb(lv_event_t* e);
static void secret_game_back_event_cb(lv_event_t* e);
static void help_button_event_cb(lv_event_t* e);

// --- Core Logic, File System & Helpers ---
void displayProfileListPage();
void enter_service_lock_mode(const char* message);
bool handle_save_new_profile_logic(const char* profile_input_name);
int scanAndCacheAllProfiles(fs::FS &fs_ref, std::vector<ProfileCacheEntry>& cache_vector);
bool readFileContentToBuffer_ino(fs::FS &fs_ref, const char * path, char* buffer, size_t buffer_size);
bool parseProfileJson(const char* jsonString, ProfileData& profile);
bool serializeProfileJson(const ProfileData& profile, char* outputBuffer, size_t bufferSize);
static void show_modal_input(lv_obj_t* target_ta, lv_keyboard_mode_t kb_mode);
static void show_info_dialog(const char* title, const char* message_text);
static void show_choice_dialog(const char* title, const char* message_text);
static void create_settings_row(lv_obj_t* parent, lv_obj_t** p_label, lv_obj_t** p_switch, const char* user_data);
static void create_custom_toggle(lv_obj_t* parent, lv_obj_t** p_label, lv_obj_t** p_toggle_box, lv_obj_t** p_lbl1, lv_obj_t** p_lbl2);
static void update_custom_toggle_ui(lv_obj_t* toggle_box, int active_index);
int roundToStep(int value, int step);
void validate_numeric_input(lv_event_t * e, int min_val, int max_val);
const char* translateSystemStatus(const char* status_msg);


// Раздел 1: Вспомогательные функции 
// ==========================================================================
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
    if (strcmp(status_msg, "ERROR:DOOR_IS_OPEN") == 0) return "ОШИБКА: Дверь открыта!";
    if (strcmp(status_msg, "FATAL_ERROR:HEATER_FAILURE") == 0) return "КРИТИЧЕСКАЯ ОШИБКА: Нагреватель";
    if (strcmp(status_msg, "WARN:COOLING_SKIPPED") == 0) return "ВНИМАНИЕ: Охлаждение пропущено";
    if (strcmp(status_msg, "WARN:NITROGEN_SKIPPED") == 0) return "ВНИМАНИЕ: Продувка азотом пропущена";
    
    // Если перевод не найден, возвращаем оригинальное сообщение
    return status_msg;
}
static void update_custom_toggle_ui(lv_obj_t* toggle_box, int active_index) {
    if (!toggle_box) return;

    lv_obj_t* btn1 = lv_obj_get_child(toggle_box, 0);
    lv_obj_t* btn2 = lv_obj_get_child(toggle_box, 1);
    if (!btn1 || !btn2) return;

    if (active_index == 0) { // Первая опция активна
        lv_obj_set_style_bg_color(btn1, lv_palette_main(LV_PALETTE_BLUE), 0);
        lv_obj_set_style_text_color(lv_obj_get_child(btn1, 0), lv_color_white(), 0);

        lv_obj_set_style_bg_color(btn2, lv_palette_lighten(LV_PALETTE_GREY, 2), 0);
        lv_obj_set_style_text_color(lv_obj_get_child(btn2, 0), lv_color_black(), 0);
    } else { // Вторая опция активна
        lv_obj_set_style_bg_color(btn1, lv_palette_lighten(LV_PALETTE_GREY, 2), 0);
        lv_obj_set_style_text_color(lv_obj_get_child(btn1, 0), lv_color_black(), 0);

        lv_obj_set_style_bg_color(btn2, lv_palette_main(LV_PALETTE_BLUE), 0);
        lv_obj_set_style_text_color(lv_obj_get_child(btn2, 0), lv_color_white(), 0);
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
    ProfileData temp_profile_data_scan;
    char temp_file_buffer_scan[FILE_CONTENT_BUFFER_SIZE];

    while (entry) {
        String entryFilenameOnly = entry.name();
        if (entryFilenameOnly.startsWith("/")) {
            entryFilenameOnly = entryFilenameOnly.substring(1);
        }
        if (!entry.isDirectory() && entryFilenameOnly.startsWith("profile_") && entryFilenameOnly.endsWith(".txt")) {
            
            // <<<--- НАЧАЛО ИЗМЕНЕНИЙ ---<<<
            
            ProfileCacheEntry new_entry; // Создаем экземпляр нашей новой структуры
            String profile_display_name_scan;

            // Копируем имя файла в структуру
            strncpy(new_entry.filename, entryFilenameOnly.c_str(), sizeof(new_entry.filename) - 1);
            new_entry.filename[sizeof(new_entry.filename) - 1] = '\0';

            // Получаем отображаемое имя из JSON, как и раньше
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

            // Копируем отображаемое имя в структуру
            strncpy(new_entry.display_name, profile_display_name_scan.c_str(), sizeof(new_entry.display_name) - 1);
            new_entry.display_name[sizeof(new_entry.display_name) - 1] = '\0';
            
            // Добавляем готовую структуру в вектор
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

    // Остальная часть функции без изменений
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
    Serial.println("-> Entering loadGlobalSettings...");
    lvgl_port_lock(-1);
    // Устанавливаем значения по умолчанию на случай, если файл не найден или ошибка
    current_global_settings.is_first_run = true;
    current_global_settings.is_heater_error = false;
    current_global_settings.nitrogen_system_enabled = false;
    current_global_settings.compressed_air_system_enabled = false;
    current_global_settings.language = 0; // 0 = ENG
    current_global_settings.theme = 0;    // 0 = Light

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
int roundToStep(int value, int step) { if (step == 0) return value; return ((value + step / 2) / step) * step; }
void validate_numeric_input(lv_event_t * e, int min_val, int max_val) {
    lv_obj_t * ta = lv_event_get_target(e); const char* txt = lv_textarea_get_text(ta); int val = atoi(txt); 
    bool changed = false; if (val < min_val) { val = min_val; changed = true; } else if (val > max_val) { val = max_val; changed = true; }
    if (changed) { char buf[10]; snprintf(buf, sizeof(buf), "%d", val); lv_textarea_set_text(ta, buf); Serial.printf("Input validated and corrected to: %d\n", val); }
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
    keys_array.add("86LH"); keys_array.add("Y5V4"); keys_array.add("83QF"); keys_array.add("7DCJ");
    keys_array.add("1KHD"); keys_array.add("8B7Z"); keys_array.add("81IC"); keys_array.add("NVTJ");
    keys_array.add("FITV"); keys_array.add("KBB4"); keys_array.add("PCY3"); keys_array.add("8WR5");
    keys_array.add("3M27"); keys_array.add("FRRD"); keys_array.add("43LZ"); keys_array.add("ZH34");
    keys_array.add("5GV7"); keys_array.add("K5UV"); keys_array.add("PPWN"); keys_array.add("SAW1");
    keys_array.add("SQYK"); keys_array.add("BFPL"); keys_array.add("RXWR"); keys_array.add("4HKX");
    keys_array.add("GJZ3"); keys_array.add("PZ56"); keys_array.add("2SM8"); keys_array.add("4T84");
    keys_array.add("KD4Y"); keys_array.add("8VNW"); keys_array.add("M8X5"); keys_array.add("X8WM");
    keys_array.add("RBSQ"); keys_array.add("BRTC"); keys_array.add("W4WX"); keys_array.add("FNER");
    keys_array.add("7L2H"); keys_array.add("LKEM"); keys_array.add("YE2F"); keys_array.add("CPT1");
    keys_array.add("7FQF"); keys_array.add("UZ85"); keys_array.add("EZ4N"); keys_array.add("U32U");
    keys_array.add("YCPK"); keys_array.add("15X4"); keys_array.add("E52A"); keys_array.add("3NKK");
    keys_array.add("L4JQ"); keys_array.add("CBJC"); keys_array.add("GH12"); keys_array.add("GRZK");
    keys_array.add("5EZ1"); keys_array.add("AN7I"); keys_array.add("IEEZ"); keys_array.add("66KU");
    keys_array.add("1ZQ3"); keys_array.add("1EYP"); keys_array.add("BNZ6"); keys_array.add("5CV5");
    keys_array.add("4QFV"); keys_array.add("MJYF"); keys_array.add("WWN5"); keys_array.add("D5KP");
    keys_array.add("XE52"); keys_array.add("Z65C"); keys_array.add("AB6P"); keys_array.add("UVBD");
    keys_array.add("W8CM"); keys_array.add("3GDV"); keys_array.add("1WQB"); keys_array.add("472Q");
    keys_array.add("4IZA"); keys_array.add("UNNA"); keys_array.add("D2PZ"); keys_array.add("Z2LP");
    keys_array.add("TPWS"); keys_array.add("TIJR"); keys_array.add("4H6H"); keys_array.add("Z4U1");
    keys_array.add("Z8CE"); keys_array.add("KN2N"); keys_array.add("3GAM"); keys_array.add("WLUL");
    keys_array.add("NMGZ"); keys_array.add("LM1C"); keys_array.add("Z61J"); keys_array.add("DUS5");
    keys_array.add("YDGG"); keys_array.add("ZPHE"); keys_array.add("L7XT"); keys_array.add("JT3W");
    keys_array.add("EQUC"); keys_array.add("ATX5"); keys_array.add("GFVP"); keys_array.add("2JWW");
    keys_array.add("CEFV"); keys_array.add("Q3J6"); keys_array.add("U7MR"); keys_array.add("EFGE");

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

void enter_service_lock_mode(const char* message) {
    Serial.printf("ENTERING SERVICE LOCK MODE. Reason: %s\n", message);
    
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
    lv_label_set_text(label_service_lock_msg, message);
    lv_scr_load(screen_service_lock);
    lvgl_port_unlock();
}

static void laboratory_mode_btn_event_cb(lv_event_t * e) {
    if (lv_event_get_code(e) == LV_EVENT_CLICKED) {
        Serial.println("Laboratory Mode button clicked.");
        // Позже здесь будет логика перехода на экран лабораторного режима
        // Например: lv_scr_load(screen_laboratory);

        // А пока просто покажем информационное сообщение
        show_info_dialog("Info", "Laboratory Mode is not yet implemented.");
    }
}


static void thermal_chamber_enable_switch_event_cb(lv_event_t * e) {
    lv_obj_t* sw = lv_event_get_target(e);
    bool is_enabled = lv_obj_has_state(sw, LV_STATE_CHECKED);

    // Управляем видимостью ЕДИНОГО контейнера с настройками
    if (is_enabled) {
        Serial.println("Thermal chamber ENABLED by user in edit screen.");
        if (thermal_elements_container) {
            lv_obj_clear_flag(thermal_elements_container, LV_OBJ_FLAG_HIDDEN);
        }
    } else {
        Serial.println("Thermal chamber DISABLED by user in edit screen.");
        if (thermal_elements_container) {
            lv_obj_add_flag(thermal_elements_container, LV_OBJ_FLAG_HIDDEN);
        }
    }
}

static void profile_detail_start_event_cb(lv_event_t * e) {
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;

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

    // 5. Сериализуем JSON в строку и отправляем ПО ЧАСТЯМ
    String output;
    serializeJson(doc, output);
    
    // Отправляем по одному символу с небольшой задержкой
    for (int i = 0; i < output.length(); i++) {
        MySerial1.print(output[i]);
        delayMicroseconds(100); // Небольшая пауза, чтобы не перегрузить буфер приемника
    }
    MySerial1.println(); // Отправляем символ новой строки для завершения команды

    Serial.println("Full process command sent (chunked):");
    Serial.println(output);

    // 6. Устанавливаем флаг, что процесс запущен.
    main_process_running = true;
}

// Раздел 2: Функции обновления UI и обработчики событий LVGL
// ==========================================================================
static void uv_mode_selector_event_cb(lv_event_t * e) {
    lv_obj_t* btnm = lv_event_get_target(e);
    lv_obj_t* label = (lv_obj_t*)lv_event_get_user_data(e);
    uint32_t id = lv_btnmatrix_get_selected_btn(btnm);

    // Просто вызываем нашу универсальную функцию для обновления UI
    update_uv_mode_selector_ui(btnm, label, id);
}

static void help_blink_timer_cb(lv_timer_t* timer) {
    if (!btn_help_section || !lv_obj_is_valid(btn_help_section)) return;

    // Получаем текущее "состояние" из user_data
    uint32_t state = (uint32_t)timer->user_data;

    if (state == 0) { // Фаза "ВКЛ"
        lv_obj_set_style_bg_color(btn_help_section, lv_palette_main(LV_PALETTE_YELLOW), 0);
        timer->period = 1000; // Следующий вызов через 1 секунду
        timer->user_data = (void*)1; // Следующее состояние - "ВЫКЛ"
    } else { // Фаза "ВЫКЛ"
        lv_obj_set_style_bg_color(btn_help_section, lv_palette_lighten(LV_PALETTE_GREY, 2), 0); // Возвращаем стандартный цвет фона
        timer->period = 2000; // Следующий вызов через 2 секунды
        timer->user_data = (void*)0; // Следующее состояние - "ВКЛ"
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
                    "5. Мерцания УФ: Первый этап облучения с мерцанием. 'Тип 1' и 'Тип 2' - разные длины волн. Можно выбрать частоту мерцаний (1-5).\n\n"
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

    Serial.printf("Displaying page %d: profiles from index %d to %d (exclusive)\n", current_profile_list_page + 1, start_index, end_index);

    for (int i = start_index; i < end_index; ++i) {
        const ProfileCacheEntry& entry = all_profile_entries_cache[i];
        
        lv_obj_t* tile = lv_obj_create(list_profiles_main);
        lv_obj_set_size(tile, 110, 175);
        
        lv_obj_set_style_bg_color(tile, lv_color_white(), 0);
        lv_obj_set_style_border_width(tile, 1, 0);
        lv_obj_set_style_border_color(tile, lv_palette_main(LV_PALETTE_GREY), 0);
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

// Меняем void на bool
bool handle_save_new_profile_logic(const char* profile_input_name) {
    // Проверки
    if (strlen(profile_input_name) == 0) {
        show_info_dialog("Error", "Profile name cannot be empty!");
        return false; // Возвращаем неудачу
    }
    if (strlen(profile_input_name) >= sizeof(ProfileData::name)) {
        show_info_dialog("Error", "Profile name is too long!");
        return false; // Возвращаем неудачу
    }
    if (!sd_card_initialized) {
        show_info_dialog("Error", "SD Card not ready!");
        return false; // Возвращаем неудачу
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
    new_profile_defaults.nitrogen_target_percent = 99;
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
        if (screen_profile_details) { lv_scr_load(screen_profile_details); }
        else { Serial.println("ERROR: screen_profile_details is NULL!"); }
        lvgl_port_unlock();
    }
}

static void profile_detail_edit_btn_event_cb(lv_event_t * e) {
    if (e && lv_event_get_code(e) != LV_EVENT_CLICKED) return;

    // Гарантированная перезагрузка данных из файла
    String full_path_to_read = "/" + String(current_selected_profile_filename);
    if (!readFileContentToBuffer_ino(SD, full_path_to_read.c_str(), file_content_buffer, FILE_CONTENT_BUFFER_SIZE) || !parseProfileJson(file_content_buffer, current_active_profile_data)) {
        show_info_dialog("Error", "Could not load profile data for editing.");
        return;
    }
    
    Serial.printf("--- Preparing REDESIGNED edit screen for profile: %s ---\n", current_active_profile_data.name);
    lvgl_port_lock(-1);

    if (screen_profile_edit) {
        // --- Заполнение полей данными из current_active_profile_data ---
        char num_buf[10];

        // Header
        lv_textarea_set_text(ta_edit_profile_name, current_active_profile_data.name);

        // Left Column (UV)
        snprintf(num_buf, sizeof(num_buf), "%d", current_active_profile_data.primary_uv_exposure_sec);
        lv_textarea_set_text(ta_edit_primary_uv, num_buf);
        snprintf(num_buf, sizeof(num_buf), "%d", current_active_profile_data.primary_uv_flicker_rate);
        lv_textarea_set_text(ta_edit_flicker_rate, num_buf);

        // --- ПРЯМОЕ УПРАВЛЕНИЕ ПЕРВИЧНЫМ ПЕРЕКЛЮЧАТЕЛЕМ ---
        {
            uint32_t mode = current_active_profile_data.primary_uv_mode;
            // 1. Жестко устанавливаем активную кнопку
            lv_btnmatrix_clear_btn_ctrl_all(btnm_primary_uv_mode, LV_BTNMATRIX_CTRL_CHECKED);
            lv_btnmatrix_set_btn_ctrl(btnm_primary_uv_mode, mode, LV_BTNMATRIX_CTRL_CHECKED);
            // 2. Жестко устанавливаем текст
            const char* text_eng; const char* text_rus;
            switch(mode) {
                case 0: text_eng = "type 1"; text_rus = "тип 1"; break;
                case 1: text_eng = "type 2"; text_rus = "тип 2"; break;
                case 2: text_eng = "both types"; text_rus = "оба типа"; break;
                default: text_eng = ""; text_rus = ""; break;
            }
            if (current_global_settings.language == 1) lv_label_set_text(label_primary_uv_mode_status, text_rus);
            else lv_label_set_text(label_primary_uv_mode_status, text_eng);
        }

        snprintf(num_buf, sizeof(num_buf), "%d", current_active_profile_data.secondary_uv_exposure_sec);
        lv_textarea_set_text(ta_edit_secondary_uv, num_buf);

        // --- ПРЯМОЕ УПРАВЛЕНИЕ ВТОРИЧНЫМ ПЕРЕКЛЮЧАТЕЛЕМ ---
        {
            uint32_t mode = current_active_profile_data.secondary_uv_mode;
            // 1. Жестко устанавливаем активную кнопку
            lv_btnmatrix_clear_btn_ctrl_all(btnm_secondary_uv_mode, LV_BTNMATRIX_CTRL_CHECKED);
            lv_btnmatrix_set_btn_ctrl(btnm_secondary_uv_mode, mode, LV_BTNMATRIX_CTRL_CHECKED);
            // 2. Жестко устанавливаем текст
            const char* text_eng; const char* text_rus;
            switch(mode) {
                case 0: text_eng = "type 1"; text_rus = "тип 1"; break;
                case 1: text_eng = "type 2"; text_rus = "тип 2"; break;
                case 2: text_eng = "both types"; text_rus = "оба типа"; break;
                default: text_eng = ""; text_rus = ""; break;
            }
            if (current_global_settings.language == 1) lv_label_set_text(label_secondary_uv_mode_status, text_rus);
            else lv_label_set_text(label_secondary_uv_mode_status, text_eng);
        }


        snprintf(num_buf, sizeof(num_buf), "%d", current_active_profile_data.tertiary_uv_exposure_sec);
        lv_textarea_set_text(ta_edit_tertiary_uv, num_buf);

        // --- ПРЯМОЕ УПРАВЛЕНИЕ ТРЕТИЧНЫМ ПЕРЕКЛЮЧАТЕЛЕМ ---
        {
            uint32_t mode = current_active_profile_data.tertiary_uv_mode;
            // 1. Жестко устанавливаем активную кнопку
            lv_btnmatrix_clear_btn_ctrl_all(btnm_tertiary_uv_mode, LV_BTNMATRIX_CTRL_CHECKED);
            lv_btnmatrix_set_btn_ctrl(btnm_tertiary_uv_mode, mode, LV_BTNMATRIX_CTRL_CHECKED);
            // 2. Жестко устанавливаем текст
            const char* text_eng; const char* text_rus;
            switch(mode) {
                case 0: text_eng = "type 1"; text_rus = "тип 1"; break;
                case 1: text_eng = "type 2"; text_rus = "тип 2"; break;
                case 2: text_eng = "both types"; text_rus = "оба типа"; break;
                default: text_eng = ""; text_rus = ""; break;
            }
            if (current_global_settings.language == 1) lv_label_set_text(label_tertiary_uv_mode_status, text_rus);
            else lv_label_set_text(label_tertiary_uv_mode_status, text_eng);
        }

        // Right Column (Polymerization)
        if (current_active_profile_data.nitrogen_use_enabled) lv_obj_add_state(sw_edit_nitrogen, LV_STATE_CHECKED); else lv_obj_clear_state(sw_edit_nitrogen, LV_STATE_CHECKED);
        lv_event_send(sw_edit_nitrogen, LV_EVENT_VALUE_CHANGED, NULL);
        snprintf(num_buf, sizeof(num_buf), "%d", current_active_profile_data.nitrogen_target_percent);
        lv_textarea_set_text(ta_edit_nitrogen_target, num_buf);
        
        if (current_active_profile_data.chamber_cooling_enabled) lv_obj_add_state(sw_edit_chamber_cooling, LV_STATE_CHECKED); else lv_obj_clear_state(sw_edit_chamber_cooling, LV_STATE_CHECKED);

        if (current_active_profile_data.thermal_chamber_enabled) lv_obj_add_state(sw_edit_thermal_chamber_enable, LV_STATE_CHECKED); else lv_obj_clear_state(sw_edit_thermal_chamber_enable, LV_STATE_CHECKED);
        lv_event_send(sw_edit_thermal_chamber_enable, LV_EVENT_VALUE_CHANGED, NULL);
        snprintf(num_buf, sizeof(num_buf), "%d", current_active_profile_data.thermal_chamber_temp);
        lv_textarea_set_text(ta_edit_thermal_temp, num_buf);
        snprintf(num_buf, sizeof(num_buf), "%d", current_active_profile_data.heat_exchange_hold_sec);
        lv_textarea_set_text(ta_edit_heat_hold, num_buf);

        // --- Перевод всех текстов ---
        if (current_global_settings.language == 1) { // RUS
            lv_label_set_text(label_edit_name_title, "Имя:");
            lv_label_set_text(label_btn_help_section, "Справка по разделу ?");
            lv_label_set_text(header_uv_params, "Параметры UV излучения");
            
            // <<<--- ИЗМЕНЕНИЕ НАЗВАНИЙ ЭТАПОВ ---<<<
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
            lv_label_set_text(label_edit_cooling_title, "Использование сжатого воздуха");
            lv_label_set_text(label_edit_thermal_chamber_title, "Термокамера");
            lv_label_set_text(label_edit_thermal_temp_title, "t нагрева (40-80C):");
            lv_label_set_text(label_edit_heat_hold_title, "Время удержания (30-180сек):");
            lv_label_set_text(label_btn_save, "Сохранить");
            lv_label_set_text(label_btn_cancel, "Отмена");
        } else { // ENG
            lv_label_set_text(label_edit_name_title, "Name:");
            lv_label_set_text(label_btn_help_section, "Section Help ?");
            lv_label_set_text(header_uv_params, "UV Parameters");

            // <<<--- ИЗМЕНЕНИЕ НАЗВАНИЙ ЭТАПОВ ---<<<
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
        
        // Запуск таймера и загрузка экрана
        if (help_blink_timer) lv_timer_del(help_blink_timer);
        help_blink_timer = lv_timer_create(help_blink_timer_cb, 1, (void*)0);
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
static void numeric_textarea_defocus_event_cb(lv_event_t * e) {
    lv_obj_t * ta = lv_event_get_target(e);
    if (ta == ta_edit_primary_uv) { validate_numeric_input(e, 5, 60); } 
    else if (ta == ta_edit_secondary_uv) { validate_numeric_input(e, 5, 200); }
    else if (ta == ta_edit_tertiary_uv) { validate_numeric_input(e, 5, 200); }
    else if (ta == ta_edit_heat_hold) { validate_numeric_input(e, 30, 180); } // <<<--- НОВАЯ ВАЛИДАЦИЯ
    else if (ta == ta_edit_flicker_rate) { validate_numeric_input(e, 1, 100); }
}

static void profile_edit_save_changes_btn_event_cb(lv_event_t * e) {
    Serial.println("Save Changes button clicked.");
    ProfileData edited_data;
    edited_data.id = current_active_profile_data.id;

    // --- Сбор данных с UI ---
    strncpy(edited_data.name, lv_textarea_get_text(ta_edit_profile_name), sizeof(edited_data.name) - 1);
    edited_data.name[sizeof(edited_data.name) - 1] = '\0'; 
    if (strlen(edited_data.name) == 0) { 
        show_info_dialog("Input Error", "Profile name cannot be empty.");
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
        show_info_dialog("Save Error", "Failed to prepare data for saving.");
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

    Serial.println("--- CANCEL button clicked by user on process screen. ---");

    // 1. Создаем и отправляем команду аварийной остановки
    StaticJsonDocument<128> doc;
    doc["command"] = "EMERGENCY_STOP";
    
    String output;
    serializeJson(doc, output);
    MySerial1.println(output); // Используем ваш MySerial1

    Serial.println("Sent command: EMERGENCY_STOP");

    // 2. Сбрасываем флаги процесса на этой (главной) плате
    main_process_running = false; 
   
    // 3. Возвращаемся на экран деталей профиля
    if (screen_profile_details) {
        Serial.println("Returning to profile details screen.");
        
        lvgl_port_lock(-1);
        
        // Обновим статус на экране процесса, чтобы было видно, что команда ушла
        if (lv_scr_act() == screen_process_execution) {
            if (current_global_settings.language == 1) { // RUS
                lv_label_set_text(label_process_status_title, "Процесс Отменен");
                lv_label_set_text(label_process_status_detail, "Сигнал аварийной остановки отправлен.\nВозврат к деталям...");
            } else { // ENG
                lv_label_set_text(label_process_status_title, "Process Cancelled");
                lv_label_set_text(label_process_status_detail, "Emergency stop signal sent.\nReturning to details...");
            }
        }

        // Возвращаемся на экран деталей, а не на главный
        lv_scr_load(screen_profile_details); 
        lvgl_port_unlock();
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
        } else if (user_data_str && strcmp(user_data_str, "air_sys") == 0) {
            current_global_settings.compressed_air_system_enabled = lv_obj_has_state(target, LV_STATE_CHECKED);
            Serial.printf("Settings: Global Compressed Air System %s\n", current_global_settings.compressed_air_system_enabled ? "Enabled" : "Disabled");
        }
    } 
    
    if (code == LV_EVENT_CLICKED) {
        if (!user_data_str) return;

        if (user_data_str && strcmp(user_data_str, "back_save") == 0) {
            Serial.println("Settings: Save and Back button clicked.");
            saveGlobalSettings(); 
            needs_list_refresh = true;
            if (screen_main_app) {
                lv_scr_load(screen_main_app); 
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
        }
        else if (strcmp(user_data_str, "theme2") == 0) {
            current_global_settings.theme = 1; // Dark
            update_custom_toggle_ui(theme_toggle_box, current_global_settings.theme);
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

            // Обновляем вид новых кастомных переключателей
            update_custom_toggle_ui(lang_toggle_box, current_global_settings.language);
            update_custom_toggle_ui(theme_toggle_box, current_global_settings.theme);

            if (current_global_settings.language == 1) { // 1 = RUS
                if (label_settings_title) { lv_label_set_text(label_settings_title, "Общие Настройки"); lv_obj_add_style(label_settings_title, &style_my_text_22, 0); }
                if (label_settings_nitrogen) { lv_label_set_text(label_settings_nitrogen, "Система Азота:"); lv_obj_add_style(label_settings_nitrogen, &style_my_text_18, 0); }
                if (label_settings_air) { lv_label_set_text(label_settings_air, "Сжатый Воздух:"); lv_obj_add_style(label_settings_air, &style_my_text_18, 0); }
                if (label_settings_language) { lv_label_set_text(label_settings_language, "Язык:"); lv_obj_add_style(label_settings_language, &style_my_text_18, 0); }
                if (label_settings_theme) { lv_label_set_text(label_settings_theme, "Тема:"); lv_obj_add_style(label_settings_theme, &style_my_text_18, 0); }
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

    // Определяем, какая кнопка была нажата
    if (btn == btn_choice_dialog_skip) {
        Serial.println("Choice Dialog: User selected SKIP.");
        choice_dialog_result = 1; // 1 = Skip/Пропустить
    } else if (btn == btn_choice_dialog_cancel) {
        Serial.println("Choice Dialog: User selected CANCEL.");
        choice_dialog_result = 2; // 2 = Cancel/Отмена
    }

    // В любом случае прячем диалоговое окно
    if (screen_choice_dialog) {
        lv_obj_add_flag(screen_choice_dialog, LV_OBJ_FLAG_HIDDEN);
    }
}

void build_info_dialog(lv_obj_t* parent_layer) { // parent_layer будет lv_layer_top()
    if (screen_info_dialog) return; // Уже создан

    screen_info_dialog = lv_obj_create(parent_layer);
    lv_obj_add_flag(screen_info_dialog, LV_OBJ_FLAG_HIDDEN); // По умолчанию скрыт
    lv_obj_set_size(screen_info_dialog, lv_pct(75), LV_SIZE_CONTENT); // Ширина 75% экрана, высота по контенту
    lv_obj_center(screen_info_dialog); 
    // Стили как у твоего диалога подтверждения удаления
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

    // Заголовок диалога (будет устанавливаться в show_info_dialog, но можно создать lv_label и здесь)
    // lv_obj_t* title_label = lv_label_create(screen_info_dialog); 
    // lv_obj_add_flag(title_label, LV_OBJ_FLAG_HIDDEN); // Если не всегда нужен заголовок

    label_info_dialog_text = lv_label_create(screen_info_dialog);
    lv_label_set_long_mode(label_info_dialog_text, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(label_info_dialog_text, lv_pct(100));
    lv_obj_set_style_text_align(label_info_dialog_text, LV_TEXT_ALIGN_CENTER, 0);
    // lv_label_set_text(label_info_dialog_text, "Placeholder message."); // Начальный текст

    btn_info_dialog_ok = lv_btn_create(screen_info_dialog);
    lv_obj_add_event_cb(btn_info_dialog_ok, info_dialog_ok_event_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_set_width(btn_info_dialog_ok, lv_pct(50)); // Ширина кнопки
    lv_obj_t* label_ok_txt = lv_label_create(btn_info_dialog_ok); 
    lv_label_set_text(label_ok_txt, "OK");
    lv_obj_center(label_ok_txt);

    Serial.println("Universal Info Dialog UI built.");
}

static void service_lock_screen_event_cb(lv_event_t* e) {
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t* target = lv_event_get_target(e);
    const char* user_data = (const char*)lv_event_get_user_data(e);

    if (code == LV_EVENT_CLICKED) {
        // --- Показать модальное окно ---
        if (user_data && strcmp(user_data, "show_modal") == 0) {
            // Копируем текущий текст из основного поля в модальное
            lv_textarea_set_text(ta_service_lock_modal_input, lv_textarea_get_text(ta_service_code_input));
            
            // Показываем оверлей и клавиатуру
            lv_obj_clear_flag(service_lock_modal_overlay, LV_OBJ_FLAG_HIDDEN);
            lv_obj_clear_flag(kb_service_code, LV_OBJ_FLAG_HIDDEN);

            // >>>>> КЛЮЧЕВОЕ ИСПРАВЛЕНИЕ: Перемещаем клавиатуру на передний план <<<<<
            // Это гарантирует, что она будет поверх затемняющего фона.
            lv_obj_move_foreground(kb_service_code);

            // Привязываем клавиатуру к модальному полю и даем ему фокус
            lv_keyboard_set_textarea(kb_service_code, ta_service_lock_modal_input);
            lv_obj_add_state(ta_service_lock_modal_input, LV_STATE_FOCUSED);
            lv_indev_reset(NULL, ta_service_lock_modal_input); // Сбрасываем, чтобы курсор появился
        }
        // --- Скрыть модальное окно (клик по фону) ---
        else if (user_data && strcmp(user_data, "hide_modal") == 0) {
            lv_event_send(kb_service_code, LV_EVENT_CANCEL, NULL); // Имитируем нажатие "Cancel" на клавиатуре
        }
        // --- Проверить код (нажатие кнопки ENTER) ---
        else if (user_data && strcmp(user_data, "check_code") == 0) {
            const char* entered_code_c = lv_textarea_get_text(ta_service_code_input);
            String entered_code(entered_code_c);
            entered_code.toUpperCase();

            bool key_is_valid = false;
            for (const auto& key : service_keys) {
                if (key == entered_code) {
                    key_is_valid = true;
                    break;
                }
            }

            if (key_is_valid) {
                Serial.printf("Valid service key '%s' entered. Unlocking device.\n", entered_code_c);
                
                // 1. Удаляем ключ
                remove_and_save_service_keys(entered_code);
                
                // 2. Сбрасываем флаг ошибки
                current_global_settings.is_heater_error = false;
            
                // 3. Сохраняем настройки, чтобы зафиксировать разблокировку
                saveGlobalSettings();

                // 4. Разблокируем - возвращаемся на главный экран
                lv_scr_load(screen_main_app);
                
                // >>>>> КЛЮЧЕВОЕ ИСПРАВЛЕНИЕ: ОБНОВЛЯЕМ СПИСОК ПРОФИЛЕЙ <<<<<
                lvgl_port_lock(-1);
                if (sd_card_initialized) {
                    Serial.println("Re-caching profiles after unlock...");
                    current_profile_next_id = scanAndCacheAllProfiles(SD, all_profile_entries_cache);
                    displayProfileListPage(); // Эта функция обновит список и состояние кнопок
                } else {
                     // Если SD карта отвалилась, показываем ошибку на главном экране
                    if (list_profiles_main) lv_list_add_text(list_profiles_main, "SD Card Error!");
                }
                lvgl_port_unlock();

            } else {
                Serial.printf("Invalid service key '%s' entered.\n", entered_code_c);
                lv_textarea_set_text(ta_service_code_input, "");
                lv_label_set_text(label_service_lock_msg, "Invalid code! Please try again.");
            }
        }
    }
}

static void build_service_lock_screen(lv_obj_t* parent_screen) {
    screen_service_lock = parent_screen; // Используем переданный объект как экран

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
    lv_obj_set_style_text_font(title, &lv_font_montserrat_22, 0);
    lv_label_set_text(title, "DEVICE LOCKED");

    // Сообщение об ошибке
    label_service_lock_msg = lv_label_create(content_cont);
    lv_obj_set_style_text_color(label_service_lock_msg, lv_color_white(), 0);
    lv_obj_set_style_text_align(label_service_lock_msg, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_long_mode(label_service_lock_msg, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(label_service_lock_msg, lv_pct(90));
    lv_label_set_text(label_service_lock_msg, "A critical error has occurred.\nPlease enter service code to unlock.");

    // Поле для ввода кода
    ta_service_code_input = lv_textarea_create(content_cont);
    lv_textarea_set_one_line(ta_service_code_input, true);
    lv_textarea_set_max_length(ta_service_code_input, 4);
    lv_obj_set_width(ta_service_code_input, 180);
    lv_textarea_set_align(ta_service_code_input, LV_TEXT_ALIGN_CENTER);
    lv_obj_set_style_text_font(ta_service_code_input, &lv_font_montserrat_20, 0);

    // >>>>> ПРАВКА 1: Добавляем плейсхолдер <<<<<
    lv_textarea_set_placeholder_text(ta_service_code_input, "Enter code here...");
    
    // >>>>> ПРАВКА 2: Привязываем событие для ПОКАЗА модального окна <<<<<
    // Используем общий обработчик, но с другим user_data
    lv_obj_add_event_cb(ta_service_code_input, service_lock_screen_event_cb, LV_EVENT_CLICKED, (void*)"show_modal");

    // --- Кнопка "Enter" для проверки кода ---
    btn_service_lock_enter = lv_btn_create(content_cont);
    lv_obj_set_width(btn_service_lock_enter, 180);
    lv_obj_add_event_cb(btn_service_lock_enter, service_lock_screen_event_cb, LV_EVENT_CLICKED, (void*)"check_code");
    lv_obj_t* label_enter = lv_label_create(btn_service_lock_enter);
    lv_label_set_text(label_enter, "ENTER");
    lv_obj_center(label_enter);

    // --- Создаем клавиатуру, она всегда будет скрыта на этом слое ---
    kb_service_code = lv_keyboard_create(lv_layer_top()); // Создаем на верхнем слое
    lv_obj_add_flag(kb_service_code, LV_OBJ_FLAG_HIDDEN); // Изначально скрыта
    lv_obj_add_event_cb(kb_service_code, service_code_keyboard_event_cb, LV_EVENT_ALL, NULL);
    lv_keyboard_set_mode(kb_service_code, LV_KEYBOARD_MODE_TEXT_UPPER);

    // --- Создаем элементы для модального окна, они тоже будут скрыты ---
    // Затемняющий фон
    service_lock_modal_overlay = lv_obj_create(lv_layer_top());
    lv_obj_remove_style_all(service_lock_modal_overlay);
    lv_obj_set_size(service_lock_modal_overlay, LV_PCT(100), LV_PCT(100));
    lv_obj_set_style_bg_color(service_lock_modal_overlay, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(service_lock_modal_overlay, LV_OPA_50, 0);
    lv_obj_add_flag(service_lock_modal_overlay, LV_OBJ_FLAG_HIDDEN);
    // Добавляем событие для закрытия по клику на фон
    lv_obj_add_event_cb(service_lock_modal_overlay, service_lock_screen_event_cb, LV_EVENT_CLICKED, (void*)"hide_modal");

    // Временное (модальное) поле ввода
    ta_service_lock_modal_input = lv_textarea_create(service_lock_modal_overlay);
    lv_textarea_set_one_line(ta_service_lock_modal_input, true);
    lv_textarea_set_max_length(ta_service_lock_modal_input, 4);
    lv_obj_set_width(ta_service_lock_modal_input, 150);
    lv_textarea_set_align(ta_service_lock_modal_input, LV_TEXT_ALIGN_CENTER);
    lv_obj_set_style_text_font(ta_service_lock_modal_input, &lv_font_montserrat_20, 0);
    lv_obj_align(ta_service_lock_modal_input, LV_ALIGN_CENTER, 0, -50); // Располагаем по центру, чуть выше
    lv_textarea_set_accepted_chars(ta_service_lock_modal_input, "ABCDEFGHIJKLMNPQRSTUVWXYZ123456789");
}

static void service_code_keyboard_event_cb(lv_event_t* e) {
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t* kb = lv_event_get_target(e);

    // Если нажата галочка (Ready) или крестик (Cancel)
    if (code == LV_EVENT_READY || code == LV_EVENT_CANCEL) {
        if (code == LV_EVENT_READY) {
            lv_textarea_set_text(ta_service_code_input, lv_textarea_get_text(ta_service_lock_modal_input));
        }
        lv_obj_add_flag(kb, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(service_lock_modal_overlay, LV_OBJ_FLAG_HIDDEN);
    }
}

void show_info_dialog(const char* title_text, const char* message_text) {
    if (!screen_info_dialog) {
        Serial.println("show_info_dialog: screen_info_dialog is NULL! Cannot show.");
        // Можно попробовать создать его здесь, если он не был создан в setup
        // build_info_dialog(lv_layer_top());
        // if (!screen_info_dialog) return; // Если и тут не создался, выходим
        return;
    }
    
    lvgl_port_lock(-1); // Блокируем LVGL

    // Если у нас есть отдельный лейбл для заголовка в диалоге:
    // lv_obj_t* title_label_in_dialog = lv_obj_get_child(screen_info_dialog, 0); // Пример, если он первый
    // if (title_label_in_dialog && lv_obj_check_type(title_label_in_dialog, &lv_label_class)) {
    //    lv_label_set_text(title_label_in_dialog, title_text);
    //    lv_obj_clear_flag(title_label_in_dialog, LV_OBJ_FLAG_HIDDEN);
    // }
    // Пока для простоты будем считать, что заголовок не используется или встроен в message_text

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
static void create_settings_row(lv_obj_t* parent, const char* text, lv_obj_t** p_switch, const char* user_data) {
    lv_obj_t* row = lv_obj_create(parent);
    lv_obj_remove_style_all(row);
    lv_obj_set_width(row, lv_pct(90));
    lv_obj_set_height(row, LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(row, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_t* label = lv_label_create(row);
    lv_label_set_text(label, text);
    *p_switch = lv_switch_create(row);
    lv_obj_add_event_cb(*p_switch, settings_screen_event_cb, LV_EVENT_VALUE_CHANGED, (void*)user_data);
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
// Обработчик кнопок "Skip" и "Cancel"
static void create_settings_row(lv_obj_t* parent, lv_obj_t** p_label, lv_obj_t** p_switch, const char* user_data) {
    lv_obj_t* row = lv_obj_create(parent);
    lv_obj_remove_style_all(row);
    lv_obj_set_width(row, lv_pct(90));
    lv_obj_set_height(row, LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(row, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    
    *p_label = lv_label_create(row); // Используем переданный указатель
    
    *p_switch = lv_switch_create(row);
    lv_obj_add_event_cb(*p_switch, settings_screen_event_cb, LV_EVENT_VALUE_CHANGED, (void*)user_data);
}

// Функция для отображения диалога
static void show_choice_dialog(const char* title, const char* message_text) {
    if (!screen_choice_dialog) return;
    
    lvgl_port_lock(-1);
    // Находим дочерние элементы (заголовок, текст) и устанавливаем им нужные значения
    lv_obj_t* title_label = lv_obj_get_child(screen_choice_dialog, 0);
    lv_obj_t* msg_label = lv_obj_get_child(screen_choice_dialog, 1);
    
    if (title_label) lv_label_set_text(title_label, title);
    if (msg_label) lv_label_set_text(msg_label, message_text);
    
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
    // ... (стили, как у других диалогов: рамка, тень и т.д.) ...
    lv_obj_set_style_bg_color(screen_choice_dialog, lv_color_white(), 0);
    lv_obj_set_style_border_width(screen_choice_dialog, 1, 0);
    lv_obj_set_style_shadow_width(screen_choice_dialog, 8, 0);
    lv_obj_set_flex_flow(screen_choice_dialog, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_all(screen_choice_dialog, 15, 0);
    lv_obj_set_style_pad_gap(screen_choice_dialog, 10, 0);

    // Заголовок
    lv_obj_t* title = lv_label_create(screen_choice_dialog);
    lv_label_set_text(title, "Warning");

    // Текст сообщения
    lv_obj_t* msg = lv_label_create(screen_choice_dialog);
    lv_label_set_long_mode(msg, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(msg, lv_pct(100));

    // Контейнер для кнопок
    lv_obj_t* btn_area = lv_obj_create(screen_choice_dialog);
    lv_obj_remove_style_all(btn_area);
    lv_obj_set_width(btn_area, lv_pct(100));
    lv_obj_set_height(btn_area, LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(btn_area, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(btn_area, LV_FLEX_ALIGN_SPACE_EVENLY, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

    // Кнопка "Cancel"
    btn_choice_dialog_cancel = lv_btn_create(btn_area);
    lv_obj_add_event_cb(btn_choice_dialog_cancel, choice_dialog_event_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t* label_cancel = lv_label_create(btn_choice_dialog_cancel);
    lv_label_set_text(label_cancel, "Cancel");
    lv_obj_set_style_bg_color(btn_choice_dialog_cancel, lv_palette_main(LV_PALETTE_RED), 0);
    
    // Кнопка "Skip"
    btn_choice_dialog_skip = lv_btn_create(btn_area);
    lv_obj_add_event_cb(btn_choice_dialog_skip, choice_dialog_event_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t* label_skip = lv_label_create(btn_choice_dialog_skip);
    lv_label_set_text(label_skip, "Skip");
}

static void profile_switch_value_changed_event_cb(lv_event_t * e) {
    lv_obj_t* sw = lv_event_get_target(e);
    const char* switch_id = (const char*)lv_event_get_user_data(e);
    bool attempted_to_enable = lv_obj_has_state(sw, LV_STATE_CHECKED); // Проверяем, ПОПЫТАЛСЯ ли пользователь включить

    if (strcmp(switch_id, "nitrogen_profile") == 0) {
        bool is_enabled_by_user = lv_obj_has_state(sw, LV_STATE_CHECKED);

        if (!current_global_settings.nitrogen_system_enabled && is_enabled_by_user) {
            lv_obj_clear_state(sw, LV_STATE_CHECKED); // Принудительно выключаем обратно
            show_info_dialog("Setting Disabled", 
                            "Nitrogen system is disabled in Global Settings.\n"
                            "Please connect the valve and enable it in Settings.");
            is_enabled_by_user = false; // Обновляем локальную переменную
        }
        
        if (is_enabled_by_user) {
            lv_obj_clear_flag(nitrogen_elements_container, LV_OBJ_FLAG_HIDDEN);
        } else {
            lv_obj_add_flag(nitrogen_elements_container, LV_OBJ_FLAG_HIDDEN);
        }
        Serial.printf("Profile Nitrogen switch new state: %s\n", is_enabled_by_user ? "ON" : "OFF");

    } else if (strcmp(switch_id, "cooling_profile") == 0) {
        if (!current_global_settings.compressed_air_system_enabled && attempted_to_enable) {
            lv_obj_clear_state(sw, LV_STATE_CHECKED); 
            show_info_dialog("Setting Disabled", 
                             "Compressed Air (Chamber Cooling) is disabled in Global Settings.\n"
                             "Please connect it and enable in Settings.");
        } else {
            Serial.printf("Profile Chamber Cooling switch new state: %s\n", attempted_to_enable ? "ON" : "OFF");
        }
    }
    // Важно: после показа диалога, LVGL продолжит обработку событий.
    // Свитч останется в том состоянии, в которое мы его принудительно установили (выключенном).
}

// Обработчик событий для глобальных клавиатур, КОГДА ОНИ ИСПОЛЬЗУЮТСЯ В МОДАЛЬНОМ РЕЖИМЕ
static void modal_input_keyboard_event_cb(lv_event_t* e) {
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t* kb = lv_event_get_target(e);

    bool close_modal = false;
    if (code == LV_EVENT_READY) {
        if (current_target_ta == ta_dummy_for_new_profile) {
            const char* new_name = lv_textarea_get_text(ta_modal_input);
            if (handle_save_new_profile_logic(new_name)) {
                close_modal = true;
            }
        } 
        else {
            if (current_target_ta && ta_modal_input) {
                lv_textarea_set_text(current_target_ta, lv_textarea_get_text(ta_modal_input));
                lv_event_send(current_target_ta, LV_EVENT_DEFOCUSED, NULL);
            }
            close_modal = true;
        }
    } else if (code == LV_EVENT_CANCEL) {
        close_modal = true;
    }

    if (close_modal) {
        if (overlay_modal_input_bg) lv_obj_add_flag(overlay_modal_input_bg, LV_OBJ_FLAG_HIDDEN);
        if (modal_input_container) lv_obj_add_flag(modal_input_container, LV_OBJ_FLAG_HIDDEN);
        if (ta_modal_input) lv_obj_add_flag(ta_modal_input, LV_OBJ_FLAG_HIDDEN);
        
        lv_obj_add_flag(kb, LV_OBJ_FLAG_HIDDEN);

        if (current_target_ta) {
            lv_obj_clear_state(current_target_ta, LV_STATE_FOCUSED);
            lv_indev_reset(NULL, current_target_ta);
        }
        current_target_ta = nullptr;
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
    if (!target_ta_original) {
        Serial.println("show_modal_input: target_ta_original is NULL!");
        return;
    }

    current_target_ta = target_ta_original;

    // 1. --- ОПРЕДЕЛЯЕМ КЛАВИАТУРУ ---
    lv_obj_t* kb_to_use = (kb_mode_to_set == LV_KEYBOARD_MODE_NUMBER) ? kb_edit_numeric : kb_edit_alpha;
    
    if (!kb_to_use) {
        Serial.println("Error: No suitable global keyboard selected! Aborting modal display.");
        return;
    }
    // Скрываем другие клавиатуры
    if (kb_edit_numeric && kb_to_use != kb_edit_numeric) lv_obj_add_flag(kb_edit_numeric, LV_OBJ_FLAG_HIDDEN);
    if (kb_edit_alpha && kb_to_use != kb_edit_alpha) lv_obj_add_flag(kb_edit_alpha, LV_OBJ_FLAG_HIDDEN);

    // 2. --- ПОДГОТОВКА ФОНА И ОСНОВНЫХ ЭЛЕМЕНТОВ ---
    if (!overlay_modal_input_bg) {
        overlay_modal_input_bg = lv_obj_create(lv_layer_top());
        lv_obj_remove_style_all(overlay_modal_input_bg);
        lv_obj_set_size(overlay_modal_input_bg, LV_PCT(100), LV_PCT(100));
        lv_obj_set_style_bg_color(overlay_modal_input_bg, lv_color_black(), 0);
        lv_obj_set_style_bg_opa(overlay_modal_input_bg, LV_OPA_50, 0);
        lv_obj_add_event_cb(overlay_modal_input_bg, modal_input_overlay_click_event_cb, LV_EVENT_CLICKED, NULL);
    }
    lv_obj_clear_flag(overlay_modal_input_bg, LV_OBJ_FLAG_HIDDEN);

    if (!ta_modal_input) {
        ta_modal_input = lv_textarea_create(lv_layer_top());
        lv_obj_add_style(ta_modal_input, &style_just_font_18, 0); 
    }
    
    // <<<--- ВОТ ОН, ВАЖНЫЙ ФИКС! ---<<<
    // Перед настройкой всегда сбрасываем фильтр символов, разрешая все (NULL)
    lv_textarea_set_accepted_chars(ta_modal_input, NULL);
    // Также сбрасываем плейсхолдер
    lv_textarea_set_placeholder_text(ta_modal_input, "");


    lv_obj_clear_flag(ta_modal_input, LV_OBJ_FLAG_HIDDEN);


    // 3. --- СТРОИМ UI В ЗАВИСИМОСТИ ОТ РЕЖИМА ---
    bool is_new_profile_mode = (target_ta_original == ta_dummy_for_new_profile);

    if (is_new_profile_mode) {
        // --- РЕЖИМ 1: Создание нового профиля ---
        if (!modal_input_container) {
            modal_input_container = lv_obj_create(lv_layer_top());
            lv_obj_set_width(modal_input_container, lv_pct(85));
            lv_obj_set_height(modal_input_container, LV_SIZE_CONTENT);
            lv_obj_set_flex_flow(modal_input_container, LV_FLEX_FLOW_COLUMN);
            lv_obj_set_style_pad_all(modal_input_container, 15, 0);
            lv_obj_set_style_pad_gap(modal_input_container, 10, 0);
            lv_obj_set_style_radius(modal_input_container, 8, 0);
            modal_input_title_label = lv_label_create(modal_input_container);
            lv_label_set_text(modal_input_title_label, "Write a profile name");
            lv_obj_set_style_text_font(modal_input_title_label, &lv_font_montserrat_18, 0);
        }
        lv_obj_set_parent(ta_modal_input, modal_input_container);
        lv_obj_clear_flag(modal_input_container, LV_OBJ_FLAG_HIDDEN);
        
        lv_textarea_set_text(ta_modal_input, "");
        lv_textarea_set_placeholder_text(ta_modal_input, "Profile Name...");
        lv_textarea_set_one_line(ta_modal_input, true);
        lv_textarea_set_max_length(ta_modal_input, 200); 
        lv_obj_set_width(ta_modal_input, lv_pct(100));

    } else {
        // --- РЕЖИМ 2: Обычное редактирование ---
        lv_obj_set_parent(ta_modal_input, lv_layer_top());
        
        lv_textarea_set_text(ta_modal_input, lv_textarea_get_text(target_ta_original));
        lv_textarea_set_one_line(ta_modal_input, lv_textarea_get_one_line(target_ta_original));
        
        // Применяем специфичный фильтр, если он есть (например, для чисел)
        lv_textarea_set_accepted_chars(ta_modal_input, lv_textarea_get_accepted_chars(target_ta_original));
        lv_textarea_set_max_length(ta_modal_input, lv_textarea_get_max_length(target_ta_original));
        lv_obj_set_width(ta_modal_input, lv_pct(80));
    }

    // 4. --- ОБЩАЯ НАСТРОЙКА КЛАВИАТУРЫ И ПОЗИЦИОНИРОВАНИЕ ---
    lv_obj_set_parent(kb_to_use, lv_layer_top());
    lv_keyboard_set_mode(kb_to_use, kb_mode_to_set);
    lv_keyboard_set_textarea(kb_to_use, ta_modal_input);
    lv_obj_remove_event_cb(kb_to_use, modal_input_keyboard_event_cb);
    lv_obj_add_event_cb(kb_to_use, modal_input_keyboard_event_cb, LV_EVENT_ALL, NULL);
    lv_obj_clear_flag(kb_to_use, LV_OBJ_FLAG_HIDDEN);

    if (is_new_profile_mode) {
        lv_obj_align_to(modal_input_container, kb_to_use, LV_ALIGN_OUT_TOP_MID, 0, -10);
    } else {
        lv_obj_align_to(ta_modal_input, kb_to_use, LV_ALIGN_OUT_TOP_MID, 0, -10);
    }

    lv_obj_move_foreground(overlay_modal_input_bg);
    lv_obj_move_foreground(kb_to_use);
    if (is_new_profile_mode) {
        lv_obj_move_foreground(modal_input_container);
    } else {
        lv_obj_move_foreground(ta_modal_input);
    }
    
    // 5. --- ФОКУС ---
    lv_group_t* g = lv_group_get_default();
    if (g) {
        lv_group_set_editing(g, true);
        lv_group_focus_obj(ta_modal_input);
    } else {
        lv_obj_add_state(ta_modal_input, LV_STATE_FOCUSED);
    }
    Serial.println("Modal input shown.");
}


// Раздел 3: Функции построения экранов LVGL
// ==========================================================================
static void build_help_screen(lv_obj_t* parent_screen) {
    screen_help = parent_screen;
    Serial.println("Building help_screen UI...");

    // Главный контейнер
    lv_obj_t* main_container = lv_obj_create(screen_help);
    lv_obj_set_size(main_container, lv_pct(100), lv_pct(100));
    lv_obj_center(main_container);
    lv_obj_set_flex_flow(main_container, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_all(main_container, 10, 0);
    lv_obj_set_style_pad_gap(main_container, 10, 0);

    // 1. Заголовок
    label_help_title = lv_label_create(main_container);
    lv_label_set_text(label_help_title, "Help Section"); // Текст по умолчанию
    lv_obj_add_style(label_help_title, &style_my_text_22, 0);
    lv_obj_set_width(label_help_title, lv_pct(100));
    lv_obj_set_style_text_align(label_help_title, LV_TEXT_ALIGN_CENTER, 0);

    // 2. Контейнер для текста с прокруткой
    lv_obj_t* text_container = lv_obj_create(main_container);
    lv_obj_set_flex_grow(text_container, 1);
    lv_obj_set_width(text_container, lv_pct(100));
    lv_obj_set_scrollbar_mode(text_container, LV_SCROLLBAR_MODE_AUTO);

    // 3. Текстовое поле
    label_help_content = lv_label_create(text_container);
    lv_obj_add_style(label_help_content, &style_my_text_18, 0);
    lv_label_set_long_mode(label_help_content, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(label_help_content, lv_pct(100));
    // Длинный текст-справка будет установлен в обработчике
    lv_label_set_text(label_help_content, "Loading help content...");

    // 4. Кнопка "Понятно"
    btn_help_close = lv_btn_create(main_container);
    lv_obj_add_event_cb(btn_help_close, help_button_event_cb, LV_EVENT_CLICKED, (void*)"close_help");
    lv_obj_set_width(btn_help_close, lv_pct(50));
    lv_obj_align(btn_help_close, LV_ALIGN_CENTER, 0, 0);

    label_btn_help_close = lv_label_create(btn_help_close);
    lv_label_set_text(label_btn_help_close, "Got it!");
    lv_obj_add_style(label_btn_help_close, &style_my_text_18_white, 0);
    lv_obj_center(label_btn_help_close);
}
static void build_splash_screen(lv_obj_t* parent_screen) {
    Serial.println("Building splash_screen UI...");

    // Устанавливаем черный фон
    lv_obj_set_style_bg_color(parent_screen, lv_color_black(), LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(parent_screen, LV_OPA_COVER, LV_STATE_DEFAULT);

    // Используем flexbox для простого центрирования
    lv_obj_set_flex_flow(parent_screen, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(parent_screen, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

    // Создаем метку с названием
    lv_obj_t* title_label = lv_label_create(parent_screen);
    lv_label_set_text(title_label, "UVTron Ultra");

    // Задаем стиль для метки: большой белый шрифт
    lv_obj_set_style_text_color(title_label, lv_color_white(), 0);
    lv_obj_set_style_text_font(title_label, &lv_font_montserrat_48, 0); // Используем большой шрифт
}

void build_main_app_screen(lv_obj_t* parent_screen) {
    Serial.println("Building main_app_screen UI (Tile View with PAGINATION)...");

    // ... (Начало функции без изменений) ...
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

    // --- Заголовок ---
    list_header_label_main = lv_label_create(main_screen_content_container);
    lv_obj_add_style(list_header_label_main, &style_my_text_22, 0);
    lv_obj_set_width(list_header_label_main, lv_pct(100));
    lv_obj_set_style_text_align(list_header_label_main, LV_TEXT_ALIGN_CENTER, 0);

    // <<<--- НАЧАЛО НОВОЙ СТРУКТУРЫ ---<<<
    
    // 1. Создаем центральную панель, которая будет содержать [Стрелка ВЛЕВО | Плитки | Стрелка ВПРАВО]
    lv_obj_t* center_panel_container = lv_obj_create(main_screen_content_container);
    lv_obj_remove_style_all(center_panel_container);
    lv_obj_set_width(center_panel_container, lv_pct(100));
    lv_obj_set_flex_grow(center_panel_container, 1);
    lv_obj_set_layout(center_panel_container, LV_LAYOUT_FLEX);
    lv_obj_set_flex_flow(center_panel_container, LV_FLEX_FLOW_ROW);

    // 2. Создаем левую кнопку-стрелку (ПРЕДЫДУЩАЯ СТРАНИЦА)
    btn_profiles_prev = lv_btn_create(center_panel_container);
    lv_obj_remove_style_all(btn_profiles_prev); // <<<--- УБИРАЕМ ВСЕ СТИЛИ (ФОН, РАМКУ И Т.Д.)
    lv_obj_set_size(btn_profiles_prev, 35, lv_pct(100)); // Размер оставляем
    lv_obj_add_event_cb(btn_profiles_prev, profile_list_prev_btn_event_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t* lbl_left = lv_label_create(btn_profiles_prev);
    lv_label_set_text(lbl_left, LV_SYMBOL_LEFT);
    lv_obj_center(lbl_left);
    // Опционально: можно сделать саму стрелку синей, чтобы она была заметнее
    lv_obj_set_style_text_color(lbl_left, lv_palette_main(LV_PALETTE_BLUE), 0);

    // 3. Создаем контейнер для ПЛИТОК 
    list_profiles_main = lv_obj_create(center_panel_container);
    lv_obj_remove_style_all(list_profiles_main);
    lv_obj_set_flex_grow(list_profiles_main, 1);
    lv_obj_set_height(list_profiles_main, lv_pct(100));
    // <<<--- ВАЖНО: Контейнер НЕ прокручиваемый. Он просто показывает ОДНУ страницу плиток
    lv_obj_clear_flag(list_profiles_main, LV_OBJ_FLAG_SCROLLABLE); 
    
    lv_obj_set_layout(list_profiles_main, LV_LAYOUT_FLEX);
    lv_obj_set_flex_flow(list_profiles_main, LV_FLEX_FLOW_ROW_WRAP);
    lv_obj_set_flex_align(list_profiles_main, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_START);
    lv_obj_set_style_pad_all(list_profiles_main, 5, 0);
    lv_obj_set_style_pad_column(list_profiles_main, 8, 0);
    lv_obj_set_style_pad_row(list_profiles_main, 8, 0);

    // 4. Создаем правую кнопку-стрелку (СЛЕДУЮЩАЯ СТРАНИЦА)
    btn_profiles_next = lv_btn_create(center_panel_container);
    lv_obj_remove_style_all(btn_profiles_next); // <<<--- УБИРАЕМ ВСЕ СТИЛИ (ФОН, РАМКУ И Т.Д.)
    lv_obj_set_size(btn_profiles_next, 35, lv_pct(100)); // Размер оставляем
    lv_obj_add_event_cb(btn_profiles_next, profile_list_next_btn_event_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t* lbl_right = lv_label_create(btn_profiles_next);
    lv_label_set_text(lbl_right, LV_SYMBOL_RIGHT);
    lv_obj_center(lbl_right);
    // Опционально: можно сделать саму стрелку синей
    lv_obj_set_style_text_color(lbl_right, lv_palette_main(LV_PALETTE_BLUE), 0);

    // ... (остальная часть функции с нижними кнопками остается без изменений) ...
    lv_obj_t* actions_btn_container = lv_obj_create(main_screen_content_container);
    lv_obj_remove_style_all(actions_btn_container);
    lv_obj_set_style_pad_all(actions_btn_container, 5, 0);
    lv_obj_set_width(actions_btn_container, lv_pct(100));
    lv_obj_set_height(actions_btn_container, LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(actions_btn_container, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(actions_btn_container, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_gap(actions_btn_container, 10, 0);

    lv_obj_t* btn_add = lv_btn_create(actions_btn_container);
    lv_obj_add_event_cb(btn_add, add_new_profile_btn_event_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_set_flex_grow(btn_add, 1);
    label_btn_add_main = lv_label_create(btn_add);
    lv_label_set_text(label_btn_add_main, LV_SYMBOL_PLUS "Add Profile"); 
    lv_obj_center(label_btn_add_main); 
    
    lv_obj_t* btn_lab_mode = lv_btn_create(actions_btn_container);
    lv_obj_add_event_cb(btn_lab_mode, laboratory_mode_btn_event_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_set_flex_grow(btn_lab_mode, 1);
    label_btn_lab_main = lv_label_create(btn_lab_mode); 
    lv_label_set_text(label_btn_lab_main, LV_SYMBOL_SETTINGS "Lab Mode"); 
    lv_obj_center(label_btn_lab_main); 

    lv_obj_t* btn_settings = lv_btn_create(actions_btn_container);
    lv_obj_add_event_cb(btn_settings, btn_goto_settings_event_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_set_flex_grow(btn_settings, 1);
    label_btn_settings_main = lv_label_create(btn_settings); 
    lv_label_set_text(label_btn_settings_main, "Settings"); 
    lv_obj_center(label_btn_settings_main); 
}

void build_profile_details_screen(lv_obj_t* parent_screen) {
    Serial.println("Building profile_details_screen UI...");

    // Создаем подложку, как на главном экране
    lv_obj_t* content_container = lv_obj_create(parent_screen);
    lv_obj_set_size(content_container, lv_pct(100), lv_pct(100));
    lv_obj_align(content_container, LV_ALIGN_TOP_LEFT, 0, 0);

    // <<<--- НАЧАЛО ИЗМЕНЕНИЙ: ЕДИНЫЙ СТИЛЬ И СТРУКТУРА ---<<<
    // Применяем компоновку и отступы, как на главном экране
    lv_obj_set_flex_flow(content_container, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_all(content_container, 5, 0);
    lv_obj_set_style_pad_gap(content_container, 10, 0);
    lv_obj_set_scrollbar_mode(content_container, LV_SCROLLBAR_MODE_OFF);
    lv_obj_clear_flag(content_container, LV_OBJ_FLAG_SCROLLABLE);

    // --- 1. Заголовок "Profile:" ---
    label_detail_header = lv_label_create(content_container); 
    lv_label_set_text(label_detail_header, "Profile name and details:");
    lv_obj_set_style_text_color(label_detail_header, lv_color_hex(0x888888), 0); 
    lv_obj_set_width(label_detail_header, lv_pct(100));                          
    lv_obj_set_style_text_align(label_detail_header, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_add_style(label_detail_header, &style_my_text_18, 0); // ДОБАВЛЕНО: Применяем стиль со шрифтом  

    // --- 2. Большое имя профиля ---
    label_detail_view_profile_name = lv_label_create(content_container);
    lv_label_set_text(label_detail_view_profile_name, "Loading...");
    lv_obj_set_style_text_font(label_detail_view_profile_name, &lv_font_montserrat_32, 0);
    lv_obj_set_width(label_detail_view_profile_name, lv_pct(100));
    lv_obj_set_style_text_align(label_detail_view_profile_name, LV_TEXT_ALIGN_CENTER, 0);

    // --- 3. Блок с деталями (ID, Temp, и т.д.) ---
    lv_obj_t* details_content_block = lv_obj_create(content_container);
    lv_obj_remove_style_all(details_content_block);
    lv_obj_set_width(details_content_block, lv_pct(100));
    lv_obj_set_flex_grow(details_content_block, 1);
    lv_obj_set_flex_flow(details_content_block, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(details_content_block, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);
    lv_obj_set_style_pad_gap(details_content_block, 15, 0); // <<<--- УВЕЛИЧИЛ ВЕРТИКАЛЬНЫЙ ОТСТУП МЕЖДУ СТРОКАМИ
    
    // <<<--- ДОБАВЛЯЕМ ГОРИЗОНТАЛЬНЫЙ ОТСТУП ОТ СТЕНОК ---<<<
    lv_obj_set_style_pad_hor(details_content_block, 15, 0);
    lv_obj_set_style_pad_ver(details_content_block, 10, 0);

    label_detail_view_id = lv_label_create(details_content_block);
    lv_label_set_text(label_detail_view_id, "ID: --");
    lv_obj_set_width(label_detail_view_id, lv_pct(100));
    lv_obj_add_style(label_detail_view_id, &style_my_text_22, 0); 

    label_detail_view_thermal_chamber_enabled = lv_label_create(details_content_block);
    lv_label_set_text(label_detail_view_thermal_chamber_enabled, "Thermal Chamber: --");
    lv_obj_set_width(label_detail_view_thermal_chamber_enabled, lv_pct(100));
    lv_obj_add_style(label_detail_view_thermal_chamber_enabled, &style_my_text_22, 0); 

    label_detail_view_thermal_chamber = lv_label_create(details_content_block);
    lv_label_set_text(label_detail_view_thermal_chamber, "Target Temp: -- C, Hold: -- s");
    lv_obj_set_width(label_detail_view_thermal_chamber, lv_pct(100));
    lv_label_set_long_mode(label_detail_view_thermal_chamber, LV_LABEL_LONG_WRAP);
    lv_obj_add_style(label_detail_view_thermal_chamber, &style_my_text_22, 0); 

    label_detail_view_chamber_cooling = lv_label_create(details_content_block);
    lv_label_set_text(label_detail_view_chamber_cooling, "Chamber Cooling: --"); 
    lv_obj_set_width(label_detail_view_chamber_cooling, lv_pct(100));
    lv_obj_add_style(label_detail_view_chamber_cooling, &style_my_text_22, 0);

    label_detail_view_nitrogen = lv_label_create(details_content_block);
    lv_label_set_text(label_detail_view_nitrogen, "Nitrogen Use: --");
    lv_obj_set_width(label_detail_view_nitrogen, lv_pct(100));
    lv_obj_add_style(label_detail_view_nitrogen, &style_my_text_22, 0); 

    label_detail_view_primary_uv = lv_label_create(details_content_block);
    lv_label_set_text(label_detail_view_primary_uv, "Primary UV: -- s");
    lv_obj_set_width(label_detail_view_primary_uv, lv_pct(100));
    lv_obj_add_style(label_detail_view_primary_uv, &style_my_text_22, 0); 

    label_detail_view_secondary_uv = lv_label_create(details_content_block);
    lv_label_set_text(label_detail_view_secondary_uv, "Secondary UV: -- s");
    lv_obj_set_width(label_detail_view_secondary_uv, lv_pct(100));
    lv_obj_add_style(label_detail_view_secondary_uv, &style_my_text_22, 0); 

    label_detail_view_tertiary_uv = lv_label_create(details_content_block);
    lv_label_set_text(label_detail_view_tertiary_uv, "Tertiary UV: -- s"); // Текст по умолчанию
    lv_obj_set_width(label_detail_view_tertiary_uv, lv_pct(100));
    lv_obj_add_style(label_detail_view_tertiary_uv, &style_my_text_22, 0);

    // Создаем ОДИН ряд для ВСЕХ кнопок
    lv_obj_t* actions_btn_container = lv_obj_create(content_container);
    lv_obj_remove_style_all(actions_btn_container);
    lv_obj_set_style_pad_all(actions_btn_container, 5, 0);
    lv_obj_set_width(actions_btn_container, lv_pct(100));
    lv_obj_set_height(actions_btn_container, LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(actions_btn_container, LV_FLEX_FLOW_ROW); // Кнопки в ряд
    lv_obj_set_style_pad_gap(actions_btn_container, 10, 0); // Отступ между кнопками
    
    // --- Кнопка Start ---
    lv_obj_t* btn_start = lv_btn_create(actions_btn_container); 
    lv_obj_add_event_cb(btn_start, profile_detail_start_event_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_set_flex_grow(btn_start, 1); // Растягивается
    label_detail_btn_start = lv_label_create(btn_start); 
    lv_label_set_text(label_detail_btn_start, "Start"); 
    lv_obj_center(label_detail_btn_start); 

    // --- Кнопка Edit ---
    lv_obj_t* btn_edit = lv_btn_create(actions_btn_container);
    lv_obj_add_event_cb(btn_edit, profile_detail_edit_btn_event_cb, LV_EVENT_CLICKED, NULL); 
    lv_obj_set_flex_grow(btn_edit, 1); // Растягивается
    label_detail_btn_edit = lv_label_create(btn_edit); 
    lv_label_set_text(label_detail_btn_edit, "Edit"); 
    lv_obj_center(label_detail_btn_edit); 
    
    // --- Кнопка Delete ---
    lv_obj_t* btn_delete = lv_btn_create(actions_btn_container);
    lv_obj_set_style_bg_color(btn_delete, lv_palette_main(LV_PALETTE_RED), 0);
    lv_obj_add_event_cb(btn_delete, profile_detail_delete_btn_event_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_set_flex_grow(btn_delete, 1); // Растягивается
    label_detail_btn_delete = lv_label_create(btn_delete); 
    lv_label_set_text(label_detail_btn_delete, "Delete"); 
    lv_obj_center(label_detail_btn_delete); 
    
    // --- Кнопка Close ---
    lv_obj_t* btn_close = lv_btn_create(actions_btn_container);
    lv_obj_add_event_cb(btn_close, profile_detail_close_event_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_set_flex_grow(btn_close, 1); // Растягивается
    label_detail_btn_close = lv_label_create(btn_close);
    lv_label_set_text(label_detail_btn_close, "Close"); 
    lv_obj_center(label_detail_btn_close); 
}

static void build_profile_edit_screen(lv_obj_t* parent_screen) {
    Serial.println("Building REDESIGNED profile_edit_screen UI...");

    const lv_coord_t TEXT_INPUT_HEIGHT = 26;

    static lv_style_t style_edit_textarea;
    lv_style_init(&style_edit_textarea);
    lv_style_set_text_align(&style_edit_textarea, LV_TEXT_ALIGN_LEFT);
    lv_style_set_pad_left(&style_edit_textarea, 5);
    lv_style_set_pad_right(&style_edit_textarea, 5);
    lv_style_set_pad_top(&style_edit_textarea, 7);
    lv_style_set_pad_bottom(&style_edit_textarea, 7);

    static lv_style_t style_column_header;
    lv_style_init(&style_column_header);
    lv_style_set_bg_color(&style_column_header, lv_palette_lighten(LV_PALETTE_GREY, 2));
    lv_style_set_bg_opa(&style_column_header, LV_OPA_COVER);
    lv_style_set_pad_all(&style_column_header, 5);
    lv_style_set_radius(&style_column_header, 3);
    lv_style_set_width(&style_column_header, lv_pct(100));
    lv_style_set_text_align(&style_column_header, LV_TEXT_ALIGN_CENTER);

    static lv_style_t style_param_block;
    lv_style_init(&style_param_block);
    lv_style_set_border_width(&style_param_block, 1);
    lv_style_set_border_color(&style_param_block, lv_palette_main(LV_PALETTE_GREY));
    lv_style_set_radius(&style_param_block, 5);
    lv_style_set_pad_all(&style_param_block, 0);
    lv_style_set_pad_row(&style_param_block, 10);
    lv_style_set_flex_flow(&style_param_block, LV_FLEX_FLOW_COLUMN);
    lv_style_set_flex_main_place(&style_param_block, LV_FLEX_ALIGN_SPACE_BETWEEN);

    lv_obj_t* main_container = lv_obj_create(parent_screen);
    lv_obj_set_size(main_container, lv_pct(100), lv_pct(100));
    lv_obj_center(main_container);
    lv_obj_set_style_pad_all(main_container, 5, 0);

    lv_obj_set_layout(main_container, LV_LAYOUT_GRID);
    static lv_coord_t main_col_dsc[] = {LV_GRID_FR(1), LV_GRID_TEMPLATE_LAST};
    static lv_coord_t main_row_dsc[] = {
        LV_GRID_CONTENT, LV_GRID_FR(1), LV_GRID_CONTENT, LV_GRID_TEMPLATE_LAST
    };
    lv_obj_set_grid_dsc_array(main_container, main_col_dsc, main_row_dsc);

    header_container = lv_obj_create(main_container);
    lv_obj_remove_style_all(header_container);
    lv_obj_set_width(header_container, lv_pct(100));
    lv_obj_set_height(header_container, LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(header_container, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(header_container, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_gap(header_container, 10, 0);
    lv_obj_set_grid_cell(header_container, LV_GRID_ALIGN_STRETCH, 0, 1, LV_GRID_ALIGN_START, 0, 1);
    lv_obj_set_style_pad_all(header_container, 0, 0);

    label_edit_name_title = lv_label_create(header_container);
    lv_obj_add_style(label_edit_name_title, &style_my_text_18, 0);

    ta_edit_profile_name = lv_textarea_create(header_container);
    lv_obj_set_flex_grow(ta_edit_profile_name, 1);
    lv_textarea_set_one_line(ta_edit_profile_name, true);
    lv_textarea_set_max_length(ta_edit_profile_name, 200);
    lv_obj_add_event_cb(ta_edit_profile_name, alpha_textarea_focus_event_cb, LV_EVENT_FOCUSED, NULL);
    lv_obj_set_height(ta_edit_profile_name, 40);
    lv_obj_add_style(ta_edit_profile_name, &style_edit_textarea, 0);
    lv_obj_set_scrollbar_mode(ta_edit_profile_name, LV_SCROLLBAR_MODE_OFF); // <<< ИСПРАВЛЕНО

    btn_help_section = lv_btn_create(header_container);
    lv_obj_set_width(btn_help_section, LV_SIZE_CONTENT);
    lv_obj_add_event_cb(btn_help_section, help_button_event_cb, LV_EVENT_CLICKED, (void*)"open_help");
    lv_obj_set_height(btn_help_section, 30);

    label_btn_help_section = lv_label_create(btn_help_section);
    lv_obj_add_style(label_btn_help_section, &style_my_text_18, 0);
    lv_obj_center(label_btn_help_section);

    main_content_container = lv_obj_create(main_container);
    lv_obj_remove_style_all(main_content_container);
    lv_obj_set_width(main_content_container, lv_pct(100));
    lv_obj_set_flex_grow(main_content_container, 1);
    lv_obj_set_layout(main_content_container, LV_LAYOUT_GRID);
    static lv_coord_t col_dsc[] = {LV_GRID_FR(1), 2, LV_GRID_FR(1), LV_GRID_TEMPLATE_LAST};
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
    lv_obj_set_grid_cell(right_column, LV_GRID_ALIGN_STRETCH, 2, 1, LV_GRID_ALIGN_STRETCH, 0, 2);
    lv_obj_set_flex_flow(right_column, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_gap(right_column, 5, 0);

    lv_obj_t* separator = lv_obj_create(main_content_container);
    lv_obj_set_grid_cell(separator, LV_GRID_ALIGN_STRETCH, 1, 1, LV_GRID_ALIGN_STRETCH, 0, 2);
    lv_obj_set_style_bg_color(separator, lv_palette_lighten(LV_PALETTE_GREY, 1), 0);

    header_uv_params = lv_label_create(left_column);
    lv_obj_add_style(header_uv_params, &style_my_text_18, 0);
    lv_obj_add_style(header_uv_params, &style_column_header, 0);
    lv_obj_set_style_pad_all(header_uv_params, 0, 0);

    block_uv_primary = lv_obj_create(left_column);
    lv_obj_add_style(block_uv_primary, &style_param_block, 0);
    lv_obj_set_flex_grow(block_uv_primary, 6);
    lv_obj_set_width(block_uv_primary, lv_pct(100));
    lv_obj_set_flex_flow(block_uv_primary, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(block_uv_primary, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_all(block_uv_primary, 5, 0);

    {
        // --- СТРОКА 1: Название, Статус, Переключатель (без изменений) ---
        lv_obj_t* row1 = lv_obj_create(block_uv_primary);
        lv_obj_remove_style_all(row1);
        lv_obj_set_width(row1, lv_pct(100));
        lv_obj_set_height(row1, LV_SIZE_CONTENT);
        lv_obj_set_flex_flow(row1, LV_FLEX_FLOW_ROW);
        lv_obj_set_flex_align(row1, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
        label_uv_primary_title = lv_label_create(row1);
        lv_obj_add_style(label_uv_primary_title, &style_my_text_18, 0);
        label_primary_uv_mode_status = lv_label_create(row1);
        lv_obj_add_style(label_primary_uv_mode_status, &style_my_text_18, 0);
        btnm_primary_uv_mode = lv_btnmatrix_create(row1);
        lv_obj_set_size(btnm_primary_uv_mode, 150, 40);
        lv_btnmatrix_set_map(btnm_primary_uv_mode, uv_btnm_map);
        lv_btnmatrix_set_btn_ctrl_all(btnm_primary_uv_mode, LV_BTNMATRIX_CTRL_CHECKABLE);
        lv_btnmatrix_set_one_checked(btnm_primary_uv_mode, true);
        lv_obj_set_style_radius(btnm_primary_uv_mode, LV_RADIUS_CIRCLE, 0);
        lv_obj_set_style_border_width(btnm_primary_uv_mode, 1, 0);
        lv_obj_set_style_border_color(btnm_primary_uv_mode, lv_color_black(), 0);
        lv_obj_set_style_pad_all(btnm_primary_uv_mode, 3, 0);
        lv_obj_set_style_bg_opa(btnm_primary_uv_mode, LV_OPA_TRANSP, LV_PART_ITEMS);
        lv_obj_set_style_border_width(btnm_primary_uv_mode, 0, LV_PART_ITEMS);
        lv_obj_set_style_radius(btnm_primary_uv_mode, LV_RADIUS_CIRCLE, LV_PART_ITEMS);
        lv_obj_set_style_text_font(btnm_primary_uv_mode, &montserrat_rus_18, LV_PART_ITEMS);
        lv_obj_set_style_bg_color(btnm_primary_uv_mode, lv_palette_main(LV_PALETTE_BLUE), LV_PART_ITEMS | LV_STATE_CHECKED);
        lv_obj_set_style_text_color(btnm_primary_uv_mode, lv_color_white(), LV_PART_ITEMS | LV_STATE_CHECKED);
        lv_obj_add_event_cb(btnm_primary_uv_mode, uv_mode_selector_event_cb, LV_EVENT_VALUE_CHANGED, label_primary_uv_mode_status);
    }
    {
        // --- СТРОКА 2: Ячейка + Кол-во вспышек ---
        lv_obj_t* row2 = lv_obj_create(block_uv_primary);
        lv_obj_remove_style_all(row2);
        lv_obj_set_size(row2, lv_pct(100), LV_SIZE_CONTENT);
        lv_obj_set_flex_flow(row2, LV_FLEX_FLOW_ROW);
        lv_obj_set_flex_align(row2, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
        lv_obj_set_style_pad_gap(row2, 10, 0);
        ta_edit_flicker_rate = lv_textarea_create(row2);
        lv_textarea_set_one_line(ta_edit_flicker_rate, true);
        lv_obj_set_size(ta_edit_flicker_rate, 80, TEXT_INPUT_HEIGHT);
        lv_textarea_set_accepted_chars(ta_edit_flicker_rate, "0123456789");
        lv_textarea_set_max_length(ta_edit_flicker_rate, 3);
        lv_obj_add_event_cb(ta_edit_flicker_rate, numeric_textarea_focus_event_cb, LV_EVENT_FOCUSED, NULL);
        lv_obj_add_event_cb(ta_edit_flicker_rate, numeric_textarea_defocus_event_cb, LV_EVENT_DEFOCUSED, NULL);
        lv_obj_add_style(ta_edit_flicker_rate, &style_edit_textarea, 0);
        lv_obj_set_scrollbar_mode(ta_edit_flicker_rate, LV_SCROLLBAR_MODE_OFF); // <<< ИСПРАВЛЕНО
        label_edit_flicker_rate_title = lv_label_create(row2);
        lv_obj_add_style(label_edit_flicker_rate_title, &style_my_text_18, 0);
    }
    {
        lv_obj_t* row3 = lv_obj_create(block_uv_primary);
        lv_obj_remove_style_all(row3);
        lv_obj_set_size(row3, lv_pct(100), LV_SIZE_CONTENT);
        lv_obj_set_flex_flow(row3, LV_FLEX_FLOW_ROW);
        lv_obj_set_flex_align(row3, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
        lv_obj_set_style_pad_gap(row3, 10, 0);
        ta_edit_primary_uv = lv_textarea_create(row3);
        lv_textarea_set_one_line(ta_edit_primary_uv, true);
        lv_obj_set_size(ta_edit_primary_uv, 80, TEXT_INPUT_HEIGHT);
        lv_textarea_set_accepted_chars(ta_edit_primary_uv, "0123456789");
        lv_textarea_set_max_length(ta_edit_primary_uv, 2);
        lv_obj_add_event_cb(ta_edit_primary_uv, numeric_textarea_focus_event_cb, LV_EVENT_FOCUSED, NULL);
        lv_obj_add_event_cb(ta_edit_primary_uv, numeric_textarea_defocus_event_cb, LV_EVENT_DEFOCUSED, NULL);
        lv_obj_add_style(ta_edit_primary_uv, &style_edit_textarea, 0);
        lv_obj_set_scrollbar_mode(ta_edit_primary_uv, LV_SCROLLBAR_MODE_OFF); // <<< ИСПРАВЛЕНО
        label_edit_primary_uv_time_title = lv_label_create(row3);
        lv_obj_add_style(label_edit_primary_uv_time_title, &style_my_text_18, 0);
    }

    // Блок: Второй этап
    block_uv_secondary = lv_obj_create(left_column);
    lv_obj_add_style(block_uv_secondary, &style_param_block, 0);
    lv_obj_set_flex_grow(block_uv_secondary, 5);
    lv_obj_set_width(block_uv_secondary, lv_pct(100));
    lv_obj_set_flex_flow(block_uv_secondary, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(block_uv_secondary, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_all(block_uv_secondary, 5, 0);

    {
        // --- СТРОКА 1: Название, Статус, Переключатель (без изменений) ---
        lv_obj_t* row1 = lv_obj_create(block_uv_secondary);
        lv_obj_remove_style_all(row1);
        lv_obj_set_width(row1, lv_pct(100));
        lv_obj_set_height(row1, LV_SIZE_CONTENT);
        lv_obj_set_flex_flow(row1, LV_FLEX_FLOW_ROW);
        lv_obj_set_flex_align(row1, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
        label_uv_secondary_title = lv_label_create(row1);
        lv_obj_add_style(label_uv_secondary_title, &style_my_text_18, 0);
        label_secondary_uv_mode_status = lv_label_create(row1);
        lv_obj_add_style(label_secondary_uv_mode_status, &style_my_text_18, 0);
        btnm_secondary_uv_mode = lv_btnmatrix_create(row1);
        lv_obj_set_size(btnm_secondary_uv_mode, 150, 40);
        lv_btnmatrix_set_map(btnm_secondary_uv_mode, uv_btnm_map); 
        lv_btnmatrix_set_btn_ctrl_all(btnm_secondary_uv_mode, LV_BTNMATRIX_CTRL_CHECKABLE);
        lv_btnmatrix_set_one_checked(btnm_secondary_uv_mode, true);
        lv_obj_set_style_radius(btnm_secondary_uv_mode, LV_RADIUS_CIRCLE, 0);
        lv_obj_set_style_border_width(btnm_secondary_uv_mode, 1, 0);
        lv_obj_set_style_border_color(btnm_secondary_uv_mode, lv_color_black(), 0);
        lv_obj_set_style_pad_all(btnm_secondary_uv_mode, 3, 0);
        lv_obj_set_style_bg_opa(btnm_secondary_uv_mode, LV_OPA_TRANSP, LV_PART_ITEMS);
        lv_obj_set_style_border_width(btnm_secondary_uv_mode, 0, LV_PART_ITEMS);
        lv_obj_set_style_radius(btnm_secondary_uv_mode, LV_RADIUS_CIRCLE, LV_PART_ITEMS);
        lv_obj_set_style_text_font(btnm_secondary_uv_mode, &montserrat_rus_18, LV_PART_ITEMS);
        lv_obj_set_style_bg_color(btnm_secondary_uv_mode, lv_palette_main(LV_PALETTE_BLUE), LV_PART_ITEMS | LV_STATE_CHECKED);
        lv_obj_set_style_text_color(btnm_secondary_uv_mode, lv_color_white(), LV_PART_ITEMS | LV_STATE_CHECKED);
        lv_obj_add_event_cb(btnm_secondary_uv_mode, uv_mode_selector_event_cb, LV_EVENT_VALUE_CHANGED, label_secondary_uv_mode_status);
    }
    {
        // --- СТРОКА 2: Ячейка + Время работы ---
        lv_obj_t* row2 = lv_obj_create(block_uv_secondary);
        lv_obj_remove_style_all(row2);
        lv_obj_set_size(row2, lv_pct(100), LV_SIZE_CONTENT);
        lv_obj_set_flex_flow(row2, LV_FLEX_FLOW_ROW);
        lv_obj_set_flex_align(row2, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
        lv_obj_set_style_pad_gap(row2, 10, 0);
        ta_edit_secondary_uv = lv_textarea_create(row2);
        lv_textarea_set_one_line(ta_edit_secondary_uv, true);
        lv_obj_set_size(ta_edit_secondary_uv, 80, TEXT_INPUT_HEIGHT);
        lv_textarea_set_accepted_chars(ta_edit_secondary_uv, "0123456789");
        lv_textarea_set_max_length(ta_edit_secondary_uv, 3);
        lv_obj_add_event_cb(ta_edit_secondary_uv, numeric_textarea_focus_event_cb, LV_EVENT_FOCUSED, NULL);
        lv_obj_add_event_cb(ta_edit_secondary_uv, numeric_textarea_defocus_event_cb, LV_EVENT_DEFOCUSED, NULL);
        lv_obj_add_style(ta_edit_secondary_uv, &style_edit_textarea, 0);
        lv_obj_set_scrollbar_mode(ta_edit_secondary_uv, LV_SCROLLBAR_MODE_OFF); // <<< ИСПРАВЛЕНО
        label_edit_secondary_uv_time_title = lv_label_create(row2);
        lv_obj_add_style(label_edit_secondary_uv_time_title, &style_my_text_18, 0);
    }

    // Блок: Третий этап
    block_uv_tertiary = lv_obj_create(left_column);
    lv_obj_add_style(block_uv_tertiary, &style_param_block, 0);
    lv_obj_set_flex_grow(block_uv_tertiary, 5);
    lv_obj_set_width(block_uv_tertiary, lv_pct(100));
    lv_obj_set_flex_flow(block_uv_tertiary, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(block_uv_tertiary, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_all(block_uv_tertiary, 5, 0);

    {
        // --- СТРОКА 1: Название, Статус, Переключатель (без изменений) ---
        lv_obj_t* row1 = lv_obj_create(block_uv_tertiary);
        lv_obj_remove_style_all(row1);
        lv_obj_set_width(row1, lv_pct(100));
        lv_obj_set_height(row1, LV_SIZE_CONTENT);
        lv_obj_set_flex_flow(row1, LV_FLEX_FLOW_ROW);
        lv_obj_set_flex_align(row1, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
        label_uv_tertiary_title = lv_label_create(row1);
        lv_obj_add_style(label_uv_tertiary_title, &style_my_text_18, 0);
        label_tertiary_uv_mode_status = lv_label_create(row1);
        lv_obj_add_style(label_tertiary_uv_mode_status, &style_my_text_18, 0);
        btnm_tertiary_uv_mode = lv_btnmatrix_create(row1);
        lv_obj_set_size(btnm_tertiary_uv_mode, 150, 40);
        lv_btnmatrix_set_map(btnm_tertiary_uv_mode, uv_btnm_map);
        lv_btnmatrix_set_btn_ctrl_all(btnm_tertiary_uv_mode, LV_BTNMATRIX_CTRL_CHECKABLE);
        lv_btnmatrix_set_one_checked(btnm_tertiary_uv_mode, true);
        lv_obj_set_style_radius(btnm_tertiary_uv_mode, LV_RADIUS_CIRCLE, 0);
        lv_obj_set_style_border_width(btnm_tertiary_uv_mode, 1, 0);
        lv_obj_set_style_border_color(btnm_tertiary_uv_mode, lv_color_black(), 0);
        lv_obj_set_style_pad_all(btnm_tertiary_uv_mode, 3, 0);
        lv_obj_set_style_bg_opa(btnm_tertiary_uv_mode, LV_OPA_TRANSP, LV_PART_ITEMS);
        lv_obj_set_style_border_width(btnm_tertiary_uv_mode, 0, LV_PART_ITEMS);
        lv_obj_set_style_radius(btnm_tertiary_uv_mode, LV_RADIUS_CIRCLE, LV_PART_ITEMS);
        lv_obj_set_style_text_font(btnm_tertiary_uv_mode, &montserrat_rus_18, LV_PART_ITEMS);
        lv_obj_set_style_bg_color(btnm_tertiary_uv_mode, lv_palette_main(LV_PALETTE_BLUE), LV_PART_ITEMS | LV_STATE_CHECKED);
        lv_obj_set_style_text_color(btnm_tertiary_uv_mode, lv_color_white(), LV_PART_ITEMS | LV_STATE_CHECKED);
        lv_obj_add_event_cb(btnm_tertiary_uv_mode, uv_mode_selector_event_cb, LV_EVENT_VALUE_CHANGED, label_tertiary_uv_mode_status);
    }
    {
        // --- СТРОКА 2: Ячейка + Время работы ---
        lv_obj_t* row2 = lv_obj_create(block_uv_tertiary);
        lv_obj_remove_style_all(row2);
        lv_obj_set_size(row2, lv_pct(100), LV_SIZE_CONTENT);
        lv_obj_set_flex_flow(row2, LV_FLEX_FLOW_ROW);
        lv_obj_set_flex_align(row2, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
        lv_obj_set_style_pad_gap(row2, 10, 0);
        ta_edit_tertiary_uv = lv_textarea_create(row2);
        lv_textarea_set_one_line(ta_edit_tertiary_uv, true);
        lv_obj_set_size(ta_edit_tertiary_uv, 80, TEXT_INPUT_HEIGHT);
        lv_textarea_set_accepted_chars(ta_edit_tertiary_uv, "0123456789");
        lv_textarea_set_max_length(ta_edit_tertiary_uv, 3);
        lv_obj_add_event_cb(ta_edit_tertiary_uv, numeric_textarea_focus_event_cb, LV_EVENT_FOCUSED, NULL);
        lv_obj_add_event_cb(ta_edit_tertiary_uv, numeric_textarea_defocus_event_cb, LV_EVENT_DEFOCUSED, NULL);
        lv_obj_add_style(ta_edit_tertiary_uv, &style_edit_textarea, 0);
        lv_obj_set_scrollbar_mode(ta_edit_tertiary_uv, LV_SCROLLBAR_MODE_OFF); // <<< ИСПРАВЛЕНО
        label_edit_tertiary_uv_time_title = lv_label_create(row2);
        lv_obj_add_style(label_edit_tertiary_uv_time_title, &style_my_text_18, 0);
    }
    
    // 5. --- Наполнение правой колонки ---
    header_poly_params = lv_label_create(right_column);
    lv_obj_add_style(header_poly_params, &style_my_text_18, 0);
    lv_obj_add_style(header_poly_params, &style_column_header, 0);
    lv_obj_set_style_pad_all(header_poly_params, 0, 0);

    // Блок: Газы
    block_gases = lv_obj_create(right_column);
    lv_obj_set_width(block_gases, lv_pct(100));
    lv_obj_set_flex_grow(block_gases, 1); 
    lv_obj_add_style(block_gases, &style_param_block, 0);
    // <<<--- Устанавливаем компоновку и отступы для блока ---<<<
    lv_obj_set_flex_flow(block_gases, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_all(block_gases, 5, 0);
    lv_obj_set_style_pad_gap(block_gases, 10, 0);
    lv_obj_set_flex_align(block_gases, LV_FLEX_ALIGN_SPACE_AROUND, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

    { // --- СТРОКА 1: Использование Азота ---
        lv_obj_t* row = lv_obj_create(block_gases);
        lv_obj_remove_style_all(row);
        lv_obj_set_size(row, lv_pct(100), 35);
        lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW);
        lv_obj_set_flex_align(row, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
        
        label_edit_nitrogen_title = lv_label_create(row);
        lv_obj_add_style(label_edit_nitrogen_title, &style_my_text_18, 0);
        
        sw_edit_nitrogen = lv_switch_create(row);
        lv_obj_add_event_cb(sw_edit_nitrogen, profile_switch_value_changed_event_cb, LV_EVENT_VALUE_CHANGED, (void*)"nitrogen_profile");
    }

    // --- Контейнер для цели Азота (будет скрываться) ---
    nitrogen_elements_container = lv_obj_create(block_gases);
    lv_obj_remove_style_all(nitrogen_elements_container);
    lv_obj_set_size(nitrogen_elements_container, lv_pct(100), LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(nitrogen_elements_container, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(nitrogen_elements_container, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_gap(nitrogen_elements_container, 10, 0);
    ta_edit_nitrogen_target = lv_textarea_create(nitrogen_elements_container);
    lv_textarea_set_one_line(ta_edit_nitrogen_target, true);
    lv_obj_set_size(ta_edit_nitrogen_target, 80, TEXT_INPUT_HEIGHT);
    lv_textarea_set_accepted_chars(ta_edit_nitrogen_target, "0123456789");
    lv_textarea_set_max_length(ta_edit_nitrogen_target, 2);
    lv_obj_add_event_cb(ta_edit_nitrogen_target, numeric_textarea_focus_event_cb, LV_EVENT_FOCUSED, NULL);
    lv_obj_add_event_cb(ta_edit_nitrogen_target, numeric_textarea_defocus_event_cb, LV_EVENT_DEFOCUSED, NULL);
    lv_obj_add_style(ta_edit_nitrogen_target, &style_edit_textarea, 0);
    lv_obj_set_scrollbar_mode(ta_edit_nitrogen_target, LV_SCROLLBAR_MODE_OFF); // <<< ИСПРАВЛЕНО
    label_edit_nitrogen_target_title = lv_label_create(nitrogen_elements_container);
    lv_obj_add_style(label_edit_nitrogen_target_title, &style_my_text_18, 0);

    { // --- СТРОКА 2: Использование сжатого воздуха ---
        lv_obj_t* row = lv_obj_create(block_gases);
        lv_obj_remove_style_all(row);
        lv_obj_set_size(row, lv_pct(100), 35);
        lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW);
        lv_obj_set_flex_align(row, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
        
        label_edit_cooling_title = lv_label_create(row);
        lv_obj_add_style(label_edit_cooling_title, &style_my_text_18, 0);
        
        sw_edit_chamber_cooling = lv_switch_create(row);
        lv_obj_add_event_cb(sw_edit_chamber_cooling, profile_switch_value_changed_event_cb, LV_EVENT_VALUE_CHANGED, (void*)"cooling_profile");
    }

    // Блок: Термокамера
    block_thermal = lv_obj_create(right_column);
    lv_obj_set_width(block_thermal, lv_pct(100));
    lv_obj_set_flex_grow(block_thermal, 1); 
    lv_obj_add_style(block_thermal, &style_param_block, 0);
    // <<<--- Устанавливаем компоновку и отступы для блока ---<<<
    lv_obj_set_flex_flow(block_thermal, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_all(block_thermal, 5, 0);
    lv_obj_set_style_pad_gap(block_thermal, 10, 0);
    lv_obj_set_flex_align(block_thermal, LV_FLEX_ALIGN_SPACE_AROUND, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

    { // --- СТРОКА 1: Термокамера ---
        lv_obj_t* row = lv_obj_create(block_thermal);
        lv_obj_remove_style_all(row);
        lv_obj_set_size(row, lv_pct(100), 35);
        lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW);
        lv_obj_set_flex_align(row, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
        
        label_edit_thermal_chamber_title = lv_label_create(row);
        lv_obj_add_style(label_edit_thermal_chamber_title, &style_my_text_18, 0);
        
        sw_edit_thermal_chamber_enable = lv_switch_create(row);
        lv_obj_add_event_cb(sw_edit_thermal_chamber_enable, thermal_chamber_enable_switch_event_cb, LV_EVENT_VALUE_CHANGED, NULL);
    }

    // --- Контейнер для параметров термокамеры (будет скрываться) ---
    thermal_elements_container = lv_obj_create(block_thermal);
    lv_obj_remove_style_all(thermal_elements_container);
    lv_obj_set_width(thermal_elements_container, lv_pct(100));
    lv_obj_set_height(thermal_elements_container, LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(thermal_elements_container, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_gap(thermal_elements_container, 10, 0);

    {
        lv_obj_t* row = lv_obj_create(thermal_elements_container);
        lv_obj_remove_style_all(row);
        lv_obj_set_size(row, lv_pct(100), LV_SIZE_CONTENT);
        lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW);
        lv_obj_set_flex_align(row, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
        lv_obj_set_style_pad_gap(row, 10, 0);
        ta_edit_thermal_temp = lv_textarea_create(row);
        lv_textarea_set_one_line(ta_edit_thermal_temp, true);
        lv_obj_set_size(ta_edit_thermal_temp, 80, TEXT_INPUT_HEIGHT);
        lv_textarea_set_accepted_chars(ta_edit_thermal_temp, "0123456789");
        lv_textarea_set_max_length(ta_edit_thermal_temp, 2);
        lv_obj_add_event_cb(ta_edit_thermal_temp, numeric_textarea_focus_event_cb, LV_EVENT_FOCUSED, NULL);
        lv_obj_add_event_cb(ta_edit_thermal_temp, numeric_textarea_defocus_event_cb, LV_EVENT_DEFOCUSED, NULL);
        lv_obj_add_style(ta_edit_thermal_temp, &style_edit_textarea, 0);
        lv_obj_set_scrollbar_mode(ta_edit_thermal_temp, LV_SCROLLBAR_MODE_OFF); // <<< ИСПРАВЛЕНО
        label_edit_thermal_temp_title = lv_label_create(row);
        lv_obj_add_style(label_edit_thermal_temp_title, &style_my_text_18, 0);
    }
    {
        lv_obj_t* row = lv_obj_create(thermal_elements_container);
        lv_obj_remove_style_all(row);
        lv_obj_set_size(row, lv_pct(100), LV_SIZE_CONTENT);
        lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW);
        lv_obj_set_flex_align(row, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
        lv_obj_set_style_pad_gap(row, 10, 0);
        ta_edit_heat_hold = lv_textarea_create(row);
        lv_textarea_set_one_line(ta_edit_heat_hold, true);
        lv_obj_set_size(ta_edit_heat_hold, 80, TEXT_INPUT_HEIGHT);
        lv_textarea_set_accepted_chars(ta_edit_heat_hold, "0123456789");
        lv_textarea_set_max_length(ta_edit_heat_hold, 3);
        lv_obj_add_event_cb(ta_edit_heat_hold, numeric_textarea_focus_event_cb, LV_EVENT_FOCUSED, NULL);
        lv_obj_add_event_cb(ta_edit_heat_hold, numeric_textarea_defocus_event_cb, LV_EVENT_DEFOCUSED, NULL);
        lv_obj_add_style(ta_edit_heat_hold, &style_edit_textarea, 0);
        lv_obj_set_scrollbar_mode(ta_edit_heat_hold, LV_SCROLLBAR_MODE_OFF); // <<< ИСПРАВЛЕНО
        label_edit_heat_hold_title = lv_label_create(row);
        lv_obj_add_style(label_edit_heat_hold_title, &style_my_text_18, 0);
    }

    // 6. --- Footer: Кнопки ---
    footer_container = lv_obj_create(main_container);
    
    lv_obj_remove_style_all(footer_container);
    lv_obj_set_width(footer_container, lv_pct(100));
    lv_obj_set_height(footer_container, LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(footer_container, LV_FLEX_FLOW_ROW);
    lv_obj_set_style_pad_gap(footer_container, 20, 0);
    lv_obj_set_style_border_width(footer_container, 0, 0);
    lv_obj_set_style_pad_all(footer_container, 5, 0);
    lv_obj_set_grid_cell(footer_container, LV_GRID_ALIGN_STRETCH, 0, 1, LV_GRID_ALIGN_END, 2, 1);

    btn_save_changes = lv_btn_create(footer_container);
    lv_obj_set_flex_grow(btn_save_changes, 1);
    lv_obj_add_event_cb(btn_save_changes, profile_edit_save_changes_btn_event_cb, LV_EVENT_CLICKED, NULL);
    label_btn_save = lv_label_create(btn_save_changes);
    lv_obj_add_style(label_btn_save, &style_my_text_18_white, 0);
    lv_obj_center(label_btn_save);
    
    btn_cancel_edit = lv_btn_create(footer_container);
    lv_obj_set_flex_grow(btn_cancel_edit, 1);
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
    Serial.println("Building process_execution_screen UI...");

    // <<<--- НАЧАЛО ИЗМЕНЕНИЙ: Создаем "подложку" ---<<<
    lv_obj_t* content_container = lv_obj_create(parent_screen);
    lv_obj_set_size(content_container, lv_pct(100), lv_pct(100));
    lv_obj_align(content_container, LV_ALIGN_TOP_LEFT, 0, 0);
    // <<<--- КОНЕЦ ИЗМЕНЕНИЙ ---<<<

    // Теперь все настройки и элементы применяются к content_container
    lv_obj_set_flex_flow(content_container, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(content_container, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_all(content_container, 20, 0);
    lv_obj_set_style_pad_gap(content_container, 20, 0);
    lv_obj_set_scrollbar_mode(content_container, LV_SCROLLBAR_MODE_OFF);

    label_process_status_title = lv_label_create(content_container); // родитель изменен
    lv_obj_add_style(label_process_status_title, &style_my_text_22, 0);
    lv_label_set_text(label_process_status_title, "Initializing Process...");
    lv_obj_set_width(label_process_status_title, LV_PCT(100));
    lv_obj_set_style_text_align(label_process_status_title, LV_TEXT_ALIGN_CENTER, 0);

    spinner_process_execution = lv_spinner_create(content_container, 1000, 60); // родитель изменен
    lv_obj_set_size(spinner_process_execution, 100, 100);
    lv_obj_center(spinner_process_execution);

    label_process_status_detail = lv_label_create(content_container); // родитель изменен
    lv_obj_add_style(label_process_status_detail, &style_my_text_18, 0);
    lv_label_set_text(label_process_status_detail, "Please wait...");
    lv_obj_set_width(label_process_status_detail, LV_PCT(100));
    lv_obj_set_style_text_align(label_process_status_detail, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_long_mode(label_process_status_detail, LV_LABEL_LONG_WRAP);

    btn_process_cancel = lv_btn_create(content_container);
    lv_obj_add_event_cb(btn_process_cancel, process_execution_cancel_btn_event_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_set_width(btn_process_cancel, LV_PCT(50));
    label_btn_process_cancel = lv_label_create(btn_process_cancel); // ИЗМЕНЕНИЕ
    lv_label_set_text(label_btn_process_cancel, "Cancel Process"); // ИЗМЕНЕНИЕ
    lv_obj_add_style(label_btn_process_cancel, &style_my_text_18_white, 0); // ДОБАВЛЕНО (белый текст и кириллица)
    lv_obj_center(label_btn_process_cancel); // ИЗМЕНЕНИЕ
    lv_obj_add_flag(btn_process_cancel, LV_OBJ_FLAG_HIDDEN);
    Serial.println("Process execution screen UI built.");
}
static void build_settings_screen(lv_obj_t* parent_screen) {
    Serial.println("Building settings_screen UI (Correct Layout)...");

    // --- Подложка на весь экран ---
    lv_obj_t* content_container = lv_obj_create(parent_screen);
    lv_obj_set_size(content_container, lv_pct(100), lv_pct(100));
    lv_obj_align(content_container, LV_ALIGN_TOP_LEFT, 0, 0);
    lv_obj_set_flex_flow(content_container, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_all(content_container, 5, 0);
    lv_obj_set_style_pad_gap(content_container, 10, 0);
    lv_obj_set_scrollbar_mode(content_container, LV_SCROLLBAR_MODE_OFF);

    // --- 1. Заголовок с секретной кнопкой ---
    lv_obj_t* header_container = lv_obj_create(content_container);
    lv_obj_remove_style_all(header_container);
    lv_obj_set_width(header_container, lv_pct(100));
    lv_obj_set_height(header_container, LV_SIZE_CONTENT);
    lv_obj_clear_flag(header_container, LV_OBJ_FLAG_CLICKABLE);

    label_settings_title = lv_label_create(header_container);
    lv_obj_add_style(label_settings_title, &style_my_text_22, 0);
    lv_obj_center(label_settings_title);
    lv_label_set_text(label_settings_title, "Global Settings"); // Начальный текст

    btn_secret_trigger = lv_btn_create(header_container);
    lv_obj_set_style_bg_opa(btn_secret_trigger, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(btn_secret_trigger, 0, 0);
    lv_obj_set_style_shadow_width(btn_secret_trigger, 0, 0);
    lv_obj_set_size(btn_secret_trigger, lv_pct(100), lv_pct(100));
    lv_obj_add_event_cb(btn_secret_trigger, secret_button_event_cb, LV_EVENT_CLICKED, NULL);

    // --- 2. Контейнер для контента настроек ---
    lv_obj_t* settings_elements_container = lv_obj_create(content_container);
    lv_obj_remove_style_all(settings_elements_container);
    lv_obj_set_width(settings_elements_container, lv_pct(100));
    lv_obj_set_flex_grow(settings_elements_container, 1);
    lv_obj_set_flex_flow(settings_elements_container, LV_FLEX_FLOW_ROW); // Две колонки в ряд
    lv_obj_set_style_pad_gap(settings_elements_container, 10, 0);

    // --- Левая колонка ---
    lv_obj_t* left_column = lv_obj_create(settings_elements_container);
    lv_obj_remove_style_all(left_column);
    lv_obj_set_flex_grow(left_column, 1);
    lv_obj_set_height(left_column, lv_pct(100));
    lv_obj_set_flex_flow(left_column, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(left_column, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_gap(left_column, 35, 0);

    // --- Правая колонка ---
    lv_obj_t* right_column = lv_obj_create(settings_elements_container);
    lv_obj_remove_style_all(right_column);
    lv_obj_set_flex_grow(right_column, 1);
    lv_obj_set_height(right_column, lv_pct(100));
    lv_obj_set_flex_flow(right_column, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(right_column, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_gap(right_column, 35, 0);

    // --- Наполнение ЛЕВОЙ колонки ---
    create_settings_row(left_column, &label_settings_nitrogen, &sw_settings_global_nitrogen_enabled, "nitro_sys");
    lv_label_set_text(label_settings_nitrogen, "Nitrogen System:"); // Начальный текст

    create_settings_row(left_column, &label_settings_air, &sw_settings_global_air_enabled, "air_sys");
    lv_label_set_text(label_settings_air, "Compressed Air:"); // Начальный текст

    // --- Наполнение ПРАВОЙ колонки ---    
    create_custom_toggle(right_column, &label_settings_language, &lang_toggle_box, &label_settings_lang_opt1, &label_settings_lang_opt2);
    lv_label_set_text(label_settings_language, "Language:");
    lv_label_set_text(label_settings_lang_opt1, "ENG");
    lv_label_set_text(label_settings_lang_opt2, "RUS");
    lv_obj_add_event_cb(lv_obj_get_child(lang_toggle_box, 0), settings_screen_event_cb, LV_EVENT_CLICKED, (void*)"lang1");
    lv_obj_add_event_cb(lv_obj_get_child(lang_toggle_box, 1), settings_screen_event_cb, LV_EVENT_CLICKED, (void*)"lang2");
    
    create_custom_toggle(right_column, &label_settings_theme, &theme_toggle_box, &label_settings_theme_opt1, &label_settings_theme_opt2);
    lv_label_set_text(label_settings_theme, "Theme:");
    lv_label_set_text(label_settings_theme_opt1, "Light");
    lv_label_set_text(label_settings_theme_opt2, "Dark");
    lv_obj_add_event_cb(lv_obj_get_child(theme_toggle_box, 0), settings_screen_event_cb, LV_EVENT_CLICKED, (void*)"theme1");
    lv_obj_add_event_cb(lv_obj_get_child(theme_toggle_box, 1), settings_screen_event_cb, LV_EVENT_CLICKED, (void*)"theme2");
    
    // --- 3. Футер с кнопкой ---
    lv_obj_t* footer_container = lv_obj_create(content_container);
    lv_obj_remove_style_all(footer_container);
    lv_obj_set_style_pad_all(footer_container, 5, 0);
    lv_obj_set_width(footer_container, lv_pct(100));
    lv_obj_set_height(footer_container, LV_SIZE_CONTENT);
    lv_obj_set_flex_align(footer_container, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

    btn_settings_save_and_back = lv_btn_create(footer_container);
    lv_obj_add_event_cb(btn_settings_save_and_back, settings_screen_event_cb, LV_EVENT_CLICKED, (void*)"back_save");
    lv_obj_set_width(btn_settings_save_and_back, lv_pct(60));
    lv_obj_t* label_btn_back = lv_label_create(btn_settings_save_and_back); // Указатель на метку кнопки может быть локальным
    lv_label_set_text(label_btn_back, "Save and Back");
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
    Serial.println("Step 1.0: Initializing board object...");
    board = new Board();
    if (!board) { handleFatalError("Failed to create Board object!"); } 
    board->init();
    Serial.println("Board object init() called.");
    esp_expander::Base* baseExpander = board->getExpander();
    if (!baseExpander) { handleFatalError("Failed to get expander from board after init!"); }
    ch422g = static_cast<esp_expander::CH422G*>(baseExpander);
    if (!ch422g) { handleFatalError("Wrong expander type or failed to cast!"); }
    Serial.println("Expander CH422G object obtained.");

    #if defined(LVGL_PORT_AVOID_TEARING_MODE) && LVGL_PORT_AVOID_TEARING_MODE
        Serial.println("Step 1.1: AVOID_TEARING_MODE is enabled, configuring frame/bounce buffers...");
        auto lcd = board->getLCD();
        if (lcd) {
            #ifndef LVGL_PORT_DISP_BUFFER_NUM
                #define LVGL_PORT_DISP_BUFFER_NUM 2
                Serial.println("Warning: LVGL_PORT_DISP_BUFFER_NUM was not defined, defaulting to 2 for tearing avoidance.");
            #endif
            lcd->configFrameBufferNumber(LVGL_PORT_DISP_BUFFER_NUM);
            Serial.printf("  Frame buffer number configured to: %d\n", (int)LVGL_PORT_DISP_BUFFER_NUM);
        #if ESP_PANEL_DRIVERS_BUS_ENABLE_RGB && CONFIG_IDF_TARGET_ESP32S3
            auto lcd_bus = lcd->getBus();
            if (lcd_bus && lcd_bus->getBasicAttributes().type == ESP_PANEL_BUS_TYPE_RGB) {
                int bounce_buffer_size_px = lcd->getFrameWidth() * 10;
                static_cast<BusRGB *>(lcd_bus)->configRGB_BounceBufferSize(bounce_buffer_size_px);
                Serial.printf("  RGB Bounce buffer size configured for up to %d pixels wide lines\n", bounce_buffer_size_px);
            }
        #endif
        } else { Serial.println("Error: Failed to get LCD from board for buffer configuration!"); }
    #else
        Serial.println("Step 1.1: AVOID_TEARING_MODE is not enabled or macro not defined.");
    #endif

    Serial.println("Step 1.2: Attempting board->begin()...");
    if (!board->begin()) { handleFatalError("Board->begin() failed!"); }
    Serial.println("Board->begin() success.");
    
    // 2. НАСТРОЙКА ПИНОВ РАСШИРИТЕЛЯ
    Serial.println("Step 2: Configuring Expander Pins...");
    if (!ch422g) { handleFatalError("CH422G not available after board->begin() for pin config."); }
    ch422g->multiPinMode(TP_RST | LCD_BL | LCD_RST, OUTPUT);
    ch422g->multiDigitalWrite(TP_RST | LCD_RST, HIGH);
    ch422g->digitalWrite(LCD_BL, HIGH);
    ch422g->multiPinMode(SD_CS | USB_SEL, OUTPUT);
    ch422g->digitalWrite(USB_SEL, LOW);
    ch422g->digitalWrite(SD_CS, HIGH);
    Serial.println("Step 2: Expander Pins configured."); 
    // delay(100);

    // 3. РАННЯЯ ИНИЦИАЛИЗАЦИЯ LVGL
    Serial.println("Step 3: Early LVGL port initialization...");
    if (!lvgl_port_init(board->getLCD(), board->getTouch())) {
        handleFatalError("lvgl_port_init failed!");
    }
    EVENT_REFRESH_PROFILES = lv_event_register_id();
    Serial.println("Step 3: LVGL port initialized.");

    // 3.1 НАСТРОЙКА СТИЛЕЙ И ТЕМЫ UI
    Serial.println("Step 3.1: Initializing UI styles...");
    
    // Инициализируем и сразу настраиваем наш новый стиль
    lv_style_init(&style_my_text_16);
    lv_style_set_text_font(&style_my_text_16, &montserrat_rus_16);
    lv_style_set_text_color(&style_my_text_16, lv_color_black());

    lv_style_init(&style_my_text_18);
    lv_style_set_text_font(&style_my_text_18, &montserrat_rus_18);
    lv_style_set_text_color(&style_my_text_18, lv_color_black());

    font_18_with_fallback = montserrat_rus_18; 
    font_18_with_fallback.fallback = &lv_font_montserrat_14; // Устанавливаем запасной шрифт
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
    // Здесь можешь добавить другие стили, если понадобятся...

    Serial.println("Step 3.1: UI styles initialized.");

    // 4. НЕМЕДЛЕННО СОЗДАЕМ И ЗАГРУЖАЕМ ЗАСТАВКУ
    if (ENABLE_SPLASH_SCREEN) {
        Serial.println("Step 4: Building and loading SPLASH SCREEN...");
        screen_splash = lv_obj_create(NULL);
        build_splash_screen(screen_splash);
        lv_scr_load(screen_splash);

        splash_screen_start_time = millis();
        is_on_splash_screen = true;
        Serial.println("Step 4: Splash screen loaded, timer started.");
    } else {
        is_on_splash_screen = false; // Убедимся, что флаг выключен
        Serial.println("Step 4: Splash screen is DISABLED. Skipping.");
    }
    // 5. ИНИЦИАЛИЗАЦИЯ SD-КАРТЫ И НАСТРОЕК
    delay(100); // Небольшая пауза
    sd_card_initialized = initializeSDCard();
    delay(100);
    Serial.println("Loading global settings...");
    loadGlobalSettings();

    // 6. СОЗДАНИЕ ОСТАЛЬНЫХ ЭКРАНОВ И UI ЭЛЕМЕНТОВ (В ФОНОВОМ РЕЖИМЕ)
    Serial.println("Step 6: Creating OTHER LVGL Screens and UI in background...");

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
    screen_secret_game = lv_obj_create(NULL);
    screen_service_lock = lv_obj_create(NULL);
    screen_help = lv_obj_create(NULL);

    // Строим все остальные экраны
    build_main_app_screen(screen_main_app);
    build_profile_details_screen(screen_profile_details);
    build_profile_edit_screen(screen_profile_edit);
    build_confirm_delete_dialog(screen_profile_details);
    build_process_execution_screen(screen_process_execution);
    build_settings_screen(screen_settings);
    build_secret_game_screen(screen_secret_game);
    build_service_lock_screen(screen_service_lock);
    build_help_screen(screen_help);

    // Создание общих клавиатур и диалогов
    kb_edit_alpha = lv_keyboard_create(lv_layer_top());
    lv_obj_add_flag(kb_edit_alpha, LV_OBJ_FLAG_HIDDEN);
    lv_keyboard_set_mode(kb_edit_alpha, LV_KEYBOARD_MODE_TEXT_LOWER);
    kb_edit_numeric = lv_keyboard_create(lv_layer_top());
    lv_obj_add_flag(kb_edit_numeric, LV_OBJ_FLAG_HIDDEN);
    lv_keyboard_set_mode(kb_edit_numeric, LV_KEYBOARD_MODE_NUMBER);
    Serial.println("Building universal info dialogs...");
    build_info_dialog(lv_layer_top());
    build_choice_dialog(lv_layer_top());

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

    Serial.println("--- System Setup Complete ---");
}

void loop() {
    
    // Эта логика будет работать, только если заставка была изначально включена
    if (ENABLE_SPLASH_SCREEN && is_on_splash_screen && (millis() - splash_screen_start_time >= 5000)) {
        is_on_splash_screen = false; 
        Serial.println("Splash screen timer elapsed. Loading appropriate screen...");
        if (current_global_settings.is_heater_error) {
            Serial.println("!!! DEVICE IS IN LOCKED STATE. Loading service screen. !!!");
            lv_label_set_text(label_service_lock_msg, "Critical Error!\nHeating element failure.\nDevice is locked.");
            lv_scr_load(screen_service_lock);
        } else {
            Serial.println("Device is OK. Loading main app screen...");
            lv_scr_load(screen_main_app);
        }
    }
    
    if (needs_list_refresh) {
        if (lv_scr_act() == screen_main_app) {
            needs_list_refresh = false; 
            Serial.println("Refreshing profile list from loop().");
            lvgl_port_lock(-1);
            displayProfileListPage();
            lvgl_port_unlock();
        }
    }

    // НОВАЯ ЛОГИКА: Постоянно слушаем UART, если запущен процесс
    if (main_process_running && MySerial1.available() > 0) {
        String response = MySerial1.readStringUntil('\n');
        response.trim();

        if (response.length() > 0) {
            Serial.printf("Data from controller: '%s'\n", response.c_str());

            // Пытаемся распарсить как JSON
            StaticJsonDocument<256> doc;
            DeserializationError error = deserializeJson(doc, response);

            // Обновляем UI, только если мы все еще на экране процесса
            if (lv_scr_act() == screen_process_execution) {
                lvgl_port_lock(-1);

                if (error) { 
                    // Не JSON -> простое статусное сообщение
                    if (current_global_settings.language == 1) { // RUS
                        // Если русский язык, сначала пытаемся перевести статус
                        lv_label_set_text(label_process_status_title, translateSystemStatus(response.c_str()));
                    } else { // ENG
                        // Если английский, выводим как есть
                        lv_label_set_text(label_process_status_title, response.c_str());
                    }
                    if(spinner_process_execution) lv_obj_add_flag(spinner_process_execution, LV_OBJ_FLAG_HIDDEN);
                } else {
                    // JSON -> пакет телеметрии
                    const char* type = doc["type"];
                    if (type && strcmp(type, "TELEMETRY") == 0) {
                        if(spinner_process_execution) lv_obj_clear_flag(spinner_process_execution, LV_OBJ_FLAG_HIDDEN);
                        
                        const char* stage = doc["stage"];
                        float temp = doc["temp"];
                        float o2 = doc["o2"];
                        int timer_rem = doc["timer_rem"];

                        if(stage) lv_label_set_text(label_process_status_title, stage);

                        char telemetry_buffer[200];
                        String timer_str = "";
                        
                        // Логика перевода для таймера
                        if (timer_rem >= 0) {
                            if (current_global_settings.language == 1) { // RUS
                                timer_str = String("Осталось: ") + String(timer_rem) + " с";
                            } else { // ENG
                                timer_str = String("Time left: ") + String(timer_rem) + "s";
                            }
                        }
                        
                        char temp_target_str[32] = ""; 
                        if (current_active_profile_data.thermal_chamber_enabled) {
                            if (current_global_settings.language == 1) { // RUS
                                snprintf(temp_target_str, sizeof(temp_target_str), " (Цель: %d C)", current_active_profile_data.thermal_chamber_temp);
                            } else { // ENG
                                snprintf(temp_target_str, sizeof(temp_target_str), " (Target: %d C)", current_active_profile_data.thermal_chamber_temp);
                            }
                        }

                        // Логика перевода для основной телеметрии
                        if (current_global_settings.language == 1) { // RUS
                            snprintf(telemetry_buffer, sizeof(telemetry_buffer),
                                    "Температура: %.1f C%s\n"
                                    "Кислород: %.1f %%\n\n"
                                    "%s",
                                    temp, temp_target_str,
                                    o2,
                                    timer_str.c_str());
                        } else { // ENG
                             snprintf(telemetry_buffer, sizeof(telemetry_buffer),
                                    "Temp: %.1f C%s\n"
                                    "Oxygen: %.1f %%\n\n"
                                    "%s",
                                    temp, temp_target_str,
                                    o2,
                                    timer_str.c_str());
                        }

                        lv_label_set_text(label_process_status_detail, telemetry_buffer);
                    }
                }

                lvgl_port_unlock();
            }

            // Логика завершения процесса
            if (response == "PROCESS_COMPLETE" || response.startsWith("ERROR:") || response.startsWith("FATAL_ERROR:")) {
                main_process_running = false; 
                
                // Обновляем кнопку, чтобы было понятно, что процесс завершен
                if (lv_scr_act() == screen_process_execution) {
                    lvgl_port_lock(-1);
                    if(spinner_process_execution) lv_obj_add_flag(spinner_process_execution, LV_OBJ_FLAG_HIDDEN);
                    if(label_btn_process_cancel) {
                        if (current_global_settings.language == 1) { // RUS
                           lv_label_set_text(label_btn_process_cancel, "Назад");
                        } else { // ENG
                           lv_label_set_text(label_btn_process_cancel, "Back");
                        }
                    }
                    lvgl_port_unlock();
                }
            }
        }
    }
}