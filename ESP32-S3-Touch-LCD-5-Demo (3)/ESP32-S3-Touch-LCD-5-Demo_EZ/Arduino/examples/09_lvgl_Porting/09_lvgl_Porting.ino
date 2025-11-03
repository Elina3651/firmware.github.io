//==========================================================================
// Includes
//==========================================================================
#include <Arduino.h>
#include <ArduinoJson.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_display_panel.hpp>
#include <lvgl.h>
#include <SD.h>
#include <SPI.h>
#include <vector>
#include <string.h>
#include "lvgl_v8_port.h"
#include "waveshare_sd_card.h"

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
    int primary_uv_exposure_sec;
    int secondary_uv_exposure_sec;
    bool chamber_cooling_enabled;
    bool primary_uv_type1_enabled;
    int  primary_uv_type1_flickers;
    bool primary_uv_type2_enabled;
    int  primary_uv_type2_flickers;
    bool secondary_uv_type1_enabled;
    bool secondary_uv_type2_enabled;
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
static uint8_t lvgl_heap[LVGL_HEAP_SIZE];
static lv_style_t style_my_text_16;
static lv_style_t style_my_text_18;
static lv_style_t style_my_text_18_white;
static lv_style_t style_just_font_18;
static lv_font_t font_18_with_fallback;
static lv_style_t style_my_text_22;
uint32_t EVENT_REFRESH_PROFILES;

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

// --- ESP-NOW Communication ---
uint8_t receiver_mac_address[] = {0xC4, 0x4F, 0x33, 0x6B, 0xAE, 0x15}; // 1
// uint8_t receiver_mac_address[] = {0xF4, 0x65, 0x0B, 0x55, 0xE4, 0xB0}; //2
esp_now_peer_info_t peerInfo;
struct_message message_to_send;
struct_message received_message;
volatile bool esp_now_send_success = false;
volatile bool esp_now_ack_received_from_peer = false;
volatile bool received_door_status_is_closed = false;
volatile int choice_dialog_result = 0;

//-------------------------------------------------
// LVGL UI Object Pointers
//-------------------------------------------------
// --- Screens & Main Containers ---
lv_obj_t * screen_splash, *screen_main_app, *screen_profile_details, *screen_profile_edit;
lv_obj_t * screen_settings, *screen_process_execution, *screen_service_lock, *screen_secret_game;
lv_obj_t * main_screen_content_container;

// --- Dialogs, Keyboards & Modal Elements ---
lv_obj_t * screen_confirm_delete_dialog, *label_confirm_delete_text;
lv_obj_t * screen_info_dialog, *label_info_dialog_text, *btn_info_dialog_ok;
lv_obj_t * screen_choice_dialog, *btn_choice_dialog_skip, *btn_choice_dialog_cancel;
lv_obj_t * kb_edit_numeric, *kb_edit_alpha, *kb_service_code;
lv_obj_t * overlay_modal_input_bg, *modal_input_container, *modal_input_title_label;
lv_obj_t * ta_modal_input, *current_target_ta;

// --- Main Screen Widgets ---
lv_obj_t * list_header_label_main, *list_profiles_main, *ta_profile_input_main;
lv_obj_t * label_status_msg_main, *ta_dummy_for_new_profile;
lv_obj_t * btn_profiles_prev, *btn_profiles_next, *btn_secret_trigger;

// --- Profile Details Screen Widgets ---
lv_obj_t * label_detail_view_profile_name, *label_detail_view_id;
lv_obj_t * label_detail_view_thermal_chamber_enabled, *label_detail_view_thermal_chamber;
lv_obj_t * label_detail_view_nitrogen, *label_detail_view_primary_uv;
lv_obj_t * label_detail_view_secondary_uv, *label_detail_view_chamber_cooling;

// --- Profile Edit Screen Widgets ---
lv_obj_t * label_edit_profile_id_val, *ta_edit_profile_name;
lv_obj_t * sw_edit_thermal_chamber_enable, *sw_edit_thermal_chamber, *slider_edit_thermal_temp;
lv_obj_t * label_edit_thermal_temp_val, *ta_edit_heat_hold, *sw_edit_nitrogen;
lv_obj_t * ta_edit_primary_uv, *ta_edit_secondary_uv, *sw_edit_chamber_cooling;
lv_obj_t * thermal_elements_container, *heat_hold_elements_container;
lv_obj_t * sw_primary_uv_type1, *slider_primary_uv_type1, *label_primary_uv_type1_val;
lv_obj_t * sw_primary_uv_type2, *slider_primary_uv_type2, *label_primary_uv_type2_val;
lv_obj_t * sw_secondary_uv_type1, *sw_secondary_uv_type2;

// --- Process Execution Screen Widgets ---
lv_obj_t * label_process_status_title, *label_process_status_detail;
lv_obj_t * spinner_process_execution, *btn_process_cancel;

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
void init_esp_now();
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
static void uv_settings_event_cb(lv_event_t * e);
static void numeric_textarea_focus_event_cb(lv_event_t * e);
static void numeric_textarea_defocus_event_cb(lv_event_t * e);
static void alpha_textarea_focus_event_cb(lv_event_t* e);
static void profile_list_prev_btn_event_cb(lv_event_t* e);
static void profile_list_next_btn_event_cb(lv_event_t* e);
static void secret_button_event_cb(lv_event_t* e);
static void secret_game_back_event_cb(lv_event_t* e);

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

// --- ESP-NOW Communication ---
void OnDataSent_callback(const uint8_t *mac_addr, esp_now_send_status_t status);
void OnDataRecv_callback(const esp_now_recv_info_t *esp_now_info, const uint8_t *incomingData, int len);
bool sendEspNowMessage(const struct_message& msg_data);

// Раздел 1: Вспомогательные функции 
// ==========================================================================
static void uv_settings_event_cb(lv_event_t * e) {
    lv_event_code_t code = lv_event_get_code(e);
    if (code != LV_EVENT_VALUE_CHANGED) return;

    lv_obj_t* target = lv_event_get_target(e);
    const char* id = (const char*)lv_event_get_user_data(e);

    // --- Логика для Primary UV ---
    if (strcmp(id, "p_uv_t1_sw") == 0 || strcmp(id, "p_uv_t2_sw") == 0) {
        bool t1_state = lv_obj_has_state(sw_primary_uv_type1, LV_STATE_CHECKED);
        bool t2_state = lv_obj_has_state(sw_primary_uv_type2, LV_STATE_CHECKED);

        if (!t1_state && !t2_state) {
            if (target == sw_primary_uv_type1) {
                lv_obj_add_state(sw_primary_uv_type2, LV_STATE_CHECKED);
            } else {
                lv_obj_add_state(sw_primary_uv_type1, LV_STATE_CHECKED);
            }
        }
        
        // Обновляем состояние слайдеров в зависимости от их свитчей
        if (lv_obj_has_state(sw_primary_uv_type1, LV_STATE_CHECKED)) {
            lv_obj_clear_state(slider_primary_uv_type1, LV_STATE_DISABLED);
            lv_obj_clear_state(label_primary_uv_type1_val, LV_STATE_DISABLED);
        } else {
            lv_obj_add_state(slider_primary_uv_type1, LV_STATE_DISABLED);
            lv_obj_add_state(label_primary_uv_type1_val, LV_STATE_DISABLED);
        }

        if (lv_obj_has_state(sw_primary_uv_type2, LV_STATE_CHECKED)) {
            lv_obj_clear_state(slider_primary_uv_type2, LV_STATE_DISABLED);
            lv_obj_clear_state(label_primary_uv_type2_val, LV_STATE_DISABLED);
        } else {
            lv_obj_add_state(slider_primary_uv_type2, LV_STATE_DISABLED);
            lv_obj_add_state(label_primary_uv_type2_val, LV_STATE_DISABLED);
        }
    }
    // --- Логика для Secondary UV ---
    else if (strcmp(id, "s_uv_t1_sw") == 0 || strcmp(id, "s_uv_t2_sw") == 0) {
        bool t1_state = lv_obj_has_state(sw_secondary_uv_type1, LV_STATE_CHECKED);
        bool t2_state = lv_obj_has_state(sw_secondary_uv_type2, LV_STATE_CHECKED);

        if (!t1_state && !t2_state) {
            if (target == sw_secondary_uv_type1) {
                lv_obj_add_state(sw_secondary_uv_type2, LV_STATE_CHECKED);
            } else {
                lv_obj_add_state(sw_secondary_uv_type1, LV_STATE_CHECKED);
            }
        }
    }
    // --- Логика для слайдеров ---
    else if (strcmp(id, "p_uv_t1_slider") == 0) {
        int value = lv_slider_get_value(target);
        lv_label_set_text_fmt(label_primary_uv_type1_val, "%d", value);
    } 
    else if (strcmp(id, "p_uv_t2_slider") == 0) {
        int value = lv_slider_get_value(target);
        lv_label_set_text_fmt(label_primary_uv_type2_val, "%d", value);
    }
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
    cache_vector.clear();
    if (!sd_card_initialized) {
        Serial.println("scanAndCacheAllProfiles: SD card not initialized.");
        return 1;
    }
    File root = fs_ref.open("/");
    if (!root) {
        Serial.println("scanAndCacheAllProfiles: Failed to open root directory.");
        return 1;
    }
    if (!root.isDirectory()) {
        Serial.println("scanAndCacheAllProfiles: Root is not a directory.");
        root.close();
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
    profile.primary_uv_exposure_sec = doc["primary_uv_exposure_sec"] | 30;
    profile.secondary_uv_exposure_sec = doc["secondary_uv_exposure_sec"] | 60;
    
    // <<<--- ЧТЕНИЕ НОВЫХ ПОЛЕЙ С ДЕФОЛТНЫМИ ЗНАЧЕНИЯМИ ---<<<
    profile.primary_uv_type1_enabled = doc["primary_uv_type1_enabled"] | true; // Включен по умолчанию
    profile.primary_uv_type1_flickers = doc["primary_uv_type1_flickers"] | 5;
    profile.primary_uv_type2_enabled = doc["primary_uv_type2_enabled"] | false; // Выключен по умолчанию
    profile.primary_uv_type2_flickers = doc["primary_uv_type2_flickers"] | 5;
    
    profile.secondary_uv_type1_enabled = doc["secondary_uv_type1_enabled"] | true; // Включен по умолчанию
    profile.secondary_uv_type2_enabled = doc["secondary_uv_type2_enabled"] | true; // Включен по умолчанию
    // >>>>> КОНЕЦ НОВЫХ ПОЛЕЙ <<<<<

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
    doc["primary_uv_exposure_sec"] = profile.primary_uv_exposure_sec;
    doc["secondary_uv_exposure_sec"] = profile.secondary_uv_exposure_sec; 
    
    // <<<--- ЗАПИСЬ НОВЫХ ПОЛЕЙ В JSON ---<<<
    doc["primary_uv_type1_enabled"] = profile.primary_uv_type1_enabled;
    doc["primary_uv_type1_flickers"] = profile.primary_uv_type1_flickers;
    doc["primary_uv_type2_enabled"] = profile.primary_uv_type2_enabled;
    doc["primary_uv_type2_flickers"] = profile.primary_uv_type2_flickers;
    
    doc["secondary_uv_type1_enabled"] = profile.secondary_uv_type1_enabled;
    doc["secondary_uv_type2_enabled"] = profile.secondary_uv_type2_enabled;
    // >>>>> КОНЕЦ НОВЫХ ПОЛЕЙ <<<<<

    doc["chamber_cooling_enabled"] = profile.chamber_cooling_enabled;
    
    size_t written = serializeJsonPretty(doc, outputBuffer, bufferSize); 
    if (written == 0 || written >= bufferSize -1 ) { Serial.println(F("serializeJsonPretty() failed or buffer too small.")); outputBuffer[bufferSize-1] = '\0'; return false; }
    return true;
}
void loadGlobalSettings() {
    Serial.println("-> Entering loadGlobalSettings...");
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

    Serial.println("<- Exiting loadGlobalSettings.");
}

void saveGlobalSettings() {
    Serial.println("   -> Entering saveGlobalSettings...");
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
    Serial.println("   <- Exiting saveGlobalSettings.");
}
int roundToStep(int value, int step) { if (step == 0) return value; return ((value + step / 2) / step) * step; }
void validate_numeric_input(lv_event_t * e, int min_val, int max_val) {
    lv_obj_t * ta = lv_event_get_target(e); const char* txt = lv_textarea_get_text(ta); int val = atoi(txt); 
    bool changed = false; if (val < min_val) { val = min_val; changed = true; } else if (val > max_val) { val = max_val; changed = true; }
    if (changed) { char buf[10]; snprintf(buf, sizeof(buf), "%d", val); lv_textarea_set_text(ta, buf); Serial.printf("Input validated and corrected to: %d\n", val); }
}
// Функция обратного вызова при отправке данных
void OnDataSent_callback(const uint8_t *mac_addr, esp_now_send_status_t status) {
    Serial.print("\r\nESP-NOW Send Status: ");
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
    if (status == ESP_NOW_SEND_SUCCESS) {
        esp_now_send_success = true;
    } else {
        esp_now_send_success = false;
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
    memset(&message_to_send, 0, sizeof(struct_message));
    strcpy(message_to_send.command, "force_stop");
    sendEspNowMessage(message_to_send);
    
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

// УПРАВЛЯЮЩАЯ ПЛАТА
void OnDataRecv_callback(const esp_now_recv_info_t *esp_now_info, const uint8_t *incomingData, int len) {
    if (len != sizeof(struct_message)) return;

    // Сначала копируем, чтобы в любом случае иметь свежие данные
    memcpy(&received_message, incomingData, sizeof(received_message));
    
    // Если процесс остановлен, выходим
    if (!main_process_running && strcmp(received_message.command, "door_status_response") != 0) {
        return; 
    }
    Serial.printf("INFO: Received command response: [%s]\n", received_message.command);

    // =================================================================================
    // === ЭТАП 0: Получен ответ о статусе двери ===
    // =================================================================================
    if (strcmp(received_message.command, "door_status_response") == 0) {
        if (!received_message.status_flag) { // Дверь открыта
            lvgl_port_lock(-1);
            lv_label_set_text(label_process_status_detail, "Warning: Door is OPEN!\nProcess Canceled.");
            lvgl_port_unlock();
            main_process_running = false; 
            return;
        }

        Serial.println("INFO: Door is closed. Sending command to confirm process start on Receiver.");
    
        // Обновляем UI, чтобы пользователь знал, что мы синхронизируемся
        lvgl_port_lock(-1);
        lv_label_set_text(label_process_status_title, "Synchronizing...");
        lv_label_set_text(label_process_status_detail, "Confirming process start...");
        lvgl_port_unlock();

        memset(&message_to_send, 0, sizeof(struct_message));
        strcpy(message_to_send.command, "confirm_process_start"); // Наша новая команда
        sendEspNowMessage(message_to_send);
        
    }

    // >>>>> ШАГ 2: ДОБАВЛЯЕМ НОВЫЙ ОБРАБОТЧИК ДЛЯ ПОДТВЕРЖДЕНИЯ СТАРТА <<<<<
    else if (strcmp(received_message.command, "ack_process_started") == 0) {
        if (!main_process_running) return;
        
        Serial.println("INFO: Receiver confirmed process start. Proceeding with logic.");
        
        // Проверяем, включена ли термокамера в профиле.
        if (current_active_profile_data.thermal_chamber_enabled) {
            // --- СЦЕНАРИЙ 1: ТЕРМОКАМЕРА ВКЛЮЧЕНА ---
            Serial.println("Profile setting: Thermal Chamber is ENABLED. Starting temperature check.");
            
            lvgl_port_lock(-1);
            lv_label_set_text(label_process_status_title, "Stage 1: Temp Check");
            lv_label_set_text(label_process_status_detail, "Requesting initial temperature...");
            lvgl_port_unlock();
            delay(1500);
            if (!main_process_running) return;

            memset(&message_to_send, 0, sizeof(struct_message));
            strcpy(message_to_send.command, "request_temp_data");
            sendEspNowMessage(message_to_send);
            
        } else {
            // --- СЦЕНАРИЙ 2: ТЕРМОКАМЕРА ВЫКЛЮЧЕНА ---
            Serial.println("Profile setting: Thermal Chamber is DISABLED. Skipping all temperature stages.");
            if (!main_process_running) return;
            
            lvgl_port_lock(-1);
            lv_label_set_text(label_process_status_title, "Stage 1: Systems ON");
            lv_label_set_text(label_process_status_detail, "Activating background systems...");
            lvgl_port_unlock();
            delay(1500);
            if (!main_process_running) return;
            
            memset(&message_to_send, 0, sizeof(struct_message));
            strcpy(message_to_send.command, "start_background_systems");
            sendEspNowMessage(message_to_send);
        }
    }
    
    // =================================================================================
    // === ЭТАП 1: Получен ответ с начальной температурой ===
    // =================================================================================
    else if (strcmp(received_message.command, "temp_data_response") == 0) {
        // --- ИСПРАВЛЕНИЕ ОТОБРАЖЕНИЯ ---
        if (received_message.value_int == -1270) {
            Serial.println("ERROR: Temperature sensor failed on receiver board!");
            lvgl_port_lock(-1);
            lv_label_set_text(label_process_status_title, "Sensor Error!");
            lv_label_set_text(label_process_status_detail, "Failed to read temperature sensor.\nProcess Halted.");
            lvgl_port_unlock();
            main_process_running = false;
            return;
        }
        int temp_int_part = received_message.value_int / 10;
        int temp_frac_part = abs(received_message.value_int % 10);
        
        lvgl_port_lock(-1);
        lv_label_set_text_fmt(label_process_status_detail, "Initial temp: %d.%d C", temp_int_part, temp_frac_part);
        lvgl_port_unlock();
        delay(1500);
        if (!main_process_running) return;

        // ЗАПУСКАЕМ ЭТАП 2
        lvgl_port_lock(-1);
        lv_label_set_text(label_process_status_title, "Stage 2: Systems ON");
        lv_label_set_text(label_process_status_detail, "Activating background systems...");
        lv_obj_clear_flag(spinner_process_execution, LV_OBJ_FLAG_HIDDEN);
        lvgl_port_unlock();
        
        memset(&message_to_send, 0, sizeof(struct_message));
        strcpy(message_to_send.command, "start_background_systems");
        sendEspNowMessage(message_to_send);
    }

    // =================================================================================
    // === ЭТАП 2: Получен ACK о включении фоновых систем ===
    // =================================================================================
    else if (strcmp(received_message.command, "ack") == 0) {
        if (!main_process_running) return;

        if (current_active_profile_data.thermal_chamber_enabled) {

            lvgl_port_lock(-1);
            lv_label_set_text(label_process_status_title, "Stage 2: Systems ON");
            lv_label_set_text(label_process_status_detail, "Background systems are active.");
            lvgl_port_unlock();
            delay(1500);
            if (!main_process_running) return;
            // --- СЦЕНАРИЙ 1: ТЕРМОКАМЕРА ВКЛЮЧЕНА. ЗАПУСКАЕМ ЭТАП 3 ---
            Serial.println("--- Starting Stage 3: Reaching Temperature ---");
            
            lvgl_port_lock(-1);
            lv_label_set_text(label_process_status_title, "Stage 3: Reaching Temp");
            lv_label_set_text(label_process_status_detail, "Getting current temperature...");
            lvgl_port_unlock();
            delay(1500);
            if (!main_process_running) return;

            memset(&message_to_send, 0, sizeof(struct_message));
            strcpy(message_to_send.command, "request_temp_data_for_stage3"); // Используем УНИКАЛЬНУЮ команду
            sendEspNowMessage(message_to_send);
            // Теперь мы ждем ответа, который будет обработан в `else if` для "temp_data_for_stage3_response"
            
        } else {
            // --- СЦЕНАРИЙ 2: ТЕРМОКАМЕРА ВЫКЛЮЧЕНА. ПРОПУСКАЕМ ЭТАП 3 ---
            lvgl_port_lock(-1);
            lv_label_set_text(label_process_status_title, "Stage 1: Systems ON");
            lv_label_set_text(label_process_status_detail, "Background systems are active.");
            lvgl_port_unlock();
            delay(1500);
            if (!main_process_running) return;

            if (current_active_profile_data.nitrogen_use_enabled) {
                if (current_global_settings.nitrogen_system_enabled){
                    lvgl_port_lock(-1);
                    lv_label_set_text(label_process_status_title, "Stage 4: Nitrogen Purge");
                    lv_label_set_text(label_process_status_detail, "Requesting initial O2...");
                    lvgl_port_unlock();
                    delay(1500); // Пауза, чтобы пользователь увидел сообщение
                    if (!main_process_running) return;

                    // --- ЗАПУСКАЕМ ЭТАП 4: АЗОТ ---                
                    memset(&message_to_send, 0, sizeof(struct_message));
                    strcpy(message_to_send.command, "request_o2_level");
                    sendEspNowMessage(message_to_send);
                } else{
                    lvgl_port_lock(-1);
                    lv_label_set_text(label_process_status_title, "Stage 4: Nitrogen Purge Error");
                    lv_label_set_text(label_process_status_detail, "Nitrogen usage is disabled in global settings");
                    lvgl_port_unlock();
                    delay(1500); // Пауза, чтобы пользователь увидел сообщение
                    if (!main_process_running) return;
                    
                    // >>>>> ИСПОЛЬЗУЕМ НОВЫЙ ДИАЛОГ <<<<<
                    show_choice_dialog("Nitrogen usage is disabled in global settings.","\nContinue without nitrogen?");
                    
                    // Входим в цикл ожидания выбора пользователя
                    while (choice_dialog_result == 0) {
                        delay(100); // Ждем, не блокируя LVGL
                        // Проверяем, не нажал ли пользователь "Cancel" на основном экране
                        if (!main_process_running) {
                            lv_obj_add_flag(screen_choice_dialog, LV_OBJ_FLAG_HIDDEN); // Прячем диалог
                            return;
                        }
                    }

                    // Обрабатываем результат
                    if (choice_dialog_result == 1) { // 1 == Skip
                        Serial.println("User chose to SKIP nitrogen stage.");
                        nitrogen_error_timer = 0;
                        // Запускаем следующий этап (Pre-cooling)
                        lvgl_port_lock(-1);
                        lv_label_set_text(label_process_status_title, "Stage 4.5: Pre-cooling");
                        lv_label_set_text(label_process_status_detail, "Activating LED radiators...");
                        lvgl_port_unlock();
                        memset(&message_to_send, 0, sizeof(struct_message));
                        strcpy(message_to_send.command, "start_extracooling");
                        sendEspNowMessage(message_to_send);
                        return;

                    } else { // 2 == Cancel
                        Serial.println("User chose to CANCEL process.");
                        // Просто останавливаем процесс, кнопка Cancel уже отправила force_stop
                        main_process_running = false;
                        nitrogen_error_timer = 0;
                        return;
                    }
                }

            } else {
                // >>>>> ЗАПУСКАЕМ ЭТАП 4.5: ОХЛАЖДЕНИЕ РАДИАТОРОВ (ПРИ ПРОПУСКЕ АЗОТА) <<<<<
                lvgl_port_lock(-1);
                lv_label_set_text(label_process_status_title, "Stage 5: Pre-cooling");
                lv_label_set_text(label_process_status_detail, "Activating LED radiators...");
                lvgl_port_unlock();
                delay(1500); // Пауза, чтобы пользователь увидел сообщение
                if (!main_process_running) return;

                memset(&message_to_send, 0, sizeof(struct_message));
                strcpy(message_to_send.command, "start_extracooling");
                sendEspNowMessage(message_to_send);
            }
        }
    }

    // =================================================================================
    // === ЭТАП 3: Получен ответ с температурой для начала регулировки (ИСПРАВЛЕНО) ===
    // =================================================================================
    else if (strcmp(received_message.command, "temp_data_for_stage3_response") == 0) {
        // --- 3.1: Получили температуру, ПРОВЕРЯЕМ и ОТОБРАЖАЕМ ---
        float current_temp_local = (float)received_message.value_int / 10.0f;

        // >>>>> ПРАВКА 1: Проверка на ошибку датчика <<<<<
        if (received_message.value_int == -1270) {
            Serial.println("ERROR: Temperature sensor failed on receiver board!");
            lvgl_port_lock(-1);
            lv_label_set_text(label_process_status_title, "Sensor Error!");
            lv_label_set_text(label_process_status_detail, "Failed to read temperature sensor.\nProcess Halted.");
            lvgl_port_unlock();
            main_process_running = false;
            return;
        }

        int target_temp = current_active_profile_data.thermal_chamber_temp;
        
        int current_int_part = (int)current_temp_local;
        int current_frac_part = abs((int)(current_temp_local * 10) % 10);

        lvgl_port_lock(-1);
        lv_label_set_text_fmt(label_process_status_detail, "Target: %d C\nCurrent: %d.%d C", target_temp, current_int_part, current_frac_part);
        lvgl_port_unlock();
        delay(1500); 
        if (!main_process_running) return;
        heater_decision = false;
        cooling_decision = false;

        // >>>>> ПРАВКА 3: Переработка логики принятия решения <<<<<
        if (current_temp_local < (float)(target_temp - 1)) {
            heater_decision = true;
            Serial.println("Decision: HEATING required.");
            sprintf(decision_text, "\nDecision: HEATING");
            sprintf(full_status_text, "Target: %d C\nCurrent: %d.%d C%s", target_temp, current_int_part, current_frac_part, decision_text);

            lvgl_port_lock(-1);
            lv_label_set_text(label_process_status_detail, full_status_text);
            lvgl_port_unlock();
            delay(1500); // Пауза, чтобы увидеть решение
            if (!main_process_running) return;

            lvgl_port_lock(-1);
            lv_label_set_text(label_process_status_detail, "Starting Heater...");
            lvgl_port_unlock();
            
            memset(&message_to_send, 0, sizeof(struct_message));
            strcpy(message_to_send.command, "start_heating_stage3");
            sendEspNowMessage(message_to_send);

        } else if (current_temp_local > (float)(target_temp + 1)) {
            if (current_global_settings.compressed_air_system_enabled) {
                cooling_decision = true;
                Serial.println("Decision: COOLING required.");
                sprintf(decision_text, "\nDecision: COOLING");
                sprintf(full_status_text, "Target: %d C\nCurrent: %d.%d C%s", target_temp, current_int_part, current_frac_part, decision_text);

                lvgl_port_lock(-1);
                lv_label_set_text(label_process_status_detail, full_status_text);
                lvgl_port_unlock();
                delay(1500); // Пауза, чтобы увидеть решение
                if (!main_process_running) return;

                lvgl_port_lock(-1);
                lv_label_set_text(label_process_status_detail, "Starting Cooling...");
                lvgl_port_unlock();
                
                memset(&message_to_send, 0, sizeof(struct_message));
                strcpy(message_to_send.command, "start_cooling_stage3");
                sendEspNowMessage(message_to_send);
            } else {
                // Охлаждение нужно, но выключено глобально
                Serial.println("Decision: COOLING required but disabled globally. SKIPPING.");
                sprintf(decision_text, "\nCooling disabled.\nSkipping stage.");
                sprintf(full_status_text, "Target: %d C\nCurrent: %d.%d C%s", target_temp, current_int_part, current_frac_part, decision_text);

                lvgl_port_lock(-1);
                lv_label_set_text(label_process_status_detail, full_status_text);
                lvgl_port_unlock();
                delay(1500); 
                if (!main_process_running) return;
                heater_decision = false;
                cooling_decision = false;

                if (current_active_profile_data.nitrogen_use_enabled) {
                    lvgl_port_lock(-1);
                    lv_label_set_text(label_process_status_title, "Stage 4: O2 Check");
                    lv_label_set_text(label_process_status_detail, "Requesting initial O2...");
                    lvgl_port_unlock();
                    delay(1500); // Пауза, чтобы пользователь увидел сообщение
                    if (!main_process_running) return;

                    // --- ЗАПУСКАЕМ ЭТАП 4: АЗОТ ---                
                    memset(&message_to_send, 0, sizeof(struct_message));
                    strcpy(message_to_send.command, "request_o2_level");
                    sendEspNowMessage(message_to_send);

                } else {
                    // >>>>> ЗАПУСКАЕМ ЭТАП 4.5: ОХЛАЖДЕНИЕ РАДИАТОРОВ (ПРИ ПРОПУСКЕ АЗОТА) <<<<<
                    lvgl_port_lock(-1);
                    lv_label_set_text(label_process_status_title, "Stage 5: Pre-cooling");
                    lv_label_set_text(label_process_status_detail, "Activating LED radiators...");
                    lvgl_port_unlock();
                    delay(1500); // Пауза, чтобы пользователь увидел сообщение
                    if (!main_process_running) return;

                    memset(&message_to_send, 0, sizeof(struct_message));
                    strcpy(message_to_send.command, "start_extracooling");
                    sendEspNowMessage(message_to_send);
                }
                return;

            }
        } else {
            // Температура уже в норме
            Serial.printf("Temperature is already at target. Skipping Reaching Temp stage.\n");
            sprintf(decision_text, "\nTemperature is OK.\nSkipping stage.");
            sprintf(full_status_text, "Target: %d C\nCurrent: %d.%d C%s", target_temp, current_int_part, current_frac_part, decision_text);

            heater_decision = true;
            cooling_decision = false;

            lvgl_port_lock(-1);
            lv_label_set_text(label_process_status_detail, full_status_text);
            lvgl_port_unlock();
            delay(1500); 
            if (!main_process_running) return;
               
            memset(&message_to_send, 0, sizeof(struct_message));
            strcpy(message_to_send.command, "start_heating_stage3");
            sendEspNowMessage(message_to_send);
        }
        
    }

    // =================================================================================
    // === ЭТАП 3.1: Контроль НАГРЕВА ===
    // =================================================================================
    else if (strcmp(received_message.command, "ack_heating_started") == 0) {
        if (!main_process_running) return;

        // При первом входе в этот режим запускаем таймер ошибки
        if (heater_decision) {
            heater_error_timer = millis();
            temp_at_heater_error_check_start = (float)received_message.value_int / 10.0f;
            heater_decision = false; // Сбрасываем флаг, чтобы больше сюда не заходить
        }

        float current_temp_local = (float)received_message.value_int / 10.0f;
        int target_temp = current_active_profile_data.thermal_chamber_temp;

        // Проверка на ошибку НАГРЕВАТЕЛЯ
        if (millis() - heater_error_timer > 100000) { // Проверяем каждые 10 секунд
            if (fabs(current_temp_local - temp_at_heater_error_check_start) < 1.0f) {
                Serial.println("CRITICAL ERROR: Temperature is not changing during HEATING. Shutting down.");
                enter_service_lock_mode("Critical Error!\nHeating element failure.");
                heater_error_timer = 0;
                return;
            }
            // Если все ОК, сбрасываем таймер и температуру для следующей проверки
            heater_error_timer = millis();
            temp_at_heater_error_check_start = current_temp_local;
        }
        
        // Отображаем статус
        int current_int_part = (int)current_temp_local;
        int current_frac_part = abs((int)(current_temp_local * 10) % 10);
        lvgl_port_lock(-1);
        lv_label_set_text(label_process_status_title, "Stage 3: Heating");
        lv_label_set_text_fmt(label_process_status_detail, "Target: %d C\nCurrent: %d.%d C", target_temp, current_int_part, current_frac_part);
        lvgl_port_unlock();

        // Проверяем, достигли ли цели
        if (current_temp_local >= target_temp) {
            Serial.println("Target temperature REACHED during heating!");
            
            lvgl_port_lock(-1);
            lv_label_set_text(label_process_status_detail, "Target Reached!\nStarting Hold phase...");
            lvgl_port_unlock();
            delay(1500);
            if (!main_process_running) return;

            // ЗАПУСКАЕМ ЭТАП УДЕРЖАНИЯ (НАГРЕВ)
            memset(&message_to_send, 0, sizeof(struct_message));
            strcpy(message_to_send.command, "start_holding_heating_stage3");
            sendEspNowMessage(message_to_send);
            return; 
        } else {
            // Цель не достигнута, продолжаем цикл
            delay(2000);
            if (!main_process_running) return;
            memset(&message_to_send, 0, sizeof(struct_message));
            strcpy(message_to_send.command, "start_heating_stage3");
            sendEspNowMessage(message_to_send);
        }
    }

    // =================================================================================
    // === ЭТАП 3.2: Контроль ОХЛАЖДЕНИЯ ===
    // =================================================================================
    else if (strcmp(received_message.command, "ack_cooling_started") == 0) {
        if (!main_process_running) return;

        // При первом входе в этот режим запускаем таймер ошибки
        if (cooling_decision) {
            air_error_timer = millis();
            temp_at_air_error_check_start = (float)received_message.value_int / 10.0f;
            cooling_decision = false; // Сбрасываем флаг
        }

        float current_temp_local = (float)received_message.value_int / 10.0f;
        int target_temp = current_active_profile_data.thermal_chamber_temp;
        
        // Проверка на ошибку СЖАТОГО ВОЗДУХА
        if (millis() - air_error_timer > 100000) { // Проверяем каждые 10 секунд
            if (fabs(current_temp_local - temp_at_air_error_check_start) < 1.0f) {
                Serial.println("WARNING: Temperature is not changing during COOLING. Skipping stage.");
                lvgl_port_lock(-1);
                lv_label_set_text(label_process_status_title, "Warning Compressed Air");
                lv_label_set_text(label_process_status_detail, "Cooling issue detected.\nCheck compressed air.\nSkipping to next stage.");
                lvgl_port_unlock();
                delay(2000);
                if (!main_process_running) return;

                if (current_active_profile_data.nitrogen_use_enabled) {
                    if (current_global_settings.nitrogen_system_enabled){
                        lvgl_port_lock(-1);
                        lv_label_set_text(label_process_status_title, "Stage 4: Nitrogen Purge");
                        lv_label_set_text(label_process_status_detail, "Requesting initial O2...");
                        lvgl_port_unlock();
                        delay(1500); // Пауза, чтобы пользователь увидел сообщение
                        if (!main_process_running) return;

                        // --- ЗАПУСКАЕМ ЭТАП 4: АЗОТ ---                
                        memset(&message_to_send, 0, sizeof(struct_message));
                        strcpy(message_to_send.command, "request_o2_level");
                        sendEspNowMessage(message_to_send);
                    } else{
                        lvgl_port_lock(-1);
                        lv_label_set_text(label_process_status_title, "Stage 4: Nitrogen Purge Error");
                        lv_label_set_text(label_process_status_detail, "Nitrogen usage is disabled in global settings");
                        lvgl_port_unlock();
                        delay(1500); // Пауза, чтобы пользователь увидел сообщение
                        if (!main_process_running) return;
                        
                        // >>>>> ИСПОЛЬЗУЕМ НОВЫЙ ДИАЛОГ <<<<<
                        show_choice_dialog("Nitrogen usage is disabled in global settings.","\nContinue without nitrogen?");
                        
                        // Входим в цикл ожидания выбора пользователя
                        while (choice_dialog_result == 0) {
                            delay(100); // Ждем, не блокируя LVGL
                            // Проверяем, не нажал ли пользователь "Cancel" на основном экране
                            if (!main_process_running) {
                                lv_obj_add_flag(screen_choice_dialog, LV_OBJ_FLAG_HIDDEN); // Прячем диалог
                                return;
                            }
                        }

                        // Обрабатываем результат
                        if (choice_dialog_result == 1) { // 1 == Skip
                            Serial.println("User chose to SKIP nitrogen stage.");
                            nitrogen_error_timer = 0;
                            // Запускаем следующий этап (Pre-cooling)
                            lvgl_port_lock(-1);
                            lv_label_set_text(label_process_status_title, "Stage 4.5: Pre-cooling");
                            lv_label_set_text(label_process_status_detail, "Activating LED radiators...");
                            lvgl_port_unlock();
                            memset(&message_to_send, 0, sizeof(struct_message));
                            strcpy(message_to_send.command, "start_extracooling");
                            sendEspNowMessage(message_to_send);
                            return;

                        } else { // 2 == Cancel
                            Serial.println("User chose to CANCEL process.");
                            // Просто останавливаем процесс, кнопка Cancel уже отправила force_stop
                            main_process_running = false;
                            nitrogen_error_timer = 0;
                            return;
                        }
                    }

                } else {
                    // >>>>> ЗАПУСКАЕМ ЭТАП 4.5: ОХЛАЖДЕНИЕ РАДИАТОРОВ (ПРИ ПРОПУСКЕ АЗОТА) <<<<<
                    lvgl_port_lock(-1);
                    lv_label_set_text(label_process_status_title, "Stage 5: Pre-cooling");
                    lv_label_set_text(label_process_status_detail, "Activating LED radiators...");
                    lvgl_port_unlock();
                    delay(1500); // Пауза, чтобы пользователь увидел сообщение
                    if (!main_process_running) return;

                    memset(&message_to_send, 0, sizeof(struct_message));
                    strcpy(message_to_send.command, "start_extracooling");
                    sendEspNowMessage(message_to_send);
                }
                return;
            }
            // Если все ОК, сбрасываем таймер и температуру
            air_error_timer = millis();
            temp_at_air_error_check_start = current_temp_local;
        }
        
        // Отображаем статус
        int current_int_part = (int)current_temp_local;
        int current_frac_part = abs((int)(current_temp_local * 10) % 10);
        lvgl_port_lock(-1);
        lv_label_set_text(label_process_status_title, "Stage 3: Cooling");
        lv_label_set_text_fmt(label_process_status_detail, "Target: %d C\nCurrent: %d.%d C", target_temp, current_int_part, current_frac_part);
        lvgl_port_unlock();

        // Проверяем, достигли ли цели
        if (current_temp_local <= target_temp) {
            Serial.println("Target temperature REACHED during cooling!");
            
            lvgl_port_lock(-1);
            lv_label_set_text(label_process_status_detail, "Target Reached!\nStarting Hold phase...");
            lvgl_port_unlock();
            delay(1500);
            if (!main_process_running) return;

            // ЗАПУСКАЕМ ЭТАП УДЕРЖАНИЯ (ОХЛАЖДЕНИЕ)
            memset(&message_to_send, 0, sizeof(struct_message));
            strcpy(message_to_send.command, "start_holding_cooling_stage3");
            sendEspNowMessage(message_to_send);
            return;
        } else {
            // Цель не достигнута, продолжаем цикл
            delay(2000);
            if (!main_process_running) return;
            memset(&message_to_send, 0, sizeof(struct_message));
            strcpy(message_to_send.command, "start_cooling_stage3");
            sendEspNowMessage(message_to_send);
        }
    }

    // =================================================================================
    // === ЭТАП 3.5: УДЕРЖАНИЕ ТЕМПЕРАТУРЫ (НАГРЕВ) ===
    // =================================================================================
    else if (strcmp(received_message.command, "ack_holding_heating_started") == 0) {
        if (!main_process_running) return;

        // При первом входе в этот блок запускаем таймер удержания
        if (hold_timer_start == 0) {
            hold_timer_start = millis();
        }

        float current_temp_local = (float)received_message.value_int / 10.0f;
        int target_temp = current_active_profile_data.thermal_chamber_temp;
        unsigned long hold_duration_ms = (unsigned long)current_active_profile_data.heat_exchange_hold_sec * 1000;
        unsigned long time_elapsed_ms = millis() - hold_timer_start;

        // 1. Проверяем, не закончилось ли время удержания
        if (time_elapsed_ms >= hold_duration_ms) {
            Serial.println("Hold time finished!");
            hold_timer_start = 0; // Сбрасываем общий таймер

            ///////////////////////////////////////////////////
            if (current_active_profile_data.nitrogen_use_enabled) {
                if (current_global_settings.nitrogen_system_enabled){
                    lvgl_port_lock(-1);
                    lv_label_set_text(label_process_status_title, "Stage 4: Nitrogen Purge");
                    lv_label_set_text(label_process_status_detail, "Requesting initial O2...");
                    lvgl_port_unlock();
                    delay(1500); // Пауза, чтобы пользователь увидел сообщение
                    if (!main_process_running) return;

                    // --- ЗАПУСКАЕМ ЭТАП 4: АЗОТ ---                
                    memset(&message_to_send, 0, sizeof(struct_message));
                    strcpy(message_to_send.command, "request_o2_level");
                    sendEspNowMessage(message_to_send);
                } else{
                    lvgl_port_lock(-1);
                    lv_label_set_text(label_process_status_title, "Stage 4: Nitrogen Purge Error");
                    lv_label_set_text(label_process_status_detail, "Nitrogen usage is disabled in global settings");
                    lvgl_port_unlock();
                    delay(1500); // Пауза, чтобы пользователь увидел сообщение
                    if (!main_process_running) return;
                    
                    // >>>>> ИСПОЛЬЗУЕМ НОВЫЙ ДИАЛОГ <<<<<
                    show_choice_dialog("Nitrogen usage is disabled in global settings.","\nContinue without nitrogen?");
                    
                    // Входим в цикл ожидания выбора пользователя
                    while (choice_dialog_result == 0) {
                        delay(100); // Ждем, не блокируя LVGL
                        // Проверяем, не нажал ли пользователь "Cancel" на основном экране
                        if (!main_process_running) {
                            lv_obj_add_flag(screen_choice_dialog, LV_OBJ_FLAG_HIDDEN); // Прячем диалог
                            return;
                        }
                    }

                    // Обрабатываем результат
                    if (choice_dialog_result == 1) { // 1 == Skip
                        Serial.println("User chose to SKIP nitrogen stage.");
                        nitrogen_error_timer = 0;
                        // Запускаем следующий этап (Pre-cooling)
                        lvgl_port_lock(-1);
                        lv_label_set_text(label_process_status_title, "Stage 4.5: Pre-cooling");
                        lv_label_set_text(label_process_status_detail, "Activating LED radiators...");
                        lvgl_port_unlock();
                        memset(&message_to_send, 0, sizeof(struct_message));
                        strcpy(message_to_send.command, "start_extracooling");
                        sendEspNowMessage(message_to_send);
                        return;

                    } else { // 2 == Cancel
                        Serial.println("User chose to CANCEL process.");
                        // Просто останавливаем процесс, кнопка Cancel уже отправила force_stop
                        main_process_running = false;
                        nitrogen_error_timer = 0;
                        return;
                    }
                }

            } else {
                // >>>>> ЗАПУСКАЕМ ЭТАП 4.5: ОХЛАЖДЕНИЕ РАДИАТОРОВ (ПРИ ПРОПУСКЕ АЗОТА) <<<<<
                lvgl_port_lock(-1);
                lv_label_set_text(label_process_status_title, "Stage 5: Pre-cooling");
                lv_label_set_text(label_process_status_detail, "Activating LED radiators...");
                lvgl_port_unlock();
                delay(1500); // Пауза, чтобы пользователь увидел сообщение
                if (!main_process_running) return;

                memset(&message_to_send, 0, sizeof(struct_message));
                strcpy(message_to_send.command, "start_extracooling");
                sendEspNowMessage(message_to_send);
            }
            //////////////////////////////////////////////////////
            return;
        }

        // 2. Отображаем статус
        int current_int_part = (int)current_temp_local;
        int current_frac_part = abs((int)(current_temp_local * 10) % 10);
        int time_left_sec = (int)((hold_duration_ms - time_elapsed_ms) / 1000);

        lvgl_port_lock(-1);
        lv_label_set_text_fmt(label_process_status_detail, "Holding at %d C\nCurrent: %d.%d C\nTime left: %d s", 
                            target_temp, current_int_part, current_frac_part, time_left_sec);
        lvgl_port_unlock();

        // 3. Принимаем решение о коррекции и проверяем на ошибку
        if (current_temp_local < target_temp) {
            // --- Температура упала, нужно подогреть ---
            Serial.println("Hold: Temp too low. Activating heater.");

            // При первом входе в режим коррекции запускаем таймер ошибки
            if (heater_error_timer == 0) {
                heater_error_timer = millis();
                temp_at_heater_error_check_start = current_temp_local;
            }

            // // Проверяем, если мы уже 10 секунд пытаемся греть безрезультатно
            // if (millis() - heater_error_timer > 100000) { 
            //     if (fabs(current_temp_local - temp_at_heater_error_check_start) < 1.0f) {
            //         Serial.println("CRITICAL ERROR: Temperature is not changing during HEATING. Shutting down.");
            //         enter_service_lock_mode("Critical Error!\nHeating element failure.");

            //         hold_timer_start = 0;
            //         return;
            //     }
            //     // Если все нормально, сбрасываем таймер для следующей 10-секундной проверки
            //     heater_error_timer = millis();
            //     temp_at_heater_error_check_start = current_temp_local;
            // }
            
            // Отправляем команду на включение нагревателя
            memset(&message_to_send, 0, sizeof(struct_message));
            strcpy(message_to_send.command, "start_holding_heating_stage3_with_heater");
            sendEspNowMessage(message_to_send);

        } else {
            // --- Температура в норме или выше, просто ждем ---
            heater_error_timer = 0; // Сбрасываем таймер ошибки, т.к. мы не греем
            
            delay(1000); // Пауза 1 секунда
            if (!main_process_running) { hold_timer_start = 0; return; }
            
            memset(&message_to_send, 0, sizeof(struct_message));
            strcpy(message_to_send.command, "start_holding_heating_stage3");
            sendEspNowMessage(message_to_send);
        }
    }

    // =================================================================================
    // === ЭТАП 3.5: УДЕРЖАНИЕ ТЕМПЕРАТУРЫ (ОХЛАЖДЕНИЕ) ===
    // =================================================================================
    else if (strcmp(received_message.command, "ack_holding_cooling_started") == 0) {
        if (!main_process_running) return;

        // При первом входе в этот блок запускаем таймер удержания
        if (hold_timer_start == 0) {
            hold_timer_start = millis();
        }

        float current_temp_local = (float)received_message.value_int / 10.0f;
        int target_temp = current_active_profile_data.thermal_chamber_temp;
        unsigned long hold_duration_ms = (unsigned long)current_active_profile_data.heat_exchange_hold_sec * 1000;
        unsigned long time_elapsed_ms = millis() - hold_timer_start;

        // 1. Проверяем, не закончилось ли время удержания
        if (time_elapsed_ms >= hold_duration_ms) {
            Serial.println("Hold time finished!");
            hold_timer_start = 0;

            ///////////////////////////////////////////////////
            if (current_active_profile_data.nitrogen_use_enabled) {
                if (current_global_settings.nitrogen_system_enabled){
                    lvgl_port_lock(-1);
                    lv_label_set_text(label_process_status_title, "Stage 4: Nitrogen Purge");
                    lv_label_set_text(label_process_status_detail, "Requesting initial O2...");
                    lvgl_port_unlock();
                    delay(1500); // Пауза, чтобы пользователь увидел сообщение
                    if (!main_process_running) return;

                    // --- ЗАПУСКАЕМ ЭТАП 4: АЗОТ ---                
                    memset(&message_to_send, 0, sizeof(struct_message));
                    strcpy(message_to_send.command, "request_o2_level");
                    sendEspNowMessage(message_to_send);
                } else{
                    lvgl_port_lock(-1);
                    lv_label_set_text(label_process_status_title, "Stage 4: Nitrogen Purge Error");
                    lv_label_set_text(label_process_status_detail, "Nitrogen usage is disabled in global settings");
                    lvgl_port_unlock();
                    delay(1500); // Пауза, чтобы пользователь увидел сообщение
                    if (!main_process_running) return;
                    
                    // >>>>> ИСПОЛЬЗУЕМ НОВЫЙ ДИАЛОГ <<<<<
                    show_choice_dialog("Nitrogen usage is disabled in global settings.","\nContinue without nitrogen?");
                    
                    // Входим в цикл ожидания выбора пользователя
                    while (choice_dialog_result == 0) {
                        delay(100); // Ждем, не блокируя LVGL
                        // Проверяем, не нажал ли пользователь "Cancel" на основном экране
                        if (!main_process_running) {
                            lv_obj_add_flag(screen_choice_dialog, LV_OBJ_FLAG_HIDDEN); // Прячем диалог
                            return;
                        }
                    }

                    // Обрабатываем результат
                    if (choice_dialog_result == 1) { // 1 == Skip
                        Serial.println("User chose to SKIP nitrogen stage.");
                        nitrogen_error_timer = 0;
                        // Запускаем следующий этап (Pre-cooling)
                        lvgl_port_lock(-1);
                        lv_label_set_text(label_process_status_title, "Stage 4.5: Pre-cooling");
                        lv_label_set_text(label_process_status_detail, "Activating LED radiators...");
                        lvgl_port_unlock();
                        memset(&message_to_send, 0, sizeof(struct_message));
                        strcpy(message_to_send.command, "start_extracooling");
                        sendEspNowMessage(message_to_send);
                        return;

                    } else { // 2 == Cancel
                        Serial.println("User chose to CANCEL process.");
                        // Просто останавливаем процесс, кнопка Cancel уже отправила force_stop
                        main_process_running = false;
                        nitrogen_error_timer = 0;
                        return;
                    }
                }

            } else {
                // >>>>> ЗАПУСКАЕМ ЭТАП 4.5: ОХЛАЖДЕНИЕ РАДИАТОРОВ (ПРИ ПРОПУСКЕ АЗОТА) <<<<<
                lvgl_port_lock(-1);
                lv_label_set_text(label_process_status_title, "Stage 5: Pre-cooling");
                lv_label_set_text(label_process_status_detail, "Activating LED radiators...");
                lvgl_port_unlock();
                delay(1500); // Пауза, чтобы пользователь увидел сообщение
                if (!main_process_running) return;

                memset(&message_to_send, 0, sizeof(struct_message));
                strcpy(message_to_send.command, "start_extracooling");
                sendEspNowMessage(message_to_send);
            }
            //////////////////////////////////////////////////////
            return;
        }

        // 2. Отображаем статус
        int current_int_part = (int)current_temp_local;
        int current_frac_part = abs((int)(current_temp_local * 10) % 10);
        int time_left_sec = (int)((hold_duration_ms - time_elapsed_ms) / 1000);
        
        lvgl_port_lock(-1);
        lv_label_set_text_fmt(label_process_status_detail, "Holding at %d C\nCurrent: %d.%d C\nTime left: %d s",
                            target_temp, current_int_part, current_frac_part, time_left_sec);
        lvgl_port_unlock();

        // 3. Принимаем решение о коррекции и проверяем на ошибку
        if (current_temp_local > target_temp) {
            // --- Температура поднялась, нужно охладить ---
            Serial.println("Hold: Temp too high. Activating air valve.");

            // При первом входе в режим коррекции запускаем таймер ошибки
            if (air_error_timer == 0) {
                air_error_timer = millis();
                temp_at_air_error_check_start = current_temp_local;
            }

            // Проверяем, если мы уже 10 секунд пытаемся охлаждать безрезультатно
            if (millis() - air_error_timer > 100000) {
                if (fabs(current_temp_local - temp_at_air_error_check_start) < 1.0f) {
                    Serial.println("WARNING: Temperature is not changing during COOLING. Skipping stage.");
                    lvgl_port_lock(-1);
                    lv_label_set_text(label_process_status_title, "Warning");
                    lv_label_set_text(label_process_status_detail, "Cooling issue detected.\nCheck compressed air.\nSkipping to next stage.");
                    lvgl_port_unlock();
                    delay(2000);

                    if (current_active_profile_data.nitrogen_use_enabled) {
                        if (current_global_settings.nitrogen_system_enabled){
                            lvgl_port_lock(-1);
                            lv_label_set_text(label_process_status_title, "Stage 4: Nitrogen Purge");
                            lv_label_set_text(label_process_status_detail, "Requesting initial O2...");
                            lvgl_port_unlock();
                            delay(1500); // Пауза, чтобы пользователь увидел сообщение
                            if (!main_process_running) return;

                            // --- ЗАПУСКАЕМ ЭТАП 4: АЗОТ ---                
                            memset(&message_to_send, 0, sizeof(struct_message));
                            strcpy(message_to_send.command, "request_o2_level");
                            sendEspNowMessage(message_to_send);
                        } else{
                            lvgl_port_lock(-1);
                            lv_label_set_text(label_process_status_title, "Stage 4: Nitrogen Purge Error");
                            lv_label_set_text(label_process_status_detail, "Nitrogen usage is disabled in global settings");
                            lvgl_port_unlock();
                            delay(1500); // Пауза, чтобы пользователь увидел сообщение
                            if (!main_process_running) return;
                            
                            // >>>>> ИСПОЛЬЗУЕМ НОВЫЙ ДИАЛОГ <<<<<
                            show_choice_dialog("Nitrogen usage is disabled in global settings.","\nContinue without nitrogen?");
                            
                            // Входим в цикл ожидания выбора пользователя
                            while (choice_dialog_result == 0) {
                                delay(100); // Ждем, не блокируя LVGL
                                // Проверяем, не нажал ли пользователь "Cancel" на основном экране
                                if (!main_process_running) {
                                    lv_obj_add_flag(screen_choice_dialog, LV_OBJ_FLAG_HIDDEN); // Прячем диалог
                                    return;
                                }
                            }

                            // Обрабатываем результат
                            if (choice_dialog_result == 1) { // 1 == Skip
                                Serial.println("User chose to SKIP nitrogen stage.");
                                nitrogen_error_timer = 0;
                                // Запускаем следующий этап (Pre-cooling)
                                lvgl_port_lock(-1);
                                lv_label_set_text(label_process_status_title, "Stage 4.5: Pre-cooling");
                                lv_label_set_text(label_process_status_detail, "Activating LED radiators...");
                                lvgl_port_unlock();
                                memset(&message_to_send, 0, sizeof(struct_message));
                                strcpy(message_to_send.command, "start_extracooling");
                                sendEspNowMessage(message_to_send);
                                return;

                            } else { // 2 == Cancel
                                Serial.println("User chose to CANCEL process.");
                                // Просто останавливаем процесс, кнопка Cancel уже отправила force_stop
                                main_process_running = false;
                                nitrogen_error_timer = 0;
                                return;
                            }
                        }

                    } else {
                        // >>>>> ЗАПУСКАЕМ ЭТАП 4.5: ОХЛАЖДЕНИЕ РАДИАТОРОВ (ПРИ ПРОПУСКЕ АЗОТА) <<<<<
                        lvgl_port_lock(-1);
                        lv_label_set_text(label_process_status_title, "Stage 5: Pre-cooling");
                        lv_label_set_text(label_process_status_detail, "Activating LED radiators...");
                        lvgl_port_unlock();
                        delay(1500); // Пауза, чтобы пользователь увидел сообщение
                        if (!main_process_running) return;

                        memset(&message_to_send, 0, sizeof(struct_message));
                        strcpy(message_to_send.command, "start_extracooling");
                        sendEspNowMessage(message_to_send);
                    }
                    return;
                }
                // Если все нормально, сбрасываем таймер для следующей 10-секундной проверки
                air_error_timer = millis();
                temp_at_air_error_check_start = current_temp_local;
            }

            // Отправляем команду на включение охлаждения
            memset(&message_to_send, 0, sizeof(struct_message));
            strcpy(message_to_send.command, "start_holding_cooling_stage3_with_air");
            sendEspNowMessage(message_to_send);
        } else {
            // --- Температура в норме или ниже, просто ждем ---
            air_error_timer = 0; // Сбрасываем таймер ошибки, т.к. мы не охлаждаем
            
            delay(1000); // Пауза 1 секунда
            if (!main_process_running) { hold_timer_start = 0; return; }
            
            memset(&message_to_send, 0, sizeof(struct_message));
            strcpy(message_to_send.command, "start_holding_cooling_stage3");
            sendEspNowMessage(message_to_send);
        }
    }
    // =================================================================================
    // === ЭТАП 4: Получен ответ с уровнем кислорода ===
    // =================================================================================
    else if (strcmp(received_message.command, "o2_level_response") == 0) {
        if (!main_process_running) return;

        // Проверяем, не пришла ли ошибка от датчика (-1.0f -> -10)
        if (received_message.value_int < 0) {
            Serial.println("ERROR: O2 sensor failed on receiver board!");
            lvgl_port_lock(-1);
            lv_label_set_text(label_process_status_title, "Sensor Error!");
            lv_label_set_text(label_process_status_detail, "Failed to read O2 sensor.\nMake Choose");
            lvgl_port_unlock();

            // >>>>> ИСПОЛЬЗУЕМ НОВЫЙ ДИАЛОГ <<<<<
            show_choice_dialog("Oxygen sensor error.\nContact service.","\nContinue without nitrogen?");
            
            // Входим в цикл ожидания выбора пользователя
            while (choice_dialog_result == 0) {
                delay(100); // Ждем, не блокируя LVGL
                // Проверяем, не нажал ли пользователь "Cancel" на основном экране
                if (!main_process_running) {
                    lv_obj_add_flag(screen_choice_dialog, LV_OBJ_FLAG_HIDDEN); // Прячем диалог
                    return;
                }
            }

            // Обрабатываем результат
            if (choice_dialog_result == 1) { // 1 == Skip
                Serial.println("User chose to SKIP nitrogen stage.");
                nitrogen_error_timer = 0;
                // Запускаем следующий этап (Pre-cooling)
                lvgl_port_lock(-1);
                lv_label_set_text(label_process_status_title, "Stage 4.5: Pre-cooling");
                lv_label_set_text(label_process_status_detail, "Activating LED radiators...");
                lvgl_port_unlock();
                memset(&message_to_send, 0, sizeof(struct_message));
                strcpy(message_to_send.command, "start_extracooling");
                sendEspNowMessage(message_to_send);
                return;

            } else { // 2 == Cancel
                Serial.println("User chose to CANCEL process.");
                // Просто останавливаем процесс, кнопка Cancel уже отправила force_stop
                main_process_running = false;
                nitrogen_error_timer = 0;
                return;
            }
        }

        float current_o2_level = (float)received_message.value_int / 10.0f;
        const float TARGET_O2_LEVEL = 5.0f;
        int target_o2_int = (int)TARGET_O2_LEVEL;

        // Разбиваем O2 на безопасные для вывода части
        int o2_int_part = (int)current_o2_level;
        int o2_frac_part = abs((int)(current_o2_level * 10) % 10);

        // Отображаем начальный статус
        lvgl_port_lock(-1);
        lv_label_set_text(label_process_status_title, "Stage 4: Nitrogen Purge");
        lv_label_set_text_fmt(label_process_status_detail, "Target O2: < %d %%\nCurrent: %d.%d %%\nAir mixture analysise...", target_o2_int, o2_int_part, o2_frac_part);
        lvgl_port_unlock();

        // Проверяем, достигли ли цели
        if (current_o2_level <= TARGET_O2_LEVEL) {
            Serial.println("Target O2 level REACHED!");
            nitrogen_error_timer = 0;
            
            // Добавляем финальное сообщение
            lvgl_port_lock(-1);
            lv_label_set_text_fmt(label_process_status_detail, "Target O2: < %d %%\nCurrent: %d.%d %%\nTarget Reached!", target_o2_int, o2_int_part, o2_frac_part);
            lvgl_port_unlock();
            delay(1500);
            if (!main_process_running) return;

            // >>>>> ЗАПУСКАЕМ ЭТАП 4.5: ОХЛАЖДЕНИЕ РАДИАТОРОВ <<<<<
            lvgl_port_lock(-1);
            lv_label_set_text(label_process_status_title, "Stage 5: Pre-cooling");
            lv_label_set_text(label_process_status_detail, "Activating LED radiators...");
            lvgl_port_unlock();
            delay(1500);
            if (!main_process_running) return;

            memset(&message_to_send, 0, sizeof(struct_message));
            strcpy(message_to_send.command, "start_extracooling");
            sendEspNowMessage(message_to_send);
            return;
        } else {
            // --- Цель не достигнута, включаем продувку и проверяем на ошибку ---
            
            // При первом входе в режим продувки запускаем таймер
            if (nitrogen_error_timer == 0) {
                nitrogen_error_timer = millis();
                o2_at_error_check_start = current_o2_level;
            }

            // Проверяем каждые 10 секунд
            if (millis() - nitrogen_error_timer > 100000) {
                // Если за 10 секунд уровень O2 не упал хотя бы на 1%
                if ((o2_at_error_check_start - current_o2_level) < 1.0f) {
                    Serial.println("WARNING: O2 level is not decreasing. Check nitrogen supply.");

                    // >>>>> ИСПОЛЬЗУЕМ НОВЫЙ ДИАЛОГ <<<<<
                    show_choice_dialog("Nitrogen Supply Issue", "O2 level is not decreasing.\nCheck nitrogen supply and valve.\n\nContinue without nitrogen?");
                    
                    // Входим в цикл ожидания выбора пользователя
                    while (choice_dialog_result == 0) {
                        delay(100); // Ждем, не блокируя LVGL
                        // Проверяем, не нажал ли пользователь "Cancel" на основном экране
                        if (!main_process_running) {
                            lv_obj_add_flag(screen_choice_dialog, LV_OBJ_FLAG_HIDDEN); // Прячем диалог
                            return;
                        }
                    }

                    // Обрабатываем результат
                    if (choice_dialog_result == 1) { // 1 == Skip
                        Serial.println("User chose to SKIP nitrogen stage.");
                        nitrogen_error_timer = 0;
                        // Запускаем следующий этап (Pre-cooling)
                        lvgl_port_lock(-1);
                        lv_label_set_text(label_process_status_title, "Stage 4.5: Pre-cooling");
                        lv_label_set_text(label_process_status_detail, "Activating LED radiators...");
                        lvgl_port_unlock();
                        memset(&message_to_send, 0, sizeof(struct_message));
                        strcpy(message_to_send.command, "start_extracooling");
                        sendEspNowMessage(message_to_send);
                        return;

                    } else { // 2 == Cancel
                        Serial.println("User chose to CANCEL process.");
                        // Просто останавливаем процесс, кнопка Cancel уже отправила force_stop
                        main_process_running = false;
                        nitrogen_error_timer = 0;
                        return;
                    }
                }
                // Если все нормально, сбрасываем таймер и точку отсчета
                nitrogen_error_timer = millis();
                o2_at_error_check_start = current_o2_level;
            }
            
            // --- Продолжаем продувку ---
            delay(1500); // Пауза, чтобы увидеть начальные цифры
            if (!main_process_running) { nitrogen_error_timer = 0; return; }

            // Формируем общую строку и выводим
            char full_status_text[128];
            sprintf(full_status_text, "Target O2: < %d %%\nCurrent: %d.%d %%\nNitrogen purge active...", target_o2_int, o2_int_part, o2_frac_part);

            lvgl_port_lock(-1);
            lv_label_set_text(label_process_status_detail, full_status_text);
            lvgl_port_unlock();

            // Отправляем команду на продувку
            memset(&message_to_send, 0, sizeof(struct_message));
            strcpy(message_to_send.command, "start_nitrogen_purge");
            sendEspNowMessage(message_to_send);
        }
    }

    // =================================================================================
    // === ЭТАП 4.1: Получен ACK о включении продувки азотом ===
    // =================================================================================
    else if (strcmp(received_message.command, "ack_nitrogen_purging") == 0) {
        if (!main_process_running) return;
        
        delay(2000); // Ждем 2 секунды, пока клапан открыт
        if (!main_process_running) return;

        // Снова запрашиваем уровень кислорода, чтобы зациклить процесс
        memset(&message_to_send, 0, sizeof(struct_message));
        strcpy(message_to_send.command, "request_o2_level");
        sendEspNowMessage(message_to_send);
    }

    // =================================================================================
    // === ЭТАП 4.5: Получен ACK о включении охлаждения радиаторов ===
    // =================================================================================
    else if (strcmp(received_message.command, "ack_extracooling_started") == 0) {
        if (!main_process_running) return;

        Serial.println("LED radiators are active. Moving to Primary UV stage.");
        lvgl_port_lock(-1);
        lv_label_set_text(label_process_status_detail, "Radiators ON.\nStarting Primary UV...");
        lvgl_port_unlock();
        delay(1500);
        if (!main_process_running) return;

        // >>>>> ЗАПУСКАЕМ ПЕРВЫЙ ЦИКЛ МЕРЦАНИЯ ЭТАПА 5 <<<<<
        Serial.println("--- Starting Stage 5: Primary UV Flicker ---");
        lvgl_port_lock(-1);
        lv_label_set_text(label_process_status_title, "Stage 5: Primary UV");
        lv_label_set_text(label_process_status_detail, "Flickering...");
        lvgl_port_unlock();

        // Готовим команду для ПП
        const int flicker_405_per_sec = current_active_profile_data.primary_uv_type1_enabled ? current_active_profile_data.primary_uv_type1_flickers : 0;
        const int flicker_430_per_sec = current_active_profile_data.primary_uv_type2_enabled ? current_active_profile_data.primary_uv_type2_flickers : 0;

        memset(&message_to_send, 0, sizeof(struct_message));
        strcpy(message_to_send.command, "do_uv_flicker");
        message_to_send.value_flickering_405 = flicker_405_per_sec;
        message_to_send.value_flickering_430 = flicker_430_per_sec;
        sendEspNowMessage(message_to_send);
        // Теперь ждем ответа "ack_flicker_done", который будет обработан в НОВОМ else if блоке
    }
    // =================================================================================
    // === ЭТАП 5: Получен ACK о завершении 1 секунды мерцания ===
    // =================================================================================
    else if (strcmp(received_message.command, "ack_flicker_done") == 0) {
        if (!main_process_running) return;

        // При первом входе в этот блок (первый ACK) запускаем таймер
        if (primary_uv_timer_start == 0) {
            primary_uv_timer_start = millis();
        }

        // --- Проверяем, не закончилось ли общее время этапа ---
        unsigned long duration_ms = (unsigned long)current_active_profile_data.primary_uv_exposure_sec * 1000;
        unsigned long time_elapsed_ms = millis() - primary_uv_timer_start;

        if (time_elapsed_ms >= duration_ms) {
            Serial.println("Primary UV stage finished.");
            primary_uv_timer_start = 0; 
            
            // >>>>> ЗАПУСКАЕМ ЭТАП 6: SECONDARY UV <<<<<
            Serial.println("--- Starting Stage 6: Secondary UV ---");
            lvgl_port_lock(-1);
            lv_label_set_text(label_process_status_title, "Stage 6: Secondary UV");
            lvgl_port_unlock();
            if (!main_process_running) return;

            memset(&message_to_send, 0, sizeof(struct_message));
            strcpy(message_to_send.command, "start_secondary_uv");
            message_to_send.uv_405_on = current_active_profile_data.secondary_uv_type1_enabled;
            message_to_send.uv_430_on = current_active_profile_data.secondary_uv_type2_enabled;
            sendEspNowMessage(message_to_send);
            return;
        }

        // --- Время еще не вышло, продолжаем цикл ---
        
        // Обновляем UI с оставшимся временем
        unsigned long time_left_sec = (duration_ms - time_elapsed_ms) / 1000;
        lvgl_port_lock(-1);
        lv_label_set_text_fmt(label_process_status_detail, "Flickering...\nTime left: %lu s", time_left_sec + 1);
        lvgl_port_unlock();

        // Отправляем команду на следующий секундный цикл мерцаний
        const int flicker_405_per_sec = current_active_profile_data.primary_uv_type1_enabled ? current_active_profile_data.primary_uv_type1_flickers : 0;
        const int flicker_430_per_sec = current_active_profile_data.primary_uv_type2_enabled ? current_active_profile_data.primary_uv_type2_flickers : 0;

        memset(&message_to_send, 0, sizeof(struct_message));
        strcpy(message_to_send.command, "do_uv_flicker");
        message_to_send.value_flickering_405 = flicker_405_per_sec;
        message_to_send.value_flickering_430 = flicker_430_per_sec;
        sendEspNowMessage(message_to_send);
    }
    // =================================================================================
    // === ЭТАП 6: Получен ACK о включении Secondary UV. Начинаем отсчет. ===
    // =================================================================================
    else if (strcmp(received_message.command, "ack_secondary_uv_started") == 0) {
        if (!main_process_running) return;

        // При первом входе в этот блок (первый ACK) запускаем таймер
        if (secondary_uv_timer_start == 0) {
            secondary_uv_timer_start = millis();
        }

        // --- Проверяем, не закончилось ли общее время этапа ---
        unsigned long duration_ms = (unsigned long)current_active_profile_data.secondary_uv_exposure_sec * 1000;
        unsigned long time_elapsed_ms = millis() - secondary_uv_timer_start;

        if (time_elapsed_ms >= duration_ms) {
            Serial.println("Secondary UV stage finished.");
            secondary_uv_timer_start = 0; // Сбрасываем таймер
            
            // Отправляем команду на выключение УФ
            memset(&message_to_send, 0, sizeof(struct_message));
            strcpy(message_to_send.command, "stop_uv");
            sendEspNowMessage(message_to_send);
            return;
        }

        // --- Время еще не вышло, продолжаем цикл ---
        
        // Обновляем UI с оставшимся временем
        unsigned long time_left_sec = (duration_ms - time_elapsed_ms) / 1000;
        lvgl_port_lock(-1);
        lv_label_set_text_fmt(label_process_status_detail, "Exposure in progress...\nTime left: %lu s", time_left_sec + 1);
        lvgl_port_unlock();

        // Ждем 1 секунду и снова отправляем команду, чтобы получить новый ACK и обновить таймер
        delay(1000);
        if (!main_process_running) { secondary_uv_timer_start = 0; return; }

        memset(&message_to_send, 0, sizeof(struct_message));
        strcpy(message_to_send.command, "start_secondary_uv");
        message_to_send.uv_405_on = current_active_profile_data.secondary_uv_type1_enabled;
        message_to_send.uv_430_on = current_active_profile_data.secondary_uv_type2_enabled;
        sendEspNowMessage(message_to_send);
    }
    // =================================================================================
    // === ЭТАП 6.5: Получен ACK о выключении УФ. Решаем, нужно ли охлаждение. ===
    // =================================================================================
    else if (strcmp(received_message.command, "ack_uv_stopped") == 0) {
        if (!main_process_running) return;

        Serial.println("UV lamps are off. Checking for final cooling stage.");
        
        // Проверяем, включено ли охлаждение в профиле И в глобальных настройках
        bool cooling_needed = current_active_profile_data.chamber_cooling_enabled && current_global_settings.compressed_air_system_enabled;

        if (cooling_needed) {
            // --- ЗАПУСКАЕМ ЭТАП 7: ФИНАЛЬНОЕ ОХЛАЖДЕНИЕ ---
            Serial.println("--- Starting Stage 7: Final Cooling ---");
            lvgl_port_lock(-1);
            lv_label_set_text(label_process_status_title, "Stage 7: Final Cooling");
            lv_label_set_text(label_process_status_detail, "Cooling down to 40 C...");
            lvgl_port_unlock();

            memset(&message_to_send, 0, sizeof(struct_message));
            strcpy(message_to_send.command, "start_final_cooling");
            sendEspNowMessage(message_to_send);
            // Теперь ждем ответа "ack_final_cooling_started"
        } else {
            // --- ОХЛАЖДЕНИЕ НЕ НУЖНО. ЗАВЕРШАЕМ ПРОЦЕСС ---
            Serial.println("Final cooling is disabled. Process complete.");
            lvgl_port_lock(-1);
            lv_label_set_text(label_process_status_title, "Process Complete!");
            lv_label_set_text(label_process_status_detail, "You can now open the door.");
            lvgl_port_unlock();
            
            // >>>>> ОТПРАВЛЯЕМ КОМАНДУ "СТОП-КРАН" <<<<<
            memset(&message_to_send, 0, sizeof(struct_message));
            strcpy(message_to_send.command, "force_stop");
            sendEspNowMessage(message_to_send);
            
            main_process_running = false; // Завершаем процесс
        }
    }
    // =================================================================================
    // === ЭТАП 7: Контроль ФИНАЛЬНОГО ОХЛАЖДЕНИЯ ===
    // =================================================================================
    else if (strcmp(received_message.command, "ack_final_cooling_started") == 0) {
        if (!main_process_running) return;

        float current_temp_local = (float)received_message.value_int / 10.0f;
        const int FINAL_TARGET_TEMP = 40;

        // Разбиваем температуру на безопасные для вывода части
        int current_int_part = (int)current_temp_local;
        int current_frac_part = abs((int)(current_temp_local * 10) % 10);

        // Отображаем статус, используя %d.%d
        lvgl_port_lock(-1);
        lv_label_set_text_fmt(label_process_status_detail, "Cooling down...\nTarget: %d C, Current: %d.%d C", FINAL_TARGET_TEMP, current_int_part, current_frac_part);
        lvgl_port_unlock();

        // Проверяем, достигли ли цели
        if (current_temp_local <= FINAL_TARGET_TEMP) {
            Serial.println("Final cooling complete. Process finished. Shutting down all systems.");
            
            // >>>>> ОТПРАВЛЯЕМ КОМАНДУ "СТОП-КРАН" <<<<<
            // Команда force_stop выключит ВСЕ реле, включая клапан воздуха и вентиляторы.
            memset(&message_to_send, 0, sizeof(struct_message));
            strcpy(message_to_send.command, "force_stop");
            sendEspNowMessage(message_to_send);

            lvgl_port_lock(-1);
            lv_label_set_text(label_process_status_title, "Process Complete!");
            lv_label_set_text(label_process_status_detail, "You can now open the door.");
            lvgl_port_unlock();
            
            main_process_running = false;
            return;
        } else {
            // Цель не достигнута, продолжаем цикл
            delay(2000);
            if (!main_process_running) return;
            
            memset(&message_to_send, 0, sizeof(struct_message));
            strcpy(message_to_send.command, "start_final_cooling");
            sendEspNowMessage(message_to_send);
        }
    }

    else if (strcmp(received_message.command, "door_opened_during_process") == 0) {
        Serial.println("CRITICAL: Received 'door_opened_during_process' from receiver!");
        main_process_running = false; // Останавливаем логику на УП
        lvgl_port_lock(-1);
        lv_label_set_text(label_process_status_title, "Door Open!");
        lv_label_set_text(label_process_status_detail, "Door was opened during the process.\nProcess has been stopped.");
        // Здесь можно скрыть спиннер, поменять цвет и т.д.
        lvgl_port_unlock();
    }
}

// Функция для инициализации ESP-NOW
void init_esp_now() {
    Serial.println("Attempting ESP-NOW Init (VERY EARLY, based on working example)...");

    WiFi.disconnect(true);
    delay(100); 
    Serial.println("Attempting WiFi.mode(WIFI_STA)...");
    if (!WiFi.mode(WIFI_STA)) {
        Serial.println("WiFi.mode(WIFI_STA) FAILED! Critical.");
        handleFatalError("WiFi STA Mode Set Failed Critically in init_esp_now!"); 
        return;
    }
    Serial.println("WiFi.mode(WIFI_STA) successful.");
    delay(100); 

    // Дать время MAC-адресу стабилизироваться
    // Можно увеличить задержку, если MAC все еще читается некорректно
    Serial.println("Waiting for MAC address stabilization...");
    delay(1000); // 1 секунда
    String mac = WiFi.macAddress();
    Serial.printf("My MAC Address (early init): %s\n", mac.c_str());
    if (mac == "00:00:00:00:00:00" || mac.length() == 0 || mac == "FF:FF:FF:FF:FF:FF") {
        Serial.println("WARNING: MAC address is zero, empty, or all FFs after early init!");
        // Это может быть проблемой для ESP-NOW.
        // Можно добавить handleFatalError, если критично, чтобы MAC был корректным на этом этапе.
        // handleFatalError("Invalid MAC address read in init_esp_now!");
        // return; 
    }

    Serial.println("Attempting esp_now_init()...");
    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW. Attempting deinit and retry...");
        esp_now_deinit();
        delay(200);
        if (esp_now_init() != ESP_OK) {
            Serial.println("Repeated error initializing ESP-NOW. Critical.");
            handleFatalError("ESP-NOW Init Failed Critically in init_esp_now!");
            return;
        }
        Serial.println("ESP-NOW initialized successfully on second attempt.");
    } else {
        Serial.println("ESP-NOW initialized successfully.");
    }

    // Регистрация коллбэков
    // Убедись, что OnDataSent_callback и OnDataRecv_callback определены и имеют правильные сигнатуры
    if (esp_now_register_send_cb(OnDataSent_callback) != ESP_OK) {
         Serial.println("Failed to register ESP-NOW Send CB");
         handleFatalError("ESP-NOW Send CB Registration Failed!");
         return;
    }
    if (esp_now_register_recv_cb(OnDataRecv_callback) != ESP_OK) {
        Serial.println("Failed to register ESP-NOW Recv CB");
        handleFatalError("ESP-NOW Recv CB Registration Failed!");
        return;
    }
    Serial.println("ESP-NOW Callbacks registered.");

    // Добавление пира (используем глобальный receiver_mac_address и peerInfo)
    memset(&peerInfo, 0, sizeof(peerInfo)); // Очищаем структуру peerInfo перед использованием
    memcpy(peerInfo.peer_addr, receiver_mac_address, 6);
    peerInfo.channel = 0;  // Канал 0 для автоматического выбора, можно поставить 1, если будут проблемы
    peerInfo.encrypt = false;
    peerInfo.ifidx = WIFI_IF_STA; // Важно для совместимости ESP-IDF

    esp_err_t addStatus = esp_now_add_peer(&peerInfo);
    if (addStatus == ESP_OK) {
        Serial.println("Peer added successfully (early init).");
    } else if (addStatus == ESP_ERR_ESPNOW_EXIST) {
        Serial.println("Peer already exists, attempting to modify (early init)...");
        if (esp_now_mod_peer(&peerInfo) == ESP_OK) {
            Serial.println("Peer modified successfully (early init).");
        } else {
            Serial.printf("Failed to modify existing peer (early init), error: %s\n", esp_err_to_name(addStatus));
            // Не фатально, но отправка может не работать
        }
    } else {
        Serial.printf("Failed to add peer (early init), error: %s\n", esp_err_to_name(addStatus));
        // Не фатально, но отправка может не работать
    }
    Serial.println("ESP-NOW Init (VERY EARLY) sequence complete.");
}

// Обобщенная функция отправки сообщения ESP-NOW
bool sendEspNowMessage(const struct_message& msg_data) {
    esp_now_send_success = false; // Сбрасываем флаг перед отправкой
    esp_err_t result = esp_now_send(receiver_mac_address, (uint8_t *) &msg_data, sizeof(msg_data));
    
    if (result == ESP_OK) {
        Serial.println("ESP-NOW: Message queued for sending.");
        // Ждем коллбэк OnDataSent_callback для подтверждения отправки
        // В реальном применении здесь может быть тайм-аут ожидания esp_now_send_success
        // Но для простоты пока просто возвращаем результат постановки в очередь
        return true; 
    } else {
        Serial.print("ESP-NOW: Error sending message. ESP-NOW Error: ");
        Serial.println(esp_err_to_name(result));
        return false;
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

    Serial.println("--- START button clicked. Sending 'check_door' command. ---");

    main_process_running = true; // Процесс пошел.

    // 1. Переходим на экран процесса и показываем, что мы ждем
    if (screen_process_execution) {
        lv_scr_load(screen_process_execution);
        lv_label_set_text(label_process_status_title, " ");
        lv_label_set_text(label_process_status_detail, " ");
        lv_obj_clear_flag(btn_process_cancel, LV_OBJ_FLAG_HIDDEN);      // Показываем кнопку Cancel
    }

    // 2. Готовим и отправляем команду
    memset(&message_to_send, 0, sizeof(struct_message));
    strcpy(message_to_send.command, "check_door");
    sendEspNowMessage(message_to_send);
}

// Раздел 2: Функции обновления UI и обработчики событий LVGL
// ==========================================================================
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
        } else { // 0 = ENG (или любое другое значение)
            title_text = "Main Menu";
            page_text = "Page";
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

    // ... (вся твоя логика создания и сохранения профиля без изменений) ...
    ProfileData new_profile_defaults;
    new_profile_defaults.id = current_profile_next_id;
    strncpy(new_profile_defaults.name, profile_input_name, sizeof(new_profile_defaults.name) - 1);
    new_profile_defaults.name[sizeof(new_profile_defaults.name) - 1] = '\0';
    new_profile_defaults.thermal_chamber_enabled = false;
    new_profile_defaults.heat_exchange_hold_sec = 60;
    new_profile_defaults.thermal_chamber_temp = 40;
    new_profile_defaults.nitrogen_use_enabled = false;
    new_profile_defaults.primary_uv_exposure_sec = 30;
    new_profile_defaults.secondary_uv_exposure_sec = 60;
    new_profile_defaults.chamber_cooling_enabled = false;

    new_profile_defaults.primary_uv_type1_enabled = true;
    new_profile_defaults.primary_uv_type1_flickers = 5;
    new_profile_defaults.primary_uv_type2_enabled = false;
    new_profile_defaults.primary_uv_type2_flickers = 5;
    
    new_profile_defaults.secondary_uv_type1_enabled = true;
    new_profile_defaults.secondary_uv_type2_enabled = true; 

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
            if (label_detail_view_profile_name) lv_label_set_text(label_detail_view_profile_name, current_active_profile_data.name);
            if (label_detail_view_id) lv_label_set_text_fmt(label_detail_view_id, "ID: %d", current_active_profile_data.id);
            
            // <<<--- ЗАМЕНИ СТАРЫЙ БЛОК НА ЭТОТ ---<<<
            if (label_detail_view_thermal_chamber_enabled) {
                lv_label_set_text_fmt(label_detail_view_thermal_chamber_enabled, "Thermal Chamber: %s", current_active_profile_data.thermal_chamber_enabled ? "ON" : "OFF");
            }
            if (label_detail_view_thermal_chamber) {
                if (current_active_profile_data.thermal_chamber_enabled) {
                    // Если включено - показываем и обновляем текст
                    lv_obj_clear_flag(label_detail_view_thermal_chamber, LV_OBJ_FLAG_HIDDEN);
                    lv_label_set_text_fmt(label_detail_view_thermal_chamber, "Target Temp: %d C, Hold: %d s", 
                                        current_active_profile_data.thermal_chamber_temp,
                                        current_active_profile_data.heat_exchange_hold_sec);
                } else {
                    // Если выключено - просто прячем эту метку
                    lv_obj_add_flag(label_detail_view_thermal_chamber, LV_OBJ_FLAG_HIDDEN);
                }
            }
            // --->>>
            
            if (label_detail_view_nitrogen) lv_label_set_text_fmt(label_detail_view_nitrogen, "Nitrogen Use: %s", current_active_profile_data.nitrogen_use_enabled ? "ON" : "OFF");
            if (label_detail_view_primary_uv) lv_label_set_text_fmt(label_detail_view_primary_uv, "Primary UV: %d s", current_active_profile_data.primary_uv_exposure_sec);
            if (label_detail_view_secondary_uv) lv_label_set_text_fmt(label_detail_view_secondary_uv, "Secondary UV: %d s", current_active_profile_data.secondary_uv_exposure_sec);
            if (label_detail_view_chamber_cooling) lv_label_set_text_fmt(label_detail_view_chamber_cooling, "Chamber Cooling: %s", current_active_profile_data.chamber_cooling_enabled ? "ON" : "OFF");
        } else { 
            if (label_detail_view_profile_name) lv_label_set_text(label_detail_view_profile_name, current_selected_profile_filename); 
            if (label_detail_view_id) lv_label_set_text(label_detail_view_id, "ID: N/A (Error)");
            if (label_detail_view_thermal_chamber) {
                lv_label_set_text_fmt(label_detail_view_thermal_chamber, "Target Temp: %d C, Hold: %d s", 
                                    current_active_profile_data.thermal_chamber_temp,
                                    current_active_profile_data.heat_exchange_hold_sec);
            }
            if (label_detail_view_nitrogen) lv_label_set_text(label_detail_view_nitrogen, "");
            if (label_detail_view_primary_uv) lv_label_set_text(label_detail_view_primary_uv, "");
            if (label_detail_view_secondary_uv) lv_label_set_text(label_detail_view_secondary_uv, "");
            if (label_detail_view_chamber_cooling) lv_label_set_text(label_detail_view_chamber_cooling, "");
        }
        if (screen_profile_details) { lv_scr_load(screen_profile_details); }
        else { Serial.println("ERROR: screen_profile_details is NULL!"); }
        lvgl_port_unlock();
    }
}

static void profile_detail_edit_btn_event_cb(lv_event_t * e) {
    lv_event_code_t code = lv_event_get_code(e);
    if (code == LV_EVENT_CLICKED) {
        Serial.printf("--- EDIT button clicked for profile file: %s (Name from data: %s). Preparing edit screen ---\n", current_selected_profile_filename, current_active_profile_data.name);
        lvgl_port_lock(-1);
        if (screen_profile_edit) {
            // ... (весь твой код для заполнения полей до UV) ...
            if(label_edit_profile_id_val) lv_label_set_text_fmt(label_edit_profile_id_val, "%d", current_active_profile_data.id);
            if(ta_edit_profile_name) lv_textarea_set_text(ta_edit_profile_name, current_active_profile_data.name);
            if (current_active_profile_data.thermal_chamber_enabled) { lv_obj_add_state(sw_edit_thermal_chamber_enable, LV_STATE_CHECKED); } else { lv_obj_clear_state(sw_edit_thermal_chamber_enable, LV_STATE_CHECKED); }
            lv_event_send(sw_edit_thermal_chamber_enable, LV_EVENT_VALUE_CHANGED, NULL);
            int slider_val = roundToStep(current_active_profile_data.thermal_chamber_temp - 40, 5) / 5;
            lv_slider_set_value(slider_edit_thermal_temp, slider_val, LV_ANIM_OFF);
            lv_label_set_text_fmt(label_edit_thermal_temp_val, "%d C", current_active_profile_data.thermal_chamber_temp);
            char heat_hold_buffer[10];
            snprintf(heat_hold_buffer, sizeof(heat_hold_buffer), "%d", current_active_profile_data.heat_exchange_hold_sec);
            lv_textarea_set_text(ta_edit_heat_hold, heat_hold_buffer);
            if (current_active_profile_data.chamber_cooling_enabled) lv_obj_add_state(sw_edit_chamber_cooling, LV_STATE_CHECKED); else lv_obj_clear_state(sw_edit_chamber_cooling, LV_STATE_CHECKED); 
            lv_obj_set_style_opa(sw_edit_chamber_cooling, current_global_settings.compressed_air_system_enabled ? LV_OPA_COVER : LV_OPA_50, 0);
            if (current_active_profile_data.nitrogen_use_enabled) lv_obj_add_state(sw_edit_nitrogen, LV_STATE_CHECKED); else lv_obj_clear_state(sw_edit_nitrogen, LV_STATE_CHECKED); 
            lv_obj_set_style_opa(sw_edit_nitrogen, current_global_settings.nitrogen_system_enabled ? LV_OPA_COVER : LV_OPA_50, 0);
            char uv_buffer[10];  
            snprintf(uv_buffer, sizeof(uv_buffer), "%d", current_active_profile_data.primary_uv_exposure_sec); 
            lv_textarea_set_text(ta_edit_primary_uv, uv_buffer); 
            snprintf(uv_buffer, sizeof(uv_buffer), "%d", current_active_profile_data.secondary_uv_exposure_sec); 
            lv_textarea_set_text(ta_edit_secondary_uv, uv_buffer); 
            
            // 4. Расширенные настройки Primary UV
            if (current_active_profile_data.primary_uv_type1_enabled) lv_obj_add_state(sw_primary_uv_type1, LV_STATE_CHECKED); else lv_obj_clear_state(sw_primary_uv_type1, LV_STATE_CHECKED);
            lv_slider_set_value(slider_primary_uv_type1, current_active_profile_data.primary_uv_type1_flickers, LV_ANIM_OFF);
            lv_label_set_text_fmt(label_primary_uv_type1_val, "%d", current_active_profile_data.primary_uv_type1_flickers);
            
            if (current_active_profile_data.primary_uv_type2_enabled) lv_obj_add_state(sw_primary_uv_type2, LV_STATE_CHECKED); else lv_obj_clear_state(sw_primary_uv_type2, LV_STATE_CHECKED);
            lv_slider_set_value(slider_primary_uv_type2, current_active_profile_data.primary_uv_type2_flickers, LV_ANIM_OFF);
            lv_label_set_text_fmt(label_primary_uv_type2_val, "%d", current_active_profile_data.primary_uv_type2_flickers);

            // <<<--- ГЛАВНЫЙ ФИКС ДЛЯ СЛАЙДЕРОВ ---<<<
            // Принудительно вызываем событие, чтобы обновить их вид (активен/неактивен)
            lv_event_send(sw_primary_uv_type1, LV_EVENT_VALUE_CHANGED, NULL);
            // lv_event_send(sw_primary_uv_type2, LV_EVENT_VALUE_CHANGED, NULL); // Этот вызов не нужен, т.к. первый обработчик проверит оба свитча
            
            // 5. Расширенные настройки Secondary UV
            if (current_active_profile_data.secondary_uv_type1_enabled) lv_obj_add_state(sw_secondary_uv_type1, LV_STATE_CHECKED); else lv_obj_clear_state(sw_secondary_uv_type1, LV_STATE_CHECKED);
            if (current_active_profile_data.secondary_uv_type2_enabled) lv_obj_add_state(sw_secondary_uv_type2, LV_STATE_CHECKED); else lv_obj_clear_state(sw_secondary_uv_type2, LV_STATE_CHECKED);
            lv_event_send(sw_secondary_uv_type1, LV_EVENT_VALUE_CHANGED, NULL);
            
            // Загружаем сам экран
            Serial.println("Loading profile_edit_screen (Corrected Logic)...");
            lv_scr_load(screen_profile_edit); 
        } else { 
            Serial.println("ERROR: screen_profile_edit is NULL!"); 
        }
        lvgl_port_unlock();
    }
}
static void profile_detail_delete_btn_event_cb(lv_event_t * e) {
    lv_event_code_t code = lv_event_get_code(e);
    if (code == LV_EVENT_CLICKED) {
        Serial.printf("--- DELETE button clicked for profile: %s. Showing custom confirm dialog ---\n", current_selected_profile_filename);
        lvgl_port_lock(-1); 
        if (screen_confirm_delete_dialog && label_confirm_delete_text) {
            char confirm_dialog_msg_buffer[256];
            snprintf(confirm_dialog_msg_buffer, sizeof(confirm_dialog_msg_buffer), "Really delete profile\n'%s'?", current_active_profile_data.name); 
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
    if (label_edit_thermal_temp_val) {
        lv_label_set_text_fmt(label_edit_thermal_temp_val, "%d C", actual_temp);
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
    else if (ta == ta_edit_heat_hold) { validate_numeric_input(e, 30, 180); } // <<<--- НОВАЯ ВАЛИДАЦИЯ
}

static void profile_edit_save_changes_btn_event_cb(lv_event_t * e) {
    Serial.println("Save Changes button clicked.");
    ProfileData edited_data; // Создаем новую структуру для редактируемых данных

    // ID профиля не меняется, берем из текущего активного
    edited_data.id = current_active_profile_data.id; 
    
    // Имя профиля
    if (ta_edit_profile_name) { 
        strncpy(edited_data.name, lv_textarea_get_text(ta_edit_profile_name), sizeof(edited_data.name) - 1);
        edited_data.name[sizeof(edited_data.name) - 1] = '\0'; 
        if (strlen(edited_data.name) == 0) { 
            Serial.println("Error: Edited name is empty!"); 
            // TODO: Показать UI ошибку пользователю, например, через show_info_dialog
            show_info_dialog("Input Error", "Profile name cannot be empty.");
            return; 
        } 
    } else { 
        strcpy(edited_data.name, "DefaultName_ERR"); // На случай, если ta_edit_profile_name не существует
    } 
    
    // Сохраняем состояние переключателя термокамеры
    if (sw_edit_thermal_chamber_enable) {
        edited_data.thermal_chamber_enabled = lv_obj_has_state(sw_edit_thermal_chamber_enable, LV_STATE_CHECKED);
    } else {
        edited_data.thermal_chamber_enabled = true; // Безопасное значение по умолчанию
    }
    
    // --- Настройки "Достижение температуры" (ранее Thermal Chamber) ---
    // Эти параметры теперь сохраняются всегда, так как этап всегда активен.
    if (slider_edit_thermal_temp) { 
        int slider_val = lv_slider_get_value(slider_edit_thermal_temp); 
        edited_data.thermal_chamber_temp = roundToStep(40 + slider_val * 5, 5); 
        edited_data.thermal_chamber_temp = constrain(edited_data.thermal_chamber_temp, 40, 80); 
    } else { 
        edited_data.thermal_chamber_temp = current_active_profile_data.thermal_chamber_temp; // Берем старое, если UI нет
    }

    if (ta_edit_heat_hold) {
        edited_data.heat_exchange_hold_sec = constrain(atoi(lv_textarea_get_text(ta_edit_heat_hold)), 30, 180);
    } else {
        edited_data.heat_exchange_hold_sec = current_active_profile_data.heat_exchange_hold_sec; // Берем старое
    }
    
    // --- Переключатель Использования Азота ---
    if (sw_edit_nitrogen) {
        // Сохраняем состояние свитча, только если он НЕ заблокирован глобальной настройкой
        if (current_global_settings.nitrogen_system_enabled) {
            edited_data.nitrogen_use_enabled = lv_obj_has_state(sw_edit_nitrogen, LV_STATE_CHECKED);
        } else {
            // Если глобально выключено, то и в профиле должно быть выключено
            edited_data.nitrogen_use_enabled = false; 
            // И можно принудительно снять галочку с UI свитча, если она там как-то оказалась
            // lv_obj_clear_state(sw_edit_nitrogen, LV_STATE_CHECKED); 
        }
    } else {
        edited_data.nitrogen_use_enabled = current_active_profile_data.nitrogen_use_enabled; // Берем старое
    }

    // --- УФ параметры ---
    int primary_uv = current_active_profile_data.primary_uv_exposure_sec; 
    int secondary_uv = current_active_profile_data.secondary_uv_exposure_sec; 
    if (ta_edit_primary_uv) primary_uv = atoi(lv_textarea_get_text(ta_edit_primary_uv));
    if (ta_edit_secondary_uv) secondary_uv = atoi(lv_textarea_get_text(ta_edit_secondary_uv));
    edited_data.primary_uv_exposure_sec = constrain(primary_uv, 5, 60);
    edited_data.secondary_uv_exposure_sec = constrain(secondary_uv, 5, 200);
    
    // --- Переключатель Охлаждения Камеры (сжатым воздухом) ---
    if (sw_edit_chamber_cooling) {
        // Сохраняем состояние свитча, только если он НЕ заблокирован глобальной настройкой
        if (current_global_settings.compressed_air_system_enabled) {
            edited_data.chamber_cooling_enabled = lv_obj_has_state(sw_edit_chamber_cooling, LV_STATE_CHECKED);
        } else {
            edited_data.chamber_cooling_enabled = false;
            // lv_obj_clear_state(sw_edit_chamber_cooling, LV_STATE_CHECKED);
        }
    } else {
        edited_data.chamber_cooling_enabled = current_active_profile_data.chamber_cooling_enabled; // Берем старое
    }

    edited_data.primary_uv_type1_enabled = lv_obj_has_state(sw_primary_uv_type1, LV_STATE_CHECKED);
    edited_data.primary_uv_type1_flickers = lv_slider_get_value(slider_primary_uv_type1);
    edited_data.primary_uv_type2_enabled = lv_obj_has_state(sw_primary_uv_type2, LV_STATE_CHECKED);
    edited_data.primary_uv_type2_flickers = lv_slider_get_value(slider_primary_uv_type2);

    edited_data.secondary_uv_type1_enabled = lv_obj_has_state(sw_secondary_uv_type1, LV_STATE_CHECKED);
    edited_data.secondary_uv_type2_enabled = lv_obj_has_state(sw_secondary_uv_type2, LV_STATE_CHECKED);
    
    // --- Сериализация и запись файла ---
    String filename_on_sd = "/" + String(current_selected_profile_filename); 
    char json_buffer_save[FILE_CONTENT_BUFFER_SIZE];
    bool serialize_ok = serializeProfileJson(edited_data, json_buffer_save, sizeof(json_buffer_save));
    
    lvgl_port_lock(-1);
    if (serialize_ok) {
        writeFile(SD, filename_on_sd.c_str(), json_buffer_save);
        Serial.printf("Profile '%s' (File: %s) updated.\n", edited_data.name, filename_on_sd.c_str());
        current_active_profile_data = edited_data; // Обновляем текущий активный профиль в памяти
        
        // Обновляем метки на экране просмотра деталей
        if (label_detail_view_profile_name) lv_label_set_text(label_detail_view_profile_name, current_active_profile_data.name);
        if (label_detail_view_id) lv_label_set_text_fmt(label_detail_view_id, "ID: %d", current_active_profile_data.id);
        
        // <<<--- ЗАМЕНИ СТАРЫЙ БЛОК НА ЭТОТ ---<<<
        if (label_detail_view_thermal_chamber_enabled) {
            lv_label_set_text_fmt(label_detail_view_thermal_chamber_enabled, "Thermal Chamber: %s", current_active_profile_data.thermal_chamber_enabled ? "ON" : "OFF");
        }
        if (label_detail_view_thermal_chamber) {
            if (current_active_profile_data.thermal_chamber_enabled) {
                // Если включено - показываем и обновляем текст
                lv_obj_clear_flag(label_detail_view_thermal_chamber, LV_OBJ_FLAG_HIDDEN);
                lv_label_set_text_fmt(label_detail_view_thermal_chamber, "Target Temp: %d C, Hold: %d s", 
                                    current_active_profile_data.thermal_chamber_temp,
                                    current_active_profile_data.heat_exchange_hold_sec);
            } else {
                // Если выключено - просто прячем эту метку
                lv_obj_add_flag(label_detail_view_thermal_chamber, LV_OBJ_FLAG_HIDDEN);
            }
        }
        // --->>>
                
        if (label_detail_view_nitrogen) lv_label_set_text_fmt(label_detail_view_nitrogen, "Nitrogen Use: %s", current_active_profile_data.nitrogen_use_enabled ? "ON" : "OFF");
        if (label_detail_view_primary_uv) lv_label_set_text_fmt(label_detail_view_primary_uv, "Primary UV: %d s", current_active_profile_data.primary_uv_exposure_sec);
        if (label_detail_view_secondary_uv) lv_label_set_text_fmt(label_detail_view_secondary_uv, "Secondary UV: %d s", current_active_profile_data.secondary_uv_exposure_sec);
        if (label_detail_view_chamber_cooling) lv_label_set_text_fmt(label_detail_view_chamber_cooling, "Chamber Cooling: %s", current_active_profile_data.chamber_cooling_enabled ? "ON" : "OFF");
        
        scanAndCacheAllProfiles(SD, all_profile_entries_cache); 
        displayProfileListPage(); 
        if (screen_profile_details) { lv_scr_load(screen_profile_details); }
    } else { 
        Serial.println("Error serializing edited profile to JSON for saving!");
        // Вместо lv_msgbox_create, используем наш универсальный show_info_dialog
        show_info_dialog("Save Error", "Failed to prepare data for saving.");
    }
    lvgl_port_unlock();
}
static void profile_edit_cancel_btn_event_cb(lv_event_t * e) {
    Serial.println("Cancel Edit button clicked. Loading profile_details_screen without saving (NO ANIMATION).");
    if (screen_profile_details) {
        lvgl_port_lock(-1);
        // Восстанавливаем отображение на экране деталей из current_active_profile_data (которое не было изменено в файле)
        if (label_detail_view_profile_name) lv_label_set_text(label_detail_view_profile_name, current_active_profile_data.name);
        if (label_detail_view_id) lv_label_set_text_fmt(label_detail_view_id, "ID: %d", current_active_profile_data.id);
        if (label_detail_view_thermal_chamber_enabled) {
            lv_label_set_text_fmt(label_detail_view_thermal_chamber_enabled, "Thermal Chamber: %s", current_active_profile_data.thermal_chamber_enabled ? "ON" : "OFF");
        }
        if (label_detail_view_thermal_chamber) {
            if (current_active_profile_data.thermal_chamber_enabled) {
                // Если включено - показываем и обновляем текст
                lv_obj_clear_flag(label_detail_view_thermal_chamber, LV_OBJ_FLAG_HIDDEN);
                lv_label_set_text_fmt(label_detail_view_thermal_chamber, "Target Temp: %d C, Hold: %d s", 
                                    current_active_profile_data.thermal_chamber_temp,
                                    current_active_profile_data.heat_exchange_hold_sec);
            } else {
                // Если выключено - просто прячем эту метку
                lv_obj_add_flag(label_detail_view_thermal_chamber, LV_OBJ_FLAG_HIDDEN);
            }
        }
        if (label_detail_view_nitrogen) lv_label_set_text_fmt(label_detail_view_nitrogen, "Nitrogen Use: %s", current_active_profile_data.nitrogen_use_enabled ? "ON" : "OFF");
        if (label_detail_view_primary_uv) lv_label_set_text_fmt(label_detail_view_primary_uv, "Primary UV: %d s", current_active_profile_data.primary_uv_exposure_sec);
        if (label_detail_view_secondary_uv) lv_label_set_text_fmt(label_detail_view_secondary_uv, "Secondary UV: %d s", current_active_profile_data.secondary_uv_exposure_sec);
        if (label_detail_view_chamber_cooling) lv_label_set_text_fmt(label_detail_view_chamber_cooling, "Chamber Cooling: %s", current_active_profile_data.chamber_cooling_enabled ? "ON" : "OFF");
        lvgl_port_unlock();
        lv_scr_load(screen_profile_details);
    }
}

// Обработчик для кнопки "Cancel" на экране выполнения процесса
static void process_execution_cancel_btn_event_cb(lv_event_t * e) {
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;

    Serial.println("--- CANCEL/RETURN button clicked by user. Halting process. ---");

    main_process_running = false; // <<<--- СБРАСЫВАЕМ ФЛАГ! Останавливаем процесс.

    // Отправляем команду "Стоп-кран" на всякий случай
    Serial.println("Sending 'force_stop' command to receiver...");
    memset(&message_to_send, 0, sizeof(struct_message));
    strcpy(message_to_send.command, "force_stop");
    sendEspNowMessage(message_to_send);
    
    // Просто возвращаемся на главный экран
    if (screen_main_app) {
        Serial.println("Returning to main screen.");
        lvgl_port_lock(-1);
        lv_scr_load(screen_main_app);
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
        if (!current_global_settings.nitrogen_system_enabled && attempted_to_enable) {
            lv_obj_clear_state(sw, LV_STATE_CHECKED); // Принудительно выключаем обратно
            show_info_dialog("Setting Disabled", 
                             "Nitrogen system is disabled in Global Settings.\n"
                             "Please connect the valve and enable it in Settings.");
        } else {
            Serial.printf("Profile Nitrogen switch new state: %s\n", attempted_to_enable ? "ON" : "OFF");
        }
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
// Вставьте эту функцию в "Раздел 3" вместе с другими функциями build_..._screen
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
    lv_obj_t * label_btn_add = lv_label_create(btn_add);
    lv_label_set_text(label_btn_add, LV_SYMBOL_PLUS " Add");
    lv_obj_center(label_btn_add);
    
    lv_obj_t* btn_lab_mode = lv_btn_create(actions_btn_container);
    lv_obj_add_event_cb(btn_lab_mode, laboratory_mode_btn_event_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_set_flex_grow(btn_lab_mode, 1);
    lv_obj_t* label_btn_lab = lv_label_create(btn_lab_mode);
    lv_label_set_text(label_btn_lab, LV_SYMBOL_SETTINGS " Lab Mode");
    lv_obj_center(label_btn_lab);

    lv_obj_t* btn_settings = lv_btn_create(actions_btn_container);
    lv_obj_add_event_cb(btn_settings, btn_goto_settings_event_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_set_flex_grow(btn_settings, 1);
    lv_obj_t * label_btn_settings = lv_label_create(btn_settings);
    lv_label_set_text(label_btn_settings, "Settings");
    lv_obj_center(label_btn_settings);

    // lv_obj_add_event_cb(parent_screen, [](lv_event_t * e) {
    //     lv_event_code_t code = lv_event_get_code(e); // Получаем код события
        
    //     // <<<--- ОБЪЕДИНЯЕМ ОБРАБОТЧИКИ В ОДИН ---<<<
    //     if (code == LV_EVENT_SCREEN_LOADED) {
    //         Serial.println("Main app screen is loaded. Updating list.");
    //         displayProfileListPage();
    //     } 
    //     else if (code == LV_EVENT_USER_1) { // <<<--- ИСПРАВЛЕНИЕ: ЛОВИМ НАШЕ СОБЫТИЕ
    //         Serial.println("Received REFRESH_PROFILES event (USER_1). Updating list.");
    //         displayProfileListPage();
    //     }
    // }, LV_EVENT_ALL, NULL); // <<<--- ИСПРАВЛЕНИЕ: СЛУШАЕМ ВСЕ СОБЫТИЯ
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
    lv_obj_t* header = lv_label_create(content_container);
    lv_label_set_text(header, "Profile name and details:");
    lv_obj_set_style_text_color(header, lv_color_hex(0x888888), 0); // Сделаем его серым
    lv_obj_set_width(header, lv_pct(100));
    lv_obj_set_style_text_align(header, LV_TEXT_ALIGN_CENTER, 0);

    // --- 2. Большое имя профиля ---
    label_detail_view_profile_name = lv_label_create(content_container);
    lv_label_set_text(label_detail_view_profile_name, "Loading...");
    lv_obj_set_style_text_font(label_detail_view_profile_name, &lv_font_montserrat_32, 0); // <<<--- Большой шрифт
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

    // ...и так далее для ВСЕХ элементов ниже...
    label_detail_view_id = lv_label_create(details_content_block);
    lv_label_set_text(label_detail_view_id, "ID: --");
    lv_obj_set_width(label_detail_view_id, lv_pct(100));
    lv_obj_set_style_text_font(label_detail_view_id, &lv_font_montserrat_22, 0); // <<<--- УВЕЛИЧИЛ ШРИФТ

    label_detail_view_thermal_chamber_enabled = lv_label_create(details_content_block);
    lv_label_set_text(label_detail_view_thermal_chamber_enabled, "Thermal Chamber: --");
    lv_obj_set_width(label_detail_view_thermal_chamber_enabled, lv_pct(100));
    lv_obj_set_style_text_font(label_detail_view_thermal_chamber_enabled, &lv_font_montserrat_22, 0); // <<<--- УВЕЛИЧИЛ ШРИФТ

    label_detail_view_thermal_chamber = lv_label_create(details_content_block);
    // <<<--- УБРАЛ СИМВОЛ "└─" ИЗ ТЕКСТА ПО УМОЛЧАНИЮ ---<<<
    lv_label_set_text(label_detail_view_thermal_chamber, "Target Temp: -- C, Hold: -- s");
    lv_obj_set_width(label_detail_view_thermal_chamber, lv_pct(100));
    lv_label_set_long_mode(label_detail_view_thermal_chamber, LV_LABEL_LONG_WRAP);
    lv_obj_set_style_text_font(label_detail_view_thermal_chamber, &lv_font_montserrat_22, 0); // <<<--- УВЕЛИЧИЛ ШРИФТ

    label_detail_view_nitrogen = lv_label_create(details_content_block);
    lv_label_set_text(label_detail_view_nitrogen, "Nitrogen Use: --");
    lv_obj_set_width(label_detail_view_nitrogen, lv_pct(100));
    lv_obj_set_style_text_font(label_detail_view_nitrogen, &lv_font_montserrat_22, 0); // <<<--- УВЕЛИЧИЛ ШРИФТ

    label_detail_view_primary_uv = lv_label_create(details_content_block);
    lv_label_set_text(label_detail_view_primary_uv, "Primary UV: -- s");
    lv_obj_set_width(label_detail_view_primary_uv, lv_pct(100));
    lv_obj_set_style_text_font(label_detail_view_primary_uv, &lv_font_montserrat_22, 0); // <<<--- УВЕЛИЧИЛ ШРИФТ

    label_detail_view_secondary_uv = lv_label_create(details_content_block);
    lv_label_set_text(label_detail_view_secondary_uv, "Secondary UV: -- s");
    lv_obj_set_width(label_detail_view_secondary_uv, lv_pct(100));
    lv_obj_set_style_text_font(label_detail_view_secondary_uv, &lv_font_montserrat_22, 0); // <<<--- УВЕЛИЧИЛ ШРИФТ
    
    label_detail_view_chamber_cooling = lv_label_create(details_content_block);
    lv_label_set_text(label_detail_view_chamber_cooling, "Chamber Cooling: --");
    lv_obj_set_width(label_detail_view_chamber_cooling, lv_pct(100));
    lv_obj_set_style_text_font(label_detail_view_chamber_cooling, &lv_font_montserrat_22, 0); // <<<--- УВЕЛИЧИЛ ШРИФТ

    // Распорка, чтобы прижать кнопки к низу
    // lv_obj_t* spacer = lv_obj_create(content_container); 
    // lv_obj_remove_style_all(spacer);
    // lv_obj_set_flex_grow(spacer, 1); // Занимает всё свободное вертикальное пространство

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
    lv_obj_t * label_start_txt = lv_label_create(btn_start);
    lv_label_set_text(label_start_txt, "Start");
    lv_obj_center(label_start_txt);

    // --- Кнопка Edit ---
    lv_obj_t* btn_edit = lv_btn_create(actions_btn_container);
    lv_obj_add_event_cb(btn_edit, profile_detail_edit_btn_event_cb, LV_EVENT_CLICKED, NULL); 
    lv_obj_set_flex_grow(btn_edit, 1); // Растягивается
    lv_obj_t * label_edit_txt = lv_label_create(btn_edit);
    lv_label_set_text(label_edit_txt, "Edit");
    lv_obj_center(label_edit_txt);
    
    // --- Кнопка Delete ---
    lv_obj_t* btn_delete = lv_btn_create(actions_btn_container);
    lv_obj_set_style_bg_color(btn_delete, lv_palette_main(LV_PALETTE_RED), 0);
    lv_obj_add_event_cb(btn_delete, profile_detail_delete_btn_event_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_set_flex_grow(btn_delete, 1); // Растягивается
    lv_obj_t * label_delete_txt = lv_label_create(btn_delete);
    lv_label_set_text(label_delete_txt, "Delete");
    lv_obj_center(label_delete_txt);
    
    // --- Кнопка Close ---
    lv_obj_t* btn_close = lv_btn_create(actions_btn_container);
    lv_obj_add_event_cb(btn_close, profile_detail_close_event_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_set_flex_grow(btn_close, 1); // Растягивается
    lv_obj_t * label_close_txt = lv_label_create(btn_close);
    lv_label_set_text(label_close_txt, "Close");
    lv_obj_center(label_close_txt);
}

static void build_profile_edit_screen(lv_obj_t* parent_screen) {
    Serial.println("Building profile_edit_screen UI (Final Clean Version)...");

    // --- Общие переменные для функции ---
    lv_font_t* font_main = (lv_font_t*)&lv_font_montserrat_18;
    lv_font_t* font_small = (lv_font_t*)&lv_font_montserrat_16;

    // Главный контейнер
    lv_obj_t* main_container = lv_obj_create(parent_screen);
    lv_obj_set_size(main_container, lv_pct(100), lv_pct(100));
    lv_obj_align(main_container, LV_ALIGN_TOP_LEFT, 0, 0);
    lv_obj_add_flag(main_container, LV_OBJ_FLAG_SCROLL_ON_FOCUS);
    lv_obj_set_scrollbar_mode(main_container, LV_SCROLLBAR_MODE_AUTO);
    lv_obj_set_flex_flow(main_container, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(main_container, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_all(main_container, 5, 0);
    lv_obj_set_style_pad_row(main_container, 8, 0); // <<< ВОЗВРАЩАЕМ НЕБОЛЬШОЙ ЗАЗОР МЕЖДУ БЛОКАМИ

    // === БЛОК 1: ID и ИМЯ ===
    lv_obj_t* block1_id_name = lv_obj_create(main_container);
    lv_obj_remove_style_all(block1_id_name);
    lv_obj_set_width(block1_id_name, lv_pct(100));
    lv_obj_set_height(block1_id_name, 60);
    lv_obj_set_flex_flow(block1_id_name, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(block1_id_name, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_gap(block1_id_name, 10, 0);
    lv_obj_set_style_border_width(block1_id_name, 0, 0); // <<< УБИРАЕМ РАМКУ
    lv_obj_set_style_pad_all(block1_id_name, 10, 0);

    lv_obj_t* label_id_title = lv_label_create(block1_id_name);
    lv_label_set_text(label_id_title, "ID:");
    lv_obj_set_style_text_font(label_id_title, font_main, 0);
    label_edit_profile_id_val = lv_label_create(block1_id_name);
    lv_label_set_text(label_edit_profile_id_val, "--");
    lv_obj_set_style_text_font(label_edit_profile_id_val, font_main, 0);
    
    lv_obj_t* label_name_title = lv_label_create(block1_id_name);
    lv_label_set_text(label_name_title, "Name:");
    lv_obj_set_style_text_font(label_name_title, font_main, 0);
    ta_edit_profile_name = lv_textarea_create(block1_id_name);
    lv_textarea_set_one_line(ta_edit_profile_name, true);
    lv_textarea_set_max_length(ta_edit_profile_name, 200); 
    lv_obj_set_flex_grow(ta_edit_profile_name, 1);
    lv_obj_set_height(ta_edit_profile_name, lv_pct(100));
    lv_obj_set_style_text_font(lv_textarea_get_label(ta_edit_profile_name), font_main, 0);
    lv_textarea_set_placeholder_text(ta_edit_profile_name, "Enter profile name");
    lv_obj_set_scrollbar_mode(ta_edit_profile_name, LV_SCROLLBAR_MODE_OFF);
    lv_obj_add_event_cb(ta_edit_profile_name, alpha_textarea_focus_event_cb, LV_EVENT_FOCUSED, NULL); 

    // === БЛОК 2: ТЕРМОКАМЕРА ===
    const lv_coord_t ROW_HEIGHT_THERMAL = 45;

    lv_obj_t* block2_thermal = lv_obj_create(main_container);
    lv_obj_remove_style_all(block2_thermal);
    lv_obj_set_width(block2_thermal, lv_pct(100));
    lv_obj_set_height(block2_thermal, 70);
    lv_obj_set_flex_flow(block2_thermal, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(block2_thermal, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_gap(block2_thermal, 10, 0);
    lv_obj_set_style_border_width(block2_thermal, 0, 0); // <<< УБИРАЕМ РАМКУ
    lv_obj_set_style_pad_all(block2_thermal, 10, 0);

    lv_obj_t* label_thermal_enable = lv_label_create(block2_thermal);
    lv_label_set_text(label_thermal_enable, "Thermal Chamber:");
    lv_obj_set_style_text_font(label_thermal_enable, font_main, 0);
    sw_edit_thermal_chamber_enable = lv_switch_create(block2_thermal);
    lv_obj_add_event_cb(sw_edit_thermal_chamber_enable, thermal_chamber_enable_switch_event_cb, LV_EVENT_VALUE_CHANGED, NULL);

    thermal_elements_container = lv_obj_create(block2_thermal);
    lv_obj_remove_style_all(thermal_elements_container);
    lv_obj_set_flex_grow(thermal_elements_container, 1);
    lv_obj_set_height(thermal_elements_container, ROW_HEIGHT_THERMAL);
    lv_obj_set_flex_flow(thermal_elements_container, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(thermal_elements_container, LV_FLEX_ALIGN_SPACE_AROUND, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_gap(thermal_elements_container, 10, 0);

    heat_hold_elements_container = lv_obj_create(thermal_elements_container);
    lv_obj_remove_style_all(heat_hold_elements_container);
    lv_obj_set_size(heat_hold_elements_container, LV_SIZE_CONTENT, lv_pct(100));
    lv_obj_set_flex_flow(heat_hold_elements_container, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(heat_hold_elements_container, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_gap(heat_hold_elements_container, 5, 0);
    lv_obj_t* label_heat_hold = lv_label_create(heat_hold_elements_container);
    lv_label_set_text(label_heat_hold, "Hold(30-180s):");
    lv_obj_set_style_text_font(label_heat_hold, font_main, 0);
    ta_edit_heat_hold = lv_textarea_create(heat_hold_elements_container);
    lv_textarea_set_one_line(ta_edit_heat_hold, true); lv_textarea_set_accepted_chars(ta_edit_heat_hold, "0123456789"); lv_textarea_set_max_length(ta_edit_heat_hold, 3);
    lv_obj_set_size(ta_edit_heat_hold, 60, lv_pct(100));
    lv_obj_set_style_text_font(lv_textarea_get_label(ta_edit_heat_hold), font_main, 0);
    lv_obj_set_scrollbar_mode(ta_edit_heat_hold, LV_SCROLLBAR_MODE_OFF);
    lv_obj_add_event_cb(ta_edit_heat_hold, numeric_textarea_focus_event_cb, LV_EVENT_FOCUSED, NULL);
    lv_obj_add_event_cb(ta_edit_heat_hold, numeric_textarea_defocus_event_cb, LV_EVENT_DEFOCUSED, NULL);
    lv_textarea_set_align(ta_edit_heat_hold, LV_TEXT_ALIGN_CENTER);

    lv_obj_t* temp_col = lv_obj_create(thermal_elements_container);
    lv_obj_remove_style_all(temp_col);
    lv_obj_set_size(temp_col, LV_SIZE_CONTENT, lv_pct(100));
    lv_obj_set_flex_grow(temp_col, 1);
    lv_obj_set_flex_flow(temp_col, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(temp_col, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_gap(temp_col, 10, 0);
    lv_obj_t* label_temp = lv_label_create(temp_col);
    lv_label_set_text(label_temp, "Temp(40-80C): ");
    lv_obj_set_style_text_font(label_temp, font_main, 0);

    slider_edit_thermal_temp = lv_slider_create(temp_col);
    lv_slider_set_range(slider_edit_thermal_temp, 0, 8);
    lv_obj_set_flex_grow(slider_edit_thermal_temp, 1);
    lv_obj_add_event_cb(slider_edit_thermal_temp, thermal_temp_slider_event_cb, LV_EVENT_VALUE_CHANGED, NULL);
    label_edit_thermal_temp_val = lv_label_create(temp_col);
    lv_label_set_text(label_edit_thermal_temp_val, "45 C");
    lv_obj_set_width(label_edit_thermal_temp_val, 70);
    lv_obj_set_style_text_font(label_edit_thermal_temp_val, font_main, 0);
    lv_obj_set_style_pad_left(label_edit_thermal_temp_val, 10, 0);

    // === БЛОК 3: ГАЗЫ ===
    lv_obj_t* block3_gases = lv_obj_create(main_container);
    lv_obj_remove_style_all(block3_gases);
    lv_obj_set_width(block3_gases, lv_pct(100));
    lv_obj_set_height(block3_gases, LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(block3_gases, LV_FLEX_FLOW_ROW);
    lv_obj_set_style_pad_gap(block3_gases, 10, 0);
    lv_obj_set_style_border_width(block3_gases, 0, 0); // <<< УБИРАЕМ РАМКУ
    lv_obj_set_style_pad_all(block3_gases, 10, 0);
    
    lv_obj_t* gas_left_col = lv_obj_create(block3_gases);
    lv_obj_remove_style_all(gas_left_col); lv_obj_set_flex_grow(gas_left_col, 1); lv_obj_set_height(gas_left_col, LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(gas_left_col, LV_FLEX_FLOW_ROW); lv_obj_set_flex_align(gas_left_col, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_gap(gas_left_col, 10, 0);
    lv_obj_t* label_nitrogen = lv_label_create(gas_left_col); lv_label_set_text(label_nitrogen, "Nitrogen Use:");
    lv_obj_set_style_text_font(label_nitrogen, font_main, 0);
    sw_edit_nitrogen = lv_switch_create(gas_left_col); lv_obj_add_event_cb(sw_edit_nitrogen, profile_switch_value_changed_event_cb, LV_EVENT_VALUE_CHANGED, (void*)"nitrogen_profile");
    
    lv_obj_t* gas_right_col = lv_obj_create(block3_gases);
    lv_obj_remove_style_all(gas_right_col); lv_obj_set_flex_grow(gas_right_col, 1); lv_obj_set_height(gas_right_col, LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(gas_right_col, LV_FLEX_FLOW_ROW); lv_obj_set_flex_align(gas_right_col, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_gap(gas_right_col, 10, 0);
    lv_obj_t* label_cooling = lv_label_create(gas_right_col); lv_label_set_text(label_cooling, "Chamber Cooling:");
    lv_obj_set_style_text_font(label_cooling, font_main, 0);
    sw_edit_chamber_cooling = lv_switch_create(gas_right_col); lv_obj_add_event_cb(sw_edit_chamber_cooling, profile_switch_value_changed_event_cb, LV_EVENT_VALUE_CHANGED, (void*)"cooling_profile");

    // === БЛОК 4: UV ===
    const lv_coord_t ROW_HEIGHT_UV_MAIN = 45;
    const lv_coord_t ROW_HEIGHT_UV_SUB = 45;
    lv_obj_t* block4_uv = lv_obj_create(main_container);
    lv_obj_remove_style_all(block4_uv);
    lv_obj_set_width(block4_uv, lv_pct(100));
    lv_obj_set_height(block4_uv, LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(block4_uv, LV_FLEX_FLOW_ROW);
    lv_obj_set_style_pad_gap(block4_uv, 10, 0);
    lv_obj_set_style_border_width(block4_uv, 0, 0); // <<< УБИРАЕМ РАМКУ
    lv_obj_set_style_pad_all(block4_uv, 10, 0);
    
    lv_obj_t* uv_left_col = lv_obj_create(block4_uv);
    lv_obj_remove_style_all(uv_left_col); lv_obj_set_flex_grow(uv_left_col, 1); lv_obj_set_height(uv_left_col, LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(uv_left_col, LV_FLEX_FLOW_COLUMN); lv_obj_set_style_pad_all(uv_left_col, 0, 0); lv_obj_set_style_pad_gap(uv_left_col, 5, 0);
    lv_obj_t* uv_right_col = lv_obj_create(block4_uv);
    lv_obj_remove_style_all(uv_right_col); lv_obj_set_flex_grow(uv_right_col, 1); lv_obj_set_height(uv_right_col, LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(uv_right_col, LV_FLEX_FLOW_COLUMN); lv_obj_set_style_pad_all(uv_right_col, 0, 0); lv_obj_set_style_pad_gap(uv_right_col, 5, 0);
    
    { // Наполняем левую колонку UV (Primary)
        // --- Строка 1: Общее время (без изменений) ---
        lv_obj_t* row_time = lv_obj_create(uv_left_col);
        lv_obj_remove_style_all(row_time); lv_obj_set_size(row_time, lv_pct(100), ROW_HEIGHT_UV_MAIN);
        lv_obj_set_flex_flow(row_time, LV_FLEX_FLOW_ROW); lv_obj_set_flex_align(row_time, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER); lv_obj_set_style_pad_gap(row_time, 5, 0);
        lv_obj_t* label = lv_label_create(row_time);
        lv_label_set_text(label, "Primary UV (5-60s):"); lv_obj_set_style_text_font(label, font_main, 0);
        ta_edit_primary_uv = lv_textarea_create(row_time);
        lv_textarea_set_one_line(ta_edit_primary_uv, true); lv_textarea_set_accepted_chars(ta_edit_primary_uv, "0123456789"); lv_textarea_set_max_length(ta_edit_primary_uv, 2);
        lv_obj_set_size(ta_edit_primary_uv, 60, lv_pct(100)); lv_obj_set_scrollbar_mode(ta_edit_primary_uv, LV_SCROLLBAR_MODE_OFF); lv_obj_set_style_text_font(lv_textarea_get_label(ta_edit_primary_uv), font_main, 0);
        lv_obj_add_event_cb(ta_edit_primary_uv, numeric_textarea_focus_event_cb, LV_EVENT_FOCUSED, NULL); lv_obj_add_event_cb(ta_edit_primary_uv, numeric_textarea_defocus_event_cb, LV_EVENT_DEFOCUSED, NULL);
        lv_textarea_set_align(ta_edit_primary_uv, LV_TEXT_ALIGN_CENTER);

        // --- Строка 2: Настройки Type 1 (ИСПРАВЛЕНО) ---
        lv_obj_t* row_type1 = lv_obj_create(uv_left_col);
        lv_obj_remove_style_all(row_type1); lv_obj_set_size(row_type1, lv_pct(100), ROW_HEIGHT_UV_SUB);
        lv_obj_clear_flag(row_type1, LV_OBJ_FLAG_SCROLLABLE); // <<< РЕШЕНИЕ 1: ЗАПРЕЩАЕМ ПРОКРУТКУ
        lv_obj_set_flex_flow(row_type1, LV_FLEX_FLOW_ROW); lv_obj_set_flex_align(row_type1, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER); lv_obj_set_style_pad_gap(row_type1, 5, 0);
        
        lv_obj_t* indent = lv_obj_create(row_type1); lv_obj_remove_style_all(indent); lv_obj_set_width(indent, 20);
        label = lv_label_create(row_type1); lv_label_set_text(label, "Type 1"); lv_obj_set_width(label, 70);
        lv_obj_set_style_text_font(label, font_main, 0);
        sw_primary_uv_type1 = lv_switch_create(row_type1);
        lv_obj_add_event_cb(sw_primary_uv_type1, uv_settings_event_cb, LV_EVENT_VALUE_CHANGED, (void*)"p_uv_t1_sw");
        
        label = lv_label_create(row_type1); lv_label_set_text(label, "flickers:");
        lv_obj_set_style_text_font(label, font_main, 0); // <<< РЕШЕНИЕ 3: УВЕЛИЧИВАЕМ ШРИФТ
        
        // <<< РЕШЕНИЕ 2: ЖЕСТКИЙ РАЗДЕЛИТЕЛЬ ВМЕСТО ПРОБЕЛА >>>
        lv_obj_t* spacer_flicker1 = lv_obj_create(row_type1);
        lv_obj_remove_style_all(spacer_flicker1);
        lv_obj_set_width(spacer_flicker1, 2); 
        
        slider_primary_uv_type1 = lv_slider_create(row_type1);
        lv_slider_set_range(slider_primary_uv_type1, 1, 5); lv_obj_set_flex_grow(slider_primary_uv_type1, 1);
        lv_obj_add_event_cb(slider_primary_uv_type1, uv_settings_event_cb, LV_EVENT_VALUE_CHANGED, (void*)"p_uv_t1_slider");
        lv_obj_set_style_bg_color(slider_primary_uv_type1, lv_palette_main(LV_PALETTE_BLUE), LV_PART_INDICATOR);
        lv_obj_set_style_bg_color(slider_primary_uv_type1, lv_palette_main(LV_PALETTE_BLUE), LV_PART_KNOB);
        lv_obj_set_style_bg_color(slider_primary_uv_type1, lv_palette_lighten(LV_PALETTE_GREY, 1), LV_PART_INDICATOR | LV_STATE_DISABLED);
        lv_obj_set_style_bg_color(slider_primary_uv_type1, lv_palette_lighten(LV_PALETTE_GREY, 1), LV_PART_KNOB | LV_STATE_DISABLED);
        
        label_primary_uv_type1_val = lv_label_create(row_type1);
        lv_label_set_text(label_primary_uv_type1_val, "5"); lv_obj_set_width(label_primary_uv_type1_val, 30);
        lv_obj_set_style_pad_left(label_primary_uv_type1_val, 5, 0); lv_obj_set_style_text_font(label_primary_uv_type1_val, font_main, 0); lv_obj_set_style_text_align(label_primary_uv_type1_val, LV_TEXT_ALIGN_CENTER, 0);

        // --- Строка 3: Настройки Type 2 (ИСПРАВЛЕНО) ---
        lv_obj_t* row_type2 = lv_obj_create(uv_left_col);
        lv_obj_remove_style_all(row_type2); lv_obj_set_size(row_type2, lv_pct(100), ROW_HEIGHT_UV_SUB);
        lv_obj_clear_flag(row_type2, LV_OBJ_FLAG_SCROLLABLE); // <<< РЕШЕНИЕ 1: ЗАПРЕЩАЕМ ПРОКРУТКУ
        lv_obj_set_flex_flow(row_type2, LV_FLEX_FLOW_ROW); lv_obj_set_flex_align(row_type2, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER); lv_obj_set_style_pad_gap(row_type2, 5, 0);
        
        indent = lv_obj_create(row_type2); lv_obj_remove_style_all(indent); lv_obj_set_width(indent, 20);
        label = lv_label_create(row_type2); lv_label_set_text(label, "Type 2"); lv_obj_set_width(label, 70);
        lv_obj_set_style_text_font(label, font_main, 0);
        sw_primary_uv_type2 = lv_switch_create(row_type2);
        lv_obj_add_event_cb(sw_primary_uv_type2, uv_settings_event_cb, LV_EVENT_VALUE_CHANGED, (void*)"p_uv_t2_sw");
        
        label = lv_label_create(row_type2); lv_label_set_text(label, "flickers:");
        lv_obj_set_style_text_font(label, font_main, 0); // <<< РЕШЕНИЕ 3: УВЕЛИЧИВАЕМ ШРИФТ
        
        // <<< РЕШЕНИЕ 2: ЖЕСТКИЙ РАЗДЕЛИТЕЛЬ ВМЕСТО ПРОБЕЛА >>>
        lv_obj_t* spacer_flicker2 = lv_obj_create(row_type2);
        lv_obj_remove_style_all(spacer_flicker2);
        lv_obj_set_width(spacer_flicker2, 2); 
        
        slider_primary_uv_type2 = lv_slider_create(row_type2);
        lv_slider_set_range(slider_primary_uv_type2, 1, 5); lv_obj_set_flex_grow(slider_primary_uv_type2, 1);
        lv_obj_add_event_cb(slider_primary_uv_type2, uv_settings_event_cb, LV_EVENT_VALUE_CHANGED, (void*)"p_uv_t2_slider");
        lv_obj_set_style_bg_color(slider_primary_uv_type2, lv_palette_main(LV_PALETTE_BLUE), LV_PART_INDICATOR);
        lv_obj_set_style_bg_color(slider_primary_uv_type2, lv_palette_main(LV_PALETTE_BLUE), LV_PART_KNOB);
        lv_obj_set_style_bg_color(slider_primary_uv_type2, lv_palette_lighten(LV_PALETTE_GREY, 1), LV_PART_INDICATOR | LV_STATE_DISABLED);
        lv_obj_set_style_bg_color(slider_primary_uv_type2, lv_palette_lighten(LV_PALETTE_GREY, 1), LV_PART_KNOB | LV_STATE_DISABLED);
        
        label_primary_uv_type2_val = lv_label_create(row_type2);
        lv_label_set_text(label_primary_uv_type2_val, "5"); lv_obj_set_width(label_primary_uv_type2_val, 30);
        lv_obj_set_style_pad_left(label_primary_uv_type2_val, 5, 0); lv_obj_set_style_text_font(label_primary_uv_type2_val, font_main, 0); lv_obj_set_style_text_align(label_primary_uv_type2_val, LV_TEXT_ALIGN_CENTER, 0);
    }
    { // Наполняем правую колонку UV (Secondary) (ИСПРАВЛЕННАЯ ВЕРСИЯ)
        // --- Строка 1 (без изменений) ---
        lv_obj_t* row = lv_obj_create(uv_right_col);
        lv_obj_remove_style_all(row); lv_obj_set_size(row, lv_pct(100), ROW_HEIGHT_UV_MAIN);
        lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW); lv_obj_set_flex_align(row, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER); lv_obj_set_style_pad_gap(row, 5, 0);
        lv_obj_t* label = lv_label_create(row);
        lv_label_set_text(label, "Secondary UV (5-200s):"); lv_obj_set_style_text_font(label, font_main, 0);
        ta_edit_secondary_uv = lv_textarea_create(row);
        lv_textarea_set_one_line(ta_edit_secondary_uv, true); lv_textarea_set_accepted_chars(ta_edit_secondary_uv, "0123456789"); lv_textarea_set_max_length(ta_edit_secondary_uv, 3);
        lv_obj_set_size(ta_edit_secondary_uv, 70, lv_pct(100)); lv_obj_set_scrollbar_mode(ta_edit_secondary_uv, LV_SCROLLBAR_MODE_OFF); lv_obj_set_style_text_font(lv_textarea_get_label(ta_edit_secondary_uv), font_main, 0);
        lv_obj_add_event_cb(ta_edit_secondary_uv, numeric_textarea_focus_event_cb, LV_EVENT_FOCUSED, NULL); lv_obj_add_event_cb(ta_edit_secondary_uv, numeric_textarea_defocus_event_cb, LV_EVENT_DEFOCUSED, NULL);
        lv_textarea_set_align(ta_edit_secondary_uv, LV_TEXT_ALIGN_CENTER);

        // --- Строка 2 (Type 1) - ИСПРАВЛЕНО ---
        row = lv_obj_create(uv_right_col);
        lv_obj_remove_style_all(row); lv_obj_set_size(row, lv_pct(100), ROW_HEIGHT_UV_SUB);
        lv_obj_clear_flag(row, LV_OBJ_FLAG_SCROLLABLE); // <<< РЕШЕНИЕ 1: ЗАПРЕЩАЕМ ПРОКРУТКУ
        lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW); lv_obj_set_flex_align(row, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER); lv_obj_set_style_pad_gap(row, 5, 0);
        lv_obj_t* indent = lv_obj_create(row); lv_obj_remove_style_all(indent); lv_obj_set_width(indent, 20);
        
        label = lv_label_create(row);
        lv_label_set_text(label, "Type 1");
        lv_obj_set_width(label, 70); // <<< ИЗМЕНЕНИЕ: Задаем ширину, а не растягивание
        lv_obj_set_style_text_font(label, font_main, 0);
        
        sw_secondary_uv_type1 = lv_switch_create(row);
        lv_obj_add_event_cb(sw_secondary_uv_type1, uv_settings_event_cb, LV_EVENT_VALUE_CHANGED, (void*)"s_uv_t1_sw");

        // --- Строка 3 (Type 2) - ИСПРАВЛЕНО ---
        row = lv_obj_create(uv_right_col);
        lv_obj_remove_style_all(row); lv_obj_set_size(row, lv_pct(100), ROW_HEIGHT_UV_SUB);
        lv_obj_clear_flag(row, LV_OBJ_FLAG_SCROLLABLE); // <<< РЕШЕНИЕ 1: ЗАПРЕЩАЕМ ПРОКРУТКУ
        lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW); lv_obj_set_flex_align(row, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER); lv_obj_set_style_pad_gap(row, 5, 0);
        indent = lv_obj_create(row); lv_obj_remove_style_all(indent); lv_obj_set_width(indent, 20);
        
        label = lv_label_create(row);
        lv_label_set_text(label, "Type 2");
        lv_obj_set_width(label, 70); // <<< ИЗМЕНЕНИЕ: Задаем ширину, а не растягивание
        lv_obj_set_style_text_font(label, font_main, 0);
        
        sw_secondary_uv_type2 = lv_switch_create(row);
        lv_obj_add_event_cb(sw_secondary_uv_type2, uv_settings_event_cb, LV_EVENT_VALUE_CHANGED, (void*)"s_uv_t2_sw");
    }

    lv_obj_t* spacer = lv_obj_create(main_container);
    lv_obj_remove_style_all(spacer);
    lv_obj_set_height(spacer, 0);
    lv_obj_set_flex_grow(spacer, 1);

    // === БЛОК 5: КНОПКИ ===
    lv_obj_t* block5_buttons = lv_obj_create(main_container);
    lv_obj_remove_style_all(block5_buttons);
    lv_obj_set_width(block5_buttons, lv_pct(100));
    lv_obj_set_height(block5_buttons, LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(block5_buttons, LV_FLEX_FLOW_ROW);
    lv_obj_set_style_pad_gap(block5_buttons, 10, 0);
    lv_obj_set_style_border_width(block5_buttons, 0, 0); // Убираем рамку
    lv_obj_set_style_pad_all(block5_buttons, 5, 0);

    lv_obj_t* btn_save_changes = lv_btn_create(block5_buttons);
    lv_obj_add_event_cb(btn_save_changes, profile_edit_save_changes_btn_event_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_set_flex_grow(btn_save_changes, 1);
    lv_obj_t* label_save_changes = lv_label_create(btn_save_changes);
    lv_label_set_text(label_save_changes, "Save Changes");
    lv_obj_set_style_text_font(label_save_changes, font_main, 0);
    lv_obj_center(label_save_changes);

    lv_obj_t* btn_cancel_edit = lv_btn_create(block5_buttons);
    lv_obj_add_event_cb(btn_cancel_edit, profile_edit_cancel_btn_event_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_set_flex_grow(btn_cancel_edit, 1);
    lv_obj_t* label_cancel_edit = lv_label_create(btn_cancel_edit);
    lv_label_set_text(label_cancel_edit, "Cancel Edit");
    lv_obj_set_style_text_font(label_cancel_edit, font_main, 0);
    lv_obj_center(label_cancel_edit);
    
    Serial.println("Profile edit screen UI built.");
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
    lv_obj_t* title = lv_label_create(screen_confirm_delete_dialog);
    lv_label_set_text(title, "Confirm Deletion");
    label_confirm_delete_text = lv_label_create(screen_confirm_delete_dialog);
    lv_label_set_long_mode(label_confirm_delete_text, LV_LABEL_LONG_WRAP);
    lv_label_set_text(label_confirm_delete_text, "Really delete 'Profile X'?"); 
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
    lv_obj_t* label_cancel_txt = lv_label_create(btn_cancel); 
    lv_label_set_text(label_cancel_txt, "Cancel");
    lv_obj_center(label_cancel_txt);
    lv_obj_t* btn_del = lv_btn_create(btn_area);
    lv_obj_set_style_bg_color(btn_del, lv_palette_main(LV_PALETTE_RED), 0);
    lv_obj_add_event_cb(btn_del, confirm_dialog_delete_btn_event_cb, LV_EVENT_CLICKED, NULL); 
    lv_obj_set_width(btn_del, 100);
    lv_obj_t* label_del_txt = lv_label_create(btn_del); 
    lv_label_set_text(label_del_txt, "Delete");
    lv_obj_center(label_del_txt);
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
    lv_obj_set_style_text_font(label_process_status_title, &lv_font_montserrat_20, 0);
    lv_label_set_text(label_process_status_title, "Initializing Process...");
    lv_obj_set_width(label_process_status_title, LV_PCT(100));
    lv_obj_set_style_text_align(label_process_status_title, LV_TEXT_ALIGN_CENTER, 0);

    spinner_process_execution = lv_spinner_create(content_container, 1000, 60); // родитель изменен
    lv_obj_set_size(spinner_process_execution, 100, 100);
    lv_obj_center(spinner_process_execution);

    label_process_status_detail = lv_label_create(content_container); // родитель изменен
    lv_label_set_text(label_process_status_detail, "Please wait...");
    lv_obj_set_width(label_process_status_detail, LV_PCT(100));
    lv_obj_set_style_text_align(label_process_status_detail, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_long_mode(label_process_status_detail, LV_LABEL_LONG_WRAP);

    btn_process_cancel = lv_btn_create(content_container); // родитель изменен
    lv_obj_add_event_cb(btn_process_cancel, process_execution_cancel_btn_event_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_set_width(btn_process_cancel, LV_PCT(50));
    lv_obj_t * label_cancel_txt = lv_label_create(btn_process_cancel);
    lv_label_set_text(label_cancel_txt, "Cancel Process");
    lv_obj_center(label_cancel_txt);
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
    
    // 0. ИНИЦИАЛИЗАЦИЯ ESP-NOW В САМОМ НАЧАЛЕ
    init_esp_now();
    
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

    // Строим все остальные экраны
    build_main_app_screen(screen_main_app);
    build_profile_details_screen(screen_profile_details);
    build_profile_edit_screen(screen_profile_edit);
    build_confirm_delete_dialog(screen_profile_details);
    build_process_execution_screen(screen_process_execution);
    build_settings_screen(screen_settings);
    build_secret_game_screen(screen_secret_game);
    build_service_lock_screen(screen_service_lock);

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
}