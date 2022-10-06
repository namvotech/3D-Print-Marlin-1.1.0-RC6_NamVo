/**
 * Marlin 3D Printer Firmware
 * Copyright (C) 2016 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include "ultralcd.h"
#if ENABLED(ULTRA_LCD)
#include "Marlin.h"
#include "language.h"
#include "cardreader.h"
#include "temperature.h"
#include "stepper.h"
#include "configuration_store.h"
#include "lifetime_stats.h"

/**
 * REVERSE_MENU_DIRECTION
 *
 * To reverse the menu direction we need a general way to reverse
 * the direction of the encoder everywhere. So encoderDirection is
 * added to allow the encoder to go the other way.
 *
 * This behavior is limited to scrolling Menus and SD card listings,
 * and is disabled in other contexts.
 */
#if ENABLED(REVERSE_MENU_DIRECTION)
  int8_t encoderDirection = 1;
   #define ENCODER_DIRECTION_NORMAL() (encoderDirection = -1)
   #define ENCODER_DIRECTION_MENUS() (encoderDirection = -1) 
  //#define ENCODER_DIRECTION_NORMAL() (encoderDirection = 1)
  //#define ENCODER_DIRECTION_MENUS() (encoderDirection = 1)  
#else
  #define ENCODER_DIRECTION_NORMAL() ;
  #define ENCODER_DIRECTION_MENUS() ;
#endif

int8_t encoderDiff; // updated from interrupt context and added to encoderPosition every LCD update

//NAMADD
bool filament_load_move_z_en=false,original_en=false ;
float original_position_e;
unsigned short original_active_extruder;
unsigned short original_feedrate_multiplier;


bool load_unload_filament_en=false, load_unload=false;//load_unload=0: unload,=1:load
#if EXTRUDERS > 1
    uint8_t load_unload_filament_ext=0;
#endif
uint32_t counter=0;
static void lcd_filament_load_unload_move_e();

#if ENABLED(ULTIPANEL)
	inline void line_to_current(AxisEnum axis);
#endif

bool encoderRateMultiplierEnabled;
int32_t lastEncoderMovementMillis;

//NAMADD Filament check
long encoderDiff_filament_a;  //NAMADD
long encoderDiff_filament_last_a=10000;  //NAMADD TEST
long filament_counter_a=0;  //NAMADD TEST

long encoderDiff_filament_b;  //NAMADD
long encoderDiff_filament_last_b=10000;  //NAMADD TEST
long filament_counter_b=0;  //NAMADD TEST

long encoderDiff_filament_c;  //NAMADD
long encoderDiff_filament_last_c=10000;  //NAMADD TEST
long filament_counter_c=0;  //NAMADD TEST

//NAMADD LED_RING
#if ENABLED(LED_RING)
	//bool led_ring_state=false;
	unsigned long led_ring_millis;
	unsigned int led_ring_counter;
#endif


int plaPreheatHotendTemp;
int plaPreheatHPBTemp;
int plaPreheatFanSpeed;

int absPreheatHotendTemp;
int absPreheatHPBTemp;
int absPreheatFanSpeed;

#if ENABLED(FILAMENT_LCD_DISPLAY)
  millis_t previous_lcd_status_ms = 0;
#endif

//NAMADD
#if ENABLED(SDSUPPORT)
  millis_t previous_lcd_status_ms_name_file = 0;
#endif

// Function pointer to menu functions.
typedef void (*menuFunc_t)();

uint8_t lcd_status_message_level;
char lcd_status_message[3 * (LCD_WIDTH) + 1] = WELCOME_MSG; // worst case is kana with up to 3*LCD_WIDTH+1

#if ENABLED(DOGLCD)
  #include "dogm_lcd_implementation.h"
#else
  #include "ultralcd_implementation_hitachi_HD44780.h"
#endif

// The main status screen
static void lcd_status_screen();

#if ENABLED(ULTIPANEL)

  #if HAS_POWER_SWITCH
    extern bool powersupply;
  #endif
  const float manual_feedrate[] = MANUAL_FEEDRATE;
  static void lcd_main_menu();
  static void lcd_tune_menu();
  static void lcd_prepare_menu();
  static void lcd_move_menu();
  static void lcd_control_menu();
  static void lcd_control_temperature_menu();
  static void lcd_control_temperature_preheat_pla_settings_menu();
  static void lcd_control_temperature_preheat_abs_settings_menu();
  static void lcd_control_motion_menu();
  static void lcd_control_volumetric_menu();
  
  //NAMADD Filament check menu
  static void lcd_filament_check_menu();
  
  static void lcd_level_bed();
  static void lcd_setting_menu(); 
  void lcd_language_menu();
  void lcd_other_menu();  
  static void lcd_life_time();
  #if ENABLED(THERMAL_PROTECTION_HOTENDS)
  #if TEMP_SENSOR_0 != 0
    void watch_temp_callback_E0();
  #endif
  #if EXTRUDERS > 1 && TEMP_SENSOR_1 != 0
    void watch_temp_callback_E1();
  #endif // EXTRUDERS > 1
  #if EXTRUDERS > 2 && TEMP_SENSOR_2 != 0
    void watch_temp_callback_E2();
  #endif // EXTRUDERS > 2
  #if EXTRUDERS > 3 && TEMP_SENSOR_3 != 0
    void watch_temp_callback_E3();
  #endif // EXTRUDERS > 3
#else
  #if TEMP_SENSOR_0 != 0
    void watch_temp_callback_E0();
  #endif
  #if EXTRUDERS > 1 && TEMP_SENSOR_1 != 0
    void watch_temp_callback_E1();
  #endif // EXTRUDERS > 1
  #if EXTRUDERS > 2 && TEMP_SENSOR_2 != 0
    void watch_temp_callback_E2();
  #endif // EXTRUDERS > 2
  #if EXTRUDERS > 3 && TEMP_SENSOR_3 != 0
    void watch_temp_callback_E3();
  #endif // EXTRUDERS > 3
#endif
  

  #if ENABLED(HAS_LCD_CONTRAST)
    static void lcd_set_contrast();
  #endif

  #if ENABLED(FWRETRACT)
    static void lcd_control_retract_menu();
  #endif

  #if ENABLED(DELTA_CALIBRATION_MENU)
    static void lcd_delta_calibrate_menu();
  #endif

  #if ENABLED(MANUAL_BED_LEVELING)
    #include "mesh_bed_leveling.h"
  #endif

  /* Different types of actions that can be used in menu items. */
  static void menu_action_back();
  static void menu_action_submenu(menuFunc_t data);
  static void menu_action_gcode(const char* pgcode);
  static void menu_action_function(menuFunc_t data);
  static void menu_action_setting_edit_bool(const char* pstr, bool* ptr);
  static void menu_action_setting_edit_int3(const char* pstr, int* ptr, int minValue, int maxValue);
  static void menu_action_setting_edit_float3(const char* pstr, float* ptr, float minValue, float maxValue);
  static void menu_action_setting_edit_float32(const char* pstr, float* ptr, float minValue, float maxValue);
  static void menu_action_setting_edit_float43(const char* pstr, float* ptr, float minValue, float maxValue);
  static void menu_action_setting_edit_float5(const char* pstr, float* ptr, float minValue, float maxValue);
  static void menu_action_setting_edit_float51(const char* pstr, float* ptr, float minValue, float maxValue);
  static void menu_action_setting_edit_float52(const char* pstr, float* ptr, float minValue, float maxValue);
  static void menu_action_setting_edit_long5(const char* pstr, unsigned long* ptr, unsigned long minValue, unsigned long maxValue);
  static void menu_action_setting_edit_callback_bool(const char* pstr, bool* ptr, menuFunc_t callbackFunc);
  static void menu_action_setting_edit_callback_int3(const char* pstr, int* ptr, int minValue, int maxValue, menuFunc_t callbackFunc);
  static void menu_action_setting_edit_callback_float3(const char* pstr, float* ptr, float minValue, float maxValue, menuFunc_t callbackFunc);
  static void menu_action_setting_edit_callback_float32(const char* pstr, float* ptr, float minValue, float maxValue, menuFunc_t callbackFunc);
  static void menu_action_setting_edit_callback_float43(const char* pstr, float* ptr, float minValue, float maxValue, menuFunc_t callbackFunc);
  static void menu_action_setting_edit_callback_float5(const char* pstr, float* ptr, float minValue, float maxValue, menuFunc_t callbackFunc);
  static void menu_action_setting_edit_callback_float51(const char* pstr, float* ptr, float minValue, float maxValue, menuFunc_t callbackFunc);
  static void menu_action_setting_edit_callback_float52(const char* pstr, float* ptr, float minValue, float maxValue, menuFunc_t callbackFunc);
  static void menu_action_setting_edit_callback_long5(const char* pstr, unsigned long* ptr, unsigned long minValue, unsigned long maxValue, menuFunc_t callbackFunc);

  #if ENABLED(SDSUPPORT)
    static void lcd_sdcard_menu();
    static void menu_action_sdfile(const char* filename, char* longFilename);
    static void menu_action_sddirectory(const char* filename, char* longFilename);
  #endif

  #define ENCODER_FEEDRATE_DEADZONE 10

  #if DISABLED(LCD_I2C_VIKI)
    #ifndef ENCODER_STEPS_PER_MENU_ITEM
      #define ENCODER_STEPS_PER_MENU_ITEM 5
    #endif
    #ifndef ENCODER_PULSES_PER_STEP
      #define ENCODER_PULSES_PER_STEP 1
    #endif
  #else
    #ifndef ENCODER_STEPS_PER_MENU_ITEM
      #define ENCODER_STEPS_PER_MENU_ITEM 2 // VIKI LCD rotary encoder uses a different number of steps per rotation
    #endif
    #ifndef ENCODER_PULSES_PER_STEP
      #define ENCODER_PULSES_PER_STEP 1
    #endif
  #endif


  /* Helper macros for menus */

  /**
   * START_MENU generates the init code for a menu function
   */
  #define START_MENU() do { \
    ENCODER_DIRECTION_MENUS(); \
    encoderRateMultiplierEnabled = false; \
    if (encoderPosition > 0x8000) encoderPosition = 0; \
    uint8_t encoderLine = encoderPosition / ENCODER_STEPS_PER_MENU_ITEM; \
    NOMORE(currentMenuViewOffset, encoderLine); \
    uint8_t _lineNr = currentMenuViewOffset, _menuItemNr; \
    bool wasClicked = LCD_CLICKED, itemSelected; \
    for (uint8_t _drawLineNr = 0; _drawLineNr < LCD_HEIGHT; _drawLineNr++, _lineNr++) { \
      _menuItemNr = 0;

  /**
   * MENU_ITEM generates draw & handler code for a menu item, potentially calling:
   *
   *   lcd_implementation_drawmenu_[type](sel, row, label, arg3...)
   *   menu_action_[type](arg3...)
   *
   * Examples:
   *   MENU_ITEM(back, MSG_WATCH)
   *     lcd_implementation_drawmenu_back(sel, row, PSTR(MSG_WATCH))
   *     menu_action_back()
   *
   *   MENU_ITEM(function, MSG_PAUSE_PRINT, lcd_sdcard_pause)
   *     lcd_implementation_drawmenu_function(sel, row, PSTR(MSG_PAUSE_PRINT), lcd_sdcard_pause)
   *     menu_action_function(lcd_sdcard_pause)
   *
   *   MENU_ITEM_EDIT(int3, MSG_SPEED, &feedrate_multiplier, 10, 999)
   *   MENU_ITEM(setting_edit_int3, MSG_SPEED, PSTR(MSG_SPEED), &feedrate_multiplier, 10, 999)
   *     lcd_implementation_drawmenu_setting_edit_int3(sel, row, PSTR(MSG_SPEED), PSTR(MSG_SPEED), &feedrate_multiplier, 10, 999)
   *     menu_action_setting_edit_int3(PSTR(MSG_SPEED), &feedrate_multiplier, 10, 999)
   *
   */
  #define _MENU_ITEM_PART_1(type, label, args...) \
    if (_menuItemNr == _lineNr) { \
      itemSelected = encoderLine == _menuItemNr; \
      if (lcdDrawUpdate) \
        lcd_implementation_drawmenu_ ## type(itemSelected, _drawLineNr, PSTR(label), ## args); \
      if (wasClicked && itemSelected) { \
        lcd_quick_feedback();\

  #define _MENU_ITEM_PART_2(type, args...) \
        menu_action_ ## type(args); \
        return; \
      } \
    } \
    _menuItemNr++

  #define MENU_ITEM(type, label, args...) do { \
      _MENU_ITEM_PART_1(type, label, ## args); \
      _MENU_ITEM_PART_2(type, ## args); \
    } while(0)

  #if ENABLED(ENCODER_RATE_MULTIPLIER)

    //#define ENCODER_RATE_MULTIPLIER_DEBUG  // If defined, output the encoder steps per second value

    /**
     * MENU_MULTIPLIER_ITEM generates drawing and handling code for a multiplier menu item
     */
    #define MENU_MULTIPLIER_ITEM(type, label, args...) do { \
        _MENU_ITEM_PART_1(type, label, ## args); \
        encoderRateMultiplierEnabled = true; \
        lastEncoderMovementMillis = 0; \
        _MENU_ITEM_PART_2(type, ## args); \
      } while(0)

  #endif //ENCODER_RATE_MULTIPLIER

  #define MENU_ITEM_DUMMY() do { _menuItemNr++; } while(0)
  #define MENU_ITEM_EDIT(type, label, args...) MENU_ITEM(setting_edit_ ## type, label, PSTR(label), ## args)
  #define MENU_ITEM_EDIT_CALLBACK(type, label, args...) MENU_ITEM(setting_edit_callback_ ## type, label, PSTR(label), ## args)
  #if ENABLED(ENCODER_RATE_MULTIPLIER)
    #define MENU_MULTIPLIER_ITEM_EDIT(type, label, args...) MENU_MULTIPLIER_ITEM(setting_edit_ ## type, label, PSTR(label), ## args)
    #define MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(type, label, args...) MENU_MULTIPLIER_ITEM(setting_edit_callback_ ## type, label, PSTR(label), ## args)
  #else //!ENCODER_RATE_MULTIPLIER
    #define MENU_MULTIPLIER_ITEM_EDIT(type, label, args...) MENU_ITEM(setting_edit_ ## type, label, PSTR(label), ## args)
    #define MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(type, label, args...) MENU_ITEM(setting_edit_callback_ ## type, label, PSTR(label), ## args)
  #endif //!ENCODER_RATE_MULTIPLIER
  #define END_MENU() \
      if (encoderLine >= _menuItemNr) { encoderPosition = _menuItemNr * (ENCODER_STEPS_PER_MENU_ITEM) - 1; encoderLine = _menuItemNr - 1; }\
      if (encoderLine >= currentMenuViewOffset + LCD_HEIGHT) { currentMenuViewOffset = encoderLine - (LCD_HEIGHT) + 1; lcdDrawUpdate = LCDVIEW_CALL_REDRAW_NEXT; _lineNr = currentMenuViewOffset - 1; _drawLineNr = -1; } \
      } } while(0)

  /** Used variables to keep track of the menu */
  volatile uint8_t buttons;  //the last checked buttons in a bit array.
  #if ENABLED(REPRAPWORLD_KEYPAD)
    volatile uint8_t buttons_reprapworld_keypad; // to store the keypad shift register values
  #endif

  #if ENABLED(LCD_HAS_SLOW_BUTTONS)
    volatile uint8_t slow_buttons; // Bits of the pressed buttons.
  #endif
  uint8_t currentMenuViewOffset;              /* scroll offset in the current menu */
  millis_t next_button_update_ms;
  uint8_t lastEncoderBits;
  uint32_t encoderPosition;

//NAMADD Filament check
  uint8_t lastEncoderBits_filament_a;
  uint32_t encoderPosition_filament;
  
  uint8_t lastEncoderBits_filament_b;  
  uint8_t lastEncoderBits_filament_c;
  
  #if PIN_EXISTS(SD_DETECT)
    uint8_t lcd_sd_status;
  #endif

#endif // ULTIPANEL

typedef struct {
  menuFunc_t menu_function;
  #if ENABLED(ULTIPANEL)
    uint32_t encoder_position;
  #endif
} menuPosition;

menuFunc_t currentMenu = lcd_status_screen; // pointer to the currently active menu handler

menuPosition menu_history[10];
uint8_t menu_history_depth = 0;

millis_t next_lcd_update_ms;
uint8_t lcd_status_update_delay;
bool ignore_click = false;
bool wait_for_unclick;
bool defer_return_to_status = false;

enum LCDViewAction {
  LCDVIEW_NONE,
  LCDVIEW_REDRAW_NOW,
  LCDVIEW_CALL_REDRAW_NEXT,
  LCDVIEW_CLEAR_CALL_REDRAW,
  LCDVIEW_CALL_NO_REDRAW
};

uint8_t lcdDrawUpdate = LCDVIEW_CLEAR_CALL_REDRAW; // Set when the LCD needs to draw, decrements after every draw. Set to 2 in LCD routines so the LCD gets at least 1 full redraw (first redraw is partial)

// Variables used when editing values.
const char* editLabel;
void* editValue;
int32_t minEditValue, maxEditValue;
menuFunc_t callbackFunc;              // call this after editing

// place-holders for Ki and Kd edits
float raw_Ki, raw_Kd;

/**
 * General function to go directly to a menu
 * Remembers the previous position
 */
static void lcd_goto_menu(menuFunc_t menu, const bool feedback = false, const uint32_t encoder = 0) {
  if (currentMenu != menu) {
    currentMenu = menu;
    lcdDrawUpdate = LCDVIEW_CLEAR_CALL_REDRAW;
    #if ENABLED(NEWPANEL)
      encoderPosition = encoder;

//NAMADD Filament check
encoderPosition_filament = encoder;  //NAMADD TEST

      if (feedback) lcd_quick_feedback();
    #endif
    if (menu == lcd_status_screen) {
      defer_return_to_status = false;
      menu_history_depth = 0;
    }
    #if ENABLED(LCD_PROGRESS_BAR)
      // For LCD_PROGRESS_BAR re-initialize custom characters
      lcd_set_custom_characters(menu == lcd_status_screen);
    #endif
  }
}

static void lcd_return_to_status() { lcd_goto_menu(lcd_status_screen); }

inline void lcd_save_previous_menu() {
  if (menu_history_depth < COUNT(menu_history)) {
    menu_history[menu_history_depth].menu_function = currentMenu;
    #if ENABLED(ULTIPANEL)
      menu_history[menu_history_depth].encoder_position = encoderPosition;
    #endif
    ++menu_history_depth;
  }
}

static void lcd_goto_previous_menu(bool feedback=false) {
  if (menu_history_depth > 0) {
    --menu_history_depth;
    lcd_goto_menu(menu_history[menu_history_depth].menu_function, feedback
      #if ENABLED(ULTIPANEL)
        , menu_history[menu_history_depth].encoder_position
      #endif
    );
  }
  else
    lcd_return_to_status();
}

	 
/**
 *
 * "Info Screen"
 *
 * This is very display-dependent, so the lcd implementation draws this.
 */

static void lcd_status_screen() {
  ENCODER_DIRECTION_NORMAL();
  encoderRateMultiplierEnabled = false;
  
  #if ENABLED(LCD_PROGRESS_BAR)
    millis_t ms = millis();
    #if DISABLED(PROGRESS_MSG_ONCE)
      if (ELAPSED(ms, progress_bar_ms + PROGRESS_BAR_MSG_TIME + PROGRESS_BAR_BAR_TIME)) {
        progress_bar_ms = ms;
      }
    #endif
    #if PROGRESS_MSG_EXPIRE > 0
      // Handle message expire
      if (expire_status_ms > 0) {
        #if ENABLED(SDSUPPORT)
          if (card.isFileOpen()) {
            // Expire the message when printing is active
            if (IS_SD_PRINTING) {
              if (ELAPSED(ms, expire_status_ms)) {
                lcd_status_message[0] = '\0';
                expire_status_ms = 0;
              }
            }
            else {
              expire_status_ms += LCD_UPDATE_INTERVAL;
            }
          }
          else {
            expire_status_ms = 0;
          }
        #else
          expire_status_ms = 0;
        #endif //SDSUPPORT
      }
    #endif
  #endif //LCD_PROGRESS_BAR

	//NAMADD Fix Fan	
	#if FAN_COUNT > 0	
		#if HAS_FAN1		
			if (degHotend(0)<50)
				fanSpeeds[1]=0;
			else 
				fanSpeeds[1]=fanValue;			
		#endif
	#endif

	  //NAMADD MAX TEMP PROTECT
  #if ENABLED(MAXTEMP_PROTECT)
 
	if ((maxtemp_protect_en)&&(current_temperature[0]>=NOZZLE_MAXTEMP_PROTECT) 
		 #if HAS_TEMP_BED
			||(current_temperature_bed >= BED_MAXTEMP_PROTECT)	
		 #endif
		)
	{
		maxtemp_protect_en=false;
		maxtemp_protect_status=true;
		disable_all_heaters();				
		enqueue_and_echo_commands_P(PSTR("M8 X Y Z"));			
	}
		
	if (!maxtemp_protect_en&&((current_temperature[0]<=NOZZLE_MAXTEMP_PROTECT-10)
		#if HAS_TEMP_BED
			&&(current_temperature_bed <= BED_MAXTEMP_PROTECT-5) 
		#endif
		))
	{
		maxtemp_protect_en=true;
		maxtemp_protect_status=false;
		lcd_setstatus("Overheat protect ok.");
		
	}else if (maxtemp_protect_status) enqueue_and_echo_commands_P(PSTR("M8"));		
  #endif
	
    //NAMADD 
	if(load_unload_filament_en)
		lcd_implementation_status_screen_filament_load_unload();
	else if (complete_state )
	{
		//NAMADD LED_RING
		#if ENABLED(LED_RING)
			led_ring_blink_en=true;
		#endif			
		lcd_implementation_status_screen_complete_state();			
	}
	else if (life_time_state)
	{
		lcd_implementation_status_screen_life_time_state();			
	}
	else
		lcd_implementation_status_screen();

  #if ENABLED(ULTIPANEL)
 
    bool current_click = LCD_CLICKED;
	bool filament_click=false;

    if (ignore_click) {
      if (wait_for_unclick) {
        if (!current_click)
          ignore_click = wait_for_unclick = false;
        else
          current_click = false;
      }
      else if (current_click) {
        lcd_quick_feedback();
        wait_for_unclick = true;
        current_click = false;
      }
    }
	//NAMADD
	if(LCD_CLICKED)
	{
		if(load_unload_filament_en)
		{				
			//NAMADD Save current_position[E_AXIS], active_extruder, feedrate_multiplier;.
			current_position[E_AXIS]= original_position_e;
			plan_set_e_position(current_position[E_AXIS]);
			active_extruder= original_active_extruder;
			feedrate_multiplier= original_feedrate_multiplier;
		
			lcd_setstatus("Load/Unload Stop.");
			load_unload_filament_en=false;			
		}
		else if(complete_state)
		{
			complete_state=false;
			enqueuecommands_P(PSTR("M5 W\nG84 S1000"));	
			
			//NAMADD LED_RING
			#if ENABLED(LED_RING)
				#if ENABLED (LED_RING_LOW_V1)
					led_ring_low_v1_sdcard_printing(0,0);
				#elif ENABLED(LED_RING_HIGH_V2)
					led_ring_high_v2_sdcard_printing(0,0);
				#endif
				led_ring_blink_en=false;
			#endif
		}
		else if (life_time_state)
		{
			life_time_state=false;
		}
		else		
			lcd_goto_menu(lcd_main_menu, true);	
		
		while(LCD_CLICKED){};
	}
	
	if (current_click) {
		
	  lcd_implementation_init( // to maybe revive the LCD if static electricity killed it.
		#if ENABLED(LCD_PROGRESS_BAR)
		  currentMenu == lcd_status_screen
		#endif	
	  );
	  #if ENABLED(FILAMENT_LCD_DISPLAY)
		previous_lcd_status_ms = millis();  // get status message to show up for a while
	  #endif
	  //NAMADD
	  #if ENABLED (SDSUPPORT)
		previous_lcd_status_ms_name_file = millis();
	  #endif	  

	}

	#if ENABLED(ULTIPANEL_FEEDMULTIPLY)
	  int new_frm = feedrate_multiplier + (int32_t)encoderPosition;
	  // Dead zone at 100% feedrate
	  if ((feedrate_multiplier < 100 && new_frm > 100) || (feedrate_multiplier > 100 && new_frm < 100)) {
		feedrate_multiplier = 100;
		encoderPosition = 0;
	  }
	  else if (feedrate_multiplier == 100) {
		if ((int32_t)encoderPosition > ENCODER_FEEDRATE_DEADZONE) {
		  feedrate_multiplier += (int32_t)encoderPosition - (ENCODER_FEEDRATE_DEADZONE);
		  encoderPosition = 0;
		}
		else if ((int32_t)encoderPosition < -(ENCODER_FEEDRATE_DEADZONE)) {
		  feedrate_multiplier += (int32_t)encoderPosition + ENCODER_FEEDRATE_DEADZONE;
		  encoderPosition = 0;
		}
	  }
	  else {
		feedrate_multiplier = new_frm;
		encoderPosition = 0;
	  }
	#endif // ULTIPANEL_FEEDMULTIPLY

	feedrate_multiplier = constrain(feedrate_multiplier, 10, 999);

//NAMADD Filament check
	if(feedrate_multiplier<=50)    //NAMADD
		filament_check_delay= 150000;
	else if (feedrate_multiplier>50 && feedrate_multiplier<=100)
		filament_check_delay= 100000;
	else if (feedrate_multiplier>100 && feedrate_multiplier<=200)
		filament_check_delay= 80000;
	else if (feedrate_multiplier>200 && feedrate_multiplier<=300)
		filament_check_delay= 70000;
	else if (feedrate_multiplier>300 && feedrate_multiplier<=400)
		filament_check_delay= 60000;
	else 
		filament_check_delay= 50000;
	
	//NAMADD Filament check test
	// u8g.setFont(u8g_font_5x8); 
	// u8g.setPrintPos(0,12);
	// u8g.print((encoderDiff_filament_a));
	
	// u8g.setPrintPos(0,22);
	// u8g.print((encoderDiff_filament_b));	

	// u8g.setPrintPos(0,32);
	// u8g.print((encoderDiff_filament_c));
	
  #endif //ULTIPANEL
}


#if ENABLED(ULTIPANEL)

inline void line_to_current(AxisEnum axis) {
  #if ENABLED(DELTA)
    calculate_delta(current_position);
    plan_buffer_line(delta[X_AXIS], delta[Y_AXIS], delta[Z_AXIS], current_position[E_AXIS], manual_feedrate[axis]/60, active_extruder);
  #else
    plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], manual_feedrate[axis]/60, active_extruder);
  #endif
}

#if ENABLED(SDSUPPORT)

  static void lcd_sdcard_pause() {	
	  card.pauseSDPrint();
		
	  #if (ENABLED(GALVO_LASER)|| ENABLED(LASER_ENGRAVING_MODULE))
	    laserPower=0;
	  #endif 
	  enqueue_and_echo_commands_P(PSTR("G91 \nG1 Z+1 E-5 F1000 \nG90 \nG0 X-126 Y-106 F5000"));	  
	  lcd_setstatus(MSG_PAUSE_PRINT,true);	  
	  lcd_return_to_status();
  }

  static void lcd_sdcard_resume() { 
    enqueue_and_echo_commands_P(PSTR("G91 \nG1 Z-1 E+5 F1000 \nG90 \nG0 X0 Y0 F5000"));	    	
	card.startFileprint();
	//NAMADD Filament check
	//filament_check_enabled_a=true;
	//filament_check_enabled_b=true;
	//filament_check_enabled_c=true;

	lcd_setstatus(MSG_RESUME_PRINT,true);
	lcd_return_to_status();
}

  static void lcd_sdcard_stop() {

    quickStop();
    card.sdprinting = false;
    card.closefile();
    autotempShutdown();
    cancel_heatup = true;
    lcd_setstatus(MSG_PRINT_ABORTED, true);
    #if (ENABLED(GALVO_LASER)|| ENABLED(LASER_ENGRAVING_MODULE))
	    laserPower=0;
	#endif  
	  lcd_return_to_status();
  }

#endif //SDSUPPORT

float move_menu_scale;

static void _lcd_move(const char* name, AxisEnum axis, float min, float max) {
  ENCODER_DIRECTION_NORMAL();
  if (encoderPosition && movesplanned() <= 3) {
    refresh_cmd_timeout();
    current_position[axis] += float((int32_t)encoderPosition) * move_menu_scale;
    if (min_software_endstops) NOLESS(current_position[axis], min);
    if (max_software_endstops) NOMORE(current_position[axis], max);
    line_to_current(axis);
    lcdDrawUpdate = LCDVIEW_REDRAW_NOW;
  }
  encoderPosition = 0;
  if (lcdDrawUpdate) lcd_implementation_drawedit(name, ftostr31(current_position[axis]));
  if (LCD_CLICKED) lcd_goto_previous_menu(true);
}
#if ENABLED(DELTA)
  static float delta_clip_radius_2 =  (DELTA_PRINTABLE_RADIUS) * (DELTA_PRINTABLE_RADIUS);
  static int delta_clip( float a ) { return sqrt(delta_clip_radius_2 - a*a); }
  static void lcd_move_x() { int clip = delta_clip(current_position[Y_AXIS]); _lcd_move(PSTR(MSG_MOVE_X), X_AXIS, max(sw_endstop_min[X_AXIS], -clip), min(sw_endstop_max[X_AXIS], clip)); }
  static void lcd_move_y() { int clip = delta_clip(current_position[X_AXIS]); _lcd_move(PSTR(MSG_MOVE_Y), Y_AXIS, max(sw_endstop_min[Y_AXIS], -clip), min(sw_endstop_max[Y_AXIS], clip)); }
#else
  static void lcd_move_x() { _lcd_move(PSTR(MSG_MOVE_X), X_AXIS, sw_endstop_min[X_AXIS], sw_endstop_max[X_AXIS]); }
  static void lcd_move_y() { _lcd_move(PSTR(MSG_MOVE_Y), Y_AXIS, sw_endstop_min[Y_AXIS], sw_endstop_max[Y_AXIS]); }
#endif
static void lcd_move_z() { _lcd_move(PSTR(MSG_MOVE_Z), Z_AXIS, sw_endstop_min[Z_AXIS], sw_endstop_max[Z_AXIS]); }
static void lcd_move_e(
  #if EXTRUDERS > 1
    uint8_t e
  #endif
) {
  ENCODER_DIRECTION_NORMAL();
  #if EXTRUDERS > 1
    unsigned short original_active_extruder = active_extruder;
    active_extruder = e;
  #endif
  if (encoderPosition && movesplanned() <= 3) {
    current_position[E_AXIS] += float((int32_t)encoderPosition) * move_menu_scale;
    line_to_current(E_AXIS);
    lcdDrawUpdate = LCDVIEW_REDRAW_NOW;
  }
  encoderPosition = 0;
  if (lcdDrawUpdate) {
    PGM_P pos_label;
    #if EXTRUDERS == 1
      pos_label = PSTR(MSG_MOVE_E);
    #else
      switch (e) {
        case 0: pos_label = PSTR(MSG_MOVE_E MSG_MOVE_E1); break;
        case 1: pos_label = PSTR(MSG_MOVE_E MSG_MOVE_E2); break;
        #if EXTRUDERS > 2
          case 2: pos_label = PSTR(MSG_MOVE_E MSG_MOVE_E3); break;
          #if EXTRUDERS > 3
            case 3: pos_label = PSTR(MSG_MOVE_E MSG_MOVE_E4); break;
          #endif //EXTRUDERS > 3
        #endif //EXTRUDERS > 2
      }
    #endif //EXTRUDERS > 1
    lcd_implementation_drawedit(pos_label, ftostr31(current_position[E_AXIS]));
  }
  if (LCD_CLICKED) lcd_goto_previous_menu(true);
  #if EXTRUDERS > 1
    active_extruder = original_active_extruder;
  #endif
}

#if EXTRUDERS > 1
  static void lcd_move_e0() { lcd_move_e(0); }
  static void lcd_move_e1() { lcd_move_e(1); }
  #if EXTRUDERS > 2
    static void lcd_move_e2() { lcd_move_e(2); }
    #if EXTRUDERS > 3
      static void lcd_move_e3() { lcd_move_e(3); }
    #endif
  #endif
#endif // EXTRUDERS > 1


/*
//NAMADD Filament Load/Unload
*/
static void lcd_filament_menu();
static void lcd_filament_load();
static void lcd_filament_unload();

static void lcd_filament_load_200();
static void lcd_filament_load_200_ext0();
static void lcd_filament_load_200_ext1();
static void lcd_filament_load_200_ext2();

static void lcd_filament_load_220();
static void lcd_filament_load_220_ext0();
static void lcd_filament_load_220_ext1();
static void lcd_filament_load_220_ext2();

static void lcd_filament_load_240();
static void lcd_filament_load_240_ext0();
static void lcd_filament_load_240_ext1();
static void lcd_filament_load_240_ext2();

static void lcd_filament_unload_180();
static void lcd_filament_unload_180_ext0();
static void lcd_filament_unload_180_ext1();
static void lcd_filament_unload_180_ext2();

static void lcd_filament_unload_200();
static void lcd_filament_unload_200_ext0();
static void lcd_filament_unload_200_ext1();
static void lcd_filament_unload_200_ext2();

static void lcd_filament_unload_240();
static void lcd_filament_unload_240_ext0();
static void lcd_filament_unload_240_ext1();
static void lcd_filament_unload_240_ext2();
static void lcd_filament_load_unload (unsigned int extruder, unsigned int pla_abs, unsigned int unload_load);


void lcd_filament_change_extruder_0();
#if EXTRUDERS != 1	
	void lcd_filament_change_extruder_1();
	#if EXTRUDERS >1
		void lcd_filament_change_extruder_2();
	#endif
#endif

#if TEMP_SENSOR_0 != 0
  void lcd_preheat_pla0();
  void lcd_preheat_abs0();
#endif

#if TEMP_SENSOR_0 != 0 && (TEMP_SENSOR_1 != 0 || TEMP_SENSOR_2 != 0 || TEMP_SENSOR_3 != 0 || TEMP_SENSOR_BED != 0)
  static void lcd_preheat_menu();
  static void lcd_preheat_pla_menu();
  static void lcd_preheat_abs_menu();
#endif


void lcd_nozzole_preheat_0(){
	#if TEMP_SENSOR_0 != 0		
		setTargetHotend0(0);
	#endif	
	#if TEMP_SENSOR_1 != 0		
		setTargetHotend1(0);
	#endif
	#if TEMP_SENSOR_2 != 0		
		setTargetHotend2(0);
	#endif	
	lcd_return_to_status();
}
void lcd_nozzole_preheat_200(){
    #if TEMP_SENSOR_0 != 0		
	  setTargetHotend0(200);
    #endif
	#if TEMP_SENSOR_1 != 0		
		setTargetHotend1(200);
	#endif
	#if TEMP_SENSOR_2 != 0		
		setTargetHotend2(200);
	#endif
    lcd_return_to_status(); 	
}
void lcd_nozzole_preheat_220(){
	#if TEMP_SENSOR_0 != 0		
		setTargetHotend0(220);
	#endif	
	#if TEMP_SENSOR_1 != 0		
		setTargetHotend1(220);
	#endif
	#if TEMP_SENSOR_2 != 0		
		setTargetHotend2(220);
	#endif	
	lcd_return_to_status();
}
void lcd_nozzole_preheat_240(){
	#if TEMP_SENSOR_0 != 0		
		setTargetHotend0(240);
	#endif
	#if TEMP_SENSOR_1 != 0		
		setTargetHotend1(240);
	#endif
	#if TEMP_SENSOR_2 != 0		
		setTargetHotend2(240);
	#endif	
	lcd_return_to_status();	
}
void lcd_nozzole_preheat_250(){
	#if TEMP_SENSOR_0 != 0		
		setTargetHotend0(250);
	#endif
	#if TEMP_SENSOR_1 != 0		
		setTargetHotend1(250);
	#endif
	#if TEMP_SENSOR_2 != 0		
		setTargetHotend2(250);
	#endif	
	lcd_return_to_status();	
}


void lcd_bed_preheat_0(){
	#if TEMP_SENSOR_BED != 0
		setTargetBed(0);
	#else
		UNUSED(0);
	#endif	
	lcd_return_to_status();
}
void lcd_bed_preheat_40(){
	#if TEMP_SENSOR_BED != 0
		setTargetBed(40);
	#else
		UNUSED(40);
	#endif	
	lcd_return_to_status();
}
void lcd_bed_preheat_50(){
	#if TEMP_SENSOR_BED != 0
		setTargetBed(50);
	#else
		UNUSED(50);
	#endif	
	lcd_return_to_status();
}
void lcd_bed_preheat_60(){
	#if TEMP_SENSOR_BED != 0
		setTargetBed(60);
	#else
		UNUSED(60);
	#endif	
	lcd_return_to_status();
}
void lcd_bed_preheat_70(){
	#if TEMP_SENSOR_BED != 0
		setTargetBed(70);
	#else
		UNUSED(70);
	#endif	
	lcd_return_to_status();
}
void lcd_bed_preheat_100(){
	#if TEMP_SENSOR_BED != 0
		setTargetBed(100);
	#else
		UNUSED(100);
	#endif
	lcd_return_to_status();	
}
void lcd_bed_preheat_130(){
	#if TEMP_SENSOR_BED != 0
		setTargetBed(130);
	#else
		UNUSED(130);
	#endif	
		lcd_return_to_status();
}
  // "Preheat" sub menu item
  // Nozzle:
  // Nozzle [1-4]:
  //
void lcd_nozzole_preheat_menu(){
	START_MENU();
	#if TEMP_SENSOR_BED!=0
		MENU_ITEM(back, MSG_PREHEAT);
	#else
		MENU_ITEM(back, MSG_MAIN);	
	#endif
	
	MENU_ITEM(function, MSG_200, lcd_nozzole_preheat_200 );
	MENU_ITEM(function, MSG_220, lcd_nozzole_preheat_220 );
	MENU_ITEM(function, MSG_240, lcd_nozzole_preheat_240 );
	MENU_ITEM(function, MSG_250, lcd_nozzole_preheat_250 );
	
  #if EXTRUDERS == 1
    #if TEMP_SENSOR_0 != 0
      MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(int3, MSG_CUSTOM, &target_temperature[0], 0, HEATER_0_MAXTEMP - 15, watch_temp_callback_E0);
    #endif
  #else //EXTRUDERS > 1
    #if TEMP_SENSOR_0 != 0
      MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(int3, MSG_CUSTOM, &target_temperature[0], 0, HEATER_0_MAXTEMP - 15, watch_temp_callback_E0);
    #endif	 

    #if (TEMP_SENSOR_1 != 0 && TEMP_SENSOR_2 != 0)
		target_temperature[1]=target_temperature[0];
		target_temperature[2]=target_temperature[0];
    #endif	
  #endif // EXTRUDERS > 1
  
	MENU_ITEM(function, MSG_0, lcd_nozzole_preheat_0 );
	END_MENU();
}

  //
  // Bed:
  //
  void lcd_bed_preheat_menu(){
	START_MENU();
	MENU_ITEM(back, MSG_PREHEAT);
	
	MENU_ITEM(function, MSG_40, lcd_bed_preheat_40 );
	MENU_ITEM(function, MSG_50, lcd_bed_preheat_50 );
	MENU_ITEM(function, MSG_60, lcd_bed_preheat_60 );
	MENU_ITEM(function, MSG_70, lcd_bed_preheat_70 );
	MENU_ITEM(function, MSG_100, lcd_bed_preheat_100 );
	//MENU_ITEM(function, MSG_130, lcd_bed_preheat_130 );	  
  #if TEMP_SENSOR_BED != 0
    MENU_MULTIPLIER_ITEM_EDIT(int3, MSG_CUSTOM, &target_temperature_bed, 0, BED_MAXTEMP - 15);
  #endif	
	MENU_ITEM(function, MSG_0, lcd_bed_preheat_0 );
	END_MENU();
  }
  
static void lcd_preheat_menu(){
	
	#if TEMP_SENSOR_BED!=0
	  START_MENU();
	  MENU_ITEM(back, MSG_MAIN);
	  MENU_ITEM(submenu,MSG_NOZZLE,lcd_nozzole_preheat_menu);
	  MENU_ITEM(submenu,MSG_BED, lcd_bed_preheat_menu);
	  END_MENU();
	#else
		lcd_nozzole_preheat_menu();	
	#endif
}

//FILAMENT
static void lcd_filament_menu()
{
  START_MENU();
  // Go back to previous menu
  MENU_ITEM(back, MSG_MAIN);

  if (!(movesplanned() || IS_SD_PRINTING)) { 
	MENU_ITEM(submenu, MSG_LOAD, lcd_filament_load);  
	MENU_ITEM(submenu, MSG_UNLOAD, lcd_filament_unload); 
  }
  
  //NAMADD Filament check menu
  #if EXTRUDERS==1
	MENU_ITEM_EDIT(bool, MSG_FILAMENT_CHECK_E, &filament_check_enabled_a);
  #elif EXTRUDERS==2 
	MENU_ITEM_EDIT(bool, MSG_FILAMENT_CHECK_E1, &filament_check_enabled_a);
	MENU_ITEM_EDIT(bool, MSG_FILAMENT_CHECK_E2, &filament_check_enabled_b);
  #else
	MENU_ITEM_EDIT(bool, MSG_FILAMENT_CHECK_E1, &filament_check_enabled_a);
	MENU_ITEM_EDIT(bool, MSG_FILAMENT_CHECK_E2, &filament_check_enabled_b);
	MENU_ITEM_EDIT(bool, MSG_FILAMENT_CHECK_E3, &filament_check_enabled_c);
  #endif

  END_MENU();
}
///LOAD FILAMENT
static void lcd_filament_load()
{
	
  START_MENU();
  // Go back to previous menu
  MENU_ITEM(back, MSG_FILAMENT);
  #if EXTRUDERS == 1
	  MENU_ITEM(function, MSG_SET_TEMP200,lcd_filament_load_200_ext0);
	  MENU_ITEM(function, MSG_SET_TEMP220,lcd_filament_load_220_ext0);	  
	  MENU_ITEM(function, MSG_SET_TEMP240,lcd_filament_load_240_ext0);
  #else
	  MENU_ITEM(submenu, MSG_SET_TEMP200, lcd_filament_load_200);
	  MENU_ITEM(submenu, MSG_SET_TEMP220, lcd_filament_load_220);
	  MENU_ITEM(submenu, MSG_SET_TEMP240, lcd_filament_load_240);
  #endif
  
  END_MENU();
}

static void lcd_filament_load_200()
{
  move_menu_scale=1;	
  START_MENU();
  MENU_ITEM(back, MSG_FILAMENT);
  MENU_ITEM(function, MSG_EXTRUDER_1, lcd_filament_load_200_ext0);  //E0
  MENU_ITEM(function, MSG_EXTRUDER_2, lcd_filament_load_200_ext1);  //E1
  #if EXTRUDERS > 2
    MENU_ITEM(function, MSG_EXTRUDER_3, lcd_filament_load_200_ext2);  //E2
  #endif
  END_MENU();	
}
static void lcd_filament_load_220()
{
	move_menu_scale=1;	
  START_MENU();
  MENU_ITEM(back, MSG_FILAMENT);
  MENU_ITEM(function, MSG_EXTRUDER_1, lcd_filament_load_220_ext0);  //E0
  MENU_ITEM(function, MSG_EXTRUDER_2, lcd_filament_load_220_ext1);  //E1
  #if EXTRUDERS > 2
    MENU_ITEM(function, MSG_EXTRUDER_3, lcd_filament_load_220_ext2);  //E2
  #endif
  END_MENU();	
}
static void lcd_filament_load_240()
{
	START_MENU();
	MENU_ITEM(back, MSG_FILAMENT);
	MENU_ITEM(function, MSG_EXTRUDER_1, lcd_filament_load_240_ext0);  //E0
	MENU_ITEM(function, MSG_EXTRUDER_2, lcd_filament_load_240_ext1);  //E1
	#if EXTRUDERS > 2
		MENU_ITEM(function, MSG_EXTRUDER_3, lcd_filament_load_240_ext2);  //E2
	#endif
	END_MENU();
}

////UNLOAD FILAMENT
static void lcd_filament_unload()
{	
  START_MENU();
  // Go back to previous menu
  MENU_ITEM(back, MSG_FILAMENT);
  #if EXTRUDERS == 1
	MENU_ITEM(function, MSG_SET_TEMP200, lcd_filament_unload_180_ext0); 
	MENU_ITEM(function, MSG_SET_TEMP220, lcd_filament_unload_200_ext0);  
	MENU_ITEM(function, MSG_SET_TEMP240, lcd_filament_unload_240_ext0);  
  #else
	MENU_ITEM(submenu, MSG_SET_TEMP200, lcd_filament_unload_180);  
	MENU_ITEM(submenu, MSG_SET_TEMP220, lcd_filament_unload_200);  
	MENU_ITEM(submenu, MSG_SET_TEMP240, lcd_filament_unload_240);  
  #endif

  END_MENU();
}

static void lcd_filament_unload_180()
{	
	START_MENU();
	MENU_ITEM(back, MSG_UNLOAD);
	MENU_ITEM(function, MSG_EXTRUDER_1, lcd_filament_unload_180_ext0);  //E0
	MENU_ITEM(function, MSG_EXTRUDER_2, lcd_filament_unload_180_ext1);  //E1
	#if EXTRUDERS > 2
		MENU_ITEM(function, MSG_EXTRUDER_3, lcd_filament_unload_180_ext2);  //E2
	#endif
	END_MENU();

}
static void lcd_filament_unload_200()
{	
	START_MENU();
	MENU_ITEM(back, MSG_UNLOAD);
	MENU_ITEM(function, MSG_EXTRUDER_1, lcd_filament_unload_200_ext0);  //E0	
	MENU_ITEM(function, MSG_EXTRUDER_2, lcd_filament_unload_200_ext1);  //E1
	#if EXTRUDERS > 2
		MENU_ITEM(function, MSG_EXTRUDER_3, lcd_filament_unload_200_ext2);  //E2
	#endif
	END_MENU();
}
static void lcd_filament_unload_240()
{	
	START_MENU();
	MENU_ITEM(back, MSG_UNLOAD);
	MENU_ITEM(function, MSG_EXTRUDER_1, lcd_filament_unload_240_ext0);  //E0
	MENU_ITEM(function, MSG_EXTRUDER_2, lcd_filament_unload_240_ext1);  //E1
	#if EXTRUDERS > 2
		MENU_ITEM(function, MSG_EXTRUDER_3, lcd_filament_unload_240_ext2);  //E2
	#endif
	END_MENU();	
}

///
static void lcd_filament_unload_180_ext0 ()
{
	original_en=true;
    enqueuecommands_P(PSTR("T0"));
    setTargetHotend(200, 0); 
	#if EXTRUDERS > 1
		load_unload_filament_ext=0;
	#endif
    load_unload=false;
	load_unload_filament_en=true;
    lcd_return_to_status();
}
static void lcd_filament_unload_200_ext0 ()
{
	original_en=true;
    enqueuecommands_P(PSTR("T0"));
    setTargetHotend(220, 0); 
	#if EXTRUDERS > 1
		load_unload_filament_ext=0;
	#endif
    load_unload=false;
	load_unload_filament_en=true;
    lcd_return_to_status();
}
static void lcd_filament_unload_240_ext0 ()
{
	original_en=true;
    enqueuecommands_P(PSTR("T0"));
    setTargetHotend(240, 0); 
    #if EXTRUDERS > 1
		load_unload_filament_ext=0;
	#endif
    load_unload=false;
	load_unload_filament_en=true;
    lcd_return_to_status();
}

static void lcd_filament_load_200_ext0 ()
{
	original_en=true;	
    enqueuecommands_P(PSTR("T0"));
    setTargetHotend(200, 0); 
    #if EXTRUDERS > 1
		load_unload_filament_ext=0;
	#endif
    load_unload=true;
	load_unload_filament_en=true;
    lcd_return_to_status();
}
static void lcd_filament_load_220_ext0 ()
{
	original_en=true;	
    enqueuecommands_P(PSTR("T0"));
    setTargetHotend(220, 0); 
    #if EXTRUDERS > 1
		load_unload_filament_ext=0;
	#endif
    load_unload=true;
	load_unload_filament_en=true;		
    lcd_return_to_status();
}
static void lcd_filament_load_240_ext0 ()
{
	original_en=true;
    enqueuecommands_P(PSTR("T0"));
    setTargetHotend(240, 0); 
    #if EXTRUDERS > 1
		load_unload_filament_ext=0;
	#endif
    load_unload=true;
	load_unload_filament_en=true;
    lcd_return_to_status();
}

static void lcd_filament_unload_180_ext1 ()
{
	original_en=true;
    enqueuecommands_P(PSTR("T0"));
    setTargetHotend(200, 0); 
    #if EXTRUDERS > 1
		load_unload_filament_ext=1;
	#endif
    load_unload=false;
	load_unload_filament_en=true;
    lcd_return_to_status();
}
static void lcd_filament_unload_200_ext1 ()
{
	original_en=true;	
    enqueuecommands_P(PSTR("T0"));
    setTargetHotend(220, 0); 
    #if EXTRUDERS > 1
		load_unload_filament_ext=1;
	#endif
    load_unload=false;
	load_unload_filament_en=true;
    lcd_return_to_status();
}
static void lcd_filament_unload_240_ext1 ()
{
	original_en=true;	
    enqueuecommands_P(PSTR("T0"));
    setTargetHotend(240, 0); 
    #if EXTRUDERS > 1
		load_unload_filament_ext=1;
	#endif
    load_unload=false;
	load_unload_filament_en=true;
    lcd_return_to_status();
}

static void lcd_filament_load_200_ext1 ()
{
	original_en=true;
    enqueuecommands_P(PSTR("T0"));
    setTargetHotend(200, 0); 
    #if EXTRUDERS > 1
		load_unload_filament_ext=1;
	#endif
    load_unload=true;
	load_unload_filament_en=true;		
    lcd_return_to_status();
}
static void lcd_filament_load_220_ext1 ()
{
	original_en=true;	
    enqueuecommands_P(PSTR("T0"));
    setTargetHotend(220, 0); 
    #if EXTRUDERS > 1
		load_unload_filament_ext=1;
	#endif
    load_unload=true;
	load_unload_filament_en=true;	
    lcd_return_to_status();
}
static void lcd_filament_load_240_ext1 ()
{
	original_en=true;
    enqueuecommands_P(PSTR("T0"));
    setTargetHotend(240, 0); 	
    #if EXTRUDERS > 1
		load_unload_filament_ext=1;
	#endif
    load_unload=true;
	load_unload_filament_en=true;		
    lcd_return_to_status();
}

static void lcd_filament_unload_180_ext2 ()
{
	original_en=true;	
    enqueuecommands_P(PSTR("T0"));
    setTargetHotend(200, 0);   
    #if EXTRUDERS > 1
		load_unload_filament_ext=2;
	#endif
    load_unload=false;
	load_unload_filament_en=true;
    lcd_return_to_status();
}
static void lcd_filament_unload_200_ext2 ()
{
	original_en=true;	
    enqueuecommands_P(PSTR("T0"));
    setTargetHotend(220, 0);   
    #if EXTRUDERS > 1
		load_unload_filament_ext=2;
	#endif
    load_unload=false;
	load_unload_filament_en=true;
    lcd_return_to_status();
}
static void lcd_filament_unload_240_ext2 ()
{
	original_en=true;	
    enqueuecommands_P(PSTR("T0"));
    setTargetHotend(240, 0);  
    #if EXTRUDERS > 1
		load_unload_filament_ext=2;
	#endif
    load_unload=false;
	load_unload_filament_en=true;
    lcd_return_to_status();
}

static void lcd_filament_load_200_ext2 ()
{
	original_en=true;
    enqueuecommands_P(PSTR("T0"));
    setTargetHotend(200, 0); 
    #if EXTRUDERS > 1
		load_unload_filament_ext=2;
	#endif
    load_unload=true;
	load_unload_filament_en=true;
    lcd_return_to_status();
}
static void lcd_filament_load_220_ext2 ()
{
	original_en=true;
    enqueuecommands_P(PSTR("T0"));
    setTargetHotend(220, 0); 
    #if EXTRUDERS > 1
		load_unload_filament_ext=2;
	#endif
    load_unload=true;
	load_unload_filament_en=true;		
    lcd_return_to_status();
}
static void lcd_filament_load_240_ext2 ()
{
	original_en=true;
    enqueuecommands_P(PSTR("T0"));
    setTargetHotend(240, 0); 
    #if EXTRUDERS > 1
		load_unload_filament_ext=2;
	#endif
    load_unload=true;
	load_unload_filament_en=true;		
    lcd_return_to_status();
}

/**
 *
 * "Main" menu
 *
 */

static void lcd_main_menu() {
  START_MENU();
  //Back Infor Menu
  MENU_ITEM(back, MSG_WATCH);
  //Print File Menu
  #if ENABLED(SDSUPPORT)
    if (card.cardOK) {
      if (card.isFileOpen()) {
        if (card.sdprinting)
          MENU_ITEM(function, MSG_PAUSE_PRINT, lcd_sdcard_pause);
        else
          MENU_ITEM(function, MSG_RESUME_PRINT, lcd_sdcard_resume);
		  MENU_ITEM(function, MSG_STOP_PRINT, lcd_sdcard_stop);
      }
      else {
        MENU_ITEM(submenu, MSG_CARD_MENU, lcd_sdcard_menu);
        #if !PIN_EXISTS(SD_DETECT)
          MENU_ITEM(gcode, MSG_CNG_SDCARD, PSTR("M21"));  // SD-card changed by user
        #endif
      }
    }
    else {
      //MENU_ITEM(submenu, MSG_NO_CARD, lcd_sdcard_menu);
      #if !PIN_EXISTS(SD_DETECT)
        MENU_ITEM(gcode, MSG_INIT_SDCARD, PSTR("M21")); // Manually initialize the SD-card via user interface
      #endif
    }
  #endif //SDSUPPORT
 
  // Preheat menu	
  MENU_ITEM(submenu, MSG_PREHEAT, lcd_preheat_menu);
	
  if (movesplanned() || IS_SD_PRINTING) {	
    MENU_ITEM(submenu, MSG_TUNE, lcd_tune_menu);
  }
	 
  // Filament menu	
  MENU_ITEM(submenu, MSG_FILAMENT, lcd_filament_menu);

  MENU_ITEM(submenu, MSG_SETTING, lcd_setting_menu); 
  END_MENU();
}


/**
 *
 * "Setting" submenu items
 *
 */
// "language" items
void lcd_tieng_viet(){
}

void lcd_english(){
}

void lcd_language_menu(){
	START_MENU();
	MENU_ITEM(back, MSG_SETTING);
	MENU_ITEM(function, MSG_TIENG_VIET, lcd_tieng_viet);
	MENU_ITEM(function, MSG_ENGLISH, lcd_english);
	END_MENU();
}


//"other" items
// void lcd_other_menu(){
	// START_MENU();
	// MENU_ITEM(back, MSG_SETTING);
	// MENU_ITEM(submenu, MSG_CONTROL, lcd_control_menu);	
	// END_MENU();
// }


/**
 *
 * "Setting" submenu
 *
 */
static void lcd_setting_menu(){
	START_MENU();
	MENU_ITEM(back, MSG_MAIN);
	if (!(movesplanned() || IS_SD_PRINTING)) {
		MENU_ITEM(submenu, MSG_MOTOR, lcd_prepare_menu);
		#if ENABLED(DELTA_CALIBRATION_MENU)
		  MENU_ITEM(submenu, MSG_DELTA_CALIBRATE, lcd_delta_calibrate_menu);
		#endif
	}
	MENU_ITEM(submenu, MSG_LANGUAGE, lcd_language_menu);
	MENU_ITEM(submenu, MSG_CONTROL, lcd_control_menu);
	MENU_ITEM(function,MSG_LIFE_TIME, lcd_life_time);	
	END_MENU();
}	
 
/**
 *
 * "Tune" submenu items
 *
 */

/**
 * Set the home offset based on the current_position
 */
void lcd_set_home_offsets() {
  // M428 Command
  enqueue_and_echo_commands_P(PSTR("M428"));
  lcd_return_to_status();
}

#if ENABLED(BABYSTEPPING)

  int babysteps_done = 0;

  static void _lcd_babystep(const int axis, const char* msg) {
    ENCODER_DIRECTION_NORMAL();
    if (encoderPosition) {
      int distance = (int32_t)encoderPosition * BABYSTEP_MULTIPLICATOR;
      encoderPosition = 0;
      lcdDrawUpdate = LCDVIEW_REDRAW_NOW;
      #if ENABLED(COREXY) || ENABLED(COREXZ)
        #if ENABLED(BABYSTEP_XY)
          switch(axis) {
            case X_AXIS: // X on CoreXY and CoreXZ
              babystepsTodo[A_AXIS] += distance * 2;
              babystepsTodo[CORE_AXIS_2] += distance * 2;
              break;
            case CORE_AXIS_2: // Y on CoreXY, Z on CoreXZ
              babystepsTodo[A_AXIS] += distance * 2;
              babystepsTodo[CORE_AXIS_2] -= distance * 2;
              break;
            case CORE_AXIS_3: // Z on CoreXY, Y on CoreXZ
              babystepsTodo[CORE_AXIS_3] += distance;
              break;
          }
        #elif ENABLED(COREXZ)
          babystepsTodo[A_AXIS] += distance * 2;
          babystepsTodo[C_AXIS] -= distance * 2;
        #else
          babystepsTodo[Z_AXIS] += distance;
        #endif
      #else
        babystepsTodo[axis] += distance;
      #endif

      babysteps_done += distance;
    }
    if (lcdDrawUpdate) lcd_implementation_drawedit(msg, itostr3sign(babysteps_done));
    if (LCD_CLICKED) lcd_goto_previous_menu(true);
  }

  #if ENABLED(BABYSTEP_XY)
    static void _lcd_babystep_x() { _lcd_babystep(X_AXIS, PSTR(MSG_BABYSTEPPING_X)); }
    static void _lcd_babystep_y() { _lcd_babystep(Y_AXIS, PSTR(MSG_BABYSTEPPING_Y)); }
    static void lcd_babystep_x() { babysteps_done = 0; lcd_goto_menu(_lcd_babystep_x); }
    static void lcd_babystep_y() { babysteps_done = 0; lcd_goto_menu(_lcd_babystep_y); }
  #endif
  static void _lcd_babystep_z() { _lcd_babystep(Z_AXIS, PSTR(MSG_BABYSTEPPING_Z)); }
  static void lcd_babystep_z() { babysteps_done = 0; lcd_goto_menu(_lcd_babystep_z); }

#endif //BABYSTEPPING

/**
 * Watch temperature callbacks
 */
#if ENABLED(THERMAL_PROTECTION_HOTENDS)
  #if TEMP_SENSOR_0 != 0
    void watch_temp_callback_E0() { start_watching_heater(0); }
  #endif
  #if EXTRUDERS > 1 && TEMP_SENSOR_1 != 0
    void watch_temp_callback_E1() { start_watching_heater(1); }
  #endif // EXTRUDERS > 1
  #if EXTRUDERS > 2 && TEMP_SENSOR_2 != 0
    void watch_temp_callback_E2() { start_watching_heater(2); }
  #endif // EXTRUDERS > 2
  #if EXTRUDERS > 3 && TEMP_SENSOR_3 != 0
    void watch_temp_callback_E3() { start_watching_heater(3); }
  #endif // EXTRUDERS > 3
#else
  #if TEMP_SENSOR_0 != 0
    void watch_temp_callback_E0() {}
  #endif
  #if EXTRUDERS > 1 && TEMP_SENSOR_1 != 0
    void watch_temp_callback_E1() {}
  #endif // EXTRUDERS > 1
  #if EXTRUDERS > 2 && TEMP_SENSOR_2 != 0
    void watch_temp_callback_E2() {}
  #endif // EXTRUDERS > 2
  #if EXTRUDERS > 3 && TEMP_SENSOR_3 != 0
    void watch_temp_callback_E3() {}
  #endif // EXTRUDERS > 3
#endif

/**
 *
 * "Tune" submenu
 *
 */
static void lcd_tune_menu() {
  START_MENU();

  //
  // ^ Main
  //
  MENU_ITEM(back, MSG_MAIN);

  //
  // Speed:
  //
  MENU_ITEM_EDIT(int3, MSG_SPEED, &feedrate_multiplier, 10, 999);

  // Manual bed leveling, Bed Z:
  #if ENABLED(MANUAL_BED_LEVELING)
    MENU_ITEM_EDIT(float43, MSG_BED_Z, &mbl.z_offset, -1, 1);
  #endif

  //
  // Nozzle:
  // Nozzle [1-4]:
  //
  #if EXTRUDERS == 1
    #if TEMP_SENSOR_0 != 0
      MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(int3, MSG_NOZZLE, &target_temperature[0], 0, HEATER_0_MAXTEMP - 15, watch_temp_callback_E0);
    #endif
  #else //EXTRUDERS > 1
    #if TEMP_SENSOR_0 != 0
      MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(int3, MSG_NOZZLE MSG_N1, &target_temperature[0], 0, HEATER_0_MAXTEMP - 15, watch_temp_callback_E0);
    #endif
    #if TEMP_SENSOR_1 != 0
      MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(int3, MSG_NOZZLE MSG_N2, &target_temperature[1], 0, HEATER_1_MAXTEMP - 15, watch_temp_callback_E1);
    #endif
    #if EXTRUDERS > 2
      #if TEMP_SENSOR_2 != 0
        MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(int3, MSG_NOZZLE MSG_N3, &target_temperature[2], 0, HEATER_2_MAXTEMP - 15, watch_temp_callback_E2);
      #endif
      #if EXTRUDERS > 3
        #if TEMP_SENSOR_3 != 0
          MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(int3, MSG_NOZZLE MSG_N4, &target_temperature[3], 0, HEATER_3_MAXTEMP - 15, watch_temp_callback_E3);
        #endif
      #endif // EXTRUDERS > 3
    #endif // EXTRUDERS > 2
  #endif // EXTRUDERS > 1

  //
  // Bed:
  //
  #if TEMP_SENSOR_BED != 0
    MENU_MULTIPLIER_ITEM_EDIT(int3, MSG_BED, &target_temperature_bed, 0, BED_MAXTEMP - 15);
  #endif

  //
  // Fan Speed:
  //
  #if FAN_COUNT > 0
    #if HAS_FAN0
      #if FAN_COUNT > 1
        #define MSG_1ST_FAN_SPEED MSG_FAN_SPEED " 1"
      #else
        #define MSG_1ST_FAN_SPEED MSG_FAN_SPEED
      #endif
      MENU_MULTIPLIER_ITEM_EDIT(int3, MSG_1ST_FAN_SPEED, &fanSpeeds[0], 0, 255);
    #endif
    #if HAS_FAN1
      //MENU_MULTIPLIER_ITEM_EDIT(int3, MSG_FAN_SPEED " 2", &fanSpeeds[1], 0, 255);
	  MENU_MULTIPLIER_ITEM_EDIT(int3, MSG_FAN_SPEED " 2", &fanValue, 0, 255);
//	  MENU_ITEM_EDIT(int3, MSG_FAN_SPEED " 2", &fanValue, 0, 255);
    #endif
    #if HAS_FAN2
	  //NAMFIX LIGHT ROOM	
      //MENU_MULTIPLIER_ITEM_EDIT(int3, MSG_FAN_SPEED " 3", &fanSpeeds[2], 0, 255);
      MENU_MULTIPLIER_ITEM_EDIT(int3, MSG_LIGHT_ROOM, &fanSpeeds[2], 0, 255);
	  #endif
  #endif // FAN_COUNT > 0

  //
  // Flow:
  // Flow 1:
  // Flow 2:
  // Flow 3:
  // Flow 4:
  //
  #if EXTRUDERS == 1
    MENU_ITEM_EDIT(int3, MSG_FLOW, &extruder_multiplier[0], 10, 999);
  #else // EXTRUDERS > 1
    MENU_ITEM_EDIT(int3, MSG_FLOW, &extruder_multiplier[active_extruder], 10, 999);
    MENU_ITEM_EDIT(int3, MSG_FLOW MSG_N1, &extruder_multiplier[0], 10, 999);
    MENU_ITEM_EDIT(int3, MSG_FLOW MSG_N2, &extruder_multiplier[1], 10, 999);
    #if EXTRUDERS > 2
      MENU_ITEM_EDIT(int3, MSG_FLOW MSG_N3, &extruder_multiplier[2], 10, 999);
      #if EXTRUDERS > 3
        MENU_ITEM_EDIT(int3, MSG_FLOW MSG_N4, &extruder_multiplier[3], 10, 999);
      #endif //EXTRUDERS > 3
    #endif //EXTRUDERS > 2
  #endif //EXTRUDERS > 1

  //
  // Babystep X:
  // Babystep Y:
  // Babystep Z:
  //
  #if ENABLED(BABYSTEPPING)
    #if ENABLED(BABYSTEP_XY)
      MENU_ITEM(submenu, MSG_BABYSTEP_X, lcd_babystep_x);
      MENU_ITEM(submenu, MSG_BABYSTEP_Y, lcd_babystep_y);
    #endif //BABYSTEP_XY
    MENU_ITEM(submenu, MSG_BABYSTEP_Z, lcd_babystep_z);
  #endif

  //
  // Change filament
  //
  #if ENABLED(FILAMENTCHANGEENABLE)
     MENU_ITEM(gcode, MSG_FILAMENTCHANGE, PSTR("M600"));
  #endif

  END_MENU();
}

/**
 *
 * "Prepare" submenu items
 *
 */
void _lcd_preheat(int endnum, const float temph, const float tempb, const int fan) {
  if (temph > 0) setTargetHotend(temph, endnum);
  #if TEMP_SENSOR_BED != 0
    setTargetBed(tempb);
  #else
    UNUSED(tempb);
  #endif
  #if FAN_COUNT > 0
    #if FAN_COUNT > 1
      fanSpeeds[active_extruder < FAN_COUNT ? active_extruder : 0] = fan;
    #else
      fanSpeeds[0] = fan;
    #endif
  #else
    UNUSED(fan);
  #endif
  lcd_return_to_status();
}

#if TEMP_SENSOR_0 != 0
  void lcd_preheat_pla0() { _lcd_preheat(0, plaPreheatHotendTemp, plaPreheatHPBTemp, plaPreheatFanSpeed);}
  void lcd_preheat_abs0() { _lcd_preheat(0, absPreheatHotendTemp, absPreheatHPBTemp, absPreheatFanSpeed);}
#endif

#if EXTRUDERS > 1
  void lcd_preheat_pla1() { _lcd_preheat(1, plaPreheatHotendTemp, plaPreheatHPBTemp, plaPreheatFanSpeed); }
  void lcd_preheat_abs1() { _lcd_preheat(1, absPreheatHotendTemp, absPreheatHPBTemp, absPreheatFanSpeed); }
  #if EXTRUDERS > 2
    void lcd_preheat_pla2() { _lcd_preheat(2, plaPreheatHotendTemp, plaPreheatHPBTemp, plaPreheatFanSpeed); }
    void lcd_preheat_abs2() { _lcd_preheat(2, absPreheatHotendTemp, absPreheatHPBTemp, absPreheatFanSpeed); }
    #if EXTRUDERS > 3
      void lcd_preheat_pla3() { _lcd_preheat(3, plaPreheatHotendTemp, plaPreheatHPBTemp, plaPreheatFanSpeed); }
      void lcd_preheat_abs3() { _lcd_preheat(3, absPreheatHotendTemp, absPreheatHPBTemp, absPreheatFanSpeed); }
    #endif
  #endif

  void lcd_preheat_pla0123() {
    #if EXTRUDERS > 1
      setTargetHotend0(plaPreheatHotendTemp);
      setTargetHotend1(plaPreheatHotendTemp);
      setTargetHotend2(plaPreheatHotendTemp);
    #endif
    _lcd_preheat(EXTRUDERS - 1, plaPreheatHotendTemp, plaPreheatHPBTemp, plaPreheatFanSpeed);
  }
  void lcd_preheat_abs0123() {
    #if EXTRUDERS > 1
      setTargetHotend0(absPreheatHotendTemp);
      setTargetHotend1(absPreheatHotendTemp);
      setTargetHotend2(absPreheatHotendTemp);
    #endif
    _lcd_preheat(EXTRUDERS - 1, absPreheatHotendTemp, absPreheatHPBTemp, absPreheatFanSpeed);
  }

#endif // EXTRUDERS > 1

#if TEMP_SENSOR_BED != 0
  void lcd_preheat_pla_bedonly() { _lcd_preheat(0, 0, plaPreheatHPBTemp, plaPreheatFanSpeed); }
  void lcd_preheat_abs_bedonly() { _lcd_preheat(0, 0, absPreheatHPBTemp, absPreheatFanSpeed); }
#endif

#if TEMP_SENSOR_0 != 0 && (TEMP_SENSOR_1 != 0 || TEMP_SENSOR_2 != 0 || TEMP_SENSOR_3 != 0 || TEMP_SENSOR_BED != 0)

  static void lcd_preheat_pla_menu() {
    START_MENU();
    MENU_ITEM(back, MSG_PREHEAT);
    #if EXTRUDERS == 1
      MENU_ITEM(function, MSG_PREHEAT_PLA, lcd_preheat_pla0);
    #else
      MENU_ITEM(function, MSG_PREHEAT_PLA_N MSG_H1, lcd_preheat_pla0);
      MENU_ITEM(function, MSG_PREHEAT_PLA_N MSG_H2, lcd_preheat_pla1);
      #if EXTRUDERS > 2
        MENU_ITEM(function, MSG_PREHEAT_PLA_N MSG_H3, lcd_preheat_pla2);
        #if EXTRUDERS > 3
          MENU_ITEM(function, MSG_PREHEAT_PLA_N MSG_H4, lcd_preheat_pla3);
        #endif
      #endif
      MENU_ITEM(function, MSG_PREHEAT_PLA_ALL, lcd_preheat_pla0123);
    #endif
    #if TEMP_SENSOR_BED != 0
      MENU_ITEM(function, MSG_PREHEAT_PLA_BEDONLY, lcd_preheat_pla_bedonly);
    #endif
    END_MENU();
  }

  static void lcd_preheat_abs_menu() {
    START_MENU();
    MENU_ITEM(back, MSG_PREHEAT);
    #if EXTRUDERS == 1
      MENU_ITEM(function, MSG_PREHEAT_ABS, lcd_preheat_abs0);
    #else
      MENU_ITEM(function, MSG_PREHEAT_ABS_N MSG_H1, lcd_preheat_abs0);
      MENU_ITEM(function, MSG_PREHEAT_ABS_N MSG_H2, lcd_preheat_abs1);
      #if EXTRUDERS > 2
        MENU_ITEM(function, MSG_PREHEAT_ABS_N MSG_H3, lcd_preheat_abs2);
        #if EXTRUDERS > 3
          MENU_ITEM(function, MSG_PREHEAT_ABS_N MSG_H4, lcd_preheat_abs3);
        #endif
      #endif
      MENU_ITEM(function, MSG_PREHEAT_ABS_ALL, lcd_preheat_abs0123);
    #endif
    #if TEMP_SENSOR_BED != 0
      MENU_ITEM(function, MSG_PREHEAT_ABS_BEDONLY, lcd_preheat_abs_bedonly);
    #endif
    END_MENU();
  }

#endif // TEMP_SENSOR_0 && (TEMP_SENSOR_1 || TEMP_SENSOR_2 || TEMP_SENSOR_3 || TEMP_SENSOR_BED)

void lcd_cooldown() {
  #if FAN_COUNT > 0
    for (uint8_t i = 0; i < FAN_COUNT; i++) fanSpeeds[i] = 0;
  #endif
  disable_all_heaters();
  lcd_return_to_status();
}

#if ENABLED(SDSUPPORT) && ENABLED(MENU_ADDAUTOSTART)

  static void lcd_autostart_sd() {
    card.autostart_index = 0;
    card.setroot();
    card.checkautostart(true);
  }

#endif

#if ENABLED(MANUAL_BED_LEVELING)

  /**
   *
   * "Prepare" > "Bed Leveling" handlers
   *
   */

  static uint8_t _lcd_level_bed_position;

  // Utility to go to the next mesh point
  // A raise is added between points if MIN_Z_HEIGHT_FOR_HOMING is in use
  // Note: During Manual Bed Leveling the homed Z position is MESH_HOME_SEARCH_Z
  // Z position will be restored with the final action, a G28
  inline void _mbl_goto_xy(float x, float y) {
    current_position[Z_AXIS] = MESH_HOME_SEARCH_Z
      #if MIN_Z_HEIGHT_FOR_HOMING > 0
        + MIN_Z_HEIGHT_FOR_HOMING
      #endif
    ;
    line_to_current(Z_AXIS);
    current_position[X_AXIS] = x + home_offset[X_AXIS];
    current_position[Y_AXIS] = y + home_offset[Y_AXIS];
    line_to_current(manual_feedrate[X_AXIS] <= manual_feedrate[Y_AXIS] ? X_AXIS : Y_AXIS);
    #if MIN_Z_HEIGHT_FOR_HOMING > 0
      current_position[Z_AXIS] = MESH_HOME_SEARCH_Z;
      line_to_current(Z_AXIS);
    #endif
    st_synchronize();
    st_synchronize();
  }

  static void _lcd_level_goto_next_point();

  static void _lcd_level_bed_done() {
    if (lcdDrawUpdate) lcd_implementation_drawedit(PSTR(MSG_LEVEL_BED_DONE));
    lcdDrawUpdate =
      #if ENABLED(DOGLCD)
        LCDVIEW_CALL_REDRAW_NEXT
      #else
        LCDVIEW_CALL_NO_REDRAW
      #endif
    ;
  }

  /**
   * Step 7: Get the Z coordinate, then goto next point or exit
   */
  static void _lcd_level_bed_get_z() {
    ENCODER_DIRECTION_NORMAL();

    // Encoder wheel adjusts the Z position
    if (encoderPosition && movesplanned() <= 3) {
      refresh_cmd_timeout();
      current_position[Z_AXIS] += float((int32_t)encoderPosition) * (MBL_Z_STEP);
      NOLESS(current_position[Z_AXIS], 0);
      NOMORE(current_position[Z_AXIS], MESH_HOME_SEARCH_Z * 2);
      line_to_current(Z_AXIS);
      lcdDrawUpdate =
        #if ENABLED(DOGLCD)
          LCDVIEW_CALL_REDRAW_NEXT
        #else
          LCDVIEW_REDRAW_NOW
        #endif
      ;
    }
    encoderPosition = 0;

    static bool debounce_click = false;
    if (LCD_CLICKED) {
      if (!debounce_click) {
        debounce_click = true; // ignore multiple "clicks" in a row
        mbl.set_zigzag_z(_lcd_level_bed_position++, current_position[Z_AXIS]);
        if (_lcd_level_bed_position == (MESH_NUM_X_POINTS) * (MESH_NUM_Y_POINTS)) {
          lcd_goto_menu(_lcd_level_bed_done, true);

          current_position[Z_AXIS] = MESH_HOME_SEARCH_Z
            #if MIN_Z_HEIGHT_FOR_HOMING > 0
              + MIN_Z_HEIGHT_FOR_HOMING
            #endif
          ;
          line_to_current(Z_AXIS);
          st_synchronize();

          mbl.active = true;
          enqueue_and_echo_commands_P(PSTR("G28"));
          lcd_return_to_status();
          //LCD_MESSAGEPGM(MSG_LEVEL_BED_DONE);
          #if HAS_BUZZER
            buzz(200, 659);
            buzz(200, 698);		
          #endif
        }
        else {
          lcd_goto_menu(_lcd_level_goto_next_point, true);
        }
      }
    }
    else {
      debounce_click = false;
    }

    // Update on first display, then only on updates to Z position
    // Show message above on clicks instead
    if (lcdDrawUpdate) {
      float v = current_position[Z_AXIS] - MESH_HOME_SEARCH_Z;
      lcd_implementation_drawedit(PSTR(MSG_MOVE_Z), ftostr43(v + (v < 0 ? -0.0001 : 0.0001), '+'));
    }

  }

  /**
   * Step 6: Display "Next point: 1 / 9" while waiting for move to finish
   */
  static void _lcd_level_bed_moving() {
    if (lcdDrawUpdate) {
      char msg[10];
      sprintf_P(msg, PSTR("%i / %u"), (int)(_lcd_level_bed_position + 1), (MESH_NUM_X_POINTS) * (MESH_NUM_Y_POINTS));
      lcd_implementation_drawedit(PSTR(MSG_LEVEL_BED_NEXT_POINT), msg);
    }

    lcdDrawUpdate =
      #if ENABLED(DOGLCD)
        LCDVIEW_CALL_REDRAW_NEXT
      #else
        LCDVIEW_CALL_NO_REDRAW
      #endif
    ;
  }

  /**
   * Step 5: Initiate a move to the next point
   */
  static void _lcd_level_goto_next_point() {
    // Set the menu to display ahead of blocking call
    lcd_goto_menu(_lcd_level_bed_moving);

    // _mbl_goto_xy runs the menu loop until the move is done
    int ix, iy;
    mbl.zigzag(_lcd_level_bed_position, ix, iy);
    _mbl_goto_xy(mbl.get_x(ix), mbl.get_y(iy));

    // After the blocking function returns, change menus
    lcd_goto_menu(_lcd_level_bed_get_z);
  }

  /**
   * Step 4: Display "Click to Begin", wait for click
   *         Move to the first probe position
   */
  static void _lcd_level_bed_homing_done() {
    if (lcdDrawUpdate) lcd_implementation_drawedit(PSTR(MSG_LEVEL_BED_WAITING));
    if (LCD_CLICKED) {
      _lcd_level_bed_position = 0;
      current_position[Z_AXIS] = MESH_HOME_SEARCH_Z;
      plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
      lcd_goto_menu(_lcd_level_goto_next_point, true);
    }
  }

  /**
   * Step 3: Display "Homing XYZ" - Wait for homing to finish
   */
  static void _lcd_level_bed_homing() {
    if (lcdDrawUpdate) lcd_implementation_drawedit(PSTR(MSG_LEVEL_BED_HOMING), NULL);
    lcdDrawUpdate =
      #if ENABLED(DOGLCD)
        LCDVIEW_CALL_REDRAW_NEXT
      #else
        LCDVIEW_CALL_NO_REDRAW
      #endif
    ;
    if (axis_homed[X_AXIS] && axis_homed[Y_AXIS] && axis_homed[Z_AXIS])
      lcd_goto_menu(_lcd_level_bed_homing_done);
  }

  /**
   * Step 2: Continue Bed Leveling...
   */
  static void _lcd_level_bed_continue() {
    defer_return_to_status = true;
    axis_homed[X_AXIS] = axis_homed[Y_AXIS] = axis_homed[Z_AXIS] = false;
    mbl.reset();
    enqueue_and_echo_commands_P(PSTR("G28")); //Home
    lcd_goto_menu(_lcd_level_bed_homing);
  }

  /**
   * Step 1: MBL entry-point: "Cancel" or "Level Bed"
   */
  static void lcd_level_bed() {
    START_MENU();
    MENU_ITEM(back, MSG_LEVEL_BED_CANCEL);
    MENU_ITEM(submenu, MSG_LEVEL_BED, _lcd_level_bed_continue);
    END_MENU();
  }

#endif  // MANUAL_BED_LEVELING

/**
 *
 * "Prepare" submenu
 *
 */

static void lcd_prepare_menu() {
  START_MENU();

  //
  // ^ Main
  //
  MENU_ITEM(back, MSG_SETTING);

  //
  // Auto Home
  //
  MENU_ITEM(gcode, MSG_AUTO_HOME, PSTR("G28"));
  MENU_ITEM(gcode, MSG_HOME_X, PSTR("G28 X0"));
  MENU_ITEM(gcode, MSG_HOME_Y, PSTR("G28 Y0"));
  MENU_ITEM(gcode, MSG_HOME_Z, PSTR("G28 Z0"));

  //
  // Set Home Offsets
  //
  MENU_ITEM(function, MSG_SET_HOME_OFFSETS, lcd_set_home_offsets);
  //MENU_ITEM(gcode, MSG_SET_ORIGIN, PSTR("G92 X0 Y0 Z0"));

  //
  // Level Bed
  //
  #if ENABLED(AUTO_BED_LEVELING_FEATURE)
    MENU_ITEM(gcode, MSG_LEVEL_BED,
      axis_homed[X_AXIS] && axis_homed[Y_AXIS] ? PSTR("G29") : PSTR("G28\nG29")
    );
  #elif ENABLED(MANUAL_BED_LEVELING)
    MENU_ITEM(submenu, MSG_LEVEL_BED, lcd_level_bed);
  #endif

  //
  // Move Axis
  //
  MENU_ITEM(submenu, MSG_MOVE_AXIS, lcd_move_menu);

  //
  // Disable Steppers
  //
  MENU_ITEM(gcode, MSG_DISABLE_STEPPERS, PSTR("M84"));

  // NAMHIDE
  // Preheat PLA
  // Preheat ABS
  //
  // #if TEMP_SENSOR_0 != 0
    // #if TEMP_SENSOR_1 != 0 || TEMP_SENSOR_2 != 0 || TEMP_SENSOR_3 != 0 || TEMP_SENSOR_BED != 0
      // MENU_ITEM(submenu, MSG_PREHEAT_PLA, lcd_preheat_pla_menu);
      // MENU_ITEM(submenu, MSG_PREHEAT_ABS, lcd_preheat_abs_menu);
    // #else
      // MENU_ITEM(function, MSG_PREHEAT_PLA, lcd_preheat_pla0);
      // MENU_ITEM(function, MSG_PREHEAT_ABS, lcd_preheat_abs0);
    // #endif
  // #endif

  //
  // Cooldown
  //
  MENU_ITEM(function, MSG_COOLDOWN, lcd_cooldown);

  //
  // Switch power on/off
  //
  #if HAS_POWER_SWITCH
    if (powersupply)
      MENU_ITEM(gcode, MSG_SWITCH_PS_OFF, PSTR("M81"));
    else
      MENU_ITEM(gcode, MSG_SWITCH_PS_ON, PSTR("M80"));
  #endif

  //
  // Autostart
  //
  #if ENABLED(SDSUPPORT) && ENABLED(MENU_ADDAUTOSTART)
    MENU_ITEM(function, MSG_AUTOSTART, lcd_autostart_sd);
  #endif

  END_MENU();
}

#if ENABLED(DELTA_CALIBRATION_MENU)

  static void lcd_delta_calibrate_menu() {
    START_MENU();
    MENU_ITEM(back, MSG_MAIN);
    MENU_ITEM(gcode, MSG_AUTO_HOME, PSTR("G28"));
    MENU_ITEM(gcode, MSG_DELTA_CALIBRATE_X, PSTR("G0 F8000 X-77.94 Y-45 Z0"));
    MENU_ITEM(gcode, MSG_DELTA_CALIBRATE_Y, PSTR("G0 F8000 X77.94 Y-45 Z0"));
    MENU_ITEM(gcode, MSG_DELTA_CALIBRATE_Z, PSTR("G0 F8000 X0 Y90 Z0"));
    MENU_ITEM(gcode, MSG_DELTA_CALIBRATE_CENTER, PSTR("G0 F8000 X0 Y0 Z0"));
    END_MENU();
  }

#endif // DELTA_CALIBRATION_MENU

/**
 *
 * "Prepare" > "Move Axis" submenu
 *
 */



/**
 *
 * "Prepare" > "Move Xmm" > "Move XYZ" submenu
 *
 */

#if ENABLED(DELTA) || ENABLED(SCARA)
  #define _MOVE_XYZ_ALLOWED (axis_homed[X_AXIS] && axis_homed[Y_AXIS] && axis_homed[Z_AXIS])
#else
  #define _MOVE_XYZ_ALLOWED true
#endif

static void _lcd_move_menu_axis() {
  START_MENU();
  MENU_ITEM(back, MSG_MOVE_AXIS);

//  if (_MOVE_XYZ_ALLOWED) { //NAMFIX
    MENU_ITEM(submenu, MSG_MOVE_X, lcd_move_x);
    MENU_ITEM(submenu, MSG_MOVE_Y, lcd_move_y);
	MENU_ITEM(submenu, MSG_MOVE_Z, lcd_move_z);//NAMADD
 // }
  if (move_menu_scale < 10.0) {
    //if (_MOVE_XYZ_ALLOWED) MENU_ITEM(submenu, MSG_MOVE_Z, lcd_move_z); //NAMHIDE
    #if EXTRUDERS == 1
      MENU_ITEM(submenu, MSG_MOVE_E, lcd_move_e);
    #else
      MENU_ITEM(submenu, MSG_MOVE_E MSG_MOVE_E1, lcd_move_e0);
      MENU_ITEM(submenu, MSG_MOVE_E MSG_MOVE_E2, lcd_move_e1);
      #if EXTRUDERS > 2
        MENU_ITEM(submenu, MSG_MOVE_E MSG_MOVE_E3, lcd_move_e2);
        #if EXTRUDERS > 3
          MENU_ITEM(submenu, MSG_MOVE_E MSG_MOVE_E4, lcd_move_e3);
        #endif
      #endif
    #endif // EXTRUDERS > 1
  }
  END_MENU();
}

static void lcd_move_menu_10mm() {
  move_menu_scale = 10.0;
  _lcd_move_menu_axis();
}
static void lcd_move_menu_1mm() {
  move_menu_scale = 1.0;
  _lcd_move_menu_axis();
}
static void lcd_move_menu_01mm() {
  move_menu_scale = 0.1;
  _lcd_move_menu_axis();
}

/**
 *
 * "Prepare" > "Move Axis" submenu
 *
 */

static void lcd_move_menu() {
  START_MENU();
  MENU_ITEM(back, MSG_MOTOR);

//  if (_MOVE_XYZ_ALLOWED)
  MENU_ITEM(submenu, MSG_MOVE_10MM, lcd_move_menu_10mm);
  MENU_ITEM(submenu, MSG_MOVE_1MM, lcd_move_menu_1mm);
  MENU_ITEM(submenu, MSG_MOVE_01MM, lcd_move_menu_01mm);
  //TODO:X,Y,Z,E
  END_MENU();
}

/**
 *
 * "Control" submenu
 *
 */

static void lcd_control_menu() {
  START_MENU();
  MENU_ITEM(back, MSG_CONTROL);
  
      // Level Bed
      #if ENABLED(AUTO_BED_LEVELING_FEATURE)
        if (axis_known_position[X_AXIS] && axis_known_position[Y_AXIS])
        MENU_ITEM(gcode, MSG_LEVEL_BED, PSTR("G29"));
      #elif ENABLED(MANUAL_BED_LEVELING)
        MENU_ITEM(submenu, MSG_LEVEL_BED, lcd_level_bed);
      #endif   
  
  MENU_ITEM(submenu, MSG_TEMPERATURE, lcd_control_temperature_menu);
  MENU_ITEM(submenu, MSG_MOTION, lcd_control_motion_menu);
  MENU_ITEM(submenu, MSG_VOLUMETRIC, lcd_control_volumetric_menu);

  #if ENABLED(HAS_LCD_CONTRAST)
    //MENU_ITEM_EDIT(int3, MSG_CONTRAST, &lcd_contrast, 0, 63);
    MENU_ITEM(submenu, MSG_CONTRAST, lcd_set_contrast);
  #endif
  #if ENABLED(FWRETRACT)
    MENU_ITEM(submenu, MSG_RETRACT, lcd_control_retract_menu);
  #endif
  #if ENABLED(EEPROM_SETTINGS)
    MENU_ITEM(function, MSG_STORE_EPROM, Config_StoreSettings);
    MENU_ITEM(function, MSG_LOAD_EPROM, Config_RetrieveSettings);
  #endif
  MENU_ITEM(function, MSG_RESTORE_FAILSAFE, Config_ResetDefault);
  END_MENU();
}

static void lcd_life_time() {
		
	life_time_state=true;	
	lcd_return_to_status();
}

/**
 *
 * "Temperature" submenu
 *
 */

#if ENABLED(PID_AUTOTUNE_MENU)

  #if ENABLED(PIDTEMP)
    int autotune_temp[EXTRUDERS] = { 150 };
    const int heater_maxtemp[EXTRUDERS] = ARRAY_BY_EXTRUDERS(HEATER_0_MAXTEMP, HEATER_1_MAXTEMP, HEATER_2_MAXTEMP, HEATER_3_MAXTEMP);
  #endif

  #if ENABLED(PIDTEMPBED)
    int autotune_temp_bed = 70;
  #endif

  static void _lcd_autotune(int e) {
    char cmd[30];
    sprintf_P(cmd, PSTR("M303 U1 E%i S%i"), e,
      #if HAS_PID_FOR_BOTH
        e < 0 ? autotune_temp_bed : autotune_temp[e]
      #elif ENABLED(PIDTEMPBED)
        autotune_temp_bed
      #else
        autotune_temp[e]
      #endif
    );
    enqueue_and_echo_command(cmd);
  }

#endif //PID_AUTOTUNE_MENU

#if ENABLED(PIDTEMP)

  // Helpers for editing PID Ki & Kd values
  // grab the PID value out of the temp variable; scale it; then update the PID driver
  void copy_and_scalePID_i(int e) {
    #if DISABLED(PID_PARAMS_PER_EXTRUDER)
      UNUSED(e);
    #endif
    PID_PARAM(Ki, e) = scalePID_i(raw_Ki);
    updatePID();
  }
  void copy_and_scalePID_d(int e) {
    #if DISABLED(PID_PARAMS_PER_EXTRUDER)
      UNUSED(e);
    #endif
    PID_PARAM(Kd, e) = scalePID_d(raw_Kd);
    updatePID();
  }
  #define _PIDTEMP_BASE_FUNCTIONS(eindex) \
    void copy_and_scalePID_i_E ## eindex() { copy_and_scalePID_i(eindex); } \
    void copy_and_scalePID_d_E ## eindex() { copy_and_scalePID_d(eindex); }

  #if ENABLED(PID_AUTOTUNE_MENU)
    #define _PIDTEMP_FUNCTIONS(eindex) \
      _PIDTEMP_BASE_FUNCTIONS(eindex); \
      void lcd_autotune_callback_E ## eindex() { _lcd_autotune(eindex); }
  #else
    #define _PIDTEMP_FUNCTIONS(eindex) _PIDTEMP_BASE_FUNCTIONS(eindex)
  #endif

  _PIDTEMP_FUNCTIONS(0);
  #if ENABLED(PID_PARAMS_PER_EXTRUDER)
    #if EXTRUDERS > 1
      _PIDTEMP_FUNCTIONS(1);
      #if EXTRUDERS > 2
        _PIDTEMP_FUNCTIONS(2);
        #if EXTRUDERS > 3
          _PIDTEMP_FUNCTIONS(3);
        #endif //EXTRUDERS > 3
      #endif //EXTRUDERS > 2
    #endif //EXTRUDERS > 1
  #endif //PID_PARAMS_PER_EXTRUDER

#endif //PIDTEMP

/**
 *
 * "Control" > "Temperature" submenu
 *
 */
static void lcd_control_temperature_menu() {
  START_MENU();

  //
  // ^ Control
  //
  MENU_ITEM(back, MSG_CONTROL);

  //
  // Nozzle:
  // Nozzle [1-4]:
  //
  #if EXTRUDERS == 1
    #if TEMP_SENSOR_0 != 0
      MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(int3, MSG_NOZZLE, &target_temperature[0], 0, HEATER_0_MAXTEMP - 15, watch_temp_callback_E0);
    #endif
  #else //EXTRUDERS > 1
    #if TEMP_SENSOR_0 != 0
      MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(int3, MSG_NOZZLE MSG_N1, &target_temperature[0], 0, HEATER_0_MAXTEMP - 15, watch_temp_callback_E0);
    #endif
    #if TEMP_SENSOR_1 != 0
      MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(int3, MSG_NOZZLE MSG_N2, &target_temperature[1], 0, HEATER_1_MAXTEMP - 15, watch_temp_callback_E1);
    #endif
    #if EXTRUDERS > 2
      #if TEMP_SENSOR_2 != 0
        MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(int3, MSG_NOZZLE MSG_N3, &target_temperature[2], 0, HEATER_2_MAXTEMP - 15, watch_temp_callback_E2);
      #endif
      #if EXTRUDERS > 3
        #if TEMP_SENSOR_3 != 0
          MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(int3, MSG_NOZZLE MSG_N4, &target_temperature[3], 0, HEATER_3_MAXTEMP - 15, watch_temp_callback_E3);
        #endif
      #endif // EXTRUDERS > 3
    #endif // EXTRUDERS > 2
  #endif // EXTRUDERS > 1

  //
  // Bed:
  //
  #if TEMP_SENSOR_BED != 0
    MENU_MULTIPLIER_ITEM_EDIT(int3, MSG_BED, &target_temperature_bed, 0, BED_MAXTEMP - 15);
  #endif

  //
  // Fan Speed:
  //
  #if FAN_COUNT > 0
    #if HAS_FAN0
      #if FAN_COUNT > 1
        #define MSG_1ST_FAN_SPEED MSG_FAN_SPEED " 1"
      #else
        #define MSG_1ST_FAN_SPEED MSG_FAN_SPEED
      #endif
      MENU_MULTIPLIER_ITEM_EDIT(int3, MSG_1ST_FAN_SPEED, &fanSpeeds[0], 0, 255);
    #endif
    #if HAS_FAN1
      //MENU_MULTIPLIER_ITEM_EDIT(int3, MSG_FAN_SPEED " 2", &fanSpeeds[1], 0, 255);
	  MENU_MULTIPLIER_ITEM_EDIT(int3, MSG_FAN_SPEED " 2", &fanValue, 0, 255);
	  
    #endif
    #if HAS_FAN2
	  //NAMFIX LIGHT ROOM	
      //MENU_MULTIPLIER_ITEM_EDIT(int3, MSG_FAN_SPEED " 3", &fanSpeeds[2], 0, 255);
      MENU_MULTIPLIER_ITEM_EDIT(int3, MSG_LIGHT_ROOM, &fanSpeeds[2], 0, 255);
    #endif
  #endif // FAN_COUNT > 0

  //
  // Autotemp, Min, Max, Fact
  //
  #if ENABLED(AUTOTEMP) && (TEMP_SENSOR_0 != 0)
    MENU_ITEM_EDIT(bool, MSG_AUTOTEMP, &autotemp_enabled);
    MENU_ITEM_EDIT(float3, MSG_MIN, &autotemp_min, 0, HEATER_0_MAXTEMP - 15);
    MENU_ITEM_EDIT(float3, MSG_MAX, &autotemp_max, 0, HEATER_0_MAXTEMP - 15);
    MENU_ITEM_EDIT(float32, MSG_FACTOR, &autotemp_factor, 0.0, 1.0);
  #endif

  //
  // PID-P, PID-I, PID-D, PID-C, PID Autotune
  // PID-P E1, PID-I E1, PID-D E1, PID-C E1, PID Autotune E1
  // PID-P E2, PID-I E2, PID-D E2, PID-C E2, PID Autotune E2
  // PID-P E3, PID-I E3, PID-D E3, PID-C E3, PID Autotune E3
  // PID-P E4, PID-I E4, PID-D E4, PID-C E4, PID Autotune E4
  //
  #if ENABLED(PIDTEMP)

    #define _PID_BASE_MENU_ITEMS(ELABEL, eindex) \
      raw_Ki = unscalePID_i(PID_PARAM(Ki, eindex)); \
      raw_Kd = unscalePID_d(PID_PARAM(Kd, eindex)); \
      MENU_ITEM_EDIT(float52, MSG_PID_P ELABEL, &PID_PARAM(Kp, eindex), 1, 9990); \
      MENU_ITEM_EDIT_CALLBACK(float52, MSG_PID_I ELABEL, &raw_Ki, 0.01, 9990, copy_and_scalePID_i_E ## eindex); \
      MENU_ITEM_EDIT_CALLBACK(float52, MSG_PID_D ELABEL, &raw_Kd, 1, 9990, copy_and_scalePID_d_E ## eindex)

    #if ENABLED(PID_ADD_EXTRUSION_RATE)
      #define _PID_MENU_ITEMS(ELABEL, eindex) \
        _PID_BASE_MENU_ITEMS(ELABEL, eindex); \
        MENU_ITEM_EDIT(float3, MSG_PID_C ELABEL, &PID_PARAM(Kc, eindex), 1, 9990)
    #else
      #define _PID_MENU_ITEMS(ELABEL, eindex) _PID_BASE_MENU_ITEMS(ELABEL, eindex)
    #endif

    #if ENABLED(PID_AUTOTUNE_MENU)
      #define PID_MENU_ITEMS(ELABEL, eindex) \
        _PID_MENU_ITEMS(ELABEL, eindex); \
        MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(int3, MSG_PID_AUTOTUNE ELABEL, &autotune_temp[eindex], 150, heater_maxtemp[eindex] - 15, lcd_autotune_callback_E ## eindex)
    #else
      #define PID_MENU_ITEMS(ELABEL, eindex) _PID_MENU_ITEMS(ELABEL, eindex)
    #endif

    #if ENABLED(PID_PARAMS_PER_EXTRUDER) && EXTRUDERS > 1
      PID_MENU_ITEMS(MSG_E1, 0);
      PID_MENU_ITEMS(MSG_E2, 1);
      #if EXTRUDERS > 2
        PID_MENU_ITEMS(MSG_E3, 2);
        #if EXTRUDERS > 3
          PID_MENU_ITEMS(MSG_E4, 3);
        #endif //EXTRUDERS > 3
      #endif //EXTRUDERS > 2
    #else //!PID_PARAMS_PER_EXTRUDER || EXTRUDERS == 1
      PID_MENU_ITEMS("", 0);
    #endif //!PID_PARAMS_PER_EXTRUDER || EXTRUDERS == 1

  #endif //PIDTEMP

  //
  // Preheat PLA conf
  //
  MENU_ITEM(submenu, MSG_PREHEAT_PLA_SETTINGS, lcd_control_temperature_preheat_pla_settings_menu);

  //
  // Preheat ABS conf
  //
  MENU_ITEM(submenu, MSG_PREHEAT_ABS_SETTINGS, lcd_control_temperature_preheat_abs_settings_menu);
  END_MENU();
}

/**
 *
 * "Temperature" > "Preheat PLA conf" submenu
 *
 */
static void lcd_control_temperature_preheat_pla_settings_menu() {
  START_MENU();
  MENU_ITEM(back, MSG_TEMPERATURE);
  MENU_ITEM_EDIT(int3, MSG_FAN_SPEED, &plaPreheatFanSpeed, 0, 255);
  #if TEMP_SENSOR_0 != 0
    MENU_ITEM_EDIT(int3, MSG_NOZZLE, &plaPreheatHotendTemp, HEATER_0_MINTEMP, HEATER_0_MAXTEMP - 15);
  #endif
  #if TEMP_SENSOR_BED != 0
    MENU_ITEM_EDIT(int3, MSG_BED, &plaPreheatHPBTemp, BED_MINTEMP, BED_MAXTEMP - 15);
  #endif
  #if ENABLED(EEPROM_SETTINGS)
    MENU_ITEM(function, MSG_STORE_EPROM, Config_StoreSettings);
  #endif
  END_MENU();
}

/**
 *
 * "Temperature" > "Preheat ABS conf" submenu
 *
 */
static void lcd_control_temperature_preheat_abs_settings_menu() {
  START_MENU();
  MENU_ITEM(back, MSG_TEMPERATURE);
  MENU_ITEM_EDIT(int3, MSG_FAN_SPEED, &absPreheatFanSpeed, 0, 255);
  #if TEMP_SENSOR_0 != 0
    MENU_ITEM_EDIT(int3, MSG_NOZZLE, &absPreheatHotendTemp, HEATER_0_MINTEMP, HEATER_0_MAXTEMP - 15);
  #endif
  #if TEMP_SENSOR_BED != 0
    MENU_ITEM_EDIT(int3, MSG_BED, &absPreheatHPBTemp, BED_MINTEMP, BED_MAXTEMP - 15);
  #endif
  #if ENABLED(EEPROM_SETTINGS)
    MENU_ITEM(function, MSG_STORE_EPROM, Config_StoreSettings);
  #endif
  END_MENU();
}

/**
 *
 * "Control" > "Motion" submenu
 *
 */
static void lcd_control_motion_menu() {
  START_MENU();
  MENU_ITEM(back, MSG_CONTROL);
  #if ENABLED(AUTO_BED_LEVELING_FEATURE)
    MENU_ITEM_EDIT(float32, MSG_ZPROBE_ZOFFSET, &zprobe_zoffset, Z_PROBE_OFFSET_RANGE_MIN, Z_PROBE_OFFSET_RANGE_MAX);
  #endif
  // Manual bed leveling, Bed Z:
  #if ENABLED(MANUAL_BED_LEVELING)
    MENU_ITEM_EDIT(float43, MSG_BED_Z, &mbl.z_offset, -1, 1);
  #endif
  MENU_ITEM_EDIT(float5, MSG_ACC, &acceleration, 10, 99000);
  MENU_ITEM_EDIT(float3, MSG_VXY_JERK, &max_xy_jerk, 1, 990);
  #if ENABLED(DELTA)
    MENU_ITEM_EDIT(float3, MSG_VZ_JERK, &max_z_jerk, 1, 990);
  #else
    MENU_ITEM_EDIT(float52, MSG_VZ_JERK, &max_z_jerk, 0.1, 990);
  #endif
  MENU_ITEM_EDIT(float3, MSG_VE_JERK, &max_e_jerk, 1, 990);
  MENU_ITEM_EDIT(float3, MSG_VMAX MSG_X, &max_feedrate[X_AXIS], 1, 999);
  MENU_ITEM_EDIT(float3, MSG_VMAX MSG_Y, &max_feedrate[Y_AXIS], 1, 999);
  MENU_ITEM_EDIT(float3, MSG_VMAX MSG_Z, &max_feedrate[Z_AXIS], 1, 999);
  MENU_ITEM_EDIT(float3, MSG_VMAX MSG_E, &max_feedrate[E_AXIS], 1, 999);
  MENU_ITEM_EDIT(float3, MSG_VMIN, &minimumfeedrate, 0, 999);
  MENU_ITEM_EDIT(float3, MSG_VTRAV_MIN, &mintravelfeedrate, 0, 999);
  MENU_ITEM_EDIT_CALLBACK(long5, MSG_AMAX MSG_X, &max_acceleration_units_per_sq_second[X_AXIS], 100, 99000, reset_acceleration_rates);
  MENU_ITEM_EDIT_CALLBACK(long5, MSG_AMAX MSG_Y, &max_acceleration_units_per_sq_second[Y_AXIS], 100, 99000, reset_acceleration_rates);
  MENU_ITEM_EDIT_CALLBACK(long5, MSG_AMAX MSG_Z, &max_acceleration_units_per_sq_second[Z_AXIS], 10, 99000, reset_acceleration_rates);
  MENU_ITEM_EDIT_CALLBACK(long5, MSG_AMAX MSG_E, &max_acceleration_units_per_sq_second[E_AXIS], 100, 99000, reset_acceleration_rates);
  MENU_ITEM_EDIT(float5, MSG_A_RETRACT, &retract_acceleration, 100, 99000);
  MENU_ITEM_EDIT(float5, MSG_A_TRAVEL, &travel_acceleration, 100, 99000);
  MENU_ITEM_EDIT(float52, MSG_XSTEPS, &axis_steps_per_unit[X_AXIS], 5, 9999);
  MENU_ITEM_EDIT(float52, MSG_YSTEPS, &axis_steps_per_unit[Y_AXIS], 5, 9999);
  #if ENABLED(DELTA)
    MENU_ITEM_EDIT(float52, MSG_ZSTEPS, &axis_steps_per_unit[Z_AXIS], 5, 9999);
  #else
    MENU_ITEM_EDIT(float51, MSG_ZSTEPS, &axis_steps_per_unit[Z_AXIS], 5, 9999);
  #endif
  MENU_ITEM_EDIT(float51, MSG_ESTEPS, &axis_steps_per_unit[E_AXIS], 5, 9999);
  #if ENABLED(ABORT_ON_ENDSTOP_HIT_FEATURE_ENABLED)
    MENU_ITEM_EDIT(bool, MSG_ENDSTOP_ABORT, &abort_on_endstop_hit);
  #endif
  #if ENABLED(SCARA)
    MENU_ITEM_EDIT(float43, MSG_XSCALE, &axis_scaling[X_AXIS], 0.5, 2);
    MENU_ITEM_EDIT(float43, MSG_YSCALE, &axis_scaling[Y_AXIS], 0.5, 2);
  #endif
  END_MENU();
}

/**
 *
 * "Control" > "Filament" submenu
 *
 */
static void lcd_control_volumetric_menu() {
  START_MENU();
  MENU_ITEM(back, MSG_CONTROL);

  MENU_ITEM_EDIT_CALLBACK(bool, MSG_VOLUMETRIC_ENABLED, &volumetric_enabled, calculate_volumetric_multipliers);

  if (volumetric_enabled) {
    #if EXTRUDERS == 1
      MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(float43, MSG_FILAMENT_DIAM, &filament_size[0], 1.5, 3.25, calculate_volumetric_multipliers);
    #else //EXTRUDERS > 1
      MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(float43, MSG_FILAMENT_DIAM MSG_DIAM_E1, &filament_size[0], 1.5, 3.25, calculate_volumetric_multipliers);
      MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(float43, MSG_FILAMENT_DIAM MSG_DIAM_E2, &filament_size[1], 1.5, 3.25, calculate_volumetric_multipliers);
      #if EXTRUDERS > 2
        MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(float43, MSG_FILAMENT_DIAM MSG_DIAM_E3, &filament_size[2], 1.5, 3.25, calculate_volumetric_multipliers);
        #if EXTRUDERS > 3
          MENU_MULTIPLIER_ITEM_EDIT_CALLBACK(float43, MSG_FILAMENT_DIAM MSG_DIAM_E4, &filament_size[3], 1.5, 3.25, calculate_volumetric_multipliers);
        #endif //EXTRUDERS > 3
      #endif //EXTRUDERS > 2
    #endif //EXTRUDERS > 1
  }

  END_MENU();
}

/**
 *
 * "Control" > "Contrast" submenu
 *
 */
#if ENABLED(HAS_LCD_CONTRAST)
  static void lcd_set_contrast() {
    ENCODER_DIRECTION_NORMAL();
    if (encoderPosition) {
      #if ENABLED(U8GLIB_LM6059_AF)
        lcd_contrast += encoderPosition;
        lcd_contrast &= 0xFF;
      #else
        lcd_contrast -= encoderPosition;
        lcd_contrast &= 0x3F;
      #endif
      encoderPosition = 0;
      lcdDrawUpdate = LCDVIEW_REDRAW_NOW;
      u8g.setContrast(lcd_contrast);
    }
    if (lcdDrawUpdate) {
      #if ENABLED(U8GLIB_LM6059_AF)
        lcd_implementation_drawedit(PSTR(MSG_CONTRAST), itostr3(lcd_contrast));
      #else
        lcd_implementation_drawedit(PSTR(MSG_CONTRAST), itostr2(lcd_contrast));
      #endif
    }
    if (LCD_CLICKED) lcd_goto_previous_menu(true);
  }
#endif // HAS_LCD_CONTRAST

/**
 *
 * "Control" > "Retract" submenu
 *
 */
#if ENABLED(FWRETRACT)
  static void lcd_control_retract_menu() {
    START_MENU();
    MENU_ITEM(back, MSG_SETTING);
    MENU_ITEM_EDIT(bool, MSG_AUTORETRACT, &autoretract_enabled);
    MENU_ITEM_EDIT(float52, MSG_CONTROL_RETRACT, &retract_length, 0, 100);
    #if EXTRUDERS > 1
      MENU_ITEM_EDIT(float52, MSG_CONTROL_RETRACT_SWAP, &retract_length_swap, 0, 100);
    #endif
    MENU_ITEM_EDIT(float3, MSG_CONTROL_RETRACTF, &retract_feedrate, 1, 999);
    MENU_ITEM_EDIT(float52, MSG_CONTROL_RETRACT_ZLIFT, &retract_zlift, 0, 999);
    MENU_ITEM_EDIT(float52, MSG_CONTROL_RETRACT_RECOVER, &retract_recover_length, 0, 100);
    #if EXTRUDERS > 1
      MENU_ITEM_EDIT(float52, MSG_CONTROL_RETRACT_RECOVER_SWAP, &retract_recover_length_swap, 0, 100);
    #endif
    MENU_ITEM_EDIT(float3, MSG_CONTROL_RETRACT_RECOVERF, &retract_recover_feedrate, 1, 999);
    END_MENU();
  }
#endif // FWRETRACT

#if ENABLED(SDSUPPORT)

  #if !PIN_EXISTS(SD_DETECT)
    static void lcd_sd_refresh() {
      card.initsd();
      currentMenuViewOffset = 0;
    }
  #endif

  static void lcd_sd_updir() {
    card.updir();
    currentMenuViewOffset = 0;
  }

  /**
   *
   * "Print from SD" submenu
   *
   */
  void lcd_sdcard_menu() {
    ENCODER_DIRECTION_MENUS();
    if (lcdDrawUpdate == 0 && LCD_CLICKED == 0) return; // nothing to do (so don't thrash the SD card)
    uint16_t fileCnt = card.getnrfilenames();
    START_MENU();
    MENU_ITEM(back, MSG_MAIN);
    card.getWorkDirName();
    if (card.filename[0] == '/') {
      #if !PIN_EXISTS(SD_DETECT)
        MENU_ITEM(function, LCD_STR_REFRESH MSG_REFRESH, lcd_sd_refresh);
      #endif
    }
    else {
      MENU_ITEM(function, LCD_STR_FOLDER "..", lcd_sd_updir);
    }

    for (uint16_t i = 0; i < fileCnt; i++) {
      if (_menuItemNr == _lineNr) {
        card.getfilename(
           #if ENABLED(SDCARD_RATHERRECENTFIRST)
             fileCnt-1 -
           #endif
           i
        );

        if (card.filenameIsDir)
          MENU_ITEM(sddirectory, MSG_CARD_MENU, card.filename, card.longFilename);
        else
          MENU_ITEM(sdfile, MSG_CARD_MENU, card.filename, card.longFilename);
	  
		lifetime_stats_print_start();
      }
      else {
        MENU_ITEM_DUMMY();
      }
    }
    END_MENU();
	

	
  }

#endif //SDSUPPORT

/**
 *
 * Functions for editing single values
 *
 * The "menu_edit_type" macro generates the functions needed to edit a numerical value.
 *
 * For example, menu_edit_type(int, int3, itostr3, 1) expands into these functions:
 *
 *   bool _menu_edit_int3();
 *   void menu_edit_int3(); // edit int (interactively)
 *   void menu_edit_callback_int3(); // edit int (interactively) with callback on completion
 *   static void _menu_action_setting_edit_int3(const char* pstr, int* ptr, int minValue, int maxValue);
 *   static void menu_action_setting_edit_int3(const char* pstr, int* ptr, int minValue, int maxValue);
 *   static void menu_action_setting_edit_callback_int3(const char* pstr, int* ptr, int minValue, int maxValue, menuFunc_t callback); // edit int with callback
 *
 * You can then use one of the menu macros to present the edit interface:
 *   MENU_ITEM_EDIT(int3, MSG_SPEED, &feedrate_multiplier, 10, 999)
 *
 * This expands into a more primitive menu item:
 *   MENU_ITEM(setting_edit_int3, MSG_SPEED, PSTR(MSG_SPEED), &feedrate_multiplier, 10, 999)
 *
 *
 * Also: MENU_MULTIPLIER_ITEM_EDIT, MENU_ITEM_EDIT_CALLBACK, and MENU_MULTIPLIER_ITEM_EDIT_CALLBACK
 *
 *       menu_action_setting_edit_int3(PSTR(MSG_SPEED), &feedrate_multiplier, 10, 999)
 */
#define menu_edit_type(_type, _name, _strFunc, scale) \
  bool _menu_edit_ ## _name () { \
    ENCODER_DIRECTION_NORMAL(); \
    bool isClicked = LCD_CLICKED; \
    if ((int32_t)encoderPosition < 0) encoderPosition = 0; \
    if ((int32_t)encoderPosition > maxEditValue) encoderPosition = maxEditValue; \
    if (lcdDrawUpdate) \
      lcd_implementation_drawedit(editLabel, _strFunc(((_type)((int32_t)encoderPosition + minEditValue)) / scale)); \
    if (isClicked) { \
      *((_type*)editValue) = ((_type)((int32_t)encoderPosition + minEditValue)) / scale; \
      lcd_goto_previous_menu(true); \
    } \
    return isClicked; \
  } \
  void menu_edit_ ## _name () { _menu_edit_ ## _name(); } \
  void menu_edit_callback_ ## _name () { if (_menu_edit_ ## _name ()) (*callbackFunc)(); } \
  static void _menu_action_setting_edit_ ## _name (const char* pstr, _type* ptr, _type minValue, _type maxValue) { \
    lcd_save_previous_menu(); \
    \
    lcdDrawUpdate = LCDVIEW_CLEAR_CALL_REDRAW; \
    \
    editLabel = pstr; \
    editValue = ptr; \
    minEditValue = minValue * scale; \
    maxEditValue = maxValue * scale - minEditValue; \
    encoderPosition = (*ptr) * scale - minEditValue; \
  } \
  static void menu_action_setting_edit_ ## _name (const char* pstr, _type* ptr, _type minValue, _type maxValue) { \
    _menu_action_setting_edit_ ## _name(pstr, ptr, minValue, maxValue); \
    currentMenu = menu_edit_ ## _name; \
  }\
  static void menu_action_setting_edit_callback_ ## _name (const char* pstr, _type* ptr, _type minValue, _type maxValue, menuFunc_t callback) { \
    _menu_action_setting_edit_ ## _name(pstr, ptr, minValue, maxValue); \
    currentMenu = menu_edit_callback_ ## _name; \
    callbackFunc = callback; \
  }
menu_edit_type(int, int3, itostr3, 1);
menu_edit_type(float, float3, ftostr3, 1);
menu_edit_type(float, float32, ftostr32, 100);
menu_edit_type(float, float43, ftostr43, 1000);
menu_edit_type(float, float5, ftostr5, 0.01);
menu_edit_type(float, float51, ftostr51, 10);
menu_edit_type(float, float52, ftostr52, 100);
menu_edit_type(unsigned long, long5, ftostr5, 0.01);

/**
 *
 * Handlers for RepRap World Keypad input
 *
 */
#if ENABLED(REPRAPWORLD_KEYPAD)
  static void reprapworld_keypad_move_z_up() {
    encoderPosition = 1;
    move_menu_scale = REPRAPWORLD_KEYPAD_MOVE_STEP;
    lcd_move_z();
  }
  static void reprapworld_keypad_move_z_down() {
    encoderPosition = -1;
    move_menu_scale = REPRAPWORLD_KEYPAD_MOVE_STEP;
    lcd_move_z();
  }
  static void reprapworld_keypad_move_x_left() {
    encoderPosition = -1;
    move_menu_scale = REPRAPWORLD_KEYPAD_MOVE_STEP;
    lcd_move_x();
  }
  static void reprapworld_keypad_move_x_right() {
    encoderPosition = 1;
    move_menu_scale = REPRAPWORLD_KEYPAD_MOVE_STEP;
    lcd_move_x();
  }
  static void reprapworld_keypad_move_y_down() {
    encoderPosition = 1;
    move_menu_scale = REPRAPWORLD_KEYPAD_MOVE_STEP;
    lcd_move_y();
  }
  static void reprapworld_keypad_move_y_up() {
    encoderPosition = -1;
    move_menu_scale = REPRAPWORLD_KEYPAD_MOVE_STEP;
    lcd_move_y();
  }
  static void reprapworld_keypad_move_home() {
    enqueue_and_echo_commands_P(PSTR("G28")); // move all axes home
  }
#endif // REPRAPWORLD_KEYPAD


/**
 *
 * Audio feedback for controller clicks
 *
 */

#if ENABLED(LCD_USE_I2C_BUZZER)
  void lcd_buzz(long duration, uint16_t freq) { // called from buzz() in Marlin_main.cpp where lcd is unknown
    lcd.buzz(duration, freq);
  }
#endif

void lcd_quick_feedback() {
	
  lcdDrawUpdate = LCDVIEW_CLEAR_CALL_REDRAW;
  next_button_update_ms = millis() + 500;

  #if ENABLED(LCD_USE_I2C_BUZZER)
    #ifndef LCD_FEEDBACK_FREQUENCY_HZ
      #define LCD_FEEDBACK_FREQUENCY_HZ 100
    #endif
    #ifndef LCD_FEEDBACK_FREQUENCY_DURATION_MS
      #define LCD_FEEDBACK_FREQUENCY_DURATION_MS (1000/6)
    #endif
    lcd.buzz(LCD_FEEDBACK_FREQUENCY_DURATION_MS, LCD_FEEDBACK_FREQUENCY_HZ);
  #elif PIN_EXISTS(BEEPER)
    #ifndef LCD_FEEDBACK_FREQUENCY_HZ
      #define LCD_FEEDBACK_FREQUENCY_HZ 5000
    #endif
    #ifndef LCD_FEEDBACK_FREQUENCY_DURATION_MS
      #define LCD_FEEDBACK_FREQUENCY_DURATION_MS 2
    #endif
    buzz(LCD_FEEDBACK_FREQUENCY_DURATION_MS, LCD_FEEDBACK_FREQUENCY_HZ);
  #else
    #ifndef LCD_FEEDBACK_FREQUENCY_DURATION_MS
      #define LCD_FEEDBACK_FREQUENCY_DURATION_MS 2
    #endif
    delay(LCD_FEEDBACK_FREQUENCY_DURATION_MS);
  #endif
}

/**
 *
 * Menu actions
 *
 */
static void menu_action_back() { lcd_goto_previous_menu(); }
static void menu_action_submenu(menuFunc_t func) { lcd_save_previous_menu(); lcd_goto_menu(func); }
static void menu_action_gcode(const char* pgcode) { enqueue_and_echo_commands_P(pgcode); }
static void menu_action_function(menuFunc_t func) { (*func)(); }

#if ENABLED(SDSUPPORT)

  static void menu_action_sdfile(const char* filename, char* longFilename) {
    UNUSED(longFilename);
    card.openAndPrintFile(filename);
    lcd_return_to_status();
  }

  static void menu_action_sddirectory(const char* filename, char* longFilename) {
    UNUSED(longFilename);
    card.chdir(filename);
    encoderPosition = 0;
  }

#endif //SDSUPPORT

static void menu_action_setting_edit_bool(const char* pstr, bool* ptr) {UNUSED(pstr); *ptr = !(*ptr); }
static void menu_action_setting_edit_callback_bool(const char* pstr, bool* ptr, menuFunc_t callback) {
  menu_action_setting_edit_bool(pstr, ptr);
  (*callback)();
}

#endif //ULTIPANEL

//NAMADD LED_RING_LOW_V1
#if ENABLED(LED_RING_LOW_V1)
	void led_ring_low_v1_sw_spi_send(unsigned int pinR,unsigned int pinG,unsigned int pinB,unsigned long valR,unsigned long valG,unsigned long valB) {
		 for (uint8_t i = 0; i < 16; i++) {
		   #if F_CPU == 20000000
			__asm__("nop\n\t");
		   #endif
		   digitalWrite(pinR,valR&&0xfffe);
		   digitalWrite(pinG,valG&&0xfffe);
		   digitalWrite(pinB,valB&&0xfffe);     
		   valR>>=1;
		   valG>>=1;
		   valB>>=1;
		   
		   digitalWrite(LR_CLK_PIN,LOW);
		   digitalWrite(LR_CLK_PIN,HIGH);
		   
		   #if F_CPU == 20000000
		   __asm__("nop\n\t""nop\n\t");
		   #endif
		}
		
	   digitalWrite(LR_LATCH_PIN,LOW);
	   digitalWrite(LR_LATCH_PIN,HIGH);		
	}	
		
	//Turn on machine or reset board
	//16x6=RGBYPC
	void led_ring_low_v1_init_1(){		
		unsigned int dataR=0xffff,dataG=0xffff,dataB=0xffff;
	    unsigned long y;
	  
		for (int i=1;i<=6;i++) 
		{		
		  dataR=0xffff;dataG=0xffff;dataB=0xffff;
		  for(int j=2;j<=18;j++)
		  {  
			 y=pow(2,j); 			 
			switch (i){
				case 1:
					dataR=y>65536?0:65536/y;
				break;
				case 2:
					dataG=y>65536?0:65536/y;
				break;
				case 3:
					dataB=y>65536?0:65536/y;
				break;
				case 4:
					dataR=y>65536?0:65536/y;
					dataG=y>65536?0:65536/y;
				break;
				case 5:
					dataR=y>65536?0:65536/y;
					dataB=y>65536?0:65536/y;			
				break;
				case 6:
					dataG=y>65536?0:65536/y;
					dataB=y>65536?0:65536/y;			
				break;			
			} 			 			 
			led_ring_low_v1_sw_spi_send(LR_RED_PIN,LR_GREEN_PIN,LR_BLUE_PIN,dataR,dataG,dataB);	
			delay(20);		
			}
		  led_ring_low_v1_sw_spi_send(LR_RED_PIN,LR_GREEN_PIN,LR_BLUE_PIN,0xffff,0xffff,0xffff);
		}
	}

	//Turn on machine opton 2
	//16 = 4R4G4B4Y
	void led_ring_low_v1_init_2(bool direction){		
		unsigned int dataR=0xffff,dataG=0xffff,dataB=0xffff;
	    unsigned long y;	  		
		dataR=0xffff;dataG=0xffff;dataB=0xffff;
	
		switch (direction)
		{
			case 0:
				for(int j=2;j<=18;j++)
				{  
					 y=pow(2,j); 			 
					switch (j){
						case 1:case 3:case 4:case 5:
							dataR=y>65536?0:65536/y;
						break;
						case 6:case 7:case 8:case 9:
							dataR=0xffff;				
							dataG=y>65536?0:65536/y;
						break;
						case 10:case 11:case 12:case 13:
							dataG=0xffff;
							dataB=y>65536?0:65536/y;
							
						break;
						case 14:case 15:case 16:case 17:
							dataR=y>65536?0:65536/y;
							dataB=y>65536?0:65536/y;
						break;						
					} 			 			 
				led_ring_low_v1_sw_spi_send(LR_RED_PIN,LR_GREEN_PIN,LR_BLUE_PIN,dataR,dataG,dataB);	
				delay(50);		
				}				
			break;
			case 1:
				for(int j=17;j>=1;j--)
				{  
					 y=pow(2,j); 			 
					switch (j){
						case 17:case 16:case 15:case 14:
							dataR=y>65536?0:65536/y;
						break;
						case 13:case 12:case 11:case 10:
							dataR=0xffff;				
							dataG=y>65536?0:65536/y;
						break;
						case 9:case 8:case 7:case 6:
							dataG=0xffff;
							dataB=y>65536?0:65536/y;
							
						break;
						case 5:case 4:case 3:case 2:
							dataR=y>65536?0:65536/y;
							dataB=y>65536?0:65536/y;
						break;						
					} 			 			 
				led_ring_low_v1_sw_spi_send(LR_RED_PIN,LR_GREEN_PIN,LR_BLUE_PIN,dataR,dataG,dataB);	
				delay(100);		
				}
			break;
		}
		led_ring_low_v1_sw_spi_send(LR_RED_PIN,LR_GREEN_PIN,LR_BLUE_PIN,0xffff,0xffff,0xffff);
	}
	
	//Turn on machine or reset board
	void led_ring_low_v1_sdcard_printing(unsigned int color, unsigned int process){		
		unsigned int dataR=0xffff,dataG=0xffff,dataB=0xffff;
	    unsigned long y;
			y=pow(2,process);
			switch (color){
				case 0: //BLACK
					dataR=0xffff;
					dataG=0xffff;
					dataB=0xffff;
				break;				
				case 1: //RED
					dataR=y>65536?0:65536/y;
					dataG=0xffff;
					dataB=0xffff;
				break;
				case 2: //GREEN
					dataR=0xffff;
					dataG=y>65536?0:65536/y;
					dataB=0xffff;					
				break;
				case 3: //BLUE
					dataR=0xffff;
					dataG=0xffff;				
					dataB=y>65536?0:65536/y;
				break;
				case 4: //YELLOW
					dataR=y>65536?0:65536/y;
					dataG=y>65536?0:65536/y;
					dataB=0xffff;
				break;
				case 5: //PINK
					dataR=y>65536?0:65536/y;
					dataG=0xffff;
					dataB=y>65536?0:65536/y;			
				break;
				case 6: //CYAN
					dataR=0xffff;
					dataG=y>65536?0:65536/y;
					dataB=y>65536?0:65536/y;			
				break;	
				case 7: //WHITE
					dataG=y>65536?0:65536/y;
					dataG=y>65536?0:65536/y;
					dataB=y>65536?0:65536/y;					
				break;		
				
				case 14: //YELLOW/RED				
					dataG=y>65536?0:65536/y;
					dataR=0;			
				break;

				case 24: //YELLOW/GREEN
					dataR=y>65536?0:65536/y;
					dataG=0;			
				break;
				
				case 15: //PINK/RED
					dataB=y>65536?0:65536/y;
					dataR=0;			
				break;	
				case 35: //PINK/BLUE
					dataR=y>65536?0:65536/y;
					dataB=0;			
				break;

				case 26: //CYAN/GREEN
					dataB=y>65536?0:65536/y;
					dataG=0;			
				break;
				
				case 36: //CYAN/BLUE
					dataG=y>65536?0:65536/y;
					dataB=0;			
				break;	
				
				default:
					dataB=y>65536?0:65536/y;
				break;
				
			}
			led_ring_low_v1_sw_spi_send(LR_RED_PIN,LR_GREEN_PIN,LR_BLUE_PIN,dataR,dataG,dataB);
	}
	
	//Start print or End Print
	void led_ring_low_v1_blinking(unsigned int color, unsigned int time, unsigned int number){
		unsigned int dataR=0xffff,dataG=0xffff,dataB=0xffff;
		unsigned int dataR1=0xffff,dataG1=0xffff,dataB1=0xffff;
		unsigned long temp_millis;
		
		switch (color){
			case 0: //BLACK
				dataR=0xffff;
				dataG=0xffff;
				dataB=0xffff;
				
				dataR1=0xffff;
				dataG1=0xffff;
				dataB1=0xffff;				
			break;			
			case 1:
				dataR=0;
			break;
			case 2:
				dataG=0;
			break;
			case 3:
				dataB=0;
			break;
			case 4:
				dataR=0;
				dataG=0;
			break;
			case 5:
				dataR=0;
				dataB=0;		
			break;
			case 6:
				dataG=0;
				dataB=0;		
			break;
			case 7:
				dataR=0;
				dataG=0;
				dataB=0;		
			break;
			
			case 12:
				dataR=0;
				dataG1=0;
			break;
			case 13:
				dataR=0;
				dataB1=0;
			break;			
			case 14:
				dataR=0;
				
				dataR1=0;
				dataG1=0;
			break;
			case 15:
				dataR=0;
				
				dataR1=0;
				dataB1=0;
			break;
			case 16:
				dataR=0;
				
				dataG1=0;
				dataB1=0;
			break;
			
			case 23:
				dataG=0;				
				dataB1=0;	
			break;
			case 24:
				dataG=0;
				
				dataR1=0;
				dataG1=0;		
			break;
			case 25:
				dataG=0;
				
				dataR1=0;
				dataB1=0;		
			break;
			case 26:
				dataG=0;
				
				dataG1=0;
				dataB1=0;		
			break;
			
			case 34:
				dataB=0;
				
				dataR1=0;
				dataG1=0;		
			break;
			case 35:
				dataB=0;
				
				dataR1=0;
				dataB1=0;		
			break;
			case 36:
				dataB=0;
				
				dataG1=0;
				dataB1=0;		
			break;
			
			case 45:
				dataR=0;
				dataG=0;
				
				dataR1=0;
				dataB1=0;		
			break;
			case 46:
				dataR=0;
				dataG=0;
				
				dataG1=0;
				dataB1=0;		
			break;

			case 56: //PINK/CYAN
				dataR=0;
				dataB=0;
				
				dataG1=0;
				dataB1=0;		
			break;
			
			default: //BLUE
				dataR=0;
				
				dataB1=0;
			break;			
		}	
					
			temp_millis =(unsigned long)(millis()-led_ring_millis);
			
			if (number)
			{
				if(temp_millis>time)
				{				
					led_ring_millis=millis();					
					led_ring_state=led_ring_state==true?false:true;	
				}
				
				if(led_ring_state==false)
					if((unsigned long)(millis()-led_ring_millis_number)>(unsigned long)(number*time*2))
					{
						led_ring_millis_number=millis();
						led_ring_low_v1_sw_spi_send(LR_RED_PIN,LR_GREEN_PIN,LR_BLUE_PIN,0xffff,0xffff,0xffff);
						led_ring_blink_en=false;
					}				
				
			}
			else
				if(temp_millis>time)
				{	
					led_ring_millis=millis();			
					led_ring_counter++;
					if (led_ring_counter<=100)
						led_ring_state=false;
					else if (led_ring_counter<=104)
						led_ring_state=true;
					else if (led_ring_counter<=108)
						led_ring_state=false;
					else if (led_ring_counter<=112)
						led_ring_state=true;
					else if (led_ring_counter<=116)
						led_ring_state=false;
					else if (led_ring_counter<=120)
						led_ring_state=true;					
					else led_ring_counter=0;
					
				}				
			
		if(time)
		{
			if(led_ring_state)
				led_ring_low_v1_sw_spi_send(LR_RED_PIN,LR_GREEN_PIN,LR_BLUE_PIN,dataR,dataG,dataB);
			else
				led_ring_low_v1_sw_spi_send(LR_RED_PIN,LR_GREEN_PIN,LR_BLUE_PIN,dataR1,dataG1,dataB1);
			
		}
		else
		{
			if (number<=5)
				for(int i=0;i<number;i++)
				{
					led_ring_low_v1_sw_spi_send(LR_RED_PIN,LR_GREEN_PIN,LR_BLUE_PIN,dataR,dataG,dataB);
					delay(100);
					led_ring_low_v1_sw_spi_send(LR_RED_PIN,LR_GREEN_PIN,LR_BLUE_PIN,dataR1,dataG1,dataB1);
					delay(100);
				}
		}
		
	}	
//NAMADD LED_RING_HIGH_V2
#elif ENABLED(LED_RING_HIGH_V2)
	void led_ring_high_v2_sw_spi_send(unsigned int pinR,unsigned int pinG,unsigned int pinB,unsigned long valR,unsigned long valG,unsigned long valB) {
		 for (uint8_t i = 16; i >0; i--) {
		   #if F_CPU == 20000000
			__asm__("nop\n\t");
		   #endif
			digitalWrite(pinR,(valR>>(i-1))&(0x01));// select each bit of data from high to low
			digitalWrite(pinG,(valG>>(i-1))&(0x01));// select each bit of data from high to low
			digitalWrite(pinB,(valB>>(i-1))&(0x01));// select each bit of data from high to low    
		   
		    digitalWrite(LR_CLK_PIN,LOW);
		    digitalWrite(LR_CLK_PIN,HIGH);
		   
		   #if F_CPU == 20000000
		   __asm__("nop\n\t""nop\n\t");
		   #endif
		}
		
	   digitalWrite(LR_LATCH_PIN,LOW);
	   digitalWrite(LR_LATCH_PIN,HIGH);		
	}
	
	//Start print or End Print
	void led_ring_high_v2_blinking(unsigned int color, unsigned int time, unsigned int number){
		unsigned int dataR=0,dataG=0,dataB=0;
		unsigned int dataR1=0,dataG1=0,dataB1=0;
		unsigned long temp_millis;
		
		switch (color){
			case 0: //BLACK
				dataR=0;
				dataG=0;
				dataB=0;
				
				dataR1=0;
				dataG1=0;
				dataB1=0;				
			break;			
			case 1:
				dataR=0xffff;
			break;
			case 2:
				dataG=0xffff;
			break;
			case 3:
				dataB=0xffff;
			break;
			case 4:
				dataR=0xffff;
				dataG=0xffff;
			break;
			case 5:
				dataR=0xffff;
				dataB=0xffff;		
			break;
			case 6:
				dataG=0xffff;
				dataB=0xffff;		
			break;
			case 7:
				dataR=0xffff;
				dataG=0xffff;
				dataB=0xffff;		
			break;
			
			case 12:
				dataR=0xffff;
				dataG1=0xffff;
			break;
			case 13:
				dataR=0xffff;
				dataB1=0xffff;
			break;			
			case 14:
				dataR=0xffff;
				
				dataR1=0xffff;
				dataG1=0xffff;
			break;
			case 15:
				dataR=0xffff;
				
				dataR1=0xffff;
				dataB1=0xffff;
			break;
			case 16:
				dataR=0xffff;
				
				dataG1=0xffff;
				dataB1=0xffff;
			break;
			
			case 23:
				dataG=0xffff;				
				dataB1=0xffff;	
			break;
			case 24:
				dataG=0xffff;
				
				dataR1=0xffff;
				dataG1=0xffff;		
			break;
			case 25:
				dataG=0xffff;
				
				dataR1=0xffff;
				dataB1=0xffff;		
			break;
			case 26:
				dataG=0xffff;
				
				dataG1=0xffff;
				dataB1=0xffff;		
			break;
			
			case 34:
				dataB=0xffff;
				
				dataR1=0xffff;
				dataG1=0xffff;		
			break;
			case 35:
				dataB=0xffff;
				
				dataR1=0xffff;
				dataB1=0xffff;		
			break;
			case 36:
				dataB=0xffff;
				
				dataG1=0xffff;
				dataB1=0xffff;		
			break;
			
			case 45:
				dataR=0xffff;
				dataG=0xffff;
				
				dataR1=0xffff;
				dataB1=0xffff;		
			break;
			case 46:
				dataR=0xffff;
				dataG=0xffff;
				
				dataG1=0xffff;
				dataB1=0xffff;		
			break;

			case 56: //PINK/CYAN
				dataR=0xffff;
				dataB=0xffff;
				
				dataG1=0xffff;
				dataB1=0xffff;		
			break;
			
			default: //BLUE
				dataR=0xffff;
				
				dataB1=0xffff;
			break;			
		}	
					
			temp_millis =(unsigned long)(millis()-led_ring_millis);
			
			if (number)
			{
				if(temp_millis>time)
				{				
					led_ring_millis=millis();					
					led_ring_state=led_ring_state==true?false:true;	
				}
				
				if(led_ring_state==false)
					if((unsigned long)(millis()-led_ring_millis_number)>(unsigned long)(number*time*2))
					{
						led_ring_millis_number=millis();
						led_ring_high_v2_sw_spi_send(LR_RED_PIN,LR_GREEN_PIN,LR_BLUE_PIN,0,0,0);
						led_ring_blink_en=false;
					}				
				
			}
			else
				if(temp_millis>time)
				{	
					led_ring_millis=millis();			
					led_ring_counter++;
					if (led_ring_counter<=100)
						led_ring_state=false;
					else if (led_ring_counter<=104)
						led_ring_state=true;
					else if (led_ring_counter<=108)
						led_ring_state=false;
					else if (led_ring_counter<=112)
						led_ring_state=true;
					else if (led_ring_counter<=116)
						led_ring_state=false;
					else if (led_ring_counter<=120)
						led_ring_state=true;					
					else led_ring_counter=0;
					
				}				
			
		if(time)
		{
			if(led_ring_state)
				led_ring_high_v2_sw_spi_send(LR_RED_PIN,LR_GREEN_PIN,LR_BLUE_PIN,dataR,dataG,dataB);
			else
				led_ring_high_v2_sw_spi_send(LR_RED_PIN,LR_GREEN_PIN,LR_BLUE_PIN,dataR1,dataG1,dataB1);
			
		}
		else
		{
			if (number<=5)
				for(int i=0;i<number;i++)
				{
					led_ring_high_v2_sw_spi_send(LR_RED_PIN,LR_GREEN_PIN,LR_BLUE_PIN,dataR,dataG,dataB);
					delay(100);
					led_ring_high_v2_sw_spi_send(LR_RED_PIN,LR_GREEN_PIN,LR_BLUE_PIN,dataR1,dataG1,dataB1);
					delay(100);
				}
		}
		
	}	

	void led_ring_high_v2_shift_one_led(unsigned int effect, unsigned time_delay){		
        unsigned int dataR,dataG,dataB;
	    unsigned int j;	  		

		switch (effect)
		{
			case 1: //Red
                dataR|=0x0001;   
				for(j=0;j<16;j++)
				{  	                                       	 								 			 
				  led_ring_high_v2_sw_spi_send(LR_RED_PIN,LR_GREEN_PIN,LR_BLUE_PIN,dataR,dataG,dataB);
	              delay(time_delay); 
                  dataR<<=1;		
				}				
			break;
			case 2: //Green
                dataG|=0x0001;   
				for(j=0;j<16;j++)
				{  	                                       	 								 			 
				  led_ring_high_v2_sw_spi_send(LR_RED_PIN,LR_GREEN_PIN,LR_BLUE_PIN,dataR,dataG,dataB);
	              delay(time_delay); 
                  dataG<<=1;		
				}				
			break;
			case 3: //Blue
                dataB|=0x0001;   
				for(j=0;j<16;j++)
				{  	                                       	 								 			 
				  led_ring_high_v2_sw_spi_send(LR_RED_PIN,LR_GREEN_PIN,LR_BLUE_PIN,dataR,dataG,dataB);
	              delay(time_delay); 
                  dataB<<=1;		
				}				
			break;
			case 4: //Yelow
                dataR|=0x0001; 
				dataG|=0x0001;
				for(j=0;j<16;j++)
				{  	                                       	 								 			 
				  led_ring_high_v2_sw_spi_send(LR_RED_PIN,LR_GREEN_PIN,LR_BLUE_PIN,dataR,dataG,dataB);
	              delay(time_delay); 
                  dataR<<=1;
                  dataG<<=1;			  
				}				
			break;
			case 5: //pink
                dataR|=0x0001; 
				dataB|=0x0001;
				for(j=0;j<16;j++)
				{  	                                       	 								 			 
				  led_ring_high_v2_sw_spi_send(LR_RED_PIN,LR_GREEN_PIN,LR_BLUE_PIN,dataR,dataG,dataB);
	              delay(time_delay); 
                  dataR<<=1;
				  dataB<<=1;
				}				
			break;
			case 6: //cyan 
				dataG|=0x0001;
				dataB|=0x0001;
				for(j=0;j<16;j++)
				{  	                                       	 								 			 
				  led_ring_high_v2_sw_spi_send(LR_RED_PIN,LR_GREEN_PIN,LR_BLUE_PIN,dataR,dataG,dataB);
	              delay(time_delay); 
                  dataG<<=1;
				  dataB<<=1;
				}				
			break;			
			case 7: //white
                dataR|=0x0001; 
				dataG|=0x0001;
				dataB|=0x0001;
				for(j=0;j<16;j++)
				{  	                                       	 								 			 
				  led_ring_high_v2_sw_spi_send(LR_RED_PIN,LR_GREEN_PIN,LR_BLUE_PIN,dataR,dataG,dataB);
	              delay(time_delay); 
                  dataR<<=1;
                  dataG<<=1;
				  dataB<<=1;
				}				
			break;			
		}
		led_ring_high_v2_sw_spi_send(LR_RED_PIN,LR_GREEN_PIN,LR_BLUE_PIN,0,0,0);
	}

	void led_ring_high_v2_shift_two_led_opposite(unsigned int effect, unsigned time_delay){		
	    unsigned int j;	
		unsigned int dataR,dataG,dataB;
		
		switch (effect)
		{
			case 1: //Red-Green

				dataR|=0x0001;  
				dataG|=0x8000;
				for(j=0;j<16;j++)
				{  	                                       	 								 			 
				  led_ring_high_v2_sw_spi_send(LR_RED_PIN,LR_GREEN_PIN,LR_BLUE_PIN,dataR,dataG,dataB);
	              delay(time_delay); 
                  dataR<<=1;
				  dataG>>=1;
				}				
			break;
			case 2: //Red-Blue
                dataR|=0x0001;  
				dataB|=0x8000;	
				for(j=0;j<16;j++)
				{  	                                       	 								 			 
				  led_ring_high_v2_sw_spi_send(LR_RED_PIN,LR_GREEN_PIN,LR_BLUE_PIN,dataR,dataG,dataB);
	              delay(time_delay); 
                  dataR<<=1;
				  dataB>>=1;
				}				
			break;
			case 3: //Green-Blue				
                dataG|=0x0001;
				dataB|=0x8000;
				for(j=0;j<16;j++)
				{  	                                       	 								 			 
				  led_ring_high_v2_sw_spi_send(LR_RED_PIN,LR_GREEN_PIN,LR_BLUE_PIN,dataR,dataG,dataB);
	              delay(time_delay); 
                  dataG<<=1;
				  dataB>>=1;
				}				
			break;			
		}
		led_ring_high_v2_sw_spi_send(LR_RED_PIN,LR_GREEN_PIN,LR_BLUE_PIN,0,0,0);
	}

	void led_ring_high_v2_sdcard_printing(unsigned int color, unsigned int process){				
		unsigned int dataR=0,dataG=0,dataB=0;		                        	 								 			 			
		 switch (color){
			case 0: //BLACK
				dataR=0;
				dataG=0;
				dataB=0;
			break;				
			 case 1: //RED				
				for(int i=0;i<process;i++)
					dataR+=pow(2,i)+0.5; 
				dataG=0;
				dataB=0;
			 break;
			case 2: //GREEN
				dataR=0;
				for(int i=0;i<process;i++)
					dataG+=pow(2,i)+0.5; 
				dataB=0;					
			break;
			case 3: //BLUE
				dataR=0;
				dataG=0;				
				for(int i=0;i<process;i++)
					dataB+=pow(2,i)+0.5; 
			break;
			case 4: //YELLOW
				for(int i=0;i<process;i++)
					dataR+=pow(2,i)+0.5; 
				for(int i=0;i<process;i++)
					dataG+=pow(2,i)+0.5; 
				dataB=0;
			break;
			case 5: //PINK
				for(int i=0;i<process;i++)
					dataR+=pow(2,i)+0.5; 
				dataG=0;
				for(int i=0;i<process;i++)
					dataB+=pow(2,i)+0.5;			
			break;
			case 6: //CYAN
				dataR=0;
				for(int i=0;i<process;i++)
					dataG+=pow(2,i)+0.5; 
				for(int i=0;i<process;i++)
					dataB+=pow(2,i)+0.5;			
			break;	
			case 7: //WHITE
				for(int i=0;i<process;i++)
					dataR+=pow(2,i)+0.5; 
				for(int i=0;i<process;i++)
					dataG+=pow(2,i)+0.5; 
				for(int i=0;i<process;i++)
					dataB+=pow(2,i)+0.5;					
			break;		
			
			case 14: //YELLOW/RED				
				for(int i=0;i<process;i++)
					dataG+=pow(2,i)+0.5; 
				dataR=0xffff;			
			break;

			case 24: //YELLOW/GREEN
				for(int i=0;i<process;i++)
					dataR+=pow(2,i)+0.5; 
				dataG=0xffff;			
			break;
			
			case 15: //PINK/RED
				for(int i=0;i<process;i++)
					dataB+=pow(2,i)+0.5;
				dataR=0xffff;			
			break;	
			case 35: //PINK/BLUE
				for(int i=0;i<process;i++)
					dataR+=pow(2,i)+0.5; 
				dataB=0xffff;			
			break;

			case 26: //CYAN/GREEN
				for(int i=0;i<process;i++)
					dataB+=pow(2,i)+0.5;
				dataG=0xffff;			
			break;
			
			case 36: //CYAN/BLUE
				for(int i=0;i<process;i++)
					dataG+=pow(2,i)+0.5; 
				dataB=0xffff;			
			break;	
			
			default:
				for(int i=0;i<process;i++)
					dataB+=pow(2,i)+0.5;
			break;
			
		 }
		led_ring_high_v2_sw_spi_send(LR_RED_PIN,LR_GREEN_PIN,LR_BLUE_PIN,dataR,dataG,dataB);
	}
			
#endif


/** LCD API **/
void lcd_init() {  
  lcd_implementation_init();
  
  #if ENABLED(NEWPANEL)
    #if BUTTON_EXISTS(EN1)
      SET_INPUT(BTN_EN1);
      WRITE(BTN_EN1, HIGH);
    #endif

    #if BUTTON_EXISTS(EN2)
      SET_INPUT(BTN_EN2);
      WRITE(BTN_EN2, HIGH);
    #endif

    #if BUTTON_EXISTS(ENC)
      SET_INPUT(BTN_ENC);
      WRITE(BTN_ENC, HIGH);
    #endif

    #if ENABLED(REPRAPWORLD_KEYPAD)
      pinMode(SHIFT_CLK, OUTPUT);
      pinMode(SHIFT_LD, OUTPUT);
      pinMode(SHIFT_OUT, INPUT);
      WRITE(SHIFT_OUT, HIGH);
      WRITE(SHIFT_LD, HIGH);
    #endif

    #ifdef RIGIDBOT_PANEL
      SET_INPUT(BTN_UP);
      SET_INPUT(BTN_DWN);
      SET_INPUT(BTN_LFT);
      SET_INPUT(BTN_RT);
    #endif

  #else  // Not NEWPANEL

    #if ENABLED(SR_LCD_2W_NL) // Non latching 2 wire shift register
      pinMode(SR_DATA_PIN, OUTPUT);
      pinMode(SR_LR_CLK_PIN, OUTPUT);
    #elif defined(SHIFT_CLK)
      pinMode(SHIFT_CLK, OUTPUT);
      pinMode(SHIFT_LD, OUTPUT);
      pinMode(SHIFT_EN, OUTPUT);
      pinMode(SHIFT_OUT, INPUT);
      WRITE(SHIFT_OUT, HIGH);
      WRITE(SHIFT_LD, HIGH);
      WRITE(SHIFT_EN, LOW);
    #endif // SR_LCD_2W_NL

  #endif//!NEWPANEL

  #if ENABLED(SDSUPPORT) && PIN_EXISTS(SD_DETECT)
    SET_INPUT(SD_DETECT_PIN);
    WRITE(SD_DETECT_PIN, HIGH);
    lcd_sd_status = 2; // UNKNOWN
  #endif

  #if ENABLED(LCD_HAS_SLOW_BUTTONS)
    slow_buttons = 0;
  #endif

  lcd_buttons_update();

  #if ENABLED(ULTIPANEL)
    encoderDiff = 0;
  #endif
}

int lcd_strlen(const char* s) {
  int i = 0, j = 0;
  while (s[i]) {
    if ((s[i] & 0xc0) != 0x80) j++;
    i++;
  }
  return j;
}

int lcd_strlen_P(const char* s) {
  int j = 0;
  while (pgm_read_byte(s)) {
    if ((pgm_read_byte(s) & 0xc0) != 0x80) j++;
    s++;
  }
  return j;
}

bool lcd_blink() {
  static uint8_t blink = 0;
  static millis_t next_blink_ms = 0;
  millis_t ms = millis();
  if (ELAPSED(ms, next_blink_ms)) {
    blink ^= 0xFF;
    next_blink_ms = ms + 1000 - LCD_UPDATE_INTERVAL / 2;
  }
  return blink != 0;
}

//NAMADD LED_RING
#if ENABLED(LED_RING)
	bool led_ring_v1_low_blink(unsigned int time) {
	  static uint8_t blink = 0;
	  static millis_t next_blink_ms = 0;
	  millis_t ms = millis();
	  if (ELAPSED(ms, next_blink_ms)) {
		blink ^= 0xFF;
		next_blink_ms = ms + time;	
	  }
	  return blink != 0;
	}
#endif

static void lcd_filament_load_unload_move_e(){	
	
  if(load_unload_filament_en)
  {	
	if (original_en)//Only update 1 first times
	{
		original_position_e = current_position[E_AXIS];
		original_feedrate_multiplier = feedrate_multiplier;	
		#if EXTRUDERS > 1
			original_active_extruder = active_extruder;			
			target_temperature[1]=target_temperature[0];
			target_temperature[2]=target_temperature[0];
		#endif	  
	
	}
	
	if ((degHotend(0)< degTargetHotend(0)-5))
	{		
		lcd_setstatus("Please Wait Heating...");		
	}	
	else
	{   
		original_en=false; //No update
		
		counter++;
		#if EXTRUDERS > 1
			active_extruder = load_unload_filament_ext;
		#endif	
		feedrate_multiplier=100;		
		if (counter==3200 )
		{
			if(load_unload) //load
			{					
				lcd_setstatus("Filament Loading...");
				current_position[E_AXIS]+=2.4;
				plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS],current_position[E_AXIS], 4, active_extruder);
			}else{
				lcd_setstatus("Filament Unloading...");
				current_position[E_AXIS]-=16;
				plan_buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS],current_position[E_AXIS], 100, active_extruder);	
			}
			
			if(current_position[E_AXIS]>2000 || current_position[E_AXIS]<-2000 )
			{
				enqueuecommands_P(PSTR("G92 E0"));
			}
			counter=0;
			lcdDrawUpdate = LCDVIEW_REDRAW_NOW;
		}							
	}
	if(LCD_CLICKED)
	{
		current_position[E_AXIS]=original_position_e;
		plan_set_e_position(current_position[E_AXIS]);
		#if EXTRUDERS>1
			active_extruder=original_active_extruder;
		#endif		
		feedrate_multiplier=original_feedrate_multiplier;				
	}
  }
}

/**
 * Update the LCD, read encoder buttons, etc.
 *   - Read button states
 *   - Check the SD Card slot state
 *   - Act on RepRap World keypad input
 *   - Update the encoder position
 *   - Apply acceleration to the encoder position
 *   - Set lcdDrawUpdate = LCDVIEW_CALL_REDRAW_NOW on controller events
 *   - Reset the Info Screen timeout if there's any input
 *   - Update status indicators, if any
 *
 *   Run the current LCD menu handler callback function:
 *   - Call the handler only if lcdDrawUpdate != LCDVIEW_NONE
 *   - Before calling the handler, LCDVIEW_CALL_NO_REDRAW => LCDVIEW_NONE
 *   - Call the menu handler. Menu handlers should do the following:
 *     - If a value changes, set lcdDrawUpdate to LCDVIEW_REDRAW_NOW and draw the value
 *       (Encoder events automatically set lcdDrawUpdate for you.)
 *     - if (lcdDrawUpdate) { redraw }
 *     - Before exiting the handler set lcdDrawUpdate to:
 *       - LCDVIEW_CLEAR_CALL_REDRAW to clear screen and set LCDVIEW_CALL_REDRAW_NEXT.
 *       - LCDVIEW_REDRAW_NOW or LCDVIEW_NONE to keep drawingm but only in this loop.
 *       - LCDVIEW_REDRAW_NEXT to keep drawing and draw on the next loop also.
 *       - LCDVIEW_CALL_NO_REDRAW to keep drawing (or start drawing) with no redraw on the next loop.
 *     - NOTE: For graphical displays menu handlers may be called 2 or more times per loop,
 *             so don't change lcdDrawUpdate without considering this.
 *
 *   After the menu handler callback runs (or not):
 *   - Clear the LCD if lcdDrawUpdate == LCDVIEW_CLEAR_CALL_REDRAW
 *   - Update lcdDrawUpdate for the next loop (i.e., move one state down, usually)
 *
 * No worries. This function is only called from the main thread.
 */
void lcd_update() {	

//NAMADD
	lcd_filament_load_unload_move_e();

  #if ENABLED(ULTIPANEL)
    static millis_t return_to_status_ms = 0;
  #endif

  lcd_buttons_update();

  #if ENABLED(SDSUPPORT) && PIN_EXISTS(SD_DETECT)

    bool sd_status = IS_SD_INSERTED;
    if (sd_status != lcd_sd_status && lcd_detected()) {
      lcdDrawUpdate = LCDVIEW_CLEAR_CALL_REDRAW;
      lcd_implementation_init( // to maybe revive the LCD if static electricity killed it.
        #if ENABLED(LCD_PROGRESS_BAR)
          currentMenu == lcd_status_screen
        #endif
      );

      if (sd_status) {
        card.initsd();
        if (lcd_sd_status != 2) LCD_MESSAGEPGM(MSG_SD_INSERTED);
      }
      else {
        card.release();
        if (lcd_sd_status != 2) LCD_MESSAGEPGM(MSG_SD_REMOVED);
      }

      lcd_sd_status = sd_status;
    }

  #endif //SDSUPPORT && SD_DETECT_PIN

  millis_t ms = millis();
  if (ELAPSED(ms, next_lcd_update_ms)) {

    next_lcd_update_ms = ms + LCD_UPDATE_INTERVAL;

    #if ENABLED(LCD_HAS_STATUS_INDICATORS)
      lcd_implementation_update_indicators();
    #endif

    #if ENABLED(LCD_HAS_SLOW_BUTTONS)
      slow_buttons = lcd_implementation_read_slow_buttons(); // buttons which take too long to read in interrupt context
    #endif

    #if ENABLED(ULTIPANEL)

      #if ENABLED(REPRAPWORLD_KEYPAD)

        #if ENABLED(DELTA) || ENABLED(SCARA)
          #define _KEYPAD_MOVE_ALLOWED (axis_homed[X_AXIS] && axis_homed[Y_AXIS] && axis_homed[Z_AXIS])
        #else
          #define _KEYPAD_MOVE_ALLOWED true
        #endif

        if (REPRAPWORLD_KEYPAD_MOVE_HOME)       reprapworld_keypad_move_home();
        if (_KEYPAD_MOVE_ALLOWED) {
          if (REPRAPWORLD_KEYPAD_MOVE_Z_UP)     reprapworld_keypad_move_z_up();
          if (REPRAPWORLD_KEYPAD_MOVE_Z_DOWN)   reprapworld_keypad_move_z_down();
          if (REPRAPWORLD_KEYPAD_MOVE_X_LEFT)   reprapworld_keypad_move_x_left();
          if (REPRAPWORLD_KEYPAD_MOVE_X_RIGHT)  reprapworld_keypad_move_x_right();
          if (REPRAPWORLD_KEYPAD_MOVE_Y_DOWN)   reprapworld_keypad_move_y_down();
          if (REPRAPWORLD_KEYPAD_MOVE_Y_UP)     reprapworld_keypad_move_y_up();
        }
      #endif

      bool encoderPastThreshold = (abs(encoderDiff) >= ENCODER_PULSES_PER_STEP);
      if (encoderPastThreshold || LCD_CLICKED) {
        if (encoderPastThreshold) {
          int32_t encoderMultiplier = 1;

          #if ENABLED(ENCODER_RATE_MULTIPLIER)

            if (encoderRateMultiplierEnabled) {
              int32_t encoderMovementSteps = abs(encoderDiff) / ENCODER_PULSES_PER_STEP;

              if (lastEncoderMovementMillis != 0) {
                // Note that the rate is always calculated between to passes through the
                // loop and that the abs of the encoderDiff value is tracked.
                float encoderStepRate = (float)(encoderMovementSteps) / ((float)(ms - lastEncoderMovementMillis)) * 1000.0;

                if (encoderStepRate >= ENCODER_100X_STEPS_PER_SEC)     encoderMultiplier = 100;
                else if (encoderStepRate >= ENCODER_10X_STEPS_PER_SEC) encoderMultiplier = 10;

                #if ENABLED(ENCODER_RATE_MULTIPLIER_DEBUG)
                  SERIAL_ECHO_START;
                  SERIAL_ECHO("Enc Step Rate: ");
                  SERIAL_ECHO(encoderStepRate);
                  SERIAL_ECHO("  Multiplier: ");
                  SERIAL_ECHO(encoderMultiplier);
                  SERIAL_ECHO("  ENCODER_10X_STEPS_PER_SEC: ");
                  SERIAL_ECHO(ENCODER_10X_STEPS_PER_SEC);
                  SERIAL_ECHO("  ENCODER_100X_STEPS_PER_SEC: ");
                  SERIAL_ECHOLN(ENCODER_100X_STEPS_PER_SEC);
                #endif //ENCODER_RATE_MULTIPLIER_DEBUG
              }

              lastEncoderMovementMillis = ms;
            } // encoderRateMultiplierEnabled
          #endif //ENCODER_RATE_MULTIPLIER

          encoderPosition += (encoderDiff * encoderMultiplier) / ENCODER_PULSES_PER_STEP;
          encoderDiff = 0;
        }
        return_to_status_ms = ms + LCD_TIMEOUT_TO_STATUS;
        lcdDrawUpdate = LCDVIEW_REDRAW_NOW;
      }
    #endif //ULTIPANEL

    // Simply redraw the Info Screen 10 times a second
    if (currentMenu == lcd_status_screen && !(++lcd_status_update_delay % 10))
      lcdDrawUpdate = LCDVIEW_REDRAW_NOW;

    if (lcdDrawUpdate) {

      switch (lcdDrawUpdate) {
        case LCDVIEW_CALL_NO_REDRAW:
          lcdDrawUpdate = LCDVIEW_NONE;
          break;
        case LCDVIEW_CLEAR_CALL_REDRAW: // set by handlers, then altered after (rarely occurs here)
        case LCDVIEW_CALL_REDRAW_NEXT:  // set by handlers, then altered after (never occurs here?)
          lcdDrawUpdate = LCDVIEW_REDRAW_NOW;
        case LCDVIEW_REDRAW_NOW:        // set above, or by a handler through LCDVIEW_CALL_REDRAW_NEXT
        case LCDVIEW_NONE:
          break;
      }

      #if ENABLED(DOGLCD)  // Changes due to different driver architecture of the DOGM display
        static int8_t dot_color = 0;
        dot_color = 1 - dot_color;
        u8g.firstPage();
        do {
          lcd_setFont(FONT_MENU);
          u8g.setPrintPos(125, 0);
          u8g.setColorIndex(dot_color); // Set color for the alive dot
          u8g.drawPixel(127, 63); // draw alive dot
          u8g.setColorIndex(1); // black on white
          (*currentMenu)();
        } while (u8g.nextPage());
      #else
        (*currentMenu)();
      #endif
    }

    #if ENABLED(ULTIPANEL)

      // Return to Status Screen after a timeout
      if (currentMenu == lcd_status_screen || defer_return_to_status)
        return_to_status_ms = ms + LCD_TIMEOUT_TO_STATUS;
      else if (ELAPSED(ms, return_to_status_ms))
        lcd_return_to_status();

    #endif // ULTIPANEL

    switch (lcdDrawUpdate) {
      case LCDVIEW_CLEAR_CALL_REDRAW:
        lcd_implementation_clear();
      case LCDVIEW_CALL_REDRAW_NEXT:
        lcdDrawUpdate = LCDVIEW_REDRAW_NOW;
        break;
      case LCDVIEW_REDRAW_NOW:
        lcdDrawUpdate = LCDVIEW_NONE;
        break;
      case LCDVIEW_NONE:
        break;
    }

  }
}

void lcd_ignore_click(bool b) {
  ignore_click = b;
  wait_for_unclick = false;
}

void lcd_finishstatus(bool persist=false) {
  #if !(ENABLED(LCD_PROGRESS_BAR) && (PROGRESS_MSG_EXPIRE > 0))
    UNUSED(persist);
  #endif

  #if ENABLED(LCD_PROGRESS_BAR)
    progress_bar_ms = millis();
    #if PROGRESS_MSG_EXPIRE > 0
      expire_status_ms = persist ? 0 : progress_bar_ms + PROGRESS_MSG_EXPIRE;
    #endif
  #endif
  lcdDrawUpdate = LCDVIEW_CLEAR_CALL_REDRAW;

  #if ENABLED(FILAMENT_LCD_DISPLAY)
    previous_lcd_status_ms = millis();  //get status message to show up for a while
  #endif
  //NAMADD
  #if ENABLED(SDSUPPORT)
    previous_lcd_status_ms_name_file = millis();
  #endif  
}

#if ENABLED(LCD_PROGRESS_BAR) && PROGRESS_MSG_EXPIRE > 0
  void dontExpireStatus() { expire_status_ms = 0; }
#endif

void set_utf_strlen(char* s, uint8_t n) {
  uint8_t i = 0, j = 0;
  while (s[i] && (j < n)) {
    if ((s[i] & 0xc0u) != 0x80u) j++;
    i++;
  }
  while (j++ < n) s[i++] = ' ';
  s[i] = 0;
}

bool lcd_hasstatus() { return (lcd_status_message[0] != '\0'); }

void lcd_setstatus(const char* message, bool persist) {
  if (lcd_status_message_level > 0) return;
  strncpy(lcd_status_message, message, 3 * (LCD_WIDTH));
  set_utf_strlen(lcd_status_message, LCD_WIDTH);
  lcd_finishstatus(persist);
}

void lcd_setstatuspgm(const char* message, uint8_t level) {
  if (level >= lcd_status_message_level) {
    strncpy_P(lcd_status_message, message, 3 * (LCD_WIDTH));
    set_utf_strlen(lcd_status_message, LCD_WIDTH);
    lcd_status_message_level = level;
    lcd_finishstatus(level > 0);
  }
}

void lcd_setalertstatuspgm(const char* message) {
  lcd_setstatuspgm(message, 1);
  #if ENABLED(ULTIPANEL)
    lcd_return_to_status();
  #endif
}

void lcd_reset_alert_level() { lcd_status_message_level = 0; }

#if ENABLED(HAS_LCD_CONTRAST)
  void lcd_setcontrast(uint8_t value) {
    lcd_contrast = value & 0x3F;
    u8g.setContrast(lcd_contrast);
  }
#endif

#if ENABLED(ULTIPANEL)

  /**
   * Setup Rotary Encoder Bit Values (for two pin encoders to indicate movement)
   * These values are independent of which pins are used for EN_A and EN_B indications
   * The rotary encoder part is also independent to the chipset used for the LCD
   */
  #if defined(EN_A) && defined(EN_B)
    #define encrot0 0
    #define encrot1 2
    #define encrot2 3
    #define encrot3 1
  #endif

  #define GET_BUTTON_STATES(DST) \
    uint8_t new_##DST = 0; \
    WRITE(SHIFT_LD, LOW); \
    WRITE(SHIFT_LD, HIGH); \
    for (int8_t i = 0; i < 8; i++) { \
      new_##DST >>= 1; \
      if (READ(SHIFT_OUT)) SBI(new_##DST, 7); \
      WRITE(SHIFT_CLK, HIGH); \
      WRITE(SHIFT_CLK, LOW); \
    } \
    DST = ~new_##DST; //invert it, because a pressed switch produces a logical 0



//NAMADD Filament check
  /**
   * Read encoder buttons from the hardware registers
   * Warning: This function is called from interrupt context!
   */
void encoder_filment_read_a()    ///NAMADD
{
    //manage encoder rotation  //
    uint8_t enc_filament_a=0;
    if (READ(BTN_EN1_FILAMENT_A)) enc_filament_a |= B01;
    if (READ(BTN_EN2_FILAMENT_A)) enc_filament_a |= B10;
		
    if(enc_filament_a != lastEncoderBits_filament_a)
    {
        switch(enc_filament_a)
        {
        case encrot_filament0_a:
            if(lastEncoderBits_filament_a==encrot_filament3_a)
                encoderDiff_filament_a++;
            else if(lastEncoderBits_filament_a==encrot_filament1_a)
                encoderDiff_filament_a--;
            break;
        case encrot_filament1_a:
            if(lastEncoderBits_filament_a==encrot_filament0_a)
                encoderDiff_filament_a++;
            else if(lastEncoderBits==encrot_filament2_a)
                encoderDiff_filament_a--;
            break;
        case encrot_filament2_a:
            if(lastEncoderBits_filament_a==encrot_filament1_a)
                encoderDiff_filament_a++;
            else if(lastEncoderBits_filament_a==encrot_filament3_a)
                encoderDiff_filament_a--;
            break;
        case encrot_filament3_a:
            if(lastEncoderBits_filament_a==encrot_filament2_a)
                encoderDiff_filament_a++;
            else if(lastEncoderBits_filament_a==encrot_filament0_a)
                encoderDiff_filament_a--;
            break;
        }
    }
    lastEncoderBits_filament_a = enc_filament_a;
	
		if (movesplanned() || IS_SD_PRINTING)
		{
			if(filament_check_enabled_a)
			{
				filament_counter_a++;
				if(filament_counter_a>= filament_check_delay) ///Delay filament monitor theo toc do chay may 
				{
					filament_counter_a=0;
					if(encoderDiff_filament_a == encoderDiff_filament_last_a)
					{
						lcd_sdcard_pause();	
					}			
					encoderDiff_filament_last_a = encoderDiff_filament_a;
				}
			}
		}
		// else
		// {
			// filament_check_enabled_a=false;   //Reset Filament Monitor
		// }	
		
		if(!filament_check_enabled_a)
		{			
			encoderDiff_filament_last_a=10000;
			encoderDiff_filament_a=0;
		}

}

void encoder_filment_read_b()    ///NAMADD
{
    //manage encoder rotation  //
    uint8_t enc_filament_b=0;
    if (READ(BTN_EN1_FILAMENT_B)) enc_filament_b |= B01;
    if (READ(BTN_EN2_FILAMENT_B)) enc_filament_b |= B10;
		
    if(enc_filament_b != lastEncoderBits_filament_b)
    {
        switch(enc_filament_b)
        {
        case encrot_filament0_b:
            if(lastEncoderBits_filament_a==encrot_filament3_b)
                encoderDiff_filament_b++;
            else if(lastEncoderBits_filament_a==encrot_filament1_b)
                encoderDiff_filament_b--;
            break;
        case encrot_filament1_b:
            if(lastEncoderBits_filament_a==encrot_filament0_b)
                encoderDiff_filament_b++;
            else if(lastEncoderBits==encrot_filament2_b)
                encoderDiff_filament_b--;
            break;
        case encrot_filament2_b:
            if(lastEncoderBits_filament_b==encrot_filament1_b)
                encoderDiff_filament_b++;
            else if(lastEncoderBits_filament_b==encrot_filament3_b)
                encoderDiff_filament_b--;
            break;
        case encrot_filament3_b:
            if(lastEncoderBits_filament_b==encrot_filament2_b)
                encoderDiff_filament_b++;
            else if(lastEncoderBits_filament_b==encrot_filament0_b)
                encoderDiff_filament_b--;
            break;
        }
    }
    lastEncoderBits_filament_b = enc_filament_b;
	
		if (movesplanned() || IS_SD_PRINTING)
		{
			if(filament_check_enabled_b)
			{
				filament_counter_b++;
				if(filament_counter_b>= filament_check_delay) ///Delay filament monitor theo toc do chay may 
				{
					filament_counter_b=0;
					if(encoderDiff_filament_b == encoderDiff_filament_last_b)
					{
						lcd_sdcard_pause();	
					}			
					encoderDiff_filament_last_b = encoderDiff_filament_b;
				}
			}
		}
		// else
		// {
			// filament_check_enabled_b=false;   //Reset Filament Monitor
		// }	
		
		if(!filament_check_enabled_b)
		{			
			encoderDiff_filament_last_b=10000;
			encoderDiff_filament_b=0;
		}

}

void encoder_filment_read_c()    ///NAMADD
{
    //manage encoder rotation  //
    uint8_t enc_filament_c=0;
    if (READ(BTN_EN1_FILAMENT_C)) enc_filament_c |= B01;
    if (READ(BTN_EN2_FILAMENT_C)) enc_filament_c |= B10;
		
    if(enc_filament_c != lastEncoderBits_filament_c)
    {
        switch(enc_filament_c)
        {
        case encrot_filament0_c:
            if(lastEncoderBits_filament_a==encrot_filament3_c)
                encoderDiff_filament_c++;
            else if(lastEncoderBits_filament_c==encrot_filament1_c)
                encoderDiff_filament_c--;
            break;
        case encrot_filament1_c:
            if(lastEncoderBits_filament_c==encrot_filament0_c)
                encoderDiff_filament_c++;
            else if(lastEncoderBits==encrot_filament2_c)
                encoderDiff_filament_c--;
            break;
        case encrot_filament2_c:
            if(lastEncoderBits_filament_c==encrot_filament1_c)
                encoderDiff_filament_c++;
            else if(lastEncoderBits_filament_a==encrot_filament3_c)
                encoderDiff_filament_c--;
            break;
        case encrot_filament3_c:
            if(lastEncoderBits_filament_c==encrot_filament2_c)
                encoderDiff_filament_c++;
            else if(lastEncoderBits_filament_c==encrot_filament0_c)
                encoderDiff_filament_c--;
            break;
        }
    }
    lastEncoderBits_filament_c = enc_filament_c;
	
		if (movesplanned() || IS_SD_PRINTING)
		{
			if(filament_check_enabled_c)
			{
				filament_counter_c++;
				if(filament_counter_c>= filament_check_delay) ///Delay filament monitor theo toc do chay may 
				{
					filament_counter_c=0;
					if(encoderDiff_filament_c == encoderDiff_filament_last_c)
					{
						lcd_sdcard_pause();	
					}			
					encoderDiff_filament_last_c = encoderDiff_filament_c;
				}
			}
		}
		// else
		// {
			// filament_check_enabled_c=false;   //Reset Filament Monitor
		// }	
		
		if(!filament_check_enabled_c)
		{			
			encoderDiff_filament_last_c=10000;
			encoderDiff_filament_c=0;
		}

}

  /**
   * Read encoder buttons from the hardware registers
   * Warning: This function is called from interrupt context!
   */
  void lcd_buttons_update() {	
	//NAMADD Filament check
	encoder_filment_read_a();  ///NAMADD TEST 
	encoder_filment_read_b();
	encoder_filment_read_c();
 
    #if ENABLED(NEWPANEL)
      uint8_t newbutton = 0;
      #if BUTTON_EXISTS(EN1)
        if (BUTTON_PRESSED(EN1)) newbutton |= EN_A;
      #endif
      #if BUTTON_EXISTS(EN2)
        if (BUTTON_PRESSED(EN2)) newbutton |= EN_B;
      #endif
      #if ENABLED(RIGIDBOT_PANEL) || BUTTON_EXISTS(ENC)
        millis_t now = millis();
      #endif
      #if ENABLED(RIGIDBOT_PANEL)
        if (ELAPSED(now, next_button_update_ms)) {
          if (BUTTON_PRESSED(UP)) {
            encoderDiff = -(ENCODER_STEPS_PER_MENU_ITEM);
            next_button_update_ms = now + 300;
          }
          else if (BUTTON_PRESSED(DWN)) {
            encoderDiff = ENCODER_STEPS_PER_MENU_ITEM;
            next_button_update_ms = now + 300;
          }
          else if (BUTTON_PRESSED(LFT)) {
            encoderDiff = -(ENCODER_PULSES_PER_STEP);
            next_button_update_ms = now + 300;
          }
          else if (BUTTON_PRESSED(RT)) {
            encoderDiff = ENCODER_PULSES_PER_STEP;
            next_button_update_ms = now + 300;
          }
        }
      #endif
      #if BUTTON_EXISTS(ENC)
        if (ELAPSED(now, next_button_update_ms) && BUTTON_PRESSED(ENC)) newbutton |= EN_C;
      #endif
      buttons = newbutton;
      #if ENABLED(LCD_HAS_SLOW_BUTTONS)
        buttons |= slow_buttons;
      #endif
      #if ENABLED(REPRAPWORLD_KEYPAD)
        GET_BUTTON_STATES(buttons_reprapworld_keypad);
      #endif
    #else
      GET_BUTTON_STATES(buttons);
    #endif //!NEWPANEL

    #if ENABLED(REVERSE_MENU_DIRECTION)
      #define ENCODER_DIFF_CW  (encoderDiff += encoderDirection)
      #define ENCODER_DIFF_CCW (encoderDiff -= encoderDirection)
    #else
      #define ENCODER_DIFF_CW  (encoderDiff++)
      #define ENCODER_DIFF_CCW (encoderDiff--)
    #endif
    #define ENCODER_SPIN(_E1, _E2) switch (lastEncoderBits) { case _E1: ENCODER_DIFF_CW; break; case _E2: ENCODER_DIFF_CCW; }

    //manage encoder rotation
    uint8_t enc = 0;
    if (buttons & EN_A) enc |= B01;
    if (buttons & EN_B) enc |= B10;
    if (enc != lastEncoderBits) {
      switch (enc) {
        case encrot0: ENCODER_SPIN(encrot3, encrot1); break;
        case encrot1: ENCODER_SPIN(encrot0, encrot2); break;
        case encrot2: ENCODER_SPIN(encrot1, encrot3); break;
        case encrot3: ENCODER_SPIN(encrot2, encrot0); break;
      }
    }
    lastEncoderBits = enc;
  }

  bool lcd_detected(void) {
    #if (ENABLED(LCD_I2C_TYPE_MCP23017) || ENABLED(LCD_I2C_TYPE_MCP23008)) && ENABLED(DETECT_DEVICE)
      return lcd.LcdDetected() == 1;
    #else
      return true;
    #endif
  }

  bool lcd_clicked() { return LCD_CLICKED; }

#endif // ULTIPANEL

/*********************************/
/** Number to string conversion **/
/*********************************/
/*********************************/
char conv[10];

// Convert float to rj string with 123 or -12 format
char *ftostr3(const float& x) { return itostr3((int)x); }

// Convert float to rj string with _123, -123, _-12, or __-1 format
char *ftostr4sign(const float& x) { return itostr4sign((int)x); }

// Convert unsigned int to string with 12 format
char* itostr2(const uint8_t& x) {
  //sprintf(conv,"%5.1f",x);
  int xx = x;
  conv[0] = (xx / 10) % 10 + '0';
  conv[1] = xx % 10 + '0';
  conv[2] = 0;
  return conv;
}

//NAMADD
// Convert float to string with +1234.5 / -1234.5 format
char* ftostr311(const float& x) {
  int xx = abs(x * 10);
  conv[0] = (x >= 0) ? '+' : '-';
  conv[1] = (xx / 10000) % 10 + '0';
  conv[2] = (xx / 1000) % 10 + '0';
  conv[3] = (xx / 100) % 10 + '0';
  conv[4] = (xx / 10) % 10 + '0';
  conv[5] = '.';
  conv[6] = xx % 10 + '0';
  conv[7] = 0;
  return conv;
}

// Convert float to string with +123.4 / -123.4 format
char* ftostr31(const float& x) {
  int xx = abs(x * 10);
  conv[0] = (x >= 0) ? '+' : '-';
  conv[1] = (xx / 1000) % 10 + '0';
  conv[2] = (xx / 100) % 10 + '0';
  conv[3] = (xx / 10) % 10 + '0';
  conv[4] = '.';
  conv[5] = xx % 10 + '0';
  conv[6] = 0;
  return conv;
}

// Convert unsigned float to string with 123.4 format, dropping sign
char* ftostr31ns(const float& x) {
  int xx = abs(x * 10);
  conv[0] = (xx / 1000) % 10 + '0';
  conv[1] = (xx / 100) % 10 + '0';
  conv[2] = (xx / 10) % 10 + '0';
  conv[3] = '.';
  conv[4] = xx % 10 + '0';
  conv[5] = 0;
  return conv;
}

// Convert signed float to string with 023.45 / -23.45 format
char *ftostr32(const float& x) {
  long xx = abs(x * 100);
  conv[0] = x >= 0 ? (xx / 10000) % 10 + '0' : '-';
  conv[1] = (xx / 1000) % 10 + '0';
  conv[2] = (xx / 100) % 10 + '0';
  conv[3] = '.';
  conv[4] = (xx / 10) % 10 + '0';
  conv[5] = xx % 10 + '0';
  conv[6] = 0;
  return conv;
}

// Convert signed float to string (6 digit) with -1.234 / _0.000 / +1.234 format
char* ftostr43(const float& x, char plus/*=' '*/) {
  long xx = x * 1000;
  if (xx == 0)
    conv[0] = ' ';
  else if (xx > 0)
    conv[0] = plus;
  else {
    xx = -xx;
    conv[0] = '-';
  }
  conv[1] = (xx / 1000) % 10 + '0';
  conv[2] = '.';
  conv[3] = (xx / 100) % 10 + '0';
  conv[4] = (xx / 10) % 10 + '0';
  conv[5] = (xx) % 10 + '0';
  conv[6] = 0;
  return conv;
}

// Convert unsigned float to string with 1.23 format
char* ftostr12ns(const float& x) {
  long xx = x * 100;
  xx = abs(xx);
  conv[0] = (xx / 100) % 10 + '0';
  conv[1] = '.';
  conv[2] = (xx / 10) % 10 + '0';
  conv[3] = (xx) % 10 + '0';
  conv[4] = 0;
  return conv;
}

// Convert signed float to space-padded string with -_23.4_ format
char* ftostr32sp(const float& x) {
  long xx = x * 100;
  uint8_t dig;
  if (xx < 0) { // negative val = -_0
    xx = -xx;
    conv[0] = '-';
    dig = (xx / 1000) % 10;
    conv[1] = dig ? '0' + dig : ' ';
  }
  else { // positive val = __0
    dig = (xx / 10000) % 10;
    if (dig) {
      conv[0] = '0' + dig;
      conv[1] = '0' + (xx / 1000) % 10;
    }
    else {
      conv[0] = ' ';
      dig = (xx / 1000) % 10;
      conv[1] = dig ? '0' + dig : ' ';
    }
  }

  conv[2] = '0' + (xx / 100) % 10; // lsd always

  dig = xx % 10;
  if (dig) { // 2 decimal places
    conv[5] = '0' + dig;
    conv[4] = '0' + (xx / 10) % 10;
    conv[3] = '.';
  }
  else { // 1 or 0 decimal place
    dig = (xx / 10) % 10;
    if (dig) {
      conv[4] = '0' + dig;
      conv[3] = '.';
    }
    else {
      conv[3] = conv[4] = ' ';
    }
    conv[5] = ' ';
  }
  conv[6] = '\0';
  return conv;
}

// Convert signed int to lj string with +012 / -012 format
char* itostr3sign(const int& x) {
  int xx;
  if (x >= 0) {
    conv[0] = '+';
    xx = x;
  }
  else {
    conv[0] = '-';
    xx = -x;
  }
  conv[1] = (xx / 100) % 10 + '0';
  conv[2] = (xx / 10) % 10 + '0';
  conv[3] = xx % 10 + '0';
  conv[4] = '.';
  conv[5] = '0';
  conv[6] = 0;
  return conv;
}

// Convert signed int to rj string with 123 or -12 format
char* itostr3(const int& x) {
  int xx = x;
  if (xx < 0) {
    conv[0] = '-';
    xx = -xx;
  }
  else
    conv[0] = xx >= 100 ? (xx / 100) % 10 + '0' : ' ';

  conv[1] = xx >= 10 ? (xx / 10) % 10 + '0' : ' ';
  conv[2] = xx % 10 + '0';
  conv[3] = 0;
  return conv;
}

// Convert unsigned int to lj string with 123 format
char* itostr3left(const int& x) {
  if (x >= 100) {
    conv[0] = (x / 100) % 10 + '0';
    conv[1] = (x / 10) % 10 + '0';
    conv[2] = x % 10 + '0';
    conv[3] = 0;
  }
  else if (x >= 10) {
    conv[0] = (x / 10) % 10 + '0';
    conv[1] = x % 10 + '0';
    conv[2] = 0;
  }
  else {
    conv[0] = x % 10 + '0';
    conv[1] = 0;
  }
  return conv;
}

// Convert unsigned int to rj string with 1234 format
char* itostr4(const int& x) {
  conv[0] = x >= 1000 ? (x / 1000) % 10 + '0' : ' ';
  conv[1] = x >= 100 ? (x / 100) % 10 + '0' : ' ';
  conv[2] = x >= 10 ? (x / 10) % 10 + '0' : ' ';
  conv[3] = x % 10 + '0';
  conv[4] = 0;
  return conv;
}

// Convert unsigned int to rj string with 12345678 format
char* itostr9(const int& x) {

  conv[0] = x >= 10000000 ? (x / 10000000) % 10 + '0' : ' ';
  conv[1] = x >= 1000000 ? (x / 1000000) % 10 + '0' : ' ';
  conv[2] = x >= 100000 ? (x / 100000) % 10 + '0' : ' ';	
  conv[3] = x >= 10000 ? (x / 10000) % 10 + '0' : ' ';  
  conv[4] = x >= 1000 ? (x / 1000) % 10 + '0' : ' ';
  conv[5] = x >= 100 ? (x / 100) % 10 + '0' : ' ';
  conv[6] = x >= 10 ? (x / 10) % 10 + '0' : ' ';
  conv[7] = x % 10 + '0';
  conv[8] = 0;
  return conv;
}

// Convert signed int to rj string with _123, -123, _-12, or __-1 format
char *itostr4sign(const int& x) {
  int xx = abs(x);
  int sign = 0;
  if (xx >= 100) {
    conv[1] = (xx / 100) % 10 + '0';
    conv[2] = (xx / 10) % 10 + '0';
  }
  else if (xx >= 10) {
    conv[0] = ' ';
    sign = 1;
    conv[2] = (xx / 10) % 10 + '0';
  }
  else {
    conv[0] = ' ';
    conv[1] = ' ';
    sign = 2;
  }
  conv[sign] = x < 0 ? '-' : ' ';
  conv[3] = xx % 10 + '0';
  conv[4] = 0;
  return conv;
}

// Convert unsigned float to rj string with 12345 format
char* ftostr5(const float& x) {
  long xx = abs(x);
  conv[0] = xx >= 10000 ? (xx / 10000) % 10 + '0' : ' ';
  conv[1] = xx >= 1000 ? (xx / 1000) % 10 + '0' : ' ';
  conv[2] = xx >= 100 ? (xx / 100) % 10 + '0' : ' ';
  conv[3] = xx >= 10 ? (xx / 10) % 10 + '0' : ' ';
  conv[4] = xx % 10 + '0';
  conv[5] = 0;
  return conv;
}

// Convert signed float to string with +1234.5 format
char* ftostr51(const float& x) {
  long xx = abs(x * 10);
  conv[0] = (x >= 0) ? '+' : '-';
  conv[1] = (xx / 10000) % 10 + '0';
  conv[2] = (xx / 1000) % 10 + '0';
  conv[3] = (xx / 100) % 10 + '0';
  conv[4] = (xx / 10) % 10 + '0';
  conv[5] = '.';
  conv[6] = xx % 10 + '0';
  conv[7] = 0;
  return conv;
}

// Convert signed float to string with +123.45 format
char* ftostr52(const float& x) {
  conv[0] = (x >= 0) ? '+' : '-';
  long xx = abs(x * 100);
  conv[1] = (xx / 10000) % 10 + '0';
  conv[2] = (xx / 1000) % 10 + '0';
  conv[3] = (xx / 100) % 10 + '0';
  conv[4] = '.';
  conv[5] = (xx / 10) % 10 + '0';
  conv[6] = xx % 10 + '0';
  conv[7] = 0;
  return conv;
}

#endif // ULTRA_LCD
