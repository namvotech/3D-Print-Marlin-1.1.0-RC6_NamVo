/**
 *
 * Applies to the following boards:
 * 3D Factory Boards EFB (Extruder, Fan, Bed)
 * Other pins_MYBOARD.h files may override these defaults
 *
 */
#define IS_RAMPS_EFB

#if !defined(__AVR_ATmega1280__) && !defined(__AVR_ATmega2560__)
  #error Oops!  Make sure you have 'Arduino Mega' selected from the 'Tools -> Boards' menu.
#endif

#define LARGE_FLASH 	true

#if ENABLED(GALVO_LASER)
	#define SERVO0_PIN          -1
#else	
	#define SERVO0_PIN          3
#endif

#define SERVO1_PIN          13
#define SERVO2_PIN          5
#define SERVO3_PIN          4

#if ENABLED(GALVO_LASER)
	#define X_STEP_PIN         1
	#define X_DIR_PIN          1
	#define X_ENABLE_PIN       1
	#define X_MIN_PIN          1
	#define X_MAX_PIN          1

	#define Y_STEP_PIN         1
	#define Y_DIR_PIN          1
	#define Y_ENABLE_PIN       1
	#define Y_MIN_PIN          1
	#define Y_MAX_PIN          1
#else
	#define X_STEP_PIN         59
	#define X_DIR_PIN          60
	#define X_ENABLE_PIN       58
	#define X_MIN_PIN          37
	#define X_MAX_PIN          40

	#define Y_STEP_PIN         6
	#define Y_DIR_PIN          17
	#define Y_ENABLE_PIN       16
	#define Y_MIN_PIN          41
	#define Y_MAX_PIN          38
#endif

#define Z_STEP_PIN         2
#define Z_DIR_PIN          23
#define Z_ENABLE_PIN       22
#define Z_MIN_PIN          18
#define Z_MAX_PIN          19

#define E0_STEP_PIN        5
#define E0_DIR_PIN         29
#define E0_ENABLE_PIN      24

#define E1_STEP_PIN        4
#define E1_DIR_PIN         14
#define E1_ENABLE_PIN      39

#define E2_STEP_PIN        47
#define E2_DIR_PIN         45
#define E2_ENABLE_PIN      69 //A15

#define SDPOWER            -1
#define SDSS               53
#define LED_PIN            13


#if ENABLED(FILAMENT_WIDTH_SENSOR)  // FMM added for Filament Extruder
  // define analog pin for the filament width sensor input
  // Use the RAMPS 1.4 Analog input 5 on the AUX2 connector
  #define FILWIDTH_PIN      5
#endif

#if ENABLED(Z_MIN_PROBE_ENDSTOP)
  // Define a pin to use as the signal pin on Arduino for the Z_PROBE endstop.
  #define Z_MIN_PROBE_PIN  32
#endif

#if ENABLED(FILAMENT_RUNOUT_SENSOR)
  // define digital pin 4 for the filament runout sensor. Use the RAMPS 1.4 digital input 4 on the servos connector
  #define FILRUNOUT_PIN     4
#endif

#define CONTROLLERFAN_PIN  -1 // Pin used for the fan to cool controller
#define FAN_PIN            7
#define FAN1_PIN		   8

#define PS_ON_PIN          12

#if ENABLED(REPRAP_DISCOUNT_SMART_CONTROLLER) || ENABLED(G3D_PANEL)
  #define KILL_PIN         -1
#else
  #define KILL_PIN         -1
#endif

#define HEATER_0_PIN     	9   // EXTRUDER 1
#define HEATER_1_PIN       10   // EXTRUDER 2 (FAN On Sprinter)
#define HEATER_2_PIN       10

#define TEMP_0_PIN         11   // ANALOG NUMBERING
#define TEMP_1_PIN         11   // ANALOG NUMBERING
#define TEMP_2_PIN         11   // ANALOG NUMBERING

#if ENABLED(SLA_PRINTER)
	#define HEATER_BED_PIN     -1    
	#define TEMP_BED_PIN       -1  
#else
	#define HEATER_BED_PIN     11    // BED
	#define TEMP_BED_PIN       12   // ANALOG NUMBERING
#endif

#if ENABLED(Z_PROBE_SLED)
  #define SLED_PIN           -1
#endif

#if ENABLED(GALVO_LASER)
	#if ENABLED (GALVO_LASER_SPI)
		#define GALVO_CS_PIN 		44	// SPI Galvo XY
		#define GALVO_LR_CLK_PIN		43		
		#define GALVO_DAT_PIN		42
	#endif
	
	#define LASER_PIN         		13	// LASER	
	
	#if ENABLED(SLS_PRINTER)	
		#define S_STEP_PIN         	6 	// trượt            //Y_STEP_PIN
		#define S_DIR_PIN      		17 	// trượt chiều	    //Y_DIR_PIN
		#define S_ENABLE_PIN 		16						//Y_ENABLE_PIN
		#define S_ENDSTOP_MIN_PIN   41	// endstop left		//Y_MIN_PIN
		#define S_ENDSTOP_MAX_PIN   38	// endstop right	//Y_MAX_PIN
		
		#define R_STEP_PIN       	59	// lăn				//X_STEP_PIN
		#define R_DIR_PIN			60	// lăn chiều		//X_DIR_PIN
		#define R_ENABLE_PIN		58						//X_ENABLE_PIN
	#endif
#endif

//NAMADD LED_RING_LOW_V1
#if ENABLED(LED_RING_LOW_V1)	
	#define LR_RST_PIN 26
	#define LR_CLK_PIN 28
	#define LR_LATCH_PIN 34
	
	//// For led pin next RGB
	//// #define LR_RED_PIN 33
	//// #define LR_GREEN_PIN 27
	//// #define LR_BLUE_PIN 25
	
	// For led pin next BRG
	#define LR_BLUE_PIN 33
	#define LR_RED_PIN 27
	#define LR_GREEN_PIN 25		

//NAMADD LED_RING_HIGH_V2
#elif ENABLED(LED_RING_HIGH_V2)	
	
	#define LR_RST_PIN 26
	#define LR_CLK_PIN 28
	#define LR_LATCH_PIN 33
	
	//For led pin next BRG
	#define LR_BLUE_PIN 25
	#define LR_RED_PIN 27
	#define LR_GREEN_PIN 34	
#endif
	

#if ENABLED(ULTRA_LCD)
	#if ENABLED(NEWPANEL)
		#if ENABLED(PANEL_ONE)
		  #define LCD_PINS_RS 		40
		  #define LCD_PINS_ENABLE 	42
		  #define LCD_PINS_D4 		65
		  #define LCD_PINS_D5 		66
		  #define LCD_PINS_D6 		44
		  #define LCD_PINS_D7 		64
		#else
		  #define LCD_PINS_RS 16
		  #define LCD_PINS_ENABLE 	1
		  #define LCD_PINS_D4 		23
		  #define LCD_PINS_D5 		25
		  #define LCD_PINS_D6 		27
		  #define LCD_PINS_D7 		29
		#endif

		#if ENABLED(REPRAP_DISCOUNT_SMART_CONTROLLER)
			
		  #if ENABLED(SLA_PRINTER)
			#define BEEPER_PIN 32
		  #else
			#define BEEPER_PIN 32  
		  #endif

		  #define BTN_EN1 30
		  #define BTN_EN2 15
		  #define BTN_ENC 31

		  #define SD_DETECT_PIN 49
		  
		  #define LCD_PINS_RS 35
		  #define LCD_PINS_ENABLE 48
		  #define LCD_PINS_D4 46
		  #define LCD_PINS_D5 -1
		  #define LCD_PINS_D6 -1
		  #define LCD_PINS_D7 -1
		  
			//NAMADD Filament check
			#define BTN_EN1_FILAMENT_A 55//A1
			#define BTN_EN2_FILAMENT_A 57//A3
			
			#define BTN_EN1_FILAMENT_B 54//A0
			#define BTN_EN2_FILAMENT_B 56//A2
			
			#define BTN_EN1_FILAMENT_C 62//A8
			#define BTN_EN2_FILAMENT_C 63//A9			

			
		#elif ENABLED(LCD_I2C_PANELOLU2)
		  #define BTN_EN1 47  // reverse if the encoder turns the wrong way.
		  #define BTN_EN2 43
		  #define BTN_ENC 32
		  #define LCD_SDSS 53
		  #define SD_DETECT_PIN -1
		  #define KILL_PIN 41
		#elif ENABLED(LCD_I2C_VIKI)
		  #define BTN_EN1 22  // reverse if the encoder turns the wrong way.
		  #define BTN_EN2 7   // http://files.panucatt.com/datasheets/viki_wiring_diagram.pdf
							  // tells about 40/42.
							  // 22/7 are unused on RAMPS_14. 22 is unused and 7 the SERVO0_PIN on RAMPS_13.
		  #define BTN_ENC -1
		  #define LCD_SDSS 53
		  #define SD_DETECT_PIN 49
		#elif ENABLED(ELB_FULL_GRAPHIC_CONTROLLER)
		  #define BTN_EN1 35  // reverse if the encoder turns the wrong way.
		  #define BTN_EN2 37
		  #define BTN_ENC 31
		  #define SD_DETECT_PIN 49
		  #define LCD_SDSS 53
		  #define KILL_PIN 41
		  #define BEEPER_PIN 23
		  #define DOGLCD_CS 29
		  #define DOGLCD_A0 27
		  #define LCD_PIN_BL 33
		#elif ENABLED(MINIPANEL)
		  #define BEEPER_PIN 42
		  // Pins for DOGM SPI LCD Support
		  #define DOGLCD_A0  44
		  #define DOGLCD_CS  66
		  #define LCD_PIN_BL 65 // backlight LED on A11/D65
		  #define SDSS   53

		  #define KILL_PIN 64
		  // GLCD features
		  //#define LCD_CONTRAST 190
		  // Uncomment screen orientation
		  //#define LCD_SCREEN_ROT_90
		  //#define LCD_SCREEN_ROT_180
		  //#define LCD_SCREEN_ROT_270
		  //The encoder and click button
		  #define BTN_EN1 40
		  #define BTN_EN2 63
		  #define BTN_ENC 59  //the click switch
		  //not connected to a pin
		  #define SD_DETECT_PIN 49

		#else

		  #define BEEPER_PIN 33  // Beeper on AUX-4
		  // buttons are directly attached using AUX-2
		  #if ENABLED(REPRAPWORLD_KEYPAD)
			#define BTN_EN1 64 // encoder
			#define BTN_EN2 59 // encoder
			#define BTN_ENC 63 // enter button
			#define SHIFT_OUT 40 // shift register
			#define SHIFT_CLK 44 // shift register
			#define SHIFT_LD 42 // shift register
		  #elif ENABLED(PANEL_ONE)
			#define BTN_EN1 59 // AUX2 PIN 3
			#define BTN_EN2 63 // AUX2 PIN 4
			#define BTN_ENC 49 // AUX3 PIN 7
		  #else
			#define BTN_EN1 37
			#define BTN_EN2 35
			#define BTN_ENC 31  // the click
		  #endif

		  #if ENABLED(G3D_PANEL)
			#define SD_DETECT_PIN 49
		  #else
			#define SD_DETECT_PIN -1  // Ramps doesn't use this
		  #endif

		#endif
	#else // !NEWPANEL (Old-style panel with shift register)

		#define BEEPER_PIN 32   // No Beeper added
		// Buttons are attached to a shift register
		// Not wired yet
		//#define SHIFT_CLK 38
		//#define SHIFT_LD 42
		//#define SHIFT_OUT 40
		//#define SHIFT_EN 17

		#define LCD_PINS_RS 	35	//CS
		#define LCD_PINS_ENABLE 48	//MOSI
		#define LCD_PINS_D4 	46  //SCK
		#define LCD_PINS_D5 	-1
		#define LCD_PINS_D6 	-1
		#define LCD_PINS_D7 	-1
		
		#if ENABLED(U8GLIB_SSD1306_SPI)
			//Use for SSD1306_SPI LCD
			#define LCD_PINS_DC		36	
		#endif
			
		#define BTN_EN1 15
		#define BTN_EN2 30
		#define BTN_ENC 31	

		//NAMADD Filament check
        //#define BTN_EN1_FILAMENT_A 43
        //#define BTN_EN2_FILAMENT_A 45			

	#endif // !NEWPANEL

#endif // ULTRA_LCD

// SPI for Max6675 Thermocouple
#if DISABLED(SDSUPPORT)
  #define MAX6675_SS       66 // Do not use pin 53 if there is even the remote possibility of using Display/SD card
#else
  #define MAX6675_SS       66 // Do not use pin 49 as this is tied to the switch inside the SD card socket to detect if there is an SD card present
#endif

#if DISABLED(SDSUPPORT)
  // these pins are defined in the SD library if building with SD support
  #define SCK_PIN          52
  #define MISO_PIN         50
  #define MOSI_PIN         51
#endif
