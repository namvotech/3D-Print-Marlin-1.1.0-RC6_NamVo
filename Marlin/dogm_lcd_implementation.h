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

/**
 * dogm_lcd_implementation.h
 *
 * Graphics LCD implementation for 128x64 pixel LCDs by STB for ErikZalm/Marlin
 * Demonstrator: http://www.reprap.org/wiki/STB_Electronics
 * License: http://opensource.org/licenses/BSD-3-Clause
 *
 * With the use of:
 * u8glib by Oliver Kraus
 * https://github.com/olikraus/U8glib_Arduino
 * License: http://opensource.org/licenses/BSD-3-Clause
 */

#ifndef DOGM_LCD_IMPLEMENTATION_H
#define DOGM_LCD_IMPLEMENTATION_H

/**
 * Implementation of the LCD display routines for a DOGM128 graphic display. These are common LCD 128x64 pixel graphic displays.
 */

#if ENABLED(ULTIPANEL)
  #define BLEN_A 0
  #define BLEN_B 1
  #define BLEN_C 2
  #define EN_A (_BV(BLEN_A))
  #define EN_B (_BV(BLEN_B))
  #define EN_C (_BV(BLEN_C))
  #define LCD_CLICKED (buttons&EN_C)
	
	//NAMADD Filament check	
	#define encrot_filament0_a 0
	#define encrot_filament1_a 2
	#define encrot_filament2_a 3
	#define encrot_filament3_a 1
	
	#define encrot_filament0_b 0
	#define encrot_filament1_b 2
	#define encrot_filament2_b 3
	#define encrot_filament3_b 1

	#define encrot_filament0_c 0
	#define encrot_filament1_c 2
	#define encrot_filament2_c 3
	#define encrot_filament3_c 1	
	
#endif

#include <U8glib.h>
#include "dogm_bitmaps.h"

#include "ultralcd.h"
#include "ultralcd_st7920_u8glib_rrd.h"
#include "Configuration.h"

#if DISABLED(MAPPER_C2C3) && DISABLED(MAPPER_NON) && ENABLED(USE_BIG_EDIT_FONT)
  #undef USE_BIG_EDIT_FONT
#endif


#if ENABLED(USE_SMALL_INFOFONT)
  #include "dogm_font_data_6x9_marlin.h"
  #define FONT_STATUSMENU_NAME u8g_font_6x9
#else
  #define FONT_STATUSMENU_NAME FONT_MENU_NAME
#endif

#include "dogm_font_data_Marlin_symbols.h"   // The Marlin special symbols
#define FONT_SPECIAL_NAME Marlin_symbols

#if DISABLED(SIMULATE_ROMFONT)
  #if ENABLED(DISPLAY_CHARSET_ISO10646_1)
    #include "dogm_font_data_ISO10646_1.h"
    #define FONT_MENU_NAME ISO10646_1_5x7
  #elif ENABLED(DISPLAY_CHARSET_ISO10646_5)
    #include "dogm_font_data_ISO10646_5_Cyrillic.h"
    #define FONT_MENU_NAME ISO10646_5_Cyrillic_5x7
  #elif ENABLED(DISPLAY_CHARSET_ISO10646_KANA)
    #include "dogm_font_data_ISO10646_Kana.h"
    #define FONT_MENU_NAME ISO10646_Kana_5x7
  #elif ENABLED(DISPLAY_CHARSET_ISO10646_CN)
    #include "dogm_font_data_ISO10646_CN.h"
    #define FONT_MENU_NAME ISO10646_CN
    #define TALL_FONT_CORRECTION 1
  #else // fall-back
    #include "dogm_font_data_ISO10646_1.h"
    #define FONT_MENU_NAME ISO10646_1_5x7
  #endif
#else // SIMULATE_ROMFONT
  #if ENABLED(DISPLAY_CHARSET_HD44780_JAPAN)
    #include "dogm_font_data_HD44780_J.h"
    #define FONT_MENU_NAME HD44780_J_5x7
  #elif ENABLED(DISPLAY_CHARSET_HD44780_WESTERN)
    #include "dogm_font_data_HD44780_W.h"
    #define FONT_MENU_NAME HD44780_W_5x7
  #elif ENABLED(DISPLAY_CHARSET_HD44780_CYRILLIC)
    #include "dogm_font_data_HD44780_C.h"
    #define FONT_MENU_NAME HD44780_C_5x7
  #else // fall-back
    #include "dogm_font_data_ISO10646_1.h"
    #define FONT_MENU_NAME ISO10646_1_5x7
  #endif
#endif // SIMULATE_ROMFONT

//#define FONT_STATUSMENU_NAME FONT_MENU_NAME

#define FONT_STATUSMENU 1
#define FONT_SPECIAL 2
#define FONT_MENU_EDIT 3
#define FONT_MENU 4

// DOGM parameters (size in pixels)
#define DOG_CHAR_WIDTH         6
#define DOG_CHAR_HEIGHT        12
#if ENABLED(USE_BIG_EDIT_FONT)
  #define FONT_MENU_EDIT_NAME u8g_font_9x18
  #define DOG_CHAR_WIDTH_EDIT  9
  #define DOG_CHAR_HEIGHT_EDIT 18
  #define LCD_WIDTH_EDIT       14
#else
  #define FONT_MENU_EDIT_NAME FONT_MENU_NAME
  #define DOG_CHAR_WIDTH_EDIT  6
  #define DOG_CHAR_HEIGHT_EDIT 12
  #define LCD_WIDTH_EDIT       22
#endif

#ifndef TALL_FONT_CORRECTION
  #define TALL_FONT_CORRECTION 0
#endif

//NAMFIX
//#define START_ROW              0
#define START_ROW              0.2

// LCD selection
#if ENABLED(U8GLIB_ST7920)
  //U8GLIB_ST7920_128X64_RRD u8g(0,0,0);
  U8GLIB_ST7920_128X64_RRD u8g(0);
#elif ENABLED(MAKRPANEL)
  // The MaKrPanel display, ST7565 controller as well
  U8GLIB_NHD_C12864 u8g(DOGLCD_CS, DOGLCD_A0);
#elif ENABLED(VIKI2) || ENABLED(miniVIKI)
  // Mini Viki and Viki 2.0 LCD, ST7565 controller as well
  U8GLIB_NHD_C12864 u8g(DOGLCD_CS, DOGLCD_A0);
#elif ENABLED(U8GLIB_LM6059_AF)
  // Based on the Adafruit ST7565 (http://www.adafruit.com/products/250)
  U8GLIB_LM6059 u8g(DOGLCD_CS, DOGLCD_A0);
#elif ENABLED(U8GLIB_SSD1306_I2C)
  // Generic support for SSD1306 OLED I2C LCDs
  U8GLIB_SSD1306_128X64 u8g(U8G_I2C_OPT_NONE);
#elif ENABLED(U8GLIB_SSD1306_SPI)
  // Generic support for SSD1306 OLED SPI LCDs
  U8GLIB_SSD1306_128X64 u8g(LCD_PINS_D4,LCD_PINS_ENABLE,LCD_PINS_RS,LCD_PINS_DC);  
#elif ENABLED(MINIPANEL)
  // The MINIPanel display
  U8GLIB_MINI12864 u8g(DOGLCD_CS, DOGLCD_A0);
#else
  // for regular DOGM128 display with HW-SPI
  U8GLIB_DOGM128 u8g(DOGLCD_CS, DOGLCD_A0);  // HW-SPI Com: CS, A0
#endif

#ifndef LCD_PIXEL_WIDTH
  #define LCD_PIXEL_WIDTH 128
#endif
#ifndef LCD_PIXEL_HEIGHT
  #define LCD_PIXEL_HEIGHT 64
#endif

#include "utf_mapper.h"

int lcd_contrast;
static char currentfont = 0;

static void lcd_setFont(char font_nr) {
  switch(font_nr) {
    case FONT_STATUSMENU : {u8g.setFont(FONT_STATUSMENU_NAME); currentfont = FONT_STATUSMENU;}; break;
    case FONT_MENU       : {u8g.setFont(FONT_MENU_NAME); currentfont = FONT_MENU;}; break;
    case FONT_SPECIAL    : {u8g.setFont(FONT_SPECIAL_NAME); currentfont = FONT_SPECIAL;}; break;
    case FONT_MENU_EDIT  : {u8g.setFont(FONT_MENU_EDIT_NAME); currentfont = FONT_MENU_EDIT;}; break;
    break;
  }
}

char lcd_print(char c) {
  if ((c > 0) && (c <= LCD_STR_SPECIAL_MAX)) {
    u8g.setFont(FONT_SPECIAL_NAME);
    u8g.print(c);
    lcd_setFont(currentfont);
    return 1;
  } else {
    return charset_mapper(c);
  }
}

char lcd_print(const char* str) {
  char c;
  int i = 0;
  char n = 0;
  while ((c = str[i++])) {
    n += lcd_print(c);
  }
  return n;
}

/* Arduino < 1.0.0 is missing a function to print PROGMEM strings, so we need to implement our own */
char lcd_printPGM(const char* str) {
  char c;
  char n = 0;
  while ((c = pgm_read_byte(str++))) {
    n += lcd_print(c);
  }
  return n;
}

#if ENABLED(SHOW_BOOTSCREEN)
  static bool show_bootscreen = true;
#endif


/* Warning: This function is called from interrupt context */
static void lcd_implementation_init() {

  #if defined(LCD_PIN_BL) && LCD_PIN_BL > -1 // Enable LCD backlight
    pinMode(LCD_PIN_BL, OUTPUT);
    digitalWrite(LCD_PIN_BL, HIGH);
  #endif

  #if defined(LCD_PIN_RESET) && LCD_PIN_RESET > -1
    pinMode(LCD_PIN_RESET, OUTPUT);
    digitalWrite(LCD_PIN_RESET, HIGH);
  #endif

  #if DISABLED(MINIPANEL) // setContrast not working for Mini Panel
    u8g.setContrast(lcd_contrast);
  #endif

  // FIXME: remove this workaround
  // Uncomment this if you have the first generation (V1.10) of STBs board
  // pinMode(17, OUTPUT); // Enable LCD backlight
  // digitalWrite(17, HIGH);

  #if ENABLED(LCD_SCREEN_ROT_90)
    u8g.setRot90();   // Rotate screen by 90°
  #elif ENABLED(LCD_SCREEN_ROT_180)
    u8g.setRot180();  // Rotate screen by 180°
  #elif ENABLED(LCD_SCREEN_ROT_270)
    u8g.setRot270();  // Rotate screen by 270°
  #endif

  #if ENABLED(SHOW_BOOTSCREEN)

	int offx=0; 
	int offy=1;
/*
    int offx = (u8g.getWidth() - (START_BMPWIDTH)) / 2;
    #if ENABLED(START_BMPHIGH)
      int offy = 0;
    #else
      int offy = DOG_CHAR_HEIGHT;
    #endif
*/
    int txt1X = (u8g.getWidth() - (sizeof(STRING_SPLASH_LINE1) - 1) * (DOG_CHAR_WIDTH)) / 2;

    u8g.firstPage();
    do {
      if (show_bootscreen) {
		  
        u8g.drawBitmapP(offx, offy, START_BMPBYTEWIDTH, START_BMPHEIGHT, start_bmp);

		// Welcome message
		//lcd_setFont(FONT_MENU);			
		//u8g.drawStr(53,10,"3D PRINTER"); 

		u8g.setFont(u8g_font_profont11);
		u8g.setFont(u8g_font_ncenB08);		

		#if ENABLED(SLA_PRINTER)
			u8g.drawStr(53,10,"SLA");		
			u8g.drawStr(53,20,"PRINTER");
		#elif ENABLED(SLS_PRINTER)
			u8g.drawStr(53,10,"SLS");		
			u8g.drawStr(53,20,"PRINTER");
		#elif ENABLED(FDM500_DIAMOND)
			u8g.drawStr(53,10,"DIAMOND");		
			u8g.drawStr(53,20,"FDM");
		#elif ENABLED(FDM200_DIAMOND_VITME)	
			u8g.drawStr(53,10,"DIAMOND");		
			u8g.drawStr(53,20,"FDM VITME");	
		#elif ENABLED(FDM_COREXY)
			#if ENABLED(FDM_Z_DUAL_COREXY)
				u8g.drawStr(53,10,"FDM Z DUAL");		
				u8g.drawStr(53,20,"PRINTER");	
			#else
				u8g.drawStr(53,10,"FDM");		
				u8g.drawStr(53,20,"PRINTER");
			#endif
		#elif ENABLED(SCARA)	
			u8g.drawStr(53,10,"SCARA");		
			u8g.drawStr(53,20,"ROBOT");		
		#else
			u8g.drawStr(53,10,"DIAMOND");		
			u8g.drawStr(53,20,"FDM");						
		#endif
		//u8g.drawStr(53,20,"V1.0.1");
		
		u8g.setFont(u8g_font_ncenB14);	
		//u8g.drawStr(46,40,"Miracles");
		u8g.drawStr(46,40,"3DE");
		//lcd_setFont(FONT_MENU);	
		u8g.setFont(u8g_font_profont10);
		u8g.drawStr(0,60,"http://3deshop.vn/");
			
/*
        #ifndef STRING_SPLASH_LINE2
          u8g.drawStr(txt1X, u8g.getHeight() - (DOG_CHAR_HEIGHT), STRING_SPLASH_LINE1);
        #else
          int txt2X = (u8g.getWidth() - (sizeof(STRING_SPLASH_LINE2) - 1) * (DOG_CHAR_WIDTH)) / 2;
          u8g.drawStr(txt1X, u8g.getHeight() - (DOG_CHAR_HEIGHT) * 3 / 2, STRING_SPLASH_LINE1);
          u8g.drawStr(txt2X, u8g.getHeight() - (DOG_CHAR_HEIGHT) * 1 / 2, STRING_SPLASH_LINE2);
        #endif
*/
      }
    } while (u8g.nextPage());

    if (show_bootscreen) {
	  delay(2000);  
      show_bootscreen = false;
    }
  #endif
}


static void lcd_implementation_clear() { } // Automatically cleared by Picture Loop

FORCE_INLINE void _draw_centered_temp(int temp, int x, int y) {
  int degsize = 6 * (temp >= 100 ? 3 : temp >= 10 ? 2 : 1); // number's pixel width
  u8g.setPrintPos(x - (18 - degsize) / 2, y); // move left if shorter
  lcd_print(itostr3(temp));
  lcd_printPGM(PSTR(LCD_STR_DEGREE " "));
}

FORCE_INLINE void _draw_heater_bed_status(int x, int heater) {
  #if HAS_TEMP_BED
    bool isBed = heater < 0;
  #else
    const bool isBed = false;
  #endif	  
	  //Bed status
  if (isBed ? isHeatingBed() : isHeatingHotend(heater)) {
/////////////////////////////////////////	  	  	  
  }

}

FORCE_INLINE void _draw_axis_label(AxisEnum axis, const char *pstr, bool blink) {
  if (blink)
    lcd_printPGM(pstr);
  else {
    if (!axis_homed[axis])
      lcd_printPGM(PSTR("?"));
    else {
      #if DISABLED(DISABLE_REDUCED_ACCURACY_WARNING)
        if (!axis_known_position[axis])
          lcd_printPGM(PSTR(" "));
        else
      #endif
      lcd_printPGM(pstr);
    }
  }
}

static void lcd_implementation_status_screen() {
  u8g.setColorIndex(1); // black on white

  bool blink = lcd_blink();
 
	//NAMADD Font
    //u8g.setFont(u8g_font_timB08);	
	u8g.setFont(u8g_font_courB08);
//	u8g.drawBitmapP(14, 0, MIRACLES_BYTEWIDTH, MIRACLES_HEIGHT, miracles_logo_bmp);
	//lcd_setFont(FONT_MENU);

	//u8g.setPrintPos(0+7, 37);
	//u8g.setPrintPos(1, 37);
	u8g.setPrintPos(64, 37);
	lcd_printPGM(PSTR("3DE "));
	//u8g.setPrintPos(17, 37);
	//u8g.setPrintPos(15+7, 37);
	//lcd_printPGM(PSTR("MIRACLES"));
	lcd_printPGM(PSTR("SHOP"));
	
  // Status Menu Font for SD info, Heater status, Fan, XYZ

  //NAMADD Font
  u8g.setFont(u8g_font_5x7);
  #if ENABLED(SDSUPPORT)
    // SD Card Symbol
	u8g.drawBitmapP(42, 42, FOLDER_ICON_BYTEWIDTH, FOLDER_ICON_HEIGHT, folder_icon_bmp);	
	
    // Progress bar frame
    u8g.drawFrame(55, 49, 73, 4 - (TALL_FONT_CORRECTION));

    // SD Card Progress bar and clock
    if (IS_SD_PRINTING) {
      // Progress bar solid part
      u8g.drawBox(56, 50, (unsigned int)(71.f * card.percentDone() / 100.f), 2 - (TALL_FONT_CORRECTION));	    
    }
			
	//NAMFIX Time
    u8g.setPrintPos(77,48);	
    millis_t time = print_job_timer.duration();		
    if (time != 0) {
      lcd_print(itostr3(time/60/60));
      lcd_print(':');
      lcd_print(itostr2((time/60)%60));
      lcd_print(':');
      lcd_print(itostr2(time%60));	  
    }

 
	// SD Card Progress bar %
	if (card.percentDone()||time != 0)
	{		
		u8g.setPrintPos(49,48);
		lcd_print(itostr3(card.percentDone()));	
		lcd_print('%');
	}

	//NAMADD LED_RING
	#if ENABLED(LED_RING)
		if(led_ring_blink_en==false)
		#if ENABLED(LED_RING_LOW_V1)	
			if (IS_SD_PRINTING)		
					led_ring_low_v1_sdcard_printing(led_ring_color,card.percentDone()/6.5+2);
			else{
				led_ring_low_v1_sdcard_printing(0,0);	
			}
		#elif ENABLED(LED_RING_HIGH_V2)
			if (IS_SD_PRINTING)
				   led_ring_high_v2_sdcard_printing(led_ring_color,card.percentDone()/6.5+1);
			else led_ring_high_v2_sdcard_printing(0,0);
				
		#endif	
	#endif
  #endif //ENABLED(SDSUPPORT)
  
 
  	lcd_setFont(FONT_STATUSMENU);
   // Extruders
   //Extruders Icon
  u8g.drawBitmapP(5, 5, HEATER_ICON_BYTEWIDTH, HEATER_ICON_HEIGHT, heater_icon_bmp);    
	//Extruders status  
  if(isHeatingHotend(active_extruder))
  {
	  u8g.drawPixel(5,0);
	  u8g.drawPixel(6,1); 
	  u8g.drawPixel(5,2);
	  u8g.drawPixel(6,3); 

	  u8g.drawPixel(8,0);
	  u8g.drawPixel(9,1); 
	  u8g.drawPixel(8,2);
	  u8g.drawPixel(9,3);  
	  
	  u8g.drawPixel(11,0);
	  u8g.drawPixel(12,1); 
	  u8g.drawPixel(11,2);
	  u8g.drawPixel(12,3);	  
  }
/* 
//test  
  u8g.drawBox(60+8, 12-2, 13, 1); 
  u8g.drawBox(59+8, 13-2, 15, 1);
  //Heated bed status
  if (isHeatingBed() ) {
	  u8g.drawPixel(61+8,7-2);
	  u8g.drawPixel(62+8,8-2); 
	  u8g.drawPixel(61+8,9-2);
	  u8g.drawPixel(62+8,10-2);
	 
	  u8g.drawPixel(66+8,7-2);
	  u8g.drawPixel(67+8,8-2); 
	  u8g.drawPixel(66+8,9-2);
	  u8g.drawPixel(67+8,10-2);

	  u8g.drawPixel(71+8,7-2);
	  u8g.drawPixel(72+8,8-2); 
	  u8g.drawPixel(71+8,9-2);
	  u8g.drawPixel(72+8,10-2); 
  }  */
  
  //Extruders Value
  u8g.setPrintPos(20, 12); 
  lcd_print(itostr3(degHotend(active_extruder)));
  lcd_print('/');
  
  if (degTargetHotend(active_extruder)>=0 && degTargetHotend(active_extruder)<10)	
	  u8g.setPrintPos(20+24-12, 12); 
  else if (degTargetHotend(active_extruder)>=10 && degTargetHotend(active_extruder)<100)		
	  u8g.setPrintPos(20+24-6, 12); 
  else		
	  u8g.setPrintPos(20+24, 12); 
  
  lcd_print(itostr3(degTargetHotend(active_extruder)));
  //lcd_print('°');
  lcd_printPGM(PSTR(LCD_STR_DEGREE));
 
 #if HAS_TEMP_BED
  // Heated bed
  // Heated bed icon
  u8g.drawBox(2, 22+1, 13, 1); 
  u8g.drawBox(1, 23+1, 15, 1);
  //Heated bed status
  if (isHeatingBed() ) {
	  u8g.drawPixel(3,17+1);
	  u8g.drawPixel(4,18+1); 
	  u8g.drawPixel(3,19+1);
	  u8g.drawPixel(4,20+1);
	 
	  u8g.drawPixel(8,17+1);
	  u8g.drawPixel(9,18+1); 
	  u8g.drawPixel(8,19+1);
	  u8g.drawPixel(9,20+1);

	  u8g.drawPixel(13,17+1);
	  u8g.drawPixel(14,18+1); 
	  u8g.drawPixel(13,19+1);
	  u8g.drawPixel(14,20+1); 
  }  
  // Heated bed Value
  u8g.setPrintPos(20, 25); 
  lcd_print(itostr3(degBed()));
  lcd_print('/');
  
  if (degTargetBed()>=0 && degTargetBed()<10)	
	  u8g.setPrintPos(20+24-12, 25); 
  else if (degTargetBed()>=10 && degTargetBed()<100)		
	  u8g.setPrintPos(20+24-6, 25); 
  else		
	  u8g.setPrintPos(20+24, 25); 
  
  lcd_print(itostr3(degTargetBed()));
  //lcd_print('°');
  lcd_printPGM(PSTR(LCD_STR_DEGREE));
 
#endif
 
  // X, Y, Z-Coordinates
  // Before homing the axis letters are blinking 'X' <-> '?'.
  // When axis is homed but axis_known_position is false the axis letters are blinking 'X' <-> ' '.
  // When everything is ok you see a constant 'X'.
  #define XYZ_BASELINE 37
  // #if ENABLED(USE_SMALL_INFOFONT)
    // u8g.drawBox(0, 29, LCD_PIXEL_WIDTH, 10);
  // #else
    // u8g.drawBox(80, 29, 50, 9);
  // #endif 
   
//  u8g.setColorIndex(0); // white on black 
  // u8g.setPrintPos(87, XYZ_BASELINE);
  // _draw_axis_label(Z_AXIS, PSTR(MSG_Z), blink);
  // lcd_print('=');
  // u8g.setPrintPos(99, XYZ_BASELINE);
  // lcd_print(ftostr32sp(current_position[Z_AXIS] + 0.00001));
//  u8g.setColorIndex(1); // black on white

  u8g.setPrintPos(6, XYZ_BASELINE);
  _draw_axis_label(Z_AXIS, PSTR(MSG_Z), blink);
  
	if (current_position[Z_AXIS]>=0 && current_position[Z_AXIS]<10) 
		u8g.setPrintPos(8, XYZ_BASELINE);
	else if (current_position[Z_AXIS]>=10 && current_position[Z_AXIS]<100) 
		u8g.setPrintPos(14, XYZ_BASELINE);
	else 	
		u8g.setPrintPos(20, XYZ_BASELINE); 
	
  lcd_print(ftostr32sp(current_position[Z_AXIS] + 0.00001));
 
  //NAMADD Font
  u8g.setFont(u8g_font_5x7);
  // Feedrate
  u8g.drawBitmapP(3, 42, FEEDRATE_ICON_BYTEWIDTH, FEEDRATE_ICON_HEIGHT, feedrate_icon_bmp);    
  if(blink){
	  u8g.drawBox(4, 44, 3, 1);
	  u8g.drawBox(2, 46, 3, 1); 
	  u8g.drawBox(0, 48, 3, 1);
  }
  u8g.setPrintPos(16, 52);
  lcd_print(itostr3(feedrate_multiplier));
  lcd_print('%');
 
   //NAMADD Font
  u8g.setFont(u8g_font_5x7);
  // Status line
  #if ENABLED(USE_SMALL_INFOFONT)
    u8g.setPrintPos(0, 62);
  #else
    u8g.setPrintPos(0, 63);
  #endif
  
 
  #if DISABLED(FILAMENT_LCD_DISPLAY)
  //NAMFIX
	#if ENABLED(SDSUPPORT)
		if (card.longFilename[0])
			if (PENDING(millis(), previous_lcd_status_ms_name_file + 20000UL))
			{
				lcd_print(lcd_status_message);
			}	
			else
				lcd_print(card.longFilename);
		else lcd_print(lcd_status_message);
	#else
		lcd_print(lcd_status_message);
	#endif
	
  #else
    if (PENDING(millis(), previous_lcd_status_ms + 5000UL)) {  //Display both Status message line and Filament display on the last line
      lcd_print(lcd_status_message);
    }
    else {
      lcd_printPGM(PSTR("dia:"));
      lcd_print(ftostr12ns(filament_width_meas));
      lcd_printPGM(PSTR(" factor:"));
      lcd_print(itostr3(100.0 * volumetric_multiplier[FILAMENT_SENSOR_EXTRUDER_NUM]));
      lcd_print('%');
    }
  #endif 
  
}


//NAMADD
static void lcd_implementation_status_screen_life_time_state(){
  u8g.setColorIndex(1); // black on white
  bool blink = lcd_blink();
	
 	//NAMADD Font
    u8g.setFont(u8g_font_ncenB10);
	u8g.setPrintPos(5, 12); 
	lcd_print("Life Time Print");
	
    u8g.setFont(u8g_font_profont10); 
	u8g.setPrintPos(0, 28);
	lcd_print("Machine open:");
	u8g.setPrintPos(67, 28);
	lcd_print(itostr9(lifetime_minutes));
	u8g.setPrintPos(110, 28);
	lcd_print("min");
	
	u8g.setPrintPos(0, 38);
	lcd_print("Printing:");
	u8g.setPrintPos(67, 38);
	lcd_print(itostr9(lifetime_print_minutes));	
	u8g.setPrintPos(110, 38);
	lcd_print("min");	

	// u8g.setPrintPos(0, 48);
	// lcd_print("Material:");
	// u8g.setPrintPos(67, 48);
	// lcd_print(itostr9(lifetime_print_centimeters));	
	// u8g.setPrintPos(110, 48);
	// lcd_print("cm");
	
	// if(lcd_blink())
	// {
		// u8g.setPrintPos(0, 40);
		// lcd_print("Hello");
		// u8g.setPrintPos(64, 50);
		// lcd_print("@@@@@@.");		
		
	// }
	
	u8g.setFont(u8g_font_5x7);
  // Status line
  #if ENABLED(USE_SMALL_INFOFONT)
    u8g.setPrintPos(0, 62);
  #else
    u8g.setPrintPos(0, 63);
  #endif
  #if DISABLED(FILAMENT_LCD_DISPLAY)
    lcd_print(lcd_status_message);
  #else
    if (PENDING(millis(), previous_lcd_status_ms + 5000UL)) {  //Display both Status message line and Filament display on the last line
      lcd_print(lcd_status_message);
    }
    else {
      lcd_printPGM(PSTR("dia:"));
      lcd_print(ftostr12ns(filament_width_meas));
      lcd_printPGM(PSTR(" factor:"));
      lcd_print(itostr3(100.0 * volumetric_multiplier[FILAMENT_SENSOR_EXTRUDER_NUM]));
      lcd_print('%');
    }
  #endif 	
}

//NAMADD
static void lcd_implementation_status_screen_complete_state(){
  u8g.setColorIndex(1); // black on white
  bool blink = lcd_blink();
	
 	//NAMADD Font
    u8g.setFont(u8g_font_ncenB10);
	u8g.setPrintPos(5, 20); 
	lcd_print("Print Job Done.");
	
    u8g.setFont(u8g_font_profont10); 
	if(lcd_blink())
	{
		u8g.setPrintPos(0, 40);
		lcd_print("Click button to");
		u8g.setPrintPos(64, 50);
		lcd_print("get products.");		
		
	}
	
	u8g.setFont(u8g_font_5x7);
  // Status line
  #if ENABLED(USE_SMALL_INFOFONT)
    u8g.setPrintPos(0, 62);
  #else
    u8g.setPrintPos(0, 63);
  #endif
  #if DISABLED(FILAMENT_LCD_DISPLAY)
    lcd_print(lcd_status_message);
  #else
    if (PENDING(millis(), previous_lcd_status_ms + 5000UL)) {  //Display both Status message line and Filament display on the last line
      lcd_print(lcd_status_message);
    }
    else {
      lcd_printPGM(PSTR("dia:"));
      lcd_print(ftostr12ns(filament_width_meas));
      lcd_printPGM(PSTR(" factor:"));
      lcd_print(itostr3(100.0 * volumetric_multiplier[FILAMENT_SENSOR_EXTRUDER_NUM]));
      lcd_print('%');
    }
  #endif 	
}

static void lcd_implementation_status_screen_filament_load_unload() {
  u8g.setColorIndex(1); // black on white
  bool blink = lcd_blink();
	
  // Status Menu Font for SD info, Heater status, Fan, XYZ
	lcd_setFont(FONT_STATUSMENU);
	
	
  u8g.drawFrame(32-5, 0, 64+10, 35);
   // Extruders
   //Extruders Icon
  u8g.drawBitmapP(63+8-35, 5+3, HEATER_ICON_BYTEWIDTH, HEATER_ICON_HEIGHT, heater_icon_bmp);    
	//Extruders status  
  if(isHeatingHotend(active_extruder))
  {
	  u8g.drawPixel(63+8-35,0+3);
	  u8g.drawPixel(64+8-35,1+3); 
	  u8g.drawPixel(63+8-35,2+3);
	  u8g.drawPixel(64+8-35,3+3); 

	  u8g.drawPixel(66+8-35,0+3);
	  u8g.drawPixel(67+8-35,1+3); 
	  u8g.drawPixel(66+8-35,2+3);
	  u8g.drawPixel(67+8-35,3+3);  
	  
	  u8g.drawPixel(69+8-35,0+3);
	  u8g.drawPixel(70+8-35,1+3); 
	  u8g.drawPixel(69+8-35,2+3);
	  u8g.drawPixel(70+8-35,3+3);	  
  }

  //Extruders Value
  u8g.setPrintPos(82-35, 12+3); 
  lcd_print(itostr3(degHotend(active_extruder)));
  lcd_print('/'); 
  
  if (degTargetHotend(active_extruder)>=0 && degTargetHotend(active_extruder)<10)	
	  u8g.setPrintPos(82+24-12-35, 12+3); 
  else if (degTargetHotend(active_extruder)>=10 && degTargetHotend(active_extruder)<100)		
	  u8g.setPrintPos(82+24-6-35, 12+3); 
  else		
	  u8g.setPrintPos(82+24-35, 12+3); 
  
  lcd_print(itostr3(degTargetHotend(active_extruder)));
  //lcd_print('°');
  lcd_printPGM(PSTR(LCD_STR_DEGREE));
 
  #define E_BASELINE 37-10+3
  u8g.setPrintPos(81-9-35, E_BASELINE);
  lcd_print("E");
   //_draw_axis_label(E_AXIS, PSTR(MSG_E), blink);	
    //NAMADD Font
    u8g.setFont(u8g_font_5x7);
	u8g.setPrintPos(93-11-35, E_BASELINE); 
    lcd_print(ftostr311(current_position[E_AXIS]));
 
 	//NAMADD Font
    u8g.setFont(u8g_font_5x7);
	u8g.setPrintPos(0, 62-12); 
	if(lcd_blink())
		lcd_print("Click button to stop.");
  // Status line
  #if ENABLED(USE_SMALL_INFOFONT)
    u8g.setPrintPos(0, 62);
  #else
    u8g.setPrintPos(0, 63);
  #endif
  #if DISABLED(FILAMENT_LCD_DISPLAY)
    lcd_print(lcd_status_message);
  #else
    if (PENDING(millis(), previous_lcd_status_ms + 5000UL)) {  //Display both Status message line and Filament display on the last line
      lcd_print(lcd_status_message);
    }
    else {
      lcd_printPGM(PSTR("dia:"));
      lcd_print(ftostr12ns(filament_width_meas));
      lcd_printPGM(PSTR(" factor:"));
      lcd_print(itostr3(100.0 * volumetric_multiplier[FILAMENT_SENSOR_EXTRUDER_NUM]));
      lcd_print('%');
    }
  #endif 
}

static void lcd_implementation_mark_as_selected(uint8_t row, bool isSelected) {
  if (isSelected) {
    u8g.setColorIndex(1);  // black on white
    u8g.drawBox(0, row * (DOG_CHAR_HEIGHT) + 3 - (TALL_FONT_CORRECTION), LCD_PIXEL_WIDTH, DOG_CHAR_HEIGHT);
    u8g.setColorIndex(0);  // following text must be white on black
  }
  else {
    u8g.setColorIndex(1); // unmarked text is black on white
  }
  u8g.setPrintPos((START_ROW) * (DOG_CHAR_WIDTH), (row + 1) * (DOG_CHAR_HEIGHT));
}

//Dấu chỉ dẫn
static void lcd_implementation_drawmenu_generic(bool isSelected, uint8_t row, const char* pstr, char pre_char, char post_char) {
  char c;
  uint8_t n = LCD_WIDTH - 2;

  lcd_implementation_mark_as_selected(row, isSelected);

  while (c = pgm_read_byte(pstr)) {
    n -= lcd_print(c);
    pstr++;
  }
  while (n--) lcd_print(' ');

  u8g.setPrintPos(LCD_PIXEL_WIDTH - (DOG_CHAR_WIDTH), (row + 1) * (DOG_CHAR_HEIGHT));
  lcd_print(post_char);
  lcd_print(' ');
}

static void _drawmenu_setting_edit_generic(bool isSelected, uint8_t row, const char* pstr, const char* data, bool pgm) {
  char c;
  uint8_t vallen = (pgm ? lcd_strlen_P(data) : (lcd_strlen((char*)data)));
  uint8_t n = LCD_WIDTH - 2 - vallen;

  lcd_implementation_mark_as_selected(row, isSelected);

  while (c = pgm_read_byte(pstr)) {
    n -= lcd_print(c);
    pstr++;
  }
  lcd_print(':');
  while (n--) lcd_print(' ');
  u8g.setPrintPos(LCD_PIXEL_WIDTH - (DOG_CHAR_WIDTH) * vallen, (row + 1) * (DOG_CHAR_HEIGHT));
  if (pgm)  lcd_printPGM(data);  else  lcd_print((char*)data);
}

#define lcd_implementation_drawmenu_setting_edit_generic(sel, row, pstr, data) _drawmenu_setting_edit_generic(sel, row, pstr, data, false)
#define lcd_implementation_drawmenu_setting_edit_generic_P(sel, row, pstr, data) _drawmenu_setting_edit_generic(sel, row, pstr, data, true)

#define lcd_implementation_drawmenu_setting_edit_int3(sel, row, pstr, pstr2, data, minValue, maxValue) lcd_implementation_drawmenu_setting_edit_generic(sel, row, pstr, itostr3(*(data)))
#define lcd_implementation_drawmenu_setting_edit_float3(sel, row, pstr, pstr2, data, minValue, maxValue) lcd_implementation_drawmenu_setting_edit_generic(sel, row, pstr, ftostr3(*(data)))
#define lcd_implementation_drawmenu_setting_edit_float32(sel, row, pstr, pstr2, data, minValue, maxValue) lcd_implementation_drawmenu_setting_edit_generic(sel, row, pstr, ftostr32(*(data)))
#define lcd_implementation_drawmenu_setting_edit_float43(sel, row, pstr, pstr2, data, minValue, maxValue) lcd_implementation_drawmenu_setting_edit_generic(sel, row, pstr, ftostr43(*(data)))
#define lcd_implementation_drawmenu_setting_edit_float5(sel, row, pstr, pstr2, data, minValue, maxValue) lcd_implementation_drawmenu_setting_edit_generic(sel, row, pstr, ftostr5(*(data)))
#define lcd_implementation_drawmenu_setting_edit_float52(sel, row, pstr, pstr2, data, minValue, maxValue) lcd_implementation_drawmenu_setting_edit_generic(sel, row, pstr, ftostr52(*(data)))
#define lcd_implementation_drawmenu_setting_edit_float51(sel, row, pstr, pstr2, data, minValue, maxValue) lcd_implementation_drawmenu_setting_edit_generic(sel, row, pstr, ftostr51(*(data)))
#define lcd_implementation_drawmenu_setting_edit_long5(sel, row, pstr, pstr2, data, minValue, maxValue) lcd_implementation_drawmenu_setting_edit_generic(sel, row, pstr, ftostr5(*(data)))
#define lcd_implementation_drawmenu_setting_edit_bool(sel, row, pstr, pstr2, data) lcd_implementation_drawmenu_setting_edit_generic_P(sel, row, pstr, (*(data))?PSTR(MSG_ON):PSTR(MSG_OFF))

//Add version for callback functions
#define lcd_implementation_drawmenu_setting_edit_callback_int3(sel, row, pstr, pstr2, data, minValue, maxValue, callback) lcd_implementation_drawmenu_setting_edit_generic(sel, row, pstr, itostr3(*(data)))
#define lcd_implementation_drawmenu_setting_edit_callback_float3(sel, row, pstr, pstr2, data, minValue, maxValue, callback) lcd_implementation_drawmenu_setting_edit_generic(sel, row, pstr, ftostr3(*(data)))
#define lcd_implementation_drawmenu_setting_edit_callback_float32(sel, row, pstr, pstr2, data, minValue, maxValue, callback) lcd_implementation_drawmenu_setting_edit_generic(sel, row, pstr, ftostr32(*(data)))
#define lcd_implementation_drawmenu_setting_edit_callback_float43(sel, row, pstr, pstr2, data, minValue, maxValue, callback) lcd_implementation_drawmenu_setting_edit_generic(sel, row, pstr, ftostr43(*(data)))
#define lcd_implementation_drawmenu_setting_edit_callback_float5(sel, row, pstr, pstr2, data, minValue, maxValue, callback) lcd_implementation_drawmenu_setting_edit_generic(sel, row, pstr, ftostr5(*(data)))
#define lcd_implementation_drawmenu_setting_edit_callback_float52(sel, row, pstr, pstr2, data, minValue, maxValue, callback) lcd_implementation_drawmenu_setting_edit_generic(sel, row, pstr, ftostr52(*(data)))
#define lcd_implementation_drawmenu_setting_edit_callback_float51(sel, row, pstr, pstr2, data, minValue, maxValue, callback) lcd_implementation_drawmenu_setting_edit_generic(sel, row, pstr, ftostr51(*(data)))
#define lcd_implementation_drawmenu_setting_edit_callback_long5(sel, row, pstr, pstr2, data, minValue, maxValue, callback) lcd_implementation_drawmenu_setting_edit_generic(sel, row, pstr, ftostr5(*(data)))
#define lcd_implementation_drawmenu_setting_edit_callback_bool(sel, row, pstr, pstr2, data, callback) lcd_implementation_drawmenu_setting_edit_generic_P(sel, row, pstr, (*(data))?PSTR(MSG_ON):PSTR(MSG_OFF))

void lcd_implementation_drawedit(const char* pstr, const char* value=NULL) {
  uint8_t rows = 1;
  uint8_t lcd_width = LCD_WIDTH, char_width = DOG_CHAR_WIDTH;
  uint8_t vallen = lcd_strlen(value);

  #if ENABLED(USE_BIG_EDIT_FONT)
    if (lcd_strlen_P(pstr) <= LCD_WIDTH_EDIT - 1) {
      lcd_setFont(FONT_MENU_EDIT);
      lcd_width = LCD_WIDTH_EDIT + 1;
      char_width = DOG_CHAR_WIDTH_EDIT;
      if (lcd_strlen_P(pstr) >= LCD_WIDTH_EDIT - vallen) rows = 2;
    }
    else {
      lcd_setFont(FONT_MENU);
    }
  #endif

  if (lcd_strlen_P(pstr) > LCD_WIDTH - 2 - vallen) rows = 2;

  const float kHalfChar = (DOG_CHAR_HEIGHT_EDIT) / 2;
  float rowHeight = u8g.getHeight() / (rows + 1); // 1/(rows+1) = 1/2 or 1/3

  u8g.setPrintPos(0, rowHeight + kHalfChar);
  lcd_printPGM(pstr);
  if (value != NULL) {
    lcd_print(':');
    u8g.setPrintPos((lcd_width - 1 - vallen) * char_width, rows * rowHeight + kHalfChar);
    lcd_print(value);
  }
}

#if ENABLED(SDSUPPORT)

  static void _drawmenu_sd(bool isSelected, uint8_t row, const char* pstr, const char* filename, char* const longFilename, bool isDir) {
    char c;
    uint8_t n = LCD_WIDTH - 1;

    if (longFilename[0]) {
      filename = longFilename;
      longFilename[n] = '\0';
    }

    lcd_implementation_mark_as_selected(row, isSelected);

    if (isDir) lcd_print(LCD_STR_FOLDER[0]);
    while ((c = *filename)) {
      n -= lcd_print(c);
      filename++;
    }
    while (n--) lcd_print(' ');
  }

  #define lcd_implementation_drawmenu_sdfile(sel, row, pstr, filename, longFilename) _drawmenu_sd(sel, row, pstr, filename, longFilename, false)
  #define lcd_implementation_drawmenu_sddirectory(sel, row, pstr, filename, longFilename) _drawmenu_sd(sel, row, pstr, filename, longFilename, true)

#endif //SDSUPPORT

#define lcd_implementation_drawmenu_back(sel, row, pstr) lcd_implementation_drawmenu_generic(sel, row, pstr, LCD_STR_UPLEVEL[0], LCD_STR_UPLEVEL[0])
#define lcd_implementation_drawmenu_submenu(sel, row, pstr, data) lcd_implementation_drawmenu_generic(sel, row, pstr, '>', LCD_STR_ARROW_RIGHT[0])
#define lcd_implementation_drawmenu_gcode(sel, row, pstr, gcode) lcd_implementation_drawmenu_generic(sel, row, pstr, '>', ' ')
#define lcd_implementation_drawmenu_function(sel, row, pstr, data) lcd_implementation_drawmenu_generic(sel, row, pstr, '>', ' ')

#endif //__DOGM_LCD_IMPLEMENTATION_H
