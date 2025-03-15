// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.3.4
// LVGL version: 8.3.6
// Project name: SquareLine_Project

#ifndef _SQUARELINE_PROJECT_UI_H
#define _SQUARELINE_PROJECT_UI_H

#ifdef __cplusplus
extern "C" {
#endif

#if defined __has_include
#if __has_include("lvgl.h")
#include "lvgl.h"
#elif __has_include("lvgl/lvgl.h")
#include "lvgl/lvgl.h"
#else
#include "lvgl.h"
#endif
#else
#include "lvgl.h"
#endif

#include "ui_helpers.h"
#include "ui_events.h"
// SCREEN: ui_Screen1
void ui_Screen1_screen_init(void);
extern lv_obj_t * ui_Screen1;
extern lv_obj_t * ui_Label1;
extern lv_obj_t * ui_BANTpnl;
extern lv_obj_t * ui_Panel6;
extern lv_obj_t * ui_STEPpnl;
extern lv_obj_t * ui_stepLBL;
extern lv_obj_t * ui_Panel1;
extern lv_obj_t * ui_batLBL;
void ui_event_Button3(lv_event_t * e);
extern lv_obj_t * ui_Button3;
extern lv_obj_t * ui_Label5;
extern lv_obj_t * ui_disLBL;
extern lv_obj_t * ui_tresLBL2;
void ui_event_Button4(lv_event_t * e);
extern lv_obj_t * ui_Button4;
extern lv_obj_t * ui_Label8;
extern lv_obj_t * ui_Label2;
extern lv_obj_t * ui_Label3;
extern lv_obj_t * ui_StepdisLBL;
extern lv_obj_t * ui_Label10;
extern lv_obj_t * ui_Label11;
extern lv_obj_t * ui_secLBL;
extern lv_obj_t * ui_Label13;
extern lv_obj_t * ui_speedLBL;
extern lv_obj_t * ui_Label12;
extern lv_obj_t * ui_Panel2;
extern lv_obj_t * ui_Label14;
extern lv_obj_t * ui_caloriesLBL;
extern lv_obj_t * ui_Label15;
extern lv_obj_t * ui_Panel4;
extern lv_obj_t * ui_Label16;
extern lv_obj_t * ui_Panel5;
extern lv_obj_t * ui_Label17;
extern lv_obj_t * ui_Label18;
extern lv_obj_t * ui_Image2;
extern lv_obj_t * ui_Label19;
extern lv_obj_t * ui_BANTpnl1;
extern lv_obj_t * ui_Spinner2;
extern lv_obj_t * ui_sleepBar;
extern lv_obj_t * ui_sndPNL;
extern lv_obj_t * ui_Panel3;
extern lv_obj_t * ui_crgPNL;
extern lv_obj_t * ui_Panel7;
extern lv_obj_t * ui_weightLBL2;
extern lv_obj_t * ui_Label29;
extern lv_obj_t * ui_Panel8;
// SCREEN: ui_setScreen
void ui_setScreen_screen_init(void);
extern lv_obj_t * ui_setScreen;
extern lv_obj_t * ui_tresLBL;
void ui_event_Button1(lv_event_t * e);
extern lv_obj_t * ui_Button1;
extern lv_obj_t * ui_Label6;
void ui_event_Button2(lv_event_t * e);
extern lv_obj_t * ui_Button2;
extern lv_obj_t * ui_Label7;
void ui_event_Button5(lv_event_t * e);
extern lv_obj_t * ui_Button5;
extern lv_obj_t * ui_Label20;
void ui_event_Button6(lv_event_t * e);
extern lv_obj_t * ui_Button6;
extern lv_obj_t * ui_Label4;
void ui_event_Button7(lv_event_t * e);
extern lv_obj_t * ui_Button7;
extern lv_obj_t * ui_Label9;
extern lv_obj_t * ui_StepdisLBL1;
void ui_event_brightSLI(lv_event_t * e);
extern lv_obj_t * ui_brightSLI;
void ui_event_weightDOWN(lv_event_t * e);
extern lv_obj_t * ui_weightDOWN;
extern lv_obj_t * ui_Label21;
extern lv_obj_t * ui_Label22;
extern lv_obj_t * ui_Label23;
void ui_event_weightUP(lv_event_t * e);
extern lv_obj_t * ui_weightUP;
extern lv_obj_t * ui_Label24;
extern lv_obj_t * ui_Label25;
extern lv_obj_t * ui_soundLBL;
extern lv_obj_t * ui_brightnessLBL;
void ui_event_soundOnBTN(lv_event_t * e);
extern lv_obj_t * ui_soundOnBTN;
extern lv_obj_t * ui_Label27;
void ui_event_soundOffBTN(lv_event_t * e);
extern lv_obj_t * ui_soundOffBTN;
extern lv_obj_t * ui_Label28;
extern lv_obj_t * ui_weightLBL;
// SCREEN: ui_Screen2
void ui_Screen2_screen_init(void);
extern lv_obj_t * ui_Screen2;
extern lv_obj_t * ui_Image1;
extern lv_obj_t * ui_Spinner1;
extern lv_obj_t * ui_Label26;
extern lv_obj_t * ui_Image3;
extern lv_obj_t * ui____initial_actions0;

LV_IMG_DECLARE(ui_img_logo_png);    // assets\logo.png
LV_IMG_DECLARE(ui_img_1537598557);    // assets\Dakirby309-Simply-Styled-YouTube.24.png
LV_IMG_DECLARE(ui_img_2038933285);    // assets\Arturo-Wibawa-Akar-Sound-off.24.png
LV_IMG_DECLARE(ui_img_597475696);    // assets\Oxygen-Icons.org-Oxygen-Status-battery-charging-080.24.png
LV_IMG_DECLARE(ui_img_440250019);    // assets\Dakirby309-Simply-Styled-YouTube.48.png

LV_FONT_DECLARE(ui_font_Font1);
LV_FONT_DECLARE(ui_font_mid);
LV_FONT_DECLARE(ui_font_mini);
LV_FONT_DECLARE(ui_font_secFont);
LV_FONT_DECLARE(ui_font_SPD);
LV_FONT_DECLARE(ui_font_STEP);
LV_FONT_DECLARE(ui_font_micro);

void ui_init(void);

#ifdef __cplusplus
} /*extern "C"*/
#endif

#endif
