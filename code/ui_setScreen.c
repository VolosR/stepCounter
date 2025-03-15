// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.3.4
// LVGL version: 8.3.6
// Project name: SquareLine_Project

#include "ui.h"

void ui_setScreen_screen_init(void)
{
    ui_setScreen = lv_obj_create(NULL);
    lv_obj_clear_flag(ui_setScreen, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_bg_color(ui_setScreen, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_setScreen, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_img_src(ui_setScreen, &ui_img_logo_png, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_img_opa(ui_setScreen, 40, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_tresLBL = lv_label_create(ui_setScreen);
    lv_obj_set_width(ui_tresLBL, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_tresLBL, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_tresLBL, -113);
    lv_obj_set_y(ui_tresLBL, 109);
    lv_obj_set_align(ui_tresLBL, LV_ALIGN_CENTER);
    lv_label_set_text(ui_tresLBL, "2");
    lv_obj_set_style_text_font(ui_tresLBL, &lv_font_montserrat_32, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Button1 = lv_btn_create(ui_setScreen);
    lv_obj_set_width(ui_Button1, 100);
    lv_obj_set_height(ui_Button1, 61);
    lv_obj_set_x(ui_Button1, -115);
    lv_obj_set_y(ui_Button1, 11);
    lv_obj_set_align(ui_Button1, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_Button1, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags
    lv_obj_clear_flag(ui_Button1, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_radius(ui_Button1, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_Button1, lv_color_hex(0x7BAAC5), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Button1, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Label6 = lv_label_create(ui_Button1);
    lv_obj_set_width(ui_Label6, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Label6, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_align(ui_Label6, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Label6, "+");
    lv_obj_set_style_text_color(ui_Label6, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_Label6, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_Label6, &lv_font_montserrat_48, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Button2 = lv_btn_create(ui_setScreen);
    lv_obj_set_width(ui_Button2, 100);
    lv_obj_set_height(ui_Button2, 61);
    lv_obj_set_x(ui_Button2, -116);
    lv_obj_set_y(ui_Button2, 167);
    lv_obj_set_align(ui_Button2, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_Button2, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags
    lv_obj_clear_flag(ui_Button2, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_radius(ui_Button2, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_Button2, lv_color_hex(0x7BAAC5), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Button2, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Label7 = lv_label_create(ui_Button2);
    lv_obj_set_width(ui_Label7, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Label7, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_align(ui_Label7, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Label7, "-");
    lv_obj_set_style_text_color(ui_Label7, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_Label7, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_Label7, &lv_font_montserrat_48, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Button5 = lv_btn_create(ui_setScreen);
    lv_obj_set_width(ui_Button5, 100);
    lv_obj_set_height(ui_Button5, 69);
    lv_obj_set_x(ui_Button5, -111);
    lv_obj_set_y(ui_Button5, -169);
    lv_obj_set_align(ui_Button5, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_Button5, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags
    lv_obj_clear_flag(ui_Button5, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_radius(ui_Button5, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_Button5, lv_color_hex(0x8B0995), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Button5, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Label20 = lv_label_create(ui_Button5);
    lv_obj_set_width(ui_Label20, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Label20, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Label20, 0);
    lv_obj_set_y(ui_Label20, 3);
    lv_obj_set_align(ui_Label20, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Label20, "OK");
    lv_obj_set_style_text_font(ui_Label20, &ui_font_mid, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Button6 = lv_btn_create(ui_setScreen);
    lv_obj_set_width(ui_Button6, 100);
    lv_obj_set_height(ui_Button6, 61);
    lv_obj_set_x(ui_Button6, 0);
    lv_obj_set_y(ui_Button6, 11);
    lv_obj_set_align(ui_Button6, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_Button6, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags
    lv_obj_clear_flag(ui_Button6, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_radius(ui_Button6, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_Button6, lv_color_hex(0x7BAAC5), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Button6, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Label4 = lv_label_create(ui_Button6);
    lv_obj_set_width(ui_Label4, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Label4, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_align(ui_Label4, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Label4, "+");
    lv_obj_set_style_text_color(ui_Label4, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_Label4, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_Label4, &lv_font_montserrat_48, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Button7 = lv_btn_create(ui_setScreen);
    lv_obj_set_width(ui_Button7, 100);
    lv_obj_set_height(ui_Button7, 61);
    lv_obj_set_x(ui_Button7, 1);
    lv_obj_set_y(ui_Button7, 167);
    lv_obj_set_align(ui_Button7, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_Button7, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags
    lv_obj_clear_flag(ui_Button7, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_radius(ui_Button7, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_Button7, lv_color_hex(0x7BAAC5), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Button7, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Label9 = lv_label_create(ui_Button7);
    lv_obj_set_width(ui_Label9, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Label9, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_align(ui_Label9, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Label9, "-");
    lv_obj_set_style_text_color(ui_Label9, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_Label9, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_Label9, &lv_font_montserrat_48, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_StepdisLBL1 = lv_label_create(ui_setScreen);
    lv_obj_set_width(ui_StepdisLBL1, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_StepdisLBL1, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_StepdisLBL1, -4);
    lv_obj_set_y(ui_StepdisLBL1, 110);
    lv_obj_set_align(ui_StepdisLBL1, LV_ALIGN_CENTER);
    lv_label_set_text(ui_StepdisLBL1, "2");
    lv_obj_set_style_text_font(ui_StepdisLBL1, &lv_font_montserrat_28, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_brightSLI = lv_slider_create(ui_setScreen);
    lv_slider_set_range(ui_brightSLI, 80, 255);
    lv_slider_set_value(ui_brightSLI, 160, LV_ANIM_OFF);
    if(lv_slider_get_mode(ui_brightSLI) == LV_SLIDER_MODE_RANGE) lv_slider_set_left_value(ui_brightSLI, 0, LV_ANIM_OFF);
    lv_obj_set_width(ui_brightSLI, 333);
    lv_obj_set_height(ui_brightSLI, 36);
    lv_obj_set_x(ui_brightSLI, 0);
    lv_obj_set_y(ui_brightSLI, -59);
    lv_obj_set_align(ui_brightSLI, LV_ALIGN_CENTER);


    ui_weightDOWN = lv_btn_create(ui_setScreen);
    lv_obj_set_width(ui_weightDOWN, 100);
    lv_obj_set_height(ui_weightDOWN, 61);
    lv_obj_set_x(ui_weightDOWN, 117);
    lv_obj_set_y(ui_weightDOWN, 166);
    lv_obj_set_align(ui_weightDOWN, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_weightDOWN, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags
    lv_obj_clear_flag(ui_weightDOWN, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_radius(ui_weightDOWN, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_weightDOWN, lv_color_hex(0x7BAAC5), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_weightDOWN, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Label21 = lv_label_create(ui_weightDOWN);
    lv_obj_set_width(ui_Label21, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Label21, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_align(ui_Label21, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Label21, "-");
    lv_obj_set_style_text_color(ui_Label21, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_Label21, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_Label21, &lv_font_montserrat_48, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Label22 = lv_label_create(ui_setScreen);
    lv_obj_set_width(ui_Label22, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Label22, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Label22, -113);
    lv_obj_set_y(ui_Label22, 64);
    lv_obj_set_align(ui_Label22, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Label22, "SENSOR\nSENSITIVITY");
    lv_obj_set_style_text_color(ui_Label22, lv_color_hex(0x94AA08), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_Label22, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_align(ui_Label22, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_Label22, &lv_font_montserrat_16, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Label23 = lv_label_create(ui_setScreen);
    lv_obj_set_width(ui_Label23, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Label23, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Label23, -3);
    lv_obj_set_y(ui_Label23, 64);
    lv_obj_set_align(ui_Label23, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Label23, "STEP\nLENGTH");
    lv_obj_set_style_text_color(ui_Label23, lv_color_hex(0x94AA08), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_Label23, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_align(ui_Label23, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_Label23, &lv_font_montserrat_16, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_weightUP = lv_btn_create(ui_setScreen);
    lv_obj_set_width(ui_weightUP, 100);
    lv_obj_set_height(ui_weightUP, 61);
    lv_obj_set_x(ui_weightUP, 115);
    lv_obj_set_y(ui_weightUP, 12);
    lv_obj_set_align(ui_weightUP, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_weightUP, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags
    lv_obj_clear_flag(ui_weightUP, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_radius(ui_weightUP, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_weightUP, lv_color_hex(0x7BAAC5), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_weightUP, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Label24 = lv_label_create(ui_weightUP);
    lv_obj_set_width(ui_Label24, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Label24, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_align(ui_Label24, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Label24, "+");
    lv_obj_set_style_text_color(ui_Label24, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_Label24, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_Label24, &lv_font_montserrat_48, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Label25 = lv_label_create(ui_setScreen);
    lv_obj_set_width(ui_Label25, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Label25, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Label25, 115);
    lv_obj_set_y(ui_Label25, 61);
    lv_obj_set_align(ui_Label25, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Label25, "WEIGHT");
    lv_obj_set_style_text_color(ui_Label25, lv_color_hex(0x94AA08), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_Label25, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_align(ui_Label25, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_Label25, &lv_font_montserrat_16, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_soundLBL = lv_label_create(ui_setScreen);
    lv_obj_set_width(ui_soundLBL, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_soundLBL, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_soundLBL, 139);
    lv_obj_set_y(ui_soundLBL, 22);
    lv_label_set_text(ui_soundLBL, "SOUND IS ON");
    lv_obj_set_style_text_font(ui_soundLBL, &lv_font_montserrat_18, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_brightnessLBL = lv_label_create(ui_setScreen);
    lv_obj_set_width(ui_brightnessLBL, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_brightnessLBL, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_brightnessLBL, -98);
    lv_obj_set_y(ui_brightnessLBL, -98);
    lv_obj_set_align(ui_brightnessLBL, LV_ALIGN_CENTER);
    lv_label_set_text(ui_brightnessLBL, "BRIGHTNESS");
    lv_obj_set_style_text_font(ui_brightnessLBL, &lv_font_montserrat_18, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_soundOnBTN = lv_btn_create(ui_setScreen);
    lv_obj_set_width(ui_soundOnBTN, 100);
    lv_obj_set_height(ui_soundOnBTN, 43);
    lv_obj_set_x(ui_soundOnBTN, 4);
    lv_obj_set_y(ui_soundOnBTN, -156);
    lv_obj_set_align(ui_soundOnBTN, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_soundOnBTN, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags
    lv_obj_clear_flag(ui_soundOnBTN, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_radius(ui_soundOnBTN, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_soundOnBTN, lv_color_hex(0x92AA0B), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_soundOnBTN, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Label27 = lv_label_create(ui_soundOnBTN);
    lv_obj_set_width(ui_Label27, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Label27, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Label27, 0);
    lv_obj_set_y(ui_Label27, 3);
    lv_obj_set_align(ui_Label27, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Label27, "ON");
    lv_obj_set_style_text_color(ui_Label27, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_Label27, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_Label27, &ui_font_mid, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_soundOffBTN = lv_btn_create(ui_setScreen);
    lv_obj_set_width(ui_soundOffBTN, 100);
    lv_obj_set_height(ui_soundOffBTN, 43);
    lv_obj_set_x(ui_soundOffBTN, 117);
    lv_obj_set_y(ui_soundOffBTN, -155);
    lv_obj_set_align(ui_soundOffBTN, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_soundOffBTN, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags
    lv_obj_clear_flag(ui_soundOffBTN, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_radius(ui_soundOffBTN, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_soundOffBTN, lv_color_hex(0xBA0921), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_soundOffBTN, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Label28 = lv_label_create(ui_soundOffBTN);
    lv_obj_set_width(ui_Label28, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Label28, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Label28, 0);
    lv_obj_set_y(ui_Label28, 3);
    lv_obj_set_align(ui_Label28, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Label28, "OFF");
    lv_obj_set_style_text_color(ui_Label28, lv_color_hex(0xDFDFDF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_Label28, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_Label28, &ui_font_mid, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_weightLBL = lv_label_create(ui_setScreen);
    lv_obj_set_width(ui_weightLBL, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_weightLBL, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_weightLBL, 115);
    lv_obj_set_y(ui_weightLBL, 110);
    lv_obj_set_align(ui_weightLBL, LV_ALIGN_CENTER);
    lv_label_set_text(ui_weightLBL, "2");
    lv_obj_set_style_text_font(ui_weightLBL, &lv_font_montserrat_28, LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_add_event_cb(ui_Button1, ui_event_Button1, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_Button2, ui_event_Button2, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_Button5, ui_event_Button5, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_Button6, ui_event_Button6, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_Button7, ui_event_Button7, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_brightSLI, ui_event_brightSLI, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_weightDOWN, ui_event_weightDOWN, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_weightUP, ui_event_weightUP, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_soundOnBTN, ui_event_soundOnBTN, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_soundOffBTN, ui_event_soundOffBTN, LV_EVENT_ALL, NULL);

}
