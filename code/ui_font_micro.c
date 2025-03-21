/*******************************************************************************
 * Size: 24 px
 * Bpp: 1
 * Opts: --bpp 1 --size 24 --font C:\Users\Danko\Desktop\AMOLED18\timeExample\assets\AGENCYB.TTF -o C:\Users\Danko\Desktop\AMOLED18\timeExample\assets\ui_font_micro.c --format lvgl -r 0x20-0x7f --no-compress --no-prefilter
 ******************************************************************************/

#include "ui.h"

#ifndef UI_FONT_MICRO
#define UI_FONT_MICRO 1
#endif

#if UI_FONT_MICRO

/*-----------------
 *    BITMAPS
 *----------------*/

/*Store the image of the glyphs*/
static LV_ATTRIBUTE_LARGE_CONST const uint8_t glyph_bitmap[] = {
    /* U+0020 " " */
    0x0,

    /* U+0021 "!" */
    0xff, 0xff, 0xff, 0xff, 0xff, 0xc0, 0xff, 0x80,

    /* U+0022 "\"" */
    0xef, 0x3c, 0xf3, 0xcf, 0x30,

    /* U+0023 "#" */
    0xc, 0xe0, 0xce, 0xc, 0xc0, 0xcc, 0x1c, 0xc7,
    0xff, 0x7f, 0xf1, 0x9c, 0x19, 0x81, 0x98, 0x19,
    0x83, 0x98, 0xff, 0xef, 0xfe, 0x33, 0x83, 0x30,
    0x33, 0x7, 0x30, 0x73, 0x0,

    /* U+0024 "$" */
    0x1c, 0x7f, 0xff, 0xfc, 0x7e, 0x3f, 0x1f, 0x8f,
    0xe0, 0x78, 0x1e, 0x7, 0x83, 0xe0, 0xf8, 0x3f,
    0x8f, 0xc7, 0xe3, 0xf1, 0xff, 0xff, 0xf1, 0xc0,
    0xe0,

    /* U+0025 "%" */
    0xfc, 0x9, 0xf8, 0x33, 0x30, 0xc6, 0x61, 0x8c,
    0xc6, 0x19, 0x8c, 0x33, 0x30, 0x66, 0x40, 0xfd,
    0x81, 0xfa, 0xfc, 0xd, 0xf8, 0x13, 0x30, 0x66,
    0x61, 0x8c, 0xc3, 0x19, 0x8c, 0x33, 0x18, 0x66,
    0x60, 0xfc, 0xc1, 0xf8,

    /* U+0026 "&" */
    0xff, 0xbf, 0xee, 0x3b, 0x8e, 0xe3, 0xb8, 0xee,
    0x3, 0xc0, 0x7f, 0xdf, 0xff, 0x3b, 0x8e, 0xe3,
    0xb8, 0xee, 0x3b, 0x8e, 0xe3, 0xbf, 0xef, 0xf8,

    /* U+0027 "'" */
    0xfb, 0x6d, 0x80,

    /* U+0028 "(" */
    0x1c, 0xe3, 0x9c, 0x71, 0xce, 0x38, 0xe3, 0x8e,
    0x38, 0xe3, 0x8e, 0x1c, 0x71, 0xc3, 0x8e, 0x18,

    /* U+0029 ")" */
    0xe1, 0xc7, 0xe, 0x38, 0xe1, 0xc7, 0x1c, 0x71,
    0xc7, 0x1c, 0x71, 0xce, 0x38, 0xe7, 0x1c, 0xe0,

    /* U+002A "*" */
    0xc, 0x13, 0x6, 0xd9, 0xfe, 0x1e, 0x1f, 0xe6,
    0xd9, 0x30, 0xc, 0x0,

    /* U+002B "+" */
    0x1c, 0x7, 0x1, 0xc3, 0xff, 0xff, 0xc7, 0x1,
    0xc0, 0x70, 0x1c, 0x0,

    /* U+002C "," */
    0xfb, 0x6c,

    /* U+002D "-" */
    0xff, 0xf0,

    /* U+002E "." */
    0xff, 0x80,

    /* U+002F "/" */
    0x1, 0xc0, 0x70, 0x18, 0xe, 0x3, 0x80, 0xc0,
    0x70, 0x18, 0xe, 0x3, 0x80, 0xc0, 0x70, 0x18,
    0xe, 0x3, 0x80, 0xc0, 0x70, 0x1c, 0x6, 0x0,

    /* U+0030 "0" */
    0xff, 0xff, 0xf8, 0xfc, 0x7e, 0x3f, 0x1f, 0x8f,
    0xc7, 0xe3, 0xf1, 0xf8, 0xfc, 0x7e, 0x3f, 0x1f,
    0x8f, 0xc7, 0xe3, 0xff, 0xff, 0xe0,

    /* U+0031 "1" */
    0x77, 0x7f, 0xff, 0x77, 0x77, 0x77, 0x77, 0x77,
    0x77, 0x70,

    /* U+0032 "2" */
    0xff, 0xff, 0xf8, 0xfc, 0x7e, 0x3f, 0x1c, 0xe,
    0xe, 0x7, 0x7, 0x3, 0x83, 0x81, 0xc1, 0xc0,
    0xe0, 0xe0, 0x70, 0x7f, 0xff, 0xe0,

    /* U+0033 "3" */
    0xff, 0xff, 0xf8, 0xfc, 0x7e, 0x3f, 0x1c, 0xe,
    0xf, 0x1e, 0xf, 0x3, 0xc0, 0x70, 0x3f, 0x1f,
    0x8f, 0xc7, 0xe3, 0xff, 0xff, 0xe0,

    /* U+0034 "4" */
    0xe, 0x3, 0x80, 0xe0, 0x70, 0x1c, 0x7, 0x3,
    0x80, 0xe0, 0x3b, 0x9c, 0xe7, 0x39, 0xce, 0xe3,
    0xbf, 0xff, 0xfc, 0xe, 0x3, 0x80, 0xe0, 0x38,

    /* U+0035 "5" */
    0xff, 0xff, 0xf8, 0x1c, 0xe, 0x7, 0x3, 0x81,
    0xff, 0xff, 0x81, 0xc0, 0xe0, 0x70, 0x3f, 0x1f,
    0x8f, 0xc7, 0xe3, 0xff, 0xff, 0xe0,

    /* U+0036 "6" */
    0xff, 0xff, 0xf8, 0xfc, 0x7e, 0x3f, 0x1f, 0x81,
    0xc0, 0xe0, 0x7f, 0xff, 0xfc, 0x7e, 0x3f, 0x1f,
    0x8f, 0xc7, 0xe3, 0xff, 0xff, 0xe0,

    /* U+0037 "7" */
    0xff, 0xff, 0xf8, 0xfc, 0x7e, 0x30, 0x38, 0x1c,
    0xe, 0x6, 0x7, 0x3, 0x81, 0xc0, 0xe0, 0x60,
    0x70, 0x38, 0x1c, 0xc, 0xe, 0x0,

    /* U+0038 "8" */
    0xff, 0xff, 0xf8, 0xfc, 0x7e, 0x3f, 0x1f, 0x8f,
    0xef, 0x7f, 0x1f, 0x3d, 0xfc, 0x7e, 0x3f, 0x1f,
    0x8f, 0xc7, 0xe3, 0xff, 0xff, 0xe0,

    /* U+0039 "9" */
    0xff, 0xff, 0xf8, 0xfc, 0x7e, 0x3f, 0x1f, 0x8f,
    0xc7, 0xe3, 0xff, 0xff, 0xe0, 0x70, 0x38, 0x1f,
    0x8f, 0xc7, 0xe3, 0xff, 0xff, 0xe0,

    /* U+003A ":" */
    0xff, 0x80, 0x7, 0xfc,

    /* U+003B ";" */
    0xff, 0x80, 0x7, 0xfb, 0x60,

    /* U+003C "<" */
    0x0, 0xc, 0x38, 0xf7, 0x9e, 0x3c, 0x3c, 0x1e,
    0x1c, 0x18, 0x0,

    /* U+003D "=" */
    0xff, 0xff, 0xc0, 0x0, 0x0, 0x7, 0xff, 0xfe,

    /* U+003E ">" */
    0x1, 0x83, 0x87, 0xc3, 0xc3, 0xc7, 0x9e, 0xf1,
    0xc3, 0x0, 0x0,

    /* U+003F "?" */
    0xff, 0xff, 0xf8, 0xfc, 0x7e, 0x3f, 0x1c, 0x1e,
    0x1e, 0x1e, 0xe, 0xe, 0x7, 0x3, 0x81, 0xc0,
    0x0, 0x0, 0x38, 0x1c, 0xe, 0x0,

    /* U+0040 "@" */
    0xff, 0xf8, 0x1, 0x80, 0x18, 0x1, 0x9f, 0x99,
    0xf9, 0x99, 0x98, 0x19, 0x9f, 0x99, 0xf9, 0x99,
    0x99, 0x99, 0x99, 0x99, 0xff, 0x9f, 0xf8, 0x0,
    0x80, 0x8, 0x0, 0xff, 0xf0,

    /* U+0041 "A" */
    0xe, 0x3, 0x81, 0xe0, 0x7c, 0x1f, 0x6, 0xc1,
    0xb0, 0x6c, 0x3b, 0xe, 0xe3, 0x38, 0xc6, 0x31,
    0x9f, 0xe7, 0xfd, 0xc7, 0x71, 0xd8, 0x76, 0xc,

    /* U+0042 "B" */
    0xff, 0xff, 0xf8, 0xfc, 0x7e, 0x3f, 0x1f, 0x8f,
    0xcf, 0xff, 0x7f, 0xb9, 0xfc, 0x7e, 0x3f, 0x1f,
    0x8f, 0xc7, 0xe3, 0xff, 0xff, 0xe0,

    /* U+0043 "C" */
    0xff, 0xff, 0xf8, 0xfc, 0x7e, 0x3f, 0x1f, 0x81,
    0xc0, 0xe0, 0x70, 0x38, 0x1c, 0xe, 0x7, 0x1f,
    0x8f, 0xc7, 0xe3, 0xff, 0xff, 0xe0,

    /* U+0044 "D" */
    0xfe, 0x7f, 0xb8, 0xfc, 0x7e, 0x3f, 0x1f, 0x8f,
    0xc7, 0xe3, 0xf1, 0xf8, 0xfc, 0x7e, 0x3f, 0x1f,
    0x8f, 0xc7, 0xe3, 0xff, 0xbf, 0x80,

    /* U+0045 "E" */
    0xff, 0xff, 0xe0, 0xe0, 0xe0, 0xe0, 0xe0, 0xe0,
    0xe0, 0xfe, 0xfe, 0xe0, 0xe0, 0xe0, 0xe0, 0xe0,
    0xe0, 0xff, 0xff,

    /* U+0046 "F" */
    0xff, 0xff, 0xe0, 0xe0, 0xe0, 0xe0, 0xe0, 0xe0,
    0xe0, 0xfe, 0xfe, 0xe0, 0xe0, 0xe0, 0xe0, 0xe0,
    0xe0, 0xe0, 0xe0,

    /* U+0047 "G" */
    0xff, 0xff, 0xf8, 0xfc, 0x7e, 0x3f, 0x1f, 0x81,
    0xc0, 0xe0, 0x77, 0xfb, 0xfc, 0x7e, 0x3f, 0x1f,
    0x8f, 0xc7, 0xe3, 0xff, 0xff, 0xe0,

    /* U+0048 "H" */
    0xe3, 0xf1, 0xf8, 0xfc, 0x7e, 0x3f, 0x1f, 0x8f,
    0xc7, 0xff, 0xff, 0xf8, 0xfc, 0x7e, 0x3f, 0x1f,
    0x8f, 0xc7, 0xe3, 0xf1, 0xf8, 0xe0,

    /* U+0049 "I" */
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x80,

    /* U+004A "J" */
    0x3, 0x81, 0xc0, 0xe0, 0x70, 0x38, 0x1c, 0xe,
    0x7, 0x3, 0x81, 0xc0, 0xe0, 0x70, 0x3f, 0x1f,
    0x8f, 0xc7, 0xe3, 0xff, 0xff, 0xe0,

    /* U+004B "K" */
    0xe3, 0xb9, 0xce, 0x73, 0x9c, 0xee, 0x3b, 0x8e,
    0xc3, 0xf0, 0xfc, 0x3e, 0xf, 0xc3, 0xf0, 0xee,
    0x3b, 0x8e, 0x73, 0x9c, 0xe7, 0x38, 0xee, 0x38,

    /* U+004C "L" */
    0xe1, 0xc3, 0x87, 0xe, 0x1c, 0x38, 0x70, 0xe1,
    0xc3, 0x87, 0xe, 0x1c, 0x38, 0x70, 0xe1, 0xff,
    0xf8,

    /* U+004D "M" */
    0xe0, 0xfc, 0x1f, 0xc7, 0xf8, 0xff, 0x1f, 0xe3,
    0xfe, 0xff, 0xdf, 0xfb, 0xff, 0x7f, 0xef, 0xf7,
    0x7e, 0xef, 0xdd, 0xfb, 0xbf, 0x37, 0xe4, 0xfc,
    0x1f, 0x83, 0x80,

    /* U+004E "N" */
    0xe3, 0xf1, 0xf8, 0xfe, 0x7f, 0x3f, 0x9f, 0xef,
    0xf7, 0xfb, 0xf7, 0xfb, 0xfd, 0xfe, 0xff, 0x3f,
    0x9f, 0xcf, 0xe3, 0xf1, 0xf8, 0xe0,

    /* U+004F "O" */
    0xff, 0xff, 0xf8, 0xfc, 0x7e, 0x3f, 0x1f, 0x8f,
    0xc7, 0xe3, 0xf1, 0xf8, 0xfc, 0x7e, 0x3f, 0x1f,
    0x8f, 0xc7, 0xe3, 0xff, 0xff, 0xe0,

    /* U+0050 "P" */
    0xff, 0xff, 0xf8, 0xfc, 0x7e, 0x3f, 0x1f, 0x8f,
    0xc7, 0xe3, 0xf1, 0xff, 0xff, 0xfe, 0x7, 0x3,
    0x81, 0xc0, 0xe0, 0x70, 0x38, 0x0,

    /* U+0051 "Q" */
    0xff, 0xbf, 0xee, 0x3b, 0x8e, 0xe3, 0xb8, 0xee,
    0x3b, 0x8e, 0xe3, 0xb8, 0xee, 0x3b, 0x8e, 0xe3,
    0xb9, 0xee, 0x7b, 0x9e, 0xe3, 0xff, 0xff, 0xfc,

    /* U+0052 "R" */
    0xff, 0xff, 0xf8, 0xfc, 0x7e, 0x3f, 0x1f, 0x8f,
    0xc7, 0xef, 0xf7, 0xfb, 0x1d, 0xce, 0xe7, 0x33,
    0x9d, 0xce, 0xe3, 0xf1, 0xf8, 0xe0,

    /* U+0053 "S" */
    0xff, 0xff, 0xf8, 0xfc, 0x7e, 0x3f, 0x1f, 0xc1,
    0xf0, 0x7c, 0x1f, 0x7, 0xc1, 0xf0, 0x7f, 0x1f,
    0x8f, 0xc7, 0xe3, 0xff, 0xff, 0xe0,

    /* U+0054 "T" */
    0xff, 0xff, 0xc7, 0x3, 0x81, 0xc0, 0xe0, 0x70,
    0x38, 0x1c, 0xe, 0x7, 0x3, 0x81, 0xc0, 0xe0,
    0x70, 0x38, 0x1c, 0xe, 0x7, 0x0,

    /* U+0055 "U" */
    0xe3, 0xf1, 0xf8, 0xfc, 0x7e, 0x3f, 0x1f, 0x8f,
    0xc7, 0xe3, 0xf1, 0xf8, 0xfc, 0x7e, 0x3f, 0x1f,
    0x8f, 0xc7, 0xe3, 0xff, 0xff, 0xe0,

    /* U+0056 "V" */
    0xe1, 0xf8, 0x76, 0x19, 0x86, 0x73, 0x9c, 0xe7,
    0x38, 0xce, 0x33, 0xc, 0xc3, 0xf0, 0xfc, 0x3f,
    0x7, 0x81, 0xe0, 0x78, 0x1e, 0x7, 0x80, 0xc0,

    /* U+0057 "W" */
    0xe3, 0x1f, 0x8c, 0x7e, 0x71, 0xb9, 0xc6, 0xe7,
    0xb9, 0x9e, 0xe6, 0x7b, 0x9d, 0xee, 0x7f, 0xb9,
    0xfe, 0xe7, 0xfb, 0x1f, 0x7c, 0x3c, 0xf0, 0xf3,
    0xc3, 0xcf, 0xf, 0x3c, 0x3c, 0xf0, 0xe3, 0x83,
    0x8e, 0x0,

    /* U+0058 "X" */
    0x71, 0xdc, 0x77, 0x38, 0xce, 0x3b, 0xf, 0xc1,
    0xf0, 0x78, 0x1e, 0x3, 0x81, 0xe0, 0x7c, 0x1f,
    0xe, 0xc3, 0xb8, 0xce, 0x71, 0x9c, 0x7e, 0x1c,

    /* U+0059 "Y" */
    0x60, 0xee, 0x39, 0xc7, 0x38, 0xe3, 0x18, 0x77,
    0xe, 0xe0, 0xd8, 0x1f, 0x3, 0xe0, 0x38, 0x7,
    0x0, 0xe0, 0x1c, 0x3, 0x80, 0x70, 0xe, 0x1,
    0xc0, 0x38, 0x0,

    /* U+005A "Z" */
    0xff, 0xff, 0x7, 0xe, 0xe, 0xe, 0x1c, 0x1c,
    0x1c, 0x38, 0x38, 0x38, 0x30, 0x70, 0x70, 0x60,
    0xe0, 0xff, 0xff,

    /* U+005B "[" */
    0xff, 0xf9, 0xce, 0x73, 0x9c, 0xe7, 0x39, 0xce,
    0x73, 0x9c, 0xe7, 0x39, 0xff, 0x80,

    /* U+005C "\\" */
    0x60, 0x1c, 0x7, 0x0, 0xc0, 0x38, 0x6, 0x1,
    0x80, 0x70, 0xc, 0x3, 0x80, 0xe0, 0x18, 0x7,
    0x0, 0xc0, 0x30, 0xe, 0x1, 0x80, 0x70, 0xc,

    /* U+005D "]" */
    0xff, 0xf1, 0xc7, 0x1c, 0x71, 0xc7, 0x1c, 0x71,
    0xc7, 0x1c, 0x71, 0xc7, 0x1c, 0x71, 0xff, 0xfc,

    /* U+005E "^" */
    0x18, 0x3c, 0x3c, 0x7e, 0x66, 0x66, 0xc3,

    /* U+005F "_" */
    0xff, 0xff, 0xf0,

    /* U+0060 "`" */
    0xe3, 0x86,

    /* U+0061 "a" */
    0xff, 0xff, 0xe7, 0xe7, 0x7, 0xff, 0xff, 0xe7,
    0xe7, 0xe7, 0xff, 0xff,

    /* U+0062 "b" */
    0xe0, 0xe0, 0xe0, 0xe0, 0xe0, 0xe0, 0xe0, 0xff,
    0xff, 0xe7, 0xe7, 0xe7, 0xe7, 0xe7, 0xe7, 0xe7,
    0xe7, 0xff, 0xfe,

    /* U+0063 "c" */
    0xff, 0xff, 0xe7, 0xe7, 0xe0, 0xe0, 0xe0, 0xe0,
    0xe7, 0xe7, 0xff, 0xff,

    /* U+0064 "d" */
    0x7, 0x7, 0x7, 0x7, 0x7, 0x7, 0x7, 0x7f,
    0xff, 0xe7, 0xe7, 0xe7, 0xe7, 0xe7, 0xe7, 0xe7,
    0xe7, 0xff, 0x7f,

    /* U+0065 "e" */
    0xff, 0xff, 0xe7, 0xe7, 0xe7, 0xff, 0xff, 0xe0,
    0xe7, 0xe7, 0xff, 0xff,

    /* U+0066 "f" */
    0x7d, 0xf7, 0x1c, 0x71, 0xc7, 0x3f, 0xfd, 0xc7,
    0x1c, 0x71, 0xc7, 0x1c, 0x71, 0xc7, 0x0,

    /* U+0067 "g" */
    0x7f, 0xff, 0xe7, 0xe7, 0xe7, 0xe7, 0xe7, 0xe7,
    0xe7, 0xe7, 0xff, 0x7f, 0x7, 0xe7, 0xff, 0xff,

    /* U+0068 "h" */
    0xe0, 0xe0, 0xe0, 0xe0, 0xe0, 0xe0, 0xe0, 0xff,
    0xff, 0xe7, 0xe7, 0xe7, 0xe7, 0xe7, 0xe7, 0xe7,
    0xe7, 0xe7, 0xe7,

    /* U+0069 "i" */
    0xff, 0x81, 0xff, 0xff, 0xff, 0xff, 0xe0,

    /* U+006A "j" */
    0x39, 0xce, 0x0, 0x1c, 0xe7, 0x39, 0xce, 0x73,
    0x9c, 0xe7, 0x39, 0xcf, 0xff, 0x80,

    /* U+006B "k" */
    0xe0, 0xe0, 0xe0, 0xe0, 0xe0, 0xe0, 0xe0, 0xe7,
    0xee, 0xee, 0xfc, 0xfc, 0xf8, 0xfc, 0xfc, 0xee,
    0xee, 0xe7, 0xe7,

    /* U+006C "l" */
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x80,

    /* U+006D "m" */
    0xff, 0xff, 0xff, 0xf9, 0xcf, 0xce, 0x7e, 0x73,
    0xf3, 0x9f, 0x9c, 0xfc, 0xe7, 0xe7, 0x3f, 0x39,
    0xf9, 0xcf, 0xce, 0x70,

    /* U+006E "n" */
    0xff, 0xff, 0xe7, 0xe7, 0xe7, 0xe7, 0xe7, 0xe7,
    0xe7, 0xe7, 0xe7, 0xe7,

    /* U+006F "o" */
    0xff, 0xff, 0xe7, 0xe7, 0xe7, 0xe7, 0xe7, 0xe7,
    0xe7, 0xe7, 0xff, 0xff,

    /* U+0070 "p" */
    0xff, 0xff, 0xe7, 0xe7, 0xe7, 0xe7, 0xe7, 0xe7,
    0xe7, 0xe7, 0xff, 0xfe, 0xe0, 0xe0, 0xe0, 0xe0,

    /* U+0071 "q" */
    0x7f, 0xff, 0xe7, 0xe7, 0xe7, 0xe7, 0xe7, 0xe7,
    0xe7, 0xe7, 0xff, 0x7f, 0x7, 0x7, 0x7, 0x7,

    /* U+0072 "r" */
    0xff, 0xff, 0xe7, 0xe7, 0xe7, 0xe0, 0xe0, 0xe0,
    0xe0, 0xe0, 0xe0, 0xe0,

    /* U+0073 "s" */
    0xff, 0xff, 0xe7, 0xe7, 0xf0, 0x7c, 0x3f, 0xf,
    0xe7, 0xe7, 0xff, 0xff,

    /* U+0074 "t" */
    0x71, 0xc7, 0x1c, 0xff, 0xf7, 0x1c, 0x71, 0xc7,
    0x1c, 0x71, 0xc7, 0xdf,

    /* U+0075 "u" */
    0xe7, 0xe7, 0xe7, 0xe7, 0xe7, 0xe7, 0xe7, 0xe7,
    0xe7, 0xe7, 0xff, 0xff,

    /* U+0076 "v" */
    0xe3, 0xb1, 0xdc, 0xce, 0x67, 0x71, 0xb8, 0xd8,
    0x7c, 0x1e, 0xf, 0x7, 0x3, 0x80,

    /* U+0077 "w" */
    0xe7, 0x3b, 0x39, 0x99, 0xcc, 0xee, 0x67, 0x77,
    0x1a, 0xb8, 0xf7, 0x87, 0xbc, 0x3d, 0xe1, 0xef,
    0x7, 0x70, 0x31, 0x80,

    /* U+0078 "x" */
    0x63, 0x3b, 0x8d, 0xc7, 0xc3, 0xe0, 0xe0, 0x70,
    0x7c, 0x36, 0x3b, 0x9d, 0xdc, 0x70,

    /* U+0079 "y" */
    0xe3, 0xb1, 0xdc, 0xce, 0x63, 0x71, 0xb8, 0xd8,
    0x7c, 0x1e, 0xf, 0x7, 0x3, 0x81, 0xc0, 0xe0,
    0x60, 0x70,

    /* U+007A "z" */
    0xff, 0xfc, 0x78, 0xe1, 0xc7, 0xe, 0x38, 0x71,
    0xc3, 0xff, 0xf0,

    /* U+007B "{" */
    0xf, 0x1f, 0x1c, 0x1c, 0x1c, 0x1c, 0x1c, 0x1c,
    0x3c, 0x78, 0x70, 0x38, 0x1c, 0x1c, 0x1c, 0x1c,
    0x1c, 0x1c, 0x1c, 0x1f, 0xf,

    /* U+007C "|" */
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xf8,

    /* U+007D "}" */
    0xf8, 0xfc, 0x1c, 0x1c, 0x1c, 0x1c, 0x1c, 0x1c,
    0x1e, 0xf, 0x7, 0x1e, 0x1c, 0x1c, 0x1c, 0x1c,
    0x1c, 0x1c, 0x1c, 0xfc, 0xf8,

    /* U+007E "~" */
    0xe3, 0xfd, 0xfb, 0xfc, 0x70
};


/*---------------------
 *  GLYPH DESCRIPTION
 *--------------------*/

static const lv_font_fmt_txt_glyph_dsc_t glyph_dsc[] = {
    {.bitmap_index = 0, .adv_w = 0, .box_w = 0, .box_h = 0, .ofs_x = 0, .ofs_y = 0} /* id = 0 reserved */,
    {.bitmap_index = 0, .adv_w = 79, .box_w = 1, .box_h = 1, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 1, .adv_w = 82, .box_w = 3, .box_h = 19, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 9, .adv_w = 133, .box_w = 6, .box_h = 6, .ofs_x = 1, .ofs_y = 13},
    {.bitmap_index = 14, .adv_w = 208, .box_w = 12, .box_h = 19, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 43, .adv_w = 176, .box_w = 9, .box_h = 22, .ofs_x = 1, .ofs_y = -2},
    {.bitmap_index = 68, .adv_w = 252, .box_w = 15, .box_h = 19, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 104, .adv_w = 187, .box_w = 10, .box_h = 19, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 128, .adv_w = 71, .box_w = 3, .box_h = 6, .ofs_x = 1, .ofs_y = 13},
    {.bitmap_index = 131, .adv_w = 117, .box_w = 6, .box_h = 21, .ofs_x = 1, .ofs_y = -2},
    {.bitmap_index = 147, .adv_w = 117, .box_w = 6, .box_h = 21, .ofs_x = 0, .ofs_y = -2},
    {.bitmap_index = 163, .adv_w = 176, .box_w = 10, .box_h = 9, .ofs_x = 1, .ofs_y = 10},
    {.bitmap_index = 175, .adv_w = 182, .box_w = 10, .box_h = 9, .ofs_x = 1, .ofs_y = 3},
    {.bitmap_index = 187, .adv_w = 71, .box_w = 3, .box_h = 5, .ofs_x = 1, .ofs_y = -2},
    {.bitmap_index = 189, .adv_w = 118, .box_w = 6, .box_h = 2, .ofs_x = 1, .ofs_y = 6},
    {.bitmap_index = 191, .adv_w = 71, .box_w = 3, .box_h = 3, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 193, .adv_w = 170, .box_w = 10, .box_h = 19, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 217, .adv_w = 179, .box_w = 9, .box_h = 19, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 239, .adv_w = 90, .box_w = 4, .box_h = 19, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 249, .adv_w = 169, .box_w = 9, .box_h = 19, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 271, .adv_w = 177, .box_w = 9, .box_h = 19, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 293, .adv_w = 175, .box_w = 10, .box_h = 19, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 317, .adv_w = 174, .box_w = 9, .box_h = 19, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 339, .adv_w = 177, .box_w = 9, .box_h = 19, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 361, .adv_w = 166, .box_w = 9, .box_h = 19, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 383, .adv_w = 180, .box_w = 9, .box_h = 19, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 405, .adv_w = 177, .box_w = 9, .box_h = 19, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 427, .adv_w = 71, .box_w = 3, .box_h = 10, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 431, .adv_w = 71, .box_w = 3, .box_h = 12, .ofs_x = 1, .ofs_y = -2},
    {.bitmap_index = 436, .adv_w = 140, .box_w = 7, .box_h = 12, .ofs_x = 1, .ofs_y = 2},
    {.bitmap_index = 447, .adv_w = 179, .box_w = 9, .box_h = 7, .ofs_x = 1, .ofs_y = 4},
    {.bitmap_index = 455, .adv_w = 140, .box_w = 7, .box_h = 12, .ofs_x = 1, .ofs_y = 2},
    {.bitmap_index = 466, .adv_w = 163, .box_w = 9, .box_h = 19, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 488, .adv_w = 224, .box_w = 12, .box_h = 19, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 517, .adv_w = 173, .box_w = 10, .box_h = 19, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 541, .adv_w = 179, .box_w = 9, .box_h = 19, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 563, .adv_w = 179, .box_w = 9, .box_h = 19, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 585, .adv_w = 180, .box_w = 9, .box_h = 19, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 607, .adv_w = 152, .box_w = 8, .box_h = 19, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 626, .adv_w = 145, .box_w = 8, .box_h = 19, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 645, .adv_w = 180, .box_w = 9, .box_h = 19, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 667, .adv_w = 182, .box_w = 9, .box_h = 19, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 689, .adv_w = 81, .box_w = 3, .box_h = 19, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 697, .adv_w = 167, .box_w = 9, .box_h = 19, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 719, .adv_w = 166, .box_w = 10, .box_h = 19, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 743, .adv_w = 141, .box_w = 7, .box_h = 19, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 760, .adv_w = 212, .box_w = 11, .box_h = 19, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 787, .adv_w = 184, .box_w = 9, .box_h = 19, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 809, .adv_w = 183, .box_w = 9, .box_h = 19, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 831, .adv_w = 173, .box_w = 9, .box_h = 19, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 853, .adv_w = 189, .box_w = 10, .box_h = 19, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 877, .adv_w = 177, .box_w = 9, .box_h = 19, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 899, .adv_w = 176, .box_w = 9, .box_h = 19, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 921, .adv_w = 150, .box_w = 9, .box_h = 19, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 943, .adv_w = 182, .box_w = 9, .box_h = 19, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 965, .adv_w = 171, .box_w = 10, .box_h = 19, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 989, .adv_w = 241, .box_w = 14, .box_h = 19, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 1023, .adv_w = 169, .box_w = 10, .box_h = 19, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 1047, .adv_w = 170, .box_w = 11, .box_h = 19, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 1074, .adv_w = 150, .box_w = 8, .box_h = 19, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 1093, .adv_w = 113, .box_w = 5, .box_h = 21, .ofs_x = 1, .ofs_y = -2},
    {.bitmap_index = 1107, .adv_w = 170, .box_w = 10, .box_h = 19, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 1131, .adv_w = 113, .box_w = 6, .box_h = 21, .ofs_x = 0, .ofs_y = -2},
    {.bitmap_index = 1147, .adv_w = 160, .box_w = 8, .box_h = 7, .ofs_x = 1, .ofs_y = 12},
    {.bitmap_index = 1154, .adv_w = 152, .box_w = 10, .box_h = 2, .ofs_x = 0, .ofs_y = -5},
    {.bitmap_index = 1157, .adv_w = 157, .box_w = 5, .box_h = 3, .ofs_x = 3, .ofs_y = 13},
    {.bitmap_index = 1159, .adv_w = 153, .box_w = 8, .box_h = 12, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 1171, .adv_w = 157, .box_w = 8, .box_h = 19, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 1190, .adv_w = 153, .box_w = 8, .box_h = 12, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 1202, .adv_w = 157, .box_w = 8, .box_h = 19, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 1221, .adv_w = 154, .box_w = 8, .box_h = 12, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 1233, .adv_w = 99, .box_w = 6, .box_h = 19, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 1248, .adv_w = 157, .box_w = 8, .box_h = 16, .ofs_x = 1, .ofs_y = -4},
    {.bitmap_index = 1264, .adv_w = 157, .box_w = 8, .box_h = 19, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 1283, .adv_w = 75, .box_w = 3, .box_h = 17, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 1290, .adv_w = 75, .box_w = 5, .box_h = 21, .ofs_x = -1, .ofs_y = -4},
    {.bitmap_index = 1304, .adv_w = 145, .box_w = 8, .box_h = 19, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 1323, .adv_w = 75, .box_w = 3, .box_h = 19, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 1331, .adv_w = 240, .box_w = 13, .box_h = 12, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 1351, .adv_w = 157, .box_w = 8, .box_h = 12, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 1363, .adv_w = 156, .box_w = 8, .box_h = 12, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 1375, .adv_w = 157, .box_w = 8, .box_h = 16, .ofs_x = 1, .ofs_y = -4},
    {.bitmap_index = 1391, .adv_w = 157, .box_w = 8, .box_h = 16, .ofs_x = 1, .ofs_y = -4},
    {.bitmap_index = 1407, .adv_w = 144, .box_w = 8, .box_h = 12, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 1419, .adv_w = 149, .box_w = 8, .box_h = 12, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 1431, .adv_w = 106, .box_w = 6, .box_h = 16, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 1443, .adv_w = 157, .box_w = 8, .box_h = 12, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 1455, .adv_w = 149, .box_w = 9, .box_h = 12, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 1469, .adv_w = 209, .box_w = 13, .box_h = 12, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 1489, .adv_w = 146, .box_w = 9, .box_h = 12, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 1503, .adv_w = 149, .box_w = 9, .box_h = 16, .ofs_x = 0, .ofs_y = -4},
    {.bitmap_index = 1521, .adv_w = 133, .box_w = 7, .box_h = 12, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 1532, .adv_w = 143, .box_w = 8, .box_h = 21, .ofs_x = 0, .ofs_y = -2},
    {.bitmap_index = 1553, .adv_w = 81, .box_w = 3, .box_h = 23, .ofs_x = 1, .ofs_y = -4},
    {.bitmap_index = 1562, .adv_w = 143, .box_w = 8, .box_h = 21, .ofs_x = 0, .ofs_y = -2},
    {.bitmap_index = 1583, .adv_w = 194, .box_w = 9, .box_h = 4, .ofs_x = 2, .ofs_y = 5}
};

/*---------------------
 *  CHARACTER MAPPING
 *--------------------*/



/*Collect the unicode lists and glyph_id offsets*/
static const lv_font_fmt_txt_cmap_t cmaps[] =
{
    {
        .range_start = 32, .range_length = 95, .glyph_id_start = 1,
        .unicode_list = NULL, .glyph_id_ofs_list = NULL, .list_length = 0, .type = LV_FONT_FMT_TXT_CMAP_FORMAT0_TINY
    }
};



/*--------------------
 *  ALL CUSTOM DATA
 *--------------------*/

#if LV_VERSION_CHECK(8, 0, 0)
/*Store all the custom data of the font*/
static  lv_font_fmt_txt_glyph_cache_t cache;
static const lv_font_fmt_txt_dsc_t font_dsc = {
#else
static lv_font_fmt_txt_dsc_t font_dsc = {
#endif
    .glyph_bitmap = glyph_bitmap,
    .glyph_dsc = glyph_dsc,
    .cmaps = cmaps,
    .kern_dsc = NULL,
    .kern_scale = 0,
    .cmap_num = 1,
    .bpp = 1,
    .kern_classes = 0,
    .bitmap_format = 0,
#if LV_VERSION_CHECK(8, 0, 0)
    .cache = &cache
#endif
};


/*-----------------
 *  PUBLIC FONT
 *----------------*/

/*Initialize a public general font descriptor*/
#if LV_VERSION_CHECK(8, 0, 0)
const lv_font_t ui_font_micro = {
#else
lv_font_t ui_font_micro = {
#endif
    .get_glyph_dsc = lv_font_get_glyph_dsc_fmt_txt,    /*Function pointer to get glyph's data*/
    .get_glyph_bitmap = lv_font_get_bitmap_fmt_txt,    /*Function pointer to get glyph's bitmap*/
    .line_height = 25,          /*The maximum line height required by the font*/
    .base_line = 5,             /*Baseline measured from the bottom of the line*/
#if !(LVGL_VERSION_MAJOR == 6 && LVGL_VERSION_MINOR == 0)
    .subpx = LV_FONT_SUBPX_NONE,
#endif
#if LV_VERSION_CHECK(7, 4, 0) || LVGL_VERSION_MAJOR >= 8
    .underline_position = -3,
    .underline_thickness = 1,
#endif
    .dsc = &font_dsc           /*The custom font data. Will be accessed by `get_glyph_bitmap/dsc` */
};



#endif /*#if UI_FONT_MICRO*/

