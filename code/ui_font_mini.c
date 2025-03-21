/*******************************************************************************
 * Size: 32 px
 * Bpp: 1
 * Opts: --bpp 1 --size 32 --font C:\Users\Danko\Desktop\AMOLED18\timeExample\assets\AGENCYR.TTF -o C:\Users\Danko\Desktop\AMOLED18\timeExample\assets\ui_font_mini.c --format lvgl -r 0x20-0x7f --no-compress --no-prefilter
 ******************************************************************************/

#include "ui.h"

#ifndef UI_FONT_MINI
#define UI_FONT_MINI 1
#endif

#if UI_FONT_MINI

/*-----------------
 *    BITMAPS
 *----------------*/

/*Store the image of the glyphs*/
static LV_ATTRIBUTE_LARGE_CONST const uint8_t glyph_bitmap[] = {
    /* U+0020 " " */
    0x0,

    /* U+0021 "!" */
    0xff, 0xff, 0xff, 0xff, 0xf0, 0xf,

    /* U+0022 "\"" */
    0xcf, 0x3c, 0xf3, 0xcf, 0x3c, 0xc0,

    /* U+0023 "#" */
    0x3, 0x1c, 0xe, 0x30, 0x18, 0x60, 0x30, 0xc0,
    0x61, 0x80, 0xc7, 0x1f, 0xff, 0xbf, 0xff, 0xc,
    0x30, 0x18, 0x60, 0x30, 0xc0, 0x63, 0x0, 0xc6,
    0x3, 0xc, 0x6, 0x18, 0xc, 0x30, 0xff, 0xfd,
    0xff, 0xf8, 0xe3, 0x1, 0x86, 0x3, 0xc, 0x6,
    0x18, 0xc, 0x70, 0x18, 0xc0,

    /* U+0024 "$" */
    0xc, 0x6, 0x1f, 0xff, 0xfc, 0xde, 0x6f, 0x37,
    0x9b, 0xcd, 0xe6, 0x1b, 0x7, 0x81, 0xc0, 0xe0,
    0x38, 0x1c, 0xf, 0x6, 0xf3, 0x79, 0xbc, 0xde,
    0x6f, 0x37, 0x9b, 0xff, 0xbf, 0xc3, 0x1, 0x80,

    /* U+0025 "%" */
    0x7f, 0x1, 0x9f, 0xe0, 0x63, 0xc, 0xc, 0x61,
    0x83, 0xc, 0x30, 0x61, 0x86, 0x18, 0x30, 0xc6,
    0x6, 0x18, 0xc0, 0xc3, 0x30, 0x18, 0x66, 0x3,
    0xfd, 0x80, 0x7f, 0xb0, 0x0, 0xd, 0xfe, 0x1,
    0xbf, 0xc0, 0x66, 0x18, 0xc, 0xc3, 0x3, 0x18,
    0x60, 0xc3, 0xc, 0x18, 0x61, 0x86, 0xc, 0x30,
    0xc1, 0x86, 0x30, 0x30, 0xc6, 0x7, 0xf9, 0x80,
    0xff,

    /* U+0026 "&" */
    0xff, 0xcf, 0xfc, 0xc0, 0xcc, 0xc, 0xc0, 0xcc,
    0xc, 0xc0, 0xc, 0x0, 0xc0, 0xf, 0x0, 0x3f,
    0xf3, 0xff, 0xf0, 0xcc, 0xc, 0xc0, 0xcc, 0xc,
    0xc0, 0xcc, 0xc, 0xc0, 0xcc, 0xc, 0xc0, 0xcc,
    0xc, 0xff, 0xcf, 0xfc,

    /* U+0027 "'" */
    0xff, 0xfc,

    /* U+0028 "(" */
    0x18, 0x63, 0xc, 0x31, 0x86, 0x18, 0xc3, 0xc,
    0x30, 0xc3, 0xc, 0x30, 0xc3, 0xc, 0x18, 0x61,
    0x86, 0xc, 0x30, 0x61, 0x80,

    /* U+0029 ")" */
    0x61, 0x83, 0xc, 0x10, 0x61, 0x86, 0xc, 0x30,
    0xc3, 0xc, 0x30, 0xc3, 0xc, 0x30, 0xc6, 0x18,
    0x63, 0xc, 0x31, 0x86, 0x0,

    /* U+002A "*" */
    0x18, 0x4, 0x32, 0x7d, 0x73, 0xe1, 0xf3, 0xaf,
    0x93, 0x8, 0xc, 0x0,

    /* U+002B "+" */
    0xc, 0x1, 0x80, 0x30, 0x6, 0xf, 0xff, 0xff,
    0xc3, 0x0, 0x60, 0xc, 0x1, 0x80,

    /* U+002C "," */
    0x6f, 0x60,

    /* U+002D "-" */
    0xff, 0xf0,

    /* U+002E "." */
    0xf0,

    /* U+002F "/" */
    0x0, 0x30, 0x7, 0x0, 0x60, 0x6, 0x0, 0xc0,
    0xc, 0x1, 0x80, 0x18, 0x3, 0x80, 0x30, 0x7,
    0x0, 0x60, 0x6, 0x0, 0xc0, 0xc, 0x1, 0x80,
    0x18, 0x3, 0x80, 0x30, 0x3, 0x0, 0x60, 0x6,
    0x0, 0xc0, 0xc, 0x0,

    /* U+0030 "0" */
    0xff, 0xff, 0xfc, 0xf, 0x3, 0xc0, 0xf0, 0x3c,
    0xf, 0x3, 0xc0, 0xf0, 0x3c, 0xf, 0x3, 0xc0,
    0xf0, 0x3c, 0xf, 0x3, 0xc0, 0xf0, 0x3c, 0xf,
    0x3, 0xc0, 0xf0, 0x3f, 0xff, 0xff,

    /* U+0031 "1" */
    0x6d, 0xb7, 0xfb, 0x6d, 0xb6, 0xdb, 0x6d, 0xb6,
    0xdb,

    /* U+0032 "2" */
    0xff, 0xff, 0xfc, 0xf, 0x3, 0xc0, 0xf0, 0x3c,
    0xc, 0x3, 0x0, 0xc0, 0x60, 0x30, 0xc, 0x6,
    0x1, 0x80, 0xc0, 0x30, 0x18, 0x6, 0x3, 0x0,
    0xc0, 0x60, 0x10, 0xf, 0xff, 0xff,

    /* U+0033 "3" */
    0xff, 0xff, 0xfc, 0xf, 0x3, 0xc0, 0xf0, 0x3c,
    0xc, 0x3, 0x1, 0xc0, 0xe0, 0xe0, 0x70, 0x7,
    0x0, 0xe0, 0xc, 0x3, 0xc0, 0xf0, 0x3c, 0xf,
    0x3, 0xc0, 0xf0, 0x3f, 0xff, 0xff,

    /* U+0034 "4" */
    0x3, 0x0, 0xe0, 0x18, 0x3, 0x0, 0xc0, 0x18,
    0x3, 0x0, 0xcc, 0x19, 0x83, 0x30, 0xc6, 0x18,
    0xc3, 0x18, 0xc3, 0x18, 0x67, 0xc, 0xff, 0xff,
    0xfc, 0x6, 0x0, 0xc0, 0x18, 0x3, 0x0, 0x60,
    0xc,

    /* U+0035 "5" */
    0xff, 0xff, 0xf0, 0x18, 0xc, 0x6, 0x3, 0x1,
    0x80, 0xc0, 0x7f, 0xff, 0xe0, 0x30, 0x18, 0xc,
    0x6, 0x3, 0xc1, 0xe0, 0xf0, 0x78, 0x3c, 0x1e,
    0xf, 0xff, 0xff,

    /* U+0036 "6" */
    0xff, 0xff, 0xfc, 0xf, 0x3, 0xc0, 0xf0, 0x3c,
    0xf, 0x0, 0xc0, 0x30, 0xc, 0x3, 0xff, 0xff,
    0xf0, 0x3c, 0xf, 0x3, 0xc0, 0xf0, 0x3c, 0xf,
    0x3, 0xc0, 0xf0, 0x3f, 0xff, 0xff,

    /* U+0037 "7" */
    0xff, 0xff, 0xec, 0x1b, 0x6, 0xc1, 0xb0, 0xe0,
    0x30, 0xc, 0x3, 0x1, 0xc0, 0x60, 0x18, 0x6,
    0x1, 0x80, 0xc0, 0x30, 0xc, 0x3, 0x1, 0xc0,
    0x60, 0x18, 0x6, 0x3, 0x80, 0xc0,

    /* U+0038 "8" */
    0xff, 0xff, 0xfc, 0xf, 0x3, 0xc0, 0xf0, 0x3c,
    0xf, 0x3, 0xc0, 0xf8, 0x77, 0xf0, 0x78, 0x7f,
    0xb8, 0x7c, 0xf, 0x3, 0xc0, 0xf0, 0x3c, 0xf,
    0x3, 0xc0, 0xf0, 0x3f, 0xff, 0xff,

    /* U+0039 "9" */
    0xff, 0xff, 0xfc, 0xf, 0x3, 0xc0, 0xf0, 0x3c,
    0xf, 0x3, 0xc0, 0xf0, 0x3c, 0xf, 0xff, 0xff,
    0xc0, 0x30, 0xc, 0x3, 0x0, 0xf0, 0x3c, 0xf,
    0x3, 0xc0, 0xf0, 0x3f, 0xff, 0xff,

    /* U+003A ":" */
    0xf0, 0x0, 0x3, 0xc0,

    /* U+003B ";" */
    0xf0, 0x0, 0x3, 0xe8,

    /* U+003C "<" */
    0x1, 0x7, 0xe, 0x18, 0x70, 0xe0, 0xe0, 0x70,
    0x1c, 0xe, 0x7, 0x1,

    /* U+003D "=" */
    0xff, 0xff, 0xfc, 0x0, 0x0, 0x0, 0x0, 0x0,
    0x3f, 0xff, 0xff,

    /* U+003E ">" */
    0x80, 0xe0, 0x70, 0x1c, 0xe, 0x7, 0x7, 0xe,
    0x38, 0x70, 0xc0, 0x80,

    /* U+003F "?" */
    0xff, 0xff, 0xf0, 0x78, 0x3c, 0x1e, 0xf, 0x7,
    0x83, 0x1, 0x81, 0x80, 0xc0, 0xc0, 0x60, 0x60,
    0x30, 0x18, 0xc, 0x6, 0x3, 0x0, 0x0, 0x0,
    0x0, 0x30, 0x18,

    /* U+0040 "@" */
    0xff, 0xfe, 0x0, 0x18, 0x0, 0x60, 0x1, 0x80,
    0x6, 0x3f, 0x18, 0xfc, 0x63, 0x31, 0x8c, 0xc6,
    0x3, 0x18, 0xc, 0x63, 0xf1, 0x8f, 0xc6, 0x33,
    0x18, 0xcc, 0x63, 0x31, 0x8c, 0xc6, 0x33, 0x18,
    0xff, 0xe0, 0x0, 0x80, 0x2, 0x0, 0x8, 0x0,
    0x3f, 0xff,

    /* U+0041 "A" */
    0xe, 0x1, 0xc0, 0x38, 0x7, 0x0, 0xe0, 0x34,
    0x6, 0xc0, 0xd8, 0x1b, 0x2, 0x60, 0xcc, 0x18,
    0xc3, 0x18, 0x63, 0xc, 0x61, 0xc, 0x61, 0x8f,
    0xf9, 0xff, 0x30, 0x66, 0xd, 0xc1, 0xb0, 0x3e,
    0x3,

    /* U+0042 "B" */
    0xff, 0xff, 0xfc, 0xf, 0x3, 0xc0, 0xf0, 0x3c,
    0xf, 0x3, 0xc0, 0xf0, 0xff, 0xf3, 0xfc, 0xc3,
    0xf0, 0x3c, 0xf, 0x3, 0xc0, 0xf0, 0x3c, 0xf,
    0x3, 0xc0, 0xf0, 0x3f, 0xff, 0xff,

    /* U+0043 "C" */
    0xff, 0xff, 0xfc, 0xf, 0x3, 0xc0, 0xf0, 0x3c,
    0xf, 0x0, 0xc0, 0x30, 0xc, 0x3, 0x0, 0xc0,
    0x30, 0xc, 0x3, 0x0, 0xc0, 0xf0, 0x3c, 0xf,
    0x3, 0xc0, 0xf0, 0x3f, 0xff, 0xff,

    /* U+0044 "D" */
    0xff, 0x3f, 0xec, 0x1f, 0x3, 0xc0, 0xf0, 0x3c,
    0xf, 0x3, 0xc0, 0xf0, 0x3c, 0xf, 0x3, 0xc0,
    0xf0, 0x3c, 0xf, 0x3, 0xc0, 0xf0, 0x3c, 0xf,
    0x3, 0xc0, 0xf0, 0x7f, 0xfb, 0xfc,

    /* U+0045 "E" */
    0xff, 0xff, 0xc0, 0xc0, 0xc0, 0xc0, 0xc0, 0xc0,
    0xc0, 0xc0, 0xc0, 0xff, 0xff, 0xc0, 0xc0, 0xc0,
    0xc0, 0xc0, 0xc0, 0xc0, 0xc0, 0xc0, 0xff, 0xff,

    /* U+0046 "F" */
    0xff, 0xff, 0xc0, 0xc0, 0xc0, 0xc0, 0xc0, 0xc0,
    0xc0, 0xc0, 0xc0, 0xff, 0xff, 0xc0, 0xc0, 0xc0,
    0xc0, 0xc0, 0xc0, 0xc0, 0xc0, 0xc0, 0xc0, 0xc0,

    /* U+0047 "G" */
    0xff, 0xff, 0xfc, 0xf, 0x3, 0xc0, 0xf0, 0x3c,
    0xf, 0x0, 0xc0, 0x30, 0xc, 0x3, 0x0, 0xc7,
    0xf1, 0xfc, 0xf, 0x3, 0xc0, 0xf0, 0x3c, 0xf,
    0x3, 0xc0, 0xf0, 0x3f, 0xff, 0xff,

    /* U+0048 "H" */
    0xc0, 0xf0, 0x3c, 0xf, 0x3, 0xc0, 0xf0, 0x3c,
    0xf, 0x3, 0xc0, 0xf0, 0x3f, 0xff, 0xff, 0xc0,
    0xf0, 0x3c, 0xf, 0x3, 0xc0, 0xf0, 0x3c, 0xf,
    0x3, 0xc0, 0xf0, 0x3c, 0xf, 0x3,

    /* U+0049 "I" */
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff,

    /* U+004A "J" */
    0x1, 0x80, 0xc0, 0x60, 0x30, 0x18, 0xc, 0x6,
    0x3, 0x1, 0x80, 0xc0, 0x60, 0x30, 0x18, 0xc,
    0x6, 0x3, 0xc1, 0xe0, 0xf0, 0x78, 0x3c, 0x1e,
    0xf, 0xff, 0xff,

    /* U+004B "K" */
    0xc1, 0xb0, 0xec, 0x33, 0x1c, 0xc6, 0x31, 0x8c,
    0xc3, 0x30, 0xd8, 0x36, 0xf, 0x3, 0xc0, 0xf0,
    0x3e, 0xd, 0x83, 0x70, 0xcc, 0x33, 0x8c, 0x63,
    0x1c, 0xc3, 0x30, 0xec, 0x1b, 0x7,

    /* U+004C "L" */
    0xc0, 0xc0, 0xc0, 0xc0, 0xc0, 0xc0, 0xc0, 0xc0,
    0xc0, 0xc0, 0xc0, 0xc0, 0xc0, 0xc0, 0xc0, 0xc0,
    0xc0, 0xc0, 0xc0, 0xc0, 0xc0, 0xc0, 0xff, 0xff,

    /* U+004D "M" */
    0xc0, 0x3c, 0x3, 0xe0, 0x7e, 0x7, 0xe0, 0x7e,
    0x7, 0xf0, 0xff, 0xf, 0xf0, 0xff, 0xf, 0xd9,
    0xbd, 0x9b, 0xd9, 0xbc, 0x93, 0xcf, 0x3c, 0xf3,
    0xcf, 0x3c, 0x63, 0xc6, 0x3c, 0x63, 0xc6, 0x3c,
    0x3, 0xc0, 0x3c, 0x3,

    /* U+004E "N" */
    0xc0, 0xf0, 0x3e, 0xf, 0x83, 0xe0, 0xfc, 0x3f,
    0xf, 0xc3, 0xd8, 0xf6, 0x3c, 0x8f, 0x33, 0xcc,
    0xf1, 0xbc, 0x6f, 0x1b, 0xc3, 0xf0, 0xfc, 0x3f,
    0x7, 0xc1, 0xf0, 0x3c, 0xf, 0x3,

    /* U+004F "O" */
    0xff, 0xff, 0xfc, 0xf, 0x3, 0xc0, 0xf0, 0x3c,
    0xf, 0x3, 0xc0, 0xf0, 0x3c, 0xf, 0x3, 0xc0,
    0xf0, 0x3c, 0xf, 0x3, 0xc0, 0xf0, 0x3c, 0xf,
    0x3, 0xc0, 0xf0, 0x3f, 0xff, 0xff,

    /* U+0050 "P" */
    0xff, 0xff, 0xfc, 0xf, 0x3, 0xc0, 0xf0, 0x3c,
    0xf, 0x3, 0xc0, 0xf0, 0x3c, 0xf, 0x3, 0xff,
    0xff, 0xfc, 0x3, 0x0, 0xc0, 0x30, 0xc, 0x3,
    0x0, 0xc0, 0x30, 0xc, 0x3, 0x0,

    /* U+0051 "Q" */
    0xff, 0xcf, 0xfc, 0xc0, 0xcc, 0xc, 0xc0, 0xcc,
    0xc, 0xc0, 0xcc, 0xc, 0xc0, 0xcc, 0xc, 0xc0,
    0xcc, 0xc, 0xc0, 0xcc, 0xc, 0xc0, 0xcc, 0xc,
    0xc0, 0xcc, 0x4c, 0xc7, 0xcc, 0x3c, 0xc1, 0xcc,
    0xf, 0xff, 0xff, 0xf9,

    /* U+0052 "R" */
    0xff, 0xff, 0xfc, 0xf, 0x3, 0xc0, 0xf0, 0x3c,
    0xf, 0x3, 0xc0, 0xf0, 0x3c, 0xf, 0x3f, 0xcf,
    0xb3, 0xc, 0x63, 0x18, 0xc7, 0x30, 0xcc, 0x33,
    0x6, 0xc1, 0xb0, 0x7c, 0xf, 0x3,

    /* U+0053 "S" */
    0xff, 0xff, 0xfc, 0xf, 0x3, 0xc0, 0xf0, 0x3c,
    0xf, 0x0, 0x60, 0xc, 0x1, 0x80, 0x30, 0xe,
    0x1, 0xc0, 0x38, 0x7, 0xc0, 0xf0, 0x3c, 0xf,
    0x3, 0xc0, 0xf0, 0x3f, 0xff, 0xff,

    /* U+0054 "T" */
    0xff, 0xff, 0xf0, 0xc0, 0x30, 0xc, 0x3, 0x0,
    0xc0, 0x30, 0xc, 0x3, 0x0, 0xc0, 0x30, 0xc,
    0x3, 0x0, 0xc0, 0x30, 0xc, 0x3, 0x0, 0xc0,
    0x30, 0xc, 0x3, 0x0, 0xc0, 0x30,

    /* U+0055 "U" */
    0xc0, 0xf0, 0x3c, 0xf, 0x3, 0xc0, 0xf0, 0x3c,
    0xf, 0x3, 0xc0, 0xf0, 0x3c, 0xf, 0x3, 0xc0,
    0xf0, 0x3c, 0xf, 0x3, 0xc0, 0xf0, 0x3c, 0xf,
    0x3, 0xc0, 0xf0, 0x3f, 0xff, 0xff,

    /* U+0056 "V" */
    0xc0, 0xf0, 0x3c, 0xf, 0x3, 0x61, 0x98, 0x66,
    0x19, 0x86, 0x61, 0x98, 0x63, 0x30, 0xcc, 0x33,
    0xc, 0xc3, 0x30, 0xcc, 0x1e, 0x7, 0x81, 0xe0,
    0x78, 0x1e, 0x3, 0x0, 0xc0, 0x30,

    /* U+0057 "W" */
    0xc3, 0x87, 0x87, 0xf, 0xe, 0x1e, 0x1c, 0x3c,
    0x38, 0x7c, 0x71, 0xd8, 0xa3, 0x33, 0x66, 0x66,
    0xcc, 0xcd, 0x99, 0x9b, 0x33, 0x36, 0x66, 0x64,
    0xc4, 0x8b, 0xf, 0x1e, 0x1e, 0x3c, 0x3c, 0x78,
    0x78, 0xf0, 0xf1, 0xe1, 0xc1, 0xc1, 0x83, 0x3,
    0x6, 0x6, 0xc, 0xc, 0x18,

    /* U+0058 "X" */
    0xc0, 0xdc, 0x19, 0x87, 0x30, 0xc7, 0x18, 0x66,
    0xc, 0xc0, 0xd8, 0x1e, 0x3, 0xc0, 0x38, 0x6,
    0x0, 0xe0, 0x3c, 0x7, 0x80, 0xd8, 0x33, 0x6,
    0x60, 0xc6, 0x30, 0xc6, 0x18, 0xc1, 0xb0, 0x36,
    0x6,

    /* U+0059 "Y" */
    0xc0, 0xf0, 0x3c, 0xd, 0x86, 0x61, 0x98, 0x63,
    0x38, 0xcc, 0x33, 0xc, 0xc1, 0xe0, 0x78, 0x1e,
    0x3, 0x0, 0xc0, 0x30, 0xc, 0x3, 0x0, 0xc0,
    0x30, 0xc, 0x3, 0x0, 0xc0, 0x30,

    /* U+005A "Z" */
    0x7f, 0xdf, 0xf0, 0x1c, 0x6, 0x1, 0x80, 0xe0,
    0x30, 0x1c, 0x6, 0x1, 0x80, 0xe0, 0x30, 0xc,
    0x7, 0x1, 0x80, 0x60, 0x38, 0xc, 0x3, 0x1,
    0xc0, 0x60, 0x38, 0xf, 0xff, 0xff,

    /* U+005B "[" */
    0xff, 0xfc, 0x30, 0xc3, 0xc, 0x30, 0xc3, 0xc,
    0x30, 0xc3, 0xc, 0x30, 0xc3, 0xc, 0x30, 0xc3,
    0xc, 0x30, 0xc3, 0xf, 0xff,

    /* U+005C "\\" */
    0xc0, 0xc, 0x0, 0x60, 0x6, 0x0, 0x30, 0x3,
    0x0, 0x38, 0x1, 0x80, 0x18, 0x0, 0xc0, 0xc,
    0x0, 0x60, 0x6, 0x0, 0x30, 0x3, 0x0, 0x30,
    0x1, 0x80, 0x18, 0x0, 0xc0, 0xc, 0x0, 0x60,
    0x6, 0x0, 0x60, 0x3,

    /* U+005D "]" */
    0xff, 0xf0, 0xc3, 0xc, 0x30, 0xc3, 0xc, 0x30,
    0xc3, 0xc, 0x30, 0xc3, 0xc, 0x30, 0xc3, 0xc,
    0x30, 0xc3, 0xc, 0x3f, 0xff,

    /* U+005E "^" */
    0x4, 0x3, 0x80, 0xe0, 0x6c, 0x1b, 0xc, 0x63,
    0x19, 0x83, 0x40, 0xc0,

    /* U+005F "_" */
    0xff, 0xff, 0xff,

    /* U+0060 "`" */
    0xc3, 0xc, 0x20,

    /* U+0061 "a" */
    0xff, 0xff, 0xf0, 0x78, 0x30, 0x18, 0xf, 0xff,
    0xff, 0xc1, 0xe0, 0xf0, 0x78, 0x3c, 0x1f, 0xff,
    0xfe,

    /* U+0062 "b" */
    0xc0, 0x60, 0x30, 0x18, 0xc, 0x6, 0x3, 0x1,
    0x80, 0xc0, 0x7f, 0xff, 0xf8, 0x3c, 0x1e, 0xf,
    0x7, 0x83, 0xc1, 0xe0, 0xf0, 0x78, 0x3c, 0x1e,
    0xf, 0xff, 0xfe,

    /* U+0063 "c" */
    0xff, 0xff, 0xc3, 0xc3, 0xc3, 0xc0, 0xc0, 0xc0,
    0xc0, 0xc0, 0xc3, 0xc3, 0xc3, 0xff, 0xff,

    /* U+0064 "d" */
    0x1, 0x80, 0xc0, 0x60, 0x30, 0x18, 0xc, 0x6,
    0x3, 0x1, 0x9f, 0xff, 0xf8, 0x3c, 0x1e, 0xf,
    0x7, 0x83, 0xc1, 0xe0, 0xf0, 0x78, 0x3c, 0x1e,
    0xf, 0xfe, 0xff,

    /* U+0065 "e" */
    0xff, 0xff, 0xc3, 0xc3, 0xc3, 0xc3, 0xff, 0xff,
    0xc0, 0xc0, 0xc3, 0xc3, 0xc3, 0xff, 0xff,

    /* U+0066 "f" */
    0x3e, 0x7c, 0xc1, 0x83, 0x6, 0xc, 0x18, 0x31,
    0xfb, 0xf1, 0x83, 0x6, 0xc, 0x18, 0x30, 0x60,
    0xc1, 0x83, 0x6, 0xc, 0x18,

    /* U+0067 "g" */
    0x7f, 0xff, 0xc3, 0xc3, 0xc3, 0xc3, 0xc3, 0xc3,
    0xc3, 0xc3, 0xc3, 0xc3, 0xc3, 0xff, 0x7f, 0x3,
    0x3, 0xc3, 0xc3, 0xff, 0xff,

    /* U+0068 "h" */
    0xc0, 0x60, 0x30, 0x18, 0xc, 0x6, 0x3, 0x1,
    0x80, 0xc0, 0x7f, 0xff, 0xf8, 0x3c, 0x1e, 0xf,
    0x7, 0x83, 0xc1, 0xe0, 0xf0, 0x78, 0x3c, 0x1e,
    0xf, 0x7, 0x83,

    /* U+0069 "i" */
    0xf0, 0xf, 0xff, 0xff, 0xff, 0xc0,

    /* U+006A "j" */
    0x18, 0xc0, 0x0, 0xc, 0x63, 0x18, 0xc6, 0x31,
    0x8c, 0x63, 0x18, 0xc6, 0x31, 0x8c, 0x63, 0xff,
    0xc0,

    /* U+006B "k" */
    0xc0, 0xc0, 0xc0, 0xc0, 0xc0, 0xc0, 0xc0, 0xc0,
    0xc0, 0xc3, 0xc6, 0xce, 0xcc, 0xd8, 0xf8, 0xf0,
    0xf0, 0xd8, 0xd8, 0xcc, 0xcc, 0xc6, 0xc7, 0xc3,

    /* U+006C "l" */
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff,

    /* U+006D "m" */
    0xff, 0xff, 0xff, 0xff, 0xc1, 0x83, 0xc1, 0x83,
    0xc1, 0x83, 0xc1, 0x83, 0xc1, 0x83, 0xc1, 0x83,
    0xc1, 0x83, 0xc1, 0x83, 0xc1, 0x83, 0xc1, 0x83,
    0xc1, 0x83, 0xc1, 0x83, 0xc1, 0x83,

    /* U+006E "n" */
    0xff, 0xff, 0xf0, 0x78, 0x3c, 0x1e, 0xf, 0x7,
    0x83, 0xc1, 0xe0, 0xf0, 0x78, 0x3c, 0x1e, 0xf,
    0x6,

    /* U+006F "o" */
    0xff, 0xff, 0xf0, 0x78, 0x3c, 0x1e, 0xf, 0x7,
    0x83, 0xc1, 0xe0, 0xf0, 0x78, 0x3c, 0x1f, 0xff,
    0xfe,

    /* U+0070 "p" */
    0xff, 0xff, 0xf0, 0x78, 0x3c, 0x1e, 0xf, 0x7,
    0x83, 0xc1, 0xe0, 0xf0, 0x78, 0x3c, 0x1f, 0xff,
    0xfd, 0x80, 0xc0, 0x60, 0x30, 0x18, 0xc, 0x0,

    /* U+0071 "q" */
    0x7f, 0xff, 0xf0, 0x78, 0x3c, 0x1e, 0xf, 0x7,
    0x83, 0xc1, 0xe0, 0xf0, 0x78, 0x3c, 0x1f, 0xfd,
    0xfe, 0x3, 0x1, 0x80, 0xc0, 0x60, 0x30, 0x18,

    /* U+0072 "r" */
    0xff, 0xff, 0xc3, 0xc3, 0xc3, 0xc0, 0xc0, 0xc0,
    0xc0, 0xc0, 0xc0, 0xc0, 0xc0, 0xc0, 0xc0,

    /* U+0073 "s" */
    0xff, 0xff, 0xc3, 0xc3, 0xc3, 0xe0, 0x30, 0x1c,
    0xe, 0x7, 0xc3, 0xc3, 0xc3, 0xff, 0xff,

    /* U+0074 "t" */
    0x30, 0x60, 0xc1, 0x83, 0x1f, 0xff, 0x98, 0x30,
    0x60, 0xc1, 0x83, 0x6, 0xc, 0x18, 0x30, 0x60,
    0xf9, 0xf0,

    /* U+0075 "u" */
    0xc1, 0xe0, 0xf0, 0x78, 0x3c, 0x1e, 0xf, 0x7,
    0x83, 0xc1, 0xe0, 0xf0, 0x78, 0x3c, 0x1f, 0xff,
    0xfe,

    /* U+0076 "v" */
    0xc1, 0xe0, 0xf8, 0x6c, 0x66, 0x33, 0x19, 0x8c,
    0x64, 0x36, 0x1b, 0xd, 0x82, 0x81, 0xc0, 0xe0,
    0x70,

    /* U+0077 "w" */
    0xc3, 0xf, 0xc, 0x3c, 0x30, 0xd1, 0xe3, 0x67,
    0x99, 0x9e, 0x66, 0x69, 0x99, 0x26, 0x24, 0xd8,
    0xf3, 0x43, 0xcf, 0xf, 0x1c, 0x38, 0x70, 0x61,
    0x81, 0x86, 0x0,

    /* U+0078 "x" */
    0xc3, 0xb1, 0x98, 0xcc, 0xc3, 0x61, 0xe0, 0x70,
    0x38, 0x1c, 0x1e, 0xd, 0x8c, 0xc6, 0x36, 0x1b,
    0x6,

    /* U+0079 "y" */
    0xc1, 0xe0, 0xf8, 0x6c, 0x66, 0x33, 0x19, 0x8c,
    0x64, 0x36, 0x1b, 0xd, 0x82, 0x81, 0xc0, 0xe0,
    0x70, 0x38, 0x18, 0xc, 0x6, 0x6, 0x3, 0x0,

    /* U+007A "z" */
    0x7f, 0x7f, 0x6, 0x6, 0xc, 0xc, 0xc, 0x18,
    0x18, 0x30, 0x30, 0x60, 0x60, 0xff, 0xff,

    /* U+007B "{" */
    0xf, 0x8f, 0xc7, 0x3, 0x1, 0x80, 0xc0, 0x60,
    0x30, 0x18, 0xc, 0x6, 0x7, 0x7, 0x7, 0x1,
    0xc0, 0x70, 0x18, 0xc, 0x6, 0x3, 0x1, 0x80,
    0xc0, 0x60, 0x30, 0x18, 0xc, 0x7, 0xe1, 0xf0,

    /* U+007C "|" */
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf0,

    /* U+007D "}" */
    0xf8, 0x7e, 0x3, 0x1, 0x80, 0xc0, 0x60, 0x30,
    0x18, 0xc, 0x6, 0x3, 0x1, 0xc0, 0x70, 0x1c,
    0x1c, 0x1c, 0xc, 0x6, 0x3, 0x1, 0x80, 0xc0,
    0x60, 0x30, 0x18, 0xc, 0x6, 0x3f, 0x1f, 0x0,

    /* U+007E "~" */
    0xf0, 0x7f, 0xcf, 0x3f, 0xe0, 0xf0
};


/*---------------------
 *  GLYPH DESCRIPTION
 *--------------------*/

static const lv_font_fmt_txt_glyph_dsc_t glyph_dsc[] = {
    {.bitmap_index = 0, .adv_w = 0, .box_w = 0, .box_h = 0, .ofs_x = 0, .ofs_y = 0} /* id = 0 reserved */,
    {.bitmap_index = 0, .adv_w = 100, .box_w = 1, .box_h = 1, .ofs_x = 0, .ofs_y = 0},
    {.bitmap_index = 1, .adv_w = 102, .box_w = 2, .box_h = 24, .ofs_x = 2, .ofs_y = 0},
    {.bitmap_index = 7, .adv_w = 157, .box_w = 6, .box_h = 7, .ofs_x = 2, .ofs_y = 17},
    {.bitmap_index = 13, .adv_w = 267, .box_w = 15, .box_h = 24, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 58, .adv_w = 213, .box_w = 9, .box_h = 28, .ofs_x = 2, .ofs_y = -2},
    {.bitmap_index = 90, .adv_w = 343, .box_w = 19, .box_h = 24, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 147, .adv_w = 223, .box_w = 12, .box_h = 24, .ofs_x = 2, .ofs_y = 0},
    {.bitmap_index = 183, .adv_w = 85, .box_w = 2, .box_h = 7, .ofs_x = 2, .ofs_y = 17},
    {.bitmap_index = 185, .adv_w = 135, .box_w = 6, .box_h = 27, .ofs_x = 2, .ofs_y = -3},
    {.bitmap_index = 206, .adv_w = 135, .box_w = 6, .box_h = 27, .ofs_x = 1, .ofs_y = -3},
    {.bitmap_index = 227, .adv_w = 203, .box_w = 9, .box_h = 10, .ofs_x = 2, .ofs_y = 14},
    {.bitmap_index = 239, .adv_w = 210, .box_w = 11, .box_h = 10, .ofs_x = 1, .ofs_y = 5},
    {.bitmap_index = 253, .adv_w = 85, .box_w = 3, .box_h = 4, .ofs_x = 1, .ofs_y = -2},
    {.bitmap_index = 255, .adv_w = 151, .box_w = 6, .box_h = 2, .ofs_x = 1, .ofs_y = 8},
    {.bitmap_index = 257, .adv_w = 85, .box_w = 2, .box_h = 2, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 258, .adv_w = 221, .box_w = 12, .box_h = 24, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 294, .adv_w = 222, .box_w = 10, .box_h = 24, .ofs_x = 2, .ofs_y = 0},
    {.bitmap_index = 324, .adv_w = 104, .box_w = 3, .box_h = 24, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 333, .adv_w = 200, .box_w = 10, .box_h = 24, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 363, .adv_w = 216, .box_w = 10, .box_h = 24, .ofs_x = 2, .ofs_y = 0},
    {.bitmap_index = 393, .adv_w = 196, .box_w = 11, .box_h = 24, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 426, .adv_w = 211, .box_w = 9, .box_h = 24, .ofs_x = 2, .ofs_y = 0},
    {.bitmap_index = 453, .adv_w = 217, .box_w = 10, .box_h = 24, .ofs_x = 2, .ofs_y = 0},
    {.bitmap_index = 483, .adv_w = 186, .box_w = 10, .box_h = 24, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 513, .adv_w = 221, .box_w = 10, .box_h = 24, .ofs_x = 2, .ofs_y = 0},
    {.bitmap_index = 543, .adv_w = 217, .box_w = 10, .box_h = 24, .ofs_x = 2, .ofs_y = 0},
    {.bitmap_index = 573, .adv_w = 85, .box_w = 2, .box_h = 13, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 577, .adv_w = 85, .box_w = 2, .box_h = 15, .ofs_x = 1, .ofs_y = -2},
    {.bitmap_index = 581, .adv_w = 175, .box_w = 8, .box_h = 12, .ofs_x = 1, .ofs_y = 4},
    {.bitmap_index = 593, .adv_w = 226, .box_w = 11, .box_h = 8, .ofs_x = 2, .ofs_y = 6},
    {.bitmap_index = 604, .adv_w = 175, .box_w = 8, .box_h = 12, .ofs_x = 2, .ofs_y = 4},
    {.bitmap_index = 616, .adv_w = 196, .box_w = 9, .box_h = 24, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 643, .adv_w = 289, .box_w = 14, .box_h = 24, .ofs_x = 2, .ofs_y = 0},
    {.bitmap_index = 685, .adv_w = 205, .box_w = 11, .box_h = 24, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 718, .adv_w = 218, .box_w = 10, .box_h = 24, .ofs_x = 2, .ofs_y = 0},
    {.bitmap_index = 748, .adv_w = 218, .box_w = 10, .box_h = 24, .ofs_x = 2, .ofs_y = 0},
    {.bitmap_index = 778, .adv_w = 222, .box_w = 10, .box_h = 24, .ofs_x = 2, .ofs_y = 0},
    {.bitmap_index = 808, .adv_w = 183, .box_w = 8, .box_h = 24, .ofs_x = 2, .ofs_y = 0},
    {.bitmap_index = 832, .adv_w = 174, .box_w = 8, .box_h = 24, .ofs_x = 2, .ofs_y = 0},
    {.bitmap_index = 856, .adv_w = 220, .box_w = 10, .box_h = 24, .ofs_x = 2, .ofs_y = 0},
    {.bitmap_index = 886, .adv_w = 226, .box_w = 10, .box_h = 24, .ofs_x = 2, .ofs_y = 0},
    {.bitmap_index = 916, .adv_w = 102, .box_w = 2, .box_h = 24, .ofs_x = 2, .ofs_y = 0},
    {.bitmap_index = 922, .adv_w = 200, .box_w = 9, .box_h = 24, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 949, .adv_w = 197, .box_w = 10, .box_h = 24, .ofs_x = 2, .ofs_y = 0},
    {.bitmap_index = 979, .adv_w = 173, .box_w = 8, .box_h = 24, .ofs_x = 2, .ofs_y = 0},
    {.bitmap_index = 1003, .adv_w = 256, .box_w = 12, .box_h = 24, .ofs_x = 2, .ofs_y = 0},
    {.bitmap_index = 1039, .adv_w = 224, .box_w = 10, .box_h = 24, .ofs_x = 2, .ofs_y = 0},
    {.bitmap_index = 1069, .adv_w = 228, .box_w = 10, .box_h = 24, .ofs_x = 2, .ofs_y = 0},
    {.bitmap_index = 1099, .adv_w = 210, .box_w = 10, .box_h = 24, .ofs_x = 2, .ofs_y = 0},
    {.bitmap_index = 1129, .adv_w = 231, .box_w = 12, .box_h = 24, .ofs_x = 2, .ofs_y = 0},
    {.bitmap_index = 1165, .adv_w = 219, .box_w = 10, .box_h = 24, .ofs_x = 2, .ofs_y = 0},
    {.bitmap_index = 1195, .adv_w = 214, .box_w = 10, .box_h = 24, .ofs_x = 2, .ofs_y = 0},
    {.bitmap_index = 1225, .adv_w = 176, .box_w = 10, .box_h = 24, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 1255, .adv_w = 225, .box_w = 10, .box_h = 24, .ofs_x = 2, .ofs_y = 0},
    {.bitmap_index = 1285, .adv_w = 200, .box_w = 10, .box_h = 24, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 1315, .adv_w = 281, .box_w = 15, .box_h = 24, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 1360, .adv_w = 197, .box_w = 11, .box_h = 24, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 1393, .adv_w = 200, .box_w = 10, .box_h = 24, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 1423, .adv_w = 176, .box_w = 10, .box_h = 24, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 1453, .adv_w = 149, .box_w = 6, .box_h = 28, .ofs_x = 2, .ofs_y = -4},
    {.bitmap_index = 1474, .adv_w = 221, .box_w = 12, .box_h = 24, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 1510, .adv_w = 149, .box_w = 6, .box_h = 28, .ofs_x = 1, .ofs_y = -4},
    {.bitmap_index = 1531, .adv_w = 205, .box_w = 10, .box_h = 9, .ofs_x = 1, .ofs_y = 16},
    {.bitmap_index = 1543, .adv_w = 185, .box_w = 12, .box_h = 2, .ofs_x = 0, .ofs_y = -6},
    {.bitmap_index = 1546, .adv_w = 205, .box_w = 5, .box_h = 4, .ofs_x = 5, .ofs_y = 16},
    {.bitmap_index = 1549, .adv_w = 192, .box_w = 9, .box_h = 15, .ofs_x = 2, .ofs_y = 0},
    {.bitmap_index = 1566, .adv_w = 194, .box_w = 9, .box_h = 24, .ofs_x = 2, .ofs_y = 0},
    {.bitmap_index = 1593, .adv_w = 189, .box_w = 8, .box_h = 15, .ofs_x = 2, .ofs_y = 0},
    {.bitmap_index = 1608, .adv_w = 193, .box_w = 9, .box_h = 24, .ofs_x = 2, .ofs_y = 0},
    {.bitmap_index = 1635, .adv_w = 191, .box_w = 8, .box_h = 15, .ofs_x = 2, .ofs_y = 0},
    {.bitmap_index = 1650, .adv_w = 122, .box_w = 7, .box_h = 24, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 1671, .adv_w = 193, .box_w = 8, .box_h = 21, .ofs_x = 2, .ofs_y = -6},
    {.bitmap_index = 1692, .adv_w = 194, .box_w = 9, .box_h = 24, .ofs_x = 2, .ofs_y = 0},
    {.bitmap_index = 1719, .adv_w = 91, .box_w = 2, .box_h = 21, .ofs_x = 2, .ofs_y = 0},
    {.bitmap_index = 1725, .adv_w = 91, .box_w = 5, .box_h = 26, .ofs_x = -1, .ofs_y = -6},
    {.bitmap_index = 1742, .adv_w = 172, .box_w = 8, .box_h = 24, .ofs_x = 2, .ofs_y = 0},
    {.bitmap_index = 1766, .adv_w = 91, .box_w = 2, .box_h = 24, .ofs_x = 2, .ofs_y = 0},
    {.bitmap_index = 1772, .adv_w = 299, .box_w = 16, .box_h = 15, .ofs_x = 2, .ofs_y = 0},
    {.bitmap_index = 1802, .adv_w = 194, .box_w = 9, .box_h = 15, .ofs_x = 2, .ofs_y = 0},
    {.bitmap_index = 1819, .adv_w = 194, .box_w = 9, .box_h = 15, .ofs_x = 2, .ofs_y = 0},
    {.bitmap_index = 1836, .adv_w = 193, .box_w = 9, .box_h = 21, .ofs_x = 2, .ofs_y = -6},
    {.bitmap_index = 1860, .adv_w = 193, .box_w = 9, .box_h = 21, .ofs_x = 2, .ofs_y = -6},
    {.bitmap_index = 1884, .adv_w = 170, .box_w = 8, .box_h = 15, .ofs_x = 2, .ofs_y = 0},
    {.bitmap_index = 1899, .adv_w = 184, .box_w = 8, .box_h = 15, .ofs_x = 2, .ofs_y = 0},
    {.bitmap_index = 1914, .adv_w = 130, .box_w = 7, .box_h = 20, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 1932, .adv_w = 194, .box_w = 9, .box_h = 15, .ofs_x = 2, .ofs_y = 0},
    {.bitmap_index = 1949, .adv_w = 178, .box_w = 9, .box_h = 15, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 1966, .adv_w = 259, .box_w = 14, .box_h = 15, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 1993, .adv_w = 170, .box_w = 9, .box_h = 15, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 2010, .adv_w = 178, .box_w = 9, .box_h = 21, .ofs_x = 1, .ofs_y = -6},
    {.bitmap_index = 2034, .adv_w = 164, .box_w = 8, .box_h = 15, .ofs_x = 1, .ofs_y = 0},
    {.bitmap_index = 2049, .adv_w = 175, .box_w = 9, .box_h = 28, .ofs_x = 0, .ofs_y = -4},
    {.bitmap_index = 2081, .adv_w = 105, .box_w = 2, .box_h = 30, .ofs_x = 2, .ofs_y = -6},
    {.bitmap_index = 2089, .adv_w = 175, .box_w = 9, .box_h = 28, .ofs_x = 1, .ofs_y = -4},
    {.bitmap_index = 2121, .adv_w = 259, .box_w = 11, .box_h = 4, .ofs_x = 3, .ofs_y = 7}
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
const lv_font_t ui_font_mini = {
#else
lv_font_t ui_font_mini = {
#endif
    .get_glyph_dsc = lv_font_get_glyph_dsc_fmt_txt,    /*Function pointer to get glyph's data*/
    .get_glyph_bitmap = lv_font_get_bitmap_fmt_txt,    /*Function pointer to get glyph's bitmap*/
    .line_height = 32,          /*The maximum line height required by the font*/
    .base_line = 6,             /*Baseline measured from the bottom of the line*/
#if !(LVGL_VERSION_MAJOR == 6 && LVGL_VERSION_MINOR == 0)
    .subpx = LV_FONT_SUBPX_NONE,
#endif
#if LV_VERSION_CHECK(7, 4, 0) || LVGL_VERSION_MAJOR >= 8
    .underline_position = -4,
    .underline_thickness = 2,
#endif
    .dsc = &font_dsc           /*The custom font data. Will be accessed by `get_glyph_bitmap/dsc` */
};



#endif /*#if UI_FONT_MINI*/

