/*
 * i2c_lcd.h
 *
 * Header file untuk driver LCD I2C HD44780.
 * Menyediakan deklarasi fungsi untuk inisialisasi LCD, pengiriman data,
 * pengaturan kursor, pembersihan layar, dan pencetakan string.
 *
 * DOKTER_FORKLIFT Project.
 */

#ifndef INC_I2C_LCD_H_
#define INC_I2C_LCD_H_

#include "stm32f1xx_hal.h" // Termasuk HAL library
#include <stdint.h>        // Untuk tipe data standar seperti uint8_t

// --- Definisi Alamat I2C LCD (PENTING! Verifikasi alamat Anda) ---
// Alamat I2C LCD default adalah 0x27 atau 0x3F.
// Ubah ini sesuai dengan alamat PCF8574 Anda.
// (Alamat di sini adalah 7-bit, yang kemudian digeser 1 bit ke kiri oleh driver HAL)
#define LCD_ADDR (0x27 << 1) // Umumnya 0x27 (jadi 0x4E). Jika tidak berfungsi, coba (0x3F << 1) (jadi 0x7E)


// --- Definisi Perintah LCD ---
#define LCD_CLEARDISPLAY 0x01
#define LCD_RETURNHOME 0x02
#define LCD_ENTRYMODESET 0x04
#define LCD_DISPLAYCONTROL 0x08
#define LCD_CURSORSHIFT 0x10
#define LCD_FUNCTIONSET 0x20
#define LCD_SETCGRAMADDR 0x40
#define LCD_SETDDRAMADDR 0x80

// --- Flags untuk Entry Mode ---
#define LCD_ENTRYRIGHT 0x00
#define LCD_ENTRYLEFT 0x02
#define LCD_ENTRYSHIFTINCREMENT 0x01
#define LCD_ENTRYSHIFTDECREMENT 0x00

// --- Flags untuk Display Control ---
#define LCD_DISPLAYON 0x04
#define LCD_DISPLAYOFF 0x00
#define LCD_CURSORON 0x02
#define LCD_CURSOROFF 0x00
#define LCD_BLINKON 0x01
#define LCD_BLINKOFF 0x00

// --- Flags untuk Display/Cursor Shift ---
#define LCD_DISPLAYMOVE 0x08
#define LCD_CURSORMOVE 0x00
#define LCD_MOVERIGHT 0x04
#define LCD_MOVELEFT 0x00

// --- Flags untuk Function Set ---
#define LCD_8BITMODE 0x10
#define LCD_4BITMODE 0x00
#define LCD_2LINE 0x08
#define LCD_1LINE 0x00
#define LCD_5x10DOTS 0x04
#define LCD_5x8DOTS 0x00

// --- Flags untuk Backlight Control ---
#define LCD_BACKLIGHT 0x08
#define LCD_NOBACKLIGHT 0x00

// --- Deklarasi Fungsi Publik ---

/**
  * @brief  Menginisialisasi modul LCD I2C.
  * @param  hi2c Pointer ke handler I2C.
  * @retval None
  */
void lcd_init(I2C_HandleTypeDef *hi2c);

/**
  * @brief  Mengirimkan data 4-bit ke LCD.
  * @param  nibble Data 4-bit yang akan dikirim (diperlukan masking &0xF0).
  * @param  mode Mode pengiriman (RS - Register Select).
  * @retval None
  */
void lcd_send_4bit(uint8_t nibble, uint8_t mode);

/**
  * @brief  Mengirimkan perintah ke LCD.
  * @param  cmd Perintah yang akan dikirim (8-bit).
  * @retval None
  */
void lcd_send_cmd(uint8_t cmd);

/**
  * @brief  Mengirimkan karakter data ke LCD.
  * @param  data Karakter data yang akan dikirim (8-bit).
  * @retval None
  */
void lcd_send_data(uint8_t data);

/**
  * @brief  Mengatur posisi kursor pada LCD.
  * @param  col Kolom (0-15 untuk 16x2).
  * @param  row Baris (0-1).
  * @retval None
  */
void lcd_set_cursor(uint8_t col, uint8_t row);

/**
  * @brief  Mengirimkan string ke LCD.
  * @param  str Pointer ke string karakter.
  * @retval None
  */
void lcd_send_string(char *str);

/**
  * @brief  Membersihkan layar LCD.
  * @param  None
  * @retval None
  */
void lcd_clear(void);

/**
  * @brief  Menyalakan/mematikan backlight LCD.
  * @param  state 1 untuk ON, 0 untuk OFF.
  * @retval None
  */
void lcd_backlight(uint8_t state);

#endif /* INC_I2C_LCD_H_ */


