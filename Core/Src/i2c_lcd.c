/*
 * i2c_lcd.c
 *
 * Source file untuk driver LCD I2C HD44780.
 * Mengimplementasikan fungsi-fungsi untuk mengontrol LCD melalui I2C.
 *
 * DOKTER_FORKLIFT Project.
 */

#include "i2c_lcd.h"
#include <stdio.h> // Untuk sprintf (jika digunakan untuk debug/logging)

// --- Variabel Global untuk Handler I2C ---
// Variabel ini diinisialisasi di sini dan diakses melalui pointer di fungsi.
static I2C_HandleTypeDef *i2c_handle_global; // Pointer ke handler I2C yang digunakan

// --- Variabel Internal LCD ---
static uint8_t _displayfunction;
static uint8_t _displaycontrol;
static uint8_t _displaymode;
static uint8_t _numlines;
static uint8_t _backlightval;

// --- Fungsi Internal (Private) ---

/**
  * @brief  Mengirimkan byte data melalui I2C ke PCF8574.
  * @param  data Byte data yang akan dikirim.
  * @retval None
  */
static void i2c_write_byte(uint8_t data) {
    // Timeout untuk transmisi I2C. Default HAL adalah 100ms.
    // Jika I2C Anda sering macet, periksa kembali wiring atau pull-up resistor.
    HAL_I2C_Master_Transmit(i2c_handle_global, LCD_ADDR, &data, 1, 1000); // BUG FIX: Meningkatkan timeout I2C ke 1 detik
}

/**
  * @brief  Mengirimkan nibble (4-bit) ke LCD dan melakukan toggle EN.
  * @param  nibble Nibble yang akan dikirim.
  * @param  mode Mode pengiriman (RS - Register Select).
  * @retval None
  */
void lcd_send_4bit(uint8_t nibble, uint8_t mode) {
    uint8_t data_to_send = (nibble & 0xF0) | mode | _backlightval;
    i2c_write_byte(data_to_send);        // Kirim data
    HAL_Delay(10); // BUG FIX: Menambah delay setelah data dikirim
    i2c_write_byte(data_to_send | 0x04); // Set EN HIGH (bit 2)
    HAL_Delay(10);                       // BUG FIX: Menambah delay untuk pulsa EN agar sangat stabil
    i2c_write_byte(data_to_send & ~0x04); // Set EN LOW
    HAL_Delay(20);                       // BUG FIX: Menambah delay setelah pulsa EN agar sangat stabil
}

/**
  * @brief  Mengirimkan perintah ke LCD.
  * @param  cmd Perintah yang akan dikirim (8-bit).
  * @retval None
  */
void lcd_send_cmd(uint8_t cmd) {
    lcd_send_4bit(cmd & 0xF0, 0x00); // Kirim 4 bit tinggi (RS=0 untuk perintah)
    lcd_send_4bit(cmd << 4, 0x00);   // Kirim 4 bit rendah (geser ke atas)
    HAL_Delay(10); // BUG FIX: Tambahan delay setelah setiap perintah
}

/**
  * @brief  Mengirimkan karakter data ke LCD.
  * @param  data Karakter data yang akan dikirim (8-bit).
  * @retval None
  */
void lcd_send_data(uint8_t data) {
    lcd_send_4bit(data & 0xF0, 0x01); // Kirim 4 bit tinggi (RS=1 untuk data)
    lcd_send_4bit(data << 4, 0x01);   // Kirim 4 bit rendah (geser ke atas)
    HAL_Delay(10); // BUG FIX: Tambahan delay setelah setiap data
}

// --- Fungsi Publik ---

/**
  * @brief  Menginisialisasi modul LCD I2C.
  * @param  hi2c Pointer ke handler I2C.
  * @retval None
  */
void lcd_init(I2C_HandleTypeDef *hi2c) {
    i2c_handle_global = hi2c; // Simpan handler I2C global

    // Konfigurasi awal untuk LCD 16x2 (paling umum)
    // Jika Anda menggunakan LCD 20x4, Anda perlu mengubah _displayfunction dan _numlines:
    // _displayfunction = LCD_4BITMODE | LCD_2LINE | LCD_5x10DOTS; // Untuk 20x4 LCD
    // _numlines = 4; // Untuk 20x4 LCD, dan row_offsets perlu 0x00, 0x40, 0x14, 0x54
    _displayfunction = LCD_4BITMODE | LCD_2LINE | LCD_5x8DOTS; // Default 16x2
    _numlines = 2; // Untuk 16x2 LCD
    _backlightval = LCD_BACKLIGHT; // Default backlight ON

    // Tunda untuk power-up LCD (PENTING! Beberapa LCD butuh lebih lama)
    HAL_Delay(500); // BUG FIX: Menambah delay awal inisialisasi agar sangat stabil (min 40ms, beri spare sangat banyak)

    // Prosedur inisialisasi 4-bit (sesuai datasheet HD44780)
    lcd_send_4bit(0x03 << 4, 0x00); // Kirim 0x03 tiga kali (diperlukan untuk inisialisasi 4-bit)
    HAL_Delay(200); // BUG FIX: Menambah delay signifikan
    lcd_send_4bit(0x03 << 4, 0x00);
    HAL_Delay(200); // BUG FIX: Menambah delay signifikan
    lcd_send_4bit(0x03 << 4, 0x00);
    HAL_Delay(50); // BUG FIX: Menambah delay signifikan

    // Sekarang atur ke mode 4-bit
    lcd_send_4bit(0x02 << 4, 0x00); // Atur LCD ke mode 4-bit
    HAL_Delay(20); // Tambahan delay setelah perintah mode set

    // Set Function Set (mode 4-bit, 2 baris, 5x8 dots)
    lcd_send_cmd(LCD_FUNCTIONSET | _displayfunction);
    HAL_Delay(20); // Tambahan delay

    // Turn the display on with no cursor or blinking default
    _displaycontrol = LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF;
    lcd_send_cmd(LCD_DISPLAYCONTROL | _displaycontrol);
    HAL_Delay(20); // Tambahan delay

    // Clear display
    lcd_clear(); // Fungsi lcd_clear sudah memiliki delay internal
    HAL_Delay(20); // Tambahan delay setelah clear

    // Set entry mode (increment cursor, no display shift)
    _displaymode = LCD_ENTRYLEFT | LCD_ENTRYSHIFTDECREMENT;
    lcd_send_cmd(LCD_ENTRYMODESET | _displaymode);
    HAL_Delay(20); // Tambahan delay
}

/**
  * @brief  Membersihkan layar LCD.
  * @param  None
  * @retval None
  */
void lcd_clear(void) {
    lcd_send_cmd(LCD_CLEARDISPLAY);
    HAL_Delay(10); // BUG FIX: Clear display membutuhkan waktu lebih lama (~1.52ms), beri spare lebih banyak
}

/**
  * @brief  Mengatur posisi kursor pada LCD.
  * @param  col Kolom (0-15 untuk 16x2).
  * @param  row Baris (0-1).
  * @retval None
  */
void lcd_set_cursor(uint8_t col, uint8_t row) {
    // Offset alamat DDRAM untuk setiap baris
    // Untuk LCD 16x2: Baris 0 dimulai dari 0x00, Baris 1 dimulai dari 0x40
    // Untuk LCD 20x4: Baris 0:0x00, Baris 1:0x40, Baris 2:0x14, Baris 3:0x54
    static uint8_t row_offsets[] = { 0x00, 0x40, 0x14, 0x54 }; // Mendukung hingga 4 baris, tapi _numlines akan membatasi

    // Batasi baris agar tidak melebihi jumlah baris yang dikonfigurasi (_numlines)
    if (row >= _numlines) {
        row = _numlines - 1;
    }
    lcd_send_cmd(LCD_SETDDRAMADDR | (col + row_offsets[row]));
    HAL_Delay(5); // BUG FIX: Tambahan delay setelah set kursor
}

/**
  * @brief  Mengirimkan string ke LCD.
  * @param  str Pointer ke string karakter.
  * @retval None
  */
void lcd_send_string(char *str) {
    while (*str) {
        lcd_send_data(*str++);
    }
}

/**
  * @brief  Menyalakan/mematikan backlight LCD.
  * @param  state 1 untuk ON, 0 untuk OFF.
  * @retval None
  */
void lcd_backlight(uint8_t state) {
    if (state) {
        _backlightval = LCD_BACKLIGHT;
    } else {
        _backlightval = LCD_NOBACKLIGHT;
    }
    // Langsung kirim byte ke PCF8574 untuk update backlight tanpa siklus EN/data nibble yang kompleks
    i2c_write_byte(_backlightval);
    HAL_Delay(1); // Tambahan delay singkat
}



