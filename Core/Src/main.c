/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Proyek Speedometer Forklift oleh DOKTER_FORKLIFT.
  * Tahap 4 (Final): Implementasi Long Press & Stabilitas Final.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "i2c_lcd.h"
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
    DISPLAY_MODE, PASSWORD_MODE, MENU_MAIN, MENU_CALIBRATE,
    MENU_SET_ALARM, MENU_SET_CUTOFF, MENU_SET_PULSES,
    MENU_FACTORY_RESET, SHOW_MESSAGE_MODE
} SystemState;

typedef struct {
    float calibration_factor;
    float alarm_speed;
    float cutoff_speed;
    uint8_t pulses_per_rev;
    uint8_t padding[3];
    uint32_t data_valid_magic;
} Settings_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DEBOUNCE_TIME 50
#define LONG_PRESS_TIME 3000 // 3 detik untuk tekan lama
#define SENSOR_TIMEOUT 2000
#define SETTINGS_FLASH_PAGE_ADDR 0x0800FC00
#define DATA_VALID_MAGIC_NUMBER 0xCAFEBABE
#define DEFAULT_CAL_FACTOR 1.296
#define NUM_FREQ_SAMPLES 5
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
extern I2C_HandleTypeDef hi2c1;
extern TIM_HandleTypeDef htim2;
extern UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
// --- State Machine & Menu ---
volatile SystemState current_state = DISPLAY_MODE;
volatile SystemState next_state_after_message = DISPLAY_MODE;
uint16_t message_delay = 0;
uint32_t message_start_time = 0;
int8_t menu_cursor = 0;
float temp_setting_value = 0;
uint8_t temp_pulses_value = 0;
bool is_setting_ref_speed = true;

// --- Password ---
char entered_password[5] = "____";
int8_t password_cursor = 0;

// --- Tombol (Edge & Long Press) ---
uint8_t menu_btn_now = 0, menu_btn_last = 0;
uint8_t up_btn_now = 0, up_btn_last = 0;
uint8_t down_btn_now = 0, down_btn_last = 0;
bool menu_just_pressed = false, up_just_pressed = false, down_just_pressed = false;
bool up_down_just_pressed = false;
uint32_t last_button_read_time = 0;
uint32_t menu_press_start_time = 0; // Timer untuk long press
bool menu_is_being_held = false;

// --- Input Capture Timer ---
volatile uint32_t ic_val1 = 0, ic_val2 = 0, diff_capture = 0;
volatile uint8_t is_first_capture = 1, new_pulse_available = 0;

// --- Perhitungan & Settings ---
float pulse_frequency = 0, speed_kmh = 0;
Settings_t settings;
float reference_speed = 10.0;

// --- Moving Average Filter ---
float freq_samples[NUM_FREQ_SAMPLES] = {0};
uint8_t freq_sample_index = 0;

// --- Timing & Error ---
uint32_t last_display_update = 0, last_pulse_time = 0;
uint8_t error_no_sensor = 0;

// --- Display Buffer ---
char lcd_line1_buffer[21], lcd_line2_buffer[21];
char prev_lcd_line1_buffer[21] = " ", prev_lcd_line2_buffer[21] = " ";
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
void Process_Sensor_Data(void);
void Display_Mode_Handler(void);
void Password_Mode_Handler(void);
void Menu_Main_Handler(void);
void Menu_Calibrate_Handler(void);
void Menu_Edit_Value_Handler(void* value_to_edit, const char* title, SystemState mode);
void Show_Message(const char* line1, const char* line2, uint16_t delay, SystemState next_state);
void Read_Buttons(void);
void Update_LCD(void);
void Load_Settings_From_Flash(void);
bool Save_Settings_To_Flash(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
  if (htim->Instance == TIM2) {
    if(current_state != SHOW_MESSAGE_MODE) HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    if (is_first_capture) {
      ic_val1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1); is_first_capture = 0;
    } else {
      ic_val2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
      if (ic_val2 > ic_val1) diff_capture = ic_val2 - ic_val1;
      else diff_capture = (0xFFFF - ic_val1) + ic_val2;
      ic_val1 = ic_val2; new_pulse_available = 1;
    }
    last_pulse_time = HAL_GetTick();
  }
}

void Read_Buttons(void) {
    menu_just_pressed = false; up_just_pressed = false; down_just_pressed = false; up_down_just_pressed = false;

    if (HAL_GetTick() - last_button_read_time > DEBOUNCE_TIME) {
        menu_btn_now = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3) == GPIO_PIN_RESET);
        up_btn_now = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4) == GPIO_PIN_RESET);
        down_btn_now = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5) == GPIO_PIN_RESET);

        // Edge detection untuk penekanan tunggal
        if (up_btn_now && !up_btn_last) up_just_pressed = true;
        if (down_btn_now && !down_btn_last) down_just_pressed = true;
        if (up_btn_now && down_btn_now && !(up_btn_last && down_btn_last)) up_down_just_pressed = true;

        // Logika Long Press untuk Tombol Menu
        if (menu_btn_now && !menu_btn_last) { // Tombol baru saja ditekan
            menu_press_start_time = HAL_GetTick();
            menu_is_being_held = true;
        }
        if (!menu_btn_now && menu_btn_last) { // Tombol baru saja dilepas
            if (menu_is_being_held) {
                if (HAL_GetTick() - menu_press_start_time < LONG_PRESS_TIME) {
                    menu_just_pressed = true; // Dianggap short press jika dilepas < 3 detik
                }
            }
            menu_is_being_held = false;
        }
        // Jika tombol masih ditahan setelah 3 detik, trigger long press
        if (menu_is_being_held && (HAL_GetTick() - menu_press_start_time > LONG_PRESS_TIME)) {
            if (current_state == DISPLAY_MODE) { // Hanya berlaku di display mode
                 password_cursor = 0;
                 sprintf(entered_password, "____");
                 current_state = PASSWORD_MODE;
                 menu_is_being_held = false; // Reset agar tidak trigger lagi
            }
        }

        menu_btn_last = menu_btn_now;
        up_btn_last = up_btn_now;
        down_btn_last = down_btn_now;
        
        if (menu_just_pressed || up_just_pressed || down_just_pressed || up_down_just_pressed) {
             last_button_read_time = HAL_GetTick();
        }
    }
}

void Process_Sensor_Data(void){
    float instantaneous_freq = 0;
    if (new_pulse_available) {
      HAL_NVIC_DisableIRQ(TIM2_IRQn);
      uint32_t local_diff_capture = diff_capture; new_pulse_available = 0;
      HAL_NVIC_EnableIRQ(TIM2_IRQn);
      if (local_diff_capture > 0) instantaneous_freq = 1000000.0f / local_diff_capture;
      error_no_sensor = 0;
    }

    if (HAL_GetTick() - last_pulse_time > SENSOR_TIMEOUT) {
      instantaneous_freq = 0; is_first_capture = 1;
      for(int i=0; i < NUM_FREQ_SAMPLES; i++) freq_samples[i] = 0.0f;
      if (HAL_GetTick() > SENSOR_TIMEOUT && last_pulse_time == 0) error_no_sensor = 1;
    }
    
    freq_samples[freq_sample_index] = instantaneous_freq;
    freq_sample_index = (freq_sample_index + 1) % NUM_FREQ_SAMPLES;

    float total_freq = 0;
    for (int i=0; i < NUM_FREQ_SAMPLES; i++) total_freq += freq_samples[i];
    pulse_frequency = total_freq / NUM_FREQ_SAMPLES;
    
    float wheel_rev_freq = pulse_frequency / settings.pulses_per_rev;
    speed_kmh = wheel_rev_freq * settings.calibration_factor;

    if (speed_kmh > settings.alarm_speed && settings.alarm_speed > 0) HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
    else HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
    if (speed_kmh > settings.cutoff_speed && settings.cutoff_speed > 0) HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
    else HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
}

void Display_Mode_Handler(void){
    if(error_no_sensor) {
        sprintf(lcd_line1_buffer, "ERROR E1:");
        sprintf(lcd_line2_buffer, "NO SENSOR SIGNAL");
    } else {
        sprintf(lcd_line1_buffer, "Speed: %.1f Km/h", speed_kmh);
        sprintf(lcd_line2_buffer, "Tahan MENU u/Set");
    }
    // Logic untuk masuk menu sudah dipindah ke Read_Buttons()
}

void Password_Mode_Handler(void) {
    const char* correct_pass = "0110";
    sprintf(lcd_line1_buffer, "Password:");
    sprintf(lcd_line2_buffer, "   %s", entered_password);

    if(up_just_pressed) entered_password[password_cursor]++;
    if(down_just_pressed) entered_password[password_cursor]--;
    if (entered_password[password_cursor] > '9') entered_password[password_cursor] = '0';
    if (entered_password[password_cursor] < '0') entered_password[password_cursor] = '9';

    if (menu_just_pressed) {
        password_cursor++;
        if (password_cursor >= 4) {
            if (strcmp(entered_password, correct_pass) == 0) {
                current_state = MENU_MAIN; menu_cursor = 0;
            } else {
                Show_Message("Password Salah!", "", 1000, DISPLAY_MODE);
            }
        }
    }
}

void Menu_Main_Handler(void) {
    const char *main_menu_items[] = {"Cal Speed", "Cal Alarm", "Cal Cutoff", "Set Pulsa/Rev", "Reset", "Save/Exit"};
    const int main_menu_size = sizeof(main_menu_items)/sizeof(char*);
    
    if (up_just_pressed) menu_cursor--;
    if (down_just_pressed) menu_cursor++;
    if (menu_cursor < 0) menu_cursor = main_menu_size - 1;
    if (menu_cursor >= main_menu_size) menu_cursor = 0;

    int top_item_index = (menu_cursor / 2) * 2;
    sprintf(lcd_line1_buffer, "%c%-14s", (menu_cursor==top_item_index ? '>' : ' '), main_menu_items[top_item_index]);
    if (top_item_index + 1 < main_menu_size) {
       sprintf(lcd_line2_buffer, "%c%-14s", (menu_cursor==top_item_index + 1 ? '>' : ' '), main_menu_items[top_item_index + 1]);
    } else {
       sprintf(lcd_line2_buffer, "");
    }
    
    if (menu_just_pressed) {
        switch(menu_cursor) {
            case 0: is_setting_ref_speed = true; temp_setting_value = reference_speed; current_state = MENU_CALIBRATE; break;
            case 1: temp_setting_value = settings.alarm_speed; current_state = MENU_SET_ALARM; break;
            case 2: temp_setting_value = settings.cutoff_speed; current_state = MENU_SET_CUTOFF; break;
            case 3: temp_pulses_value = settings.pulses_per_rev; current_state = MENU_SET_PULSES; break;
            case 4: current_state = MENU_FACTORY_RESET; break;
            case 5: current_state = DISPLAY_MODE; break;
        }
    }
}

void Menu_Calibrate_Handler(void) {
    if (is_setting_ref_speed) {
        sprintf(lcd_line1_buffer, "Set Ref. Speed:");
        sprintf(lcd_line2_buffer, "-> %.1f Km/h", temp_setting_value);

        if(up_just_pressed) temp_setting_value += 1.0;
        if(down_just_pressed) temp_setting_value -= 1.0;
        if(temp_setting_value < 1.0) temp_setting_value = 1.0;

        if(menu_just_pressed) {
            reference_speed = temp_setting_value;
            is_setting_ref_speed = false;
        }
    } else {
        sprintf(lcd_line1_buffer, "Jlnkn %.1fKm/h", reference_speed);
        sprintf(lcd_line2_buffer, "UP+DN=Set,MENU=Batal");

        float avg_pulse_freq = 0;
        for(int i=0; i<NUM_FREQ_SAMPLES; i++) avg_pulse_freq += freq_samples[i];
        avg_pulse_freq /= NUM_FREQ_SAMPLES;
        
        float avg_wheel_freq = avg_pulse_freq / settings.pulses_per_rev;

        if (up_down_just_pressed && avg_wheel_freq > 0.1) {
            settings.calibration_factor = reference_speed / avg_wheel_freq;
            if (Save_Settings_To_Flash()) Show_Message("Kalibrasi OK!", "", 1500, MENU_MAIN);
            else Show_Message("Save Gagal!", "", 1500, MENU_MAIN);
            menu_cursor = 0;
        }
        if(menu_just_pressed) {
            current_state = MENU_MAIN;
            menu_cursor = 0;
        }
    }
}

void Menu_Edit_Value_Handler(void* value_to_edit, const char* title, SystemState mode) {
    sprintf(lcd_line1_buffer, title);
    
    switch(mode) {
        case MENU_SET_ALARM:
        case MENU_SET_CUTOFF:
            if(up_just_pressed) temp_setting_value += 1.0;
            if(down_just_pressed) temp_setting_value -= 1.0;
            if (temp_setting_value < 0) temp_setting_value = 0;
            sprintf(lcd_line2_buffer, "-> %.1f", temp_setting_value);
            break;
        case MENU_SET_PULSES:
            if(up_just_pressed) temp_pulses_value++;
            if(down_just_pressed) temp_pulses_value--;
            if(temp_pulses_value < 1) temp_pulses_value = 1;
            if(temp_pulses_value > 20) temp_pulses_value = 20;
            sprintf(lcd_line2_buffer, "-> %d", temp_pulses_value);
            break;
        default: break;
    }

    if(up_down_just_pressed) {
        if(mode == MENU_SET_PULSES) *((uint8_t*)value_to_edit) = temp_pulses_value;
        else *((float*)value_to_edit) = temp_setting_value;

        if (Save_Settings_To_Flash()) Show_Message("Pengaturan", "Tersimpan!", 1500, MENU_MAIN);
        else Show_Message("Save Gagal!", "", 1500, MENU_MAIN);
        menu_cursor = 0;
    }
    
    if(menu_just_pressed) {
        current_state = MENU_MAIN;
        menu_cursor = 0;
    }
}

void Show_Message(const char* line1, const char* line2, uint16_t delay, SystemState next_state) {
    current_state = SHOW_MESSAGE_MODE;
    next_state_after_message = next_state;
    message_delay = delay;
    message_start_time = HAL_GetTick();
    sprintf(lcd_line1_buffer, line1);
    sprintf(lcd_line2_buffer, line2);
}

void Update_LCD() {
    char temp_buffer[21];
    if (strcmp(prev_lcd_line1_buffer, lcd_line1_buffer) != 0) {
        sprintf(temp_buffer, "%-16s", lcd_line1_buffer);
        lcd_set_cursor(0, 0); lcd_send_string(temp_buffer);
        strcpy(prev_lcd_line1_buffer, lcd_line1_buffer);
    }
    if (strcmp(prev_lcd_line2_buffer, lcd_line2_buffer) != 0) {
        sprintf(temp_buffer, "%-16s", lcd_line2_buffer);
        lcd_set_cursor(1, 0); lcd_send_string(temp_buffer);
        strcpy(prev_lcd_line2_buffer, lcd_line2_buffer);
    }
}

bool Save_Settings_To_Flash() {
    HAL_NVIC_DisableIRQ(TIM2_IRQn);
    HAL_FLASH_Unlock();
    FLASH_EraseInitTypeDef EraseInitStruct = {FLASH_TYPEERASE_PAGES, SETTINGS_FLASH_PAGE_ADDR, 1};
    uint32_t PageError = 0;
    if (HAL_FLASHEx_Erase(&EraseInitStruct, &PageError) != HAL_OK) {
        HAL_FLASH_Lock(); HAL_NVIC_EnableIRQ(TIM2_IRQn); return false;
    }
    
    uint32_t address = SETTINGS_FLASH_PAGE_ADDR;
    uint32_t* data_ptr = (uint32_t*)&settings;
    for (uint_fast8_t i = 0; i < sizeof(Settings_t)/4; i++) {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address, data_ptr[i]) != HAL_OK) {
            HAL_FLASH_Lock(); HAL_NVIC_EnableIRQ(TIM2_IRQn); return false;
        }
        address += 4;
    }
    HAL_FLASH_Lock();
    HAL_NVIC_EnableIRQ(TIM2_IRQn);
    return true;
}

void Load_Settings_From_Flash() {
    Settings_t temp_settings;
    uint32_t address = SETTINGS_FLASH_PAGE_ADDR;
    uint32_t* data_ptr = (uint32_t*)&temp_settings;
    for (uint_fast8_t i = 0; i < sizeof(Settings_t)/4; i++) {
        data_ptr[i] = *(__IO uint32_t *)address;
        address += 4;
    }

    if (temp_settings.data_valid_magic == DATA_VALID_MAGIC_NUMBER) {
        settings = temp_settings;
        if (settings.calibration_factor <= 0.0f) settings.calibration_factor = DEFAULT_CAL_FACTOR;
        if (settings.pulses_per_rev == 0) settings.pulses_per_rev = 1;
    } else {
        settings.calibration_factor = DEFAULT_CAL_FACTOR;
        settings.alarm_speed = 15.0;
        settings.cutoff_speed = 20.0;
        settings.pulses_per_rev = 1;
        settings.data_valid_magic = DATA_VALID_MAGIC_NUMBER;
        Save_Settings_To_Flash();
    }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/
  HAL_Init();
  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  SystemClock_Config();
  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  lcd_init(&hi2c1);
  lcd_set_cursor(0, 2); lcd_send_string("DOKTER FORKLIFT"); HAL_Delay(1500);
  lcd_clear(); lcd_set_cursor(0, 0); lcd_send_string("Memuat data...");
  Load_Settings_From_Flash(); HAL_Delay(1000);
  lcd_clear();
  current_state = DISPLAY_MODE;
  
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
  last_pulse_time = HAL_GetTick();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    
    Read_Buttons();
    Process_Sensor_Data();

    switch(current_state) {
        case DISPLAY_MODE: Display_Mode_Handler(); break;
        case PASSWORD_MODE: Password_Mode_Handler(); break;
        case MENU_MAIN: Menu_Main_Handler(); break;
        case MENU_CALIBRATE: Menu_Calibrate_Handler(); break;
        case MENU_SET_ALARM: Menu_Edit_Value_Handler(&settings.alarm_speed, "Set Alarm Speed:", MENU_SET_ALARM); break;
        case MENU_SET_CUTOFF: Menu_Edit_Value_Handler(&settings.cutoff_speed, "Set Cutoff Speed:", MENU_SET_CUTOFF); break;
        case MENU_SET_PULSES: Menu_Edit_Value_Handler(&settings.pulses_per_rev, "Set Pulsa/Rev:", MENU_SET_PULSES); break;
        case MENU_FACTORY_RESET:
             settings.calibration_factor = DEFAULT_CAL_FACTOR;
             settings.alarm_speed = 15.0;
             settings.cutoff_speed = 20.0;
             settings.pulses_per_rev = 1;
             Save_Settings_To_Flash();
             Show_Message("Reset Berhasil!", "", 1500, MENU_MAIN);
             break;
        case SHOW_MESSAGE_MODE:
             if (HAL_GetTick() - message_start_time > message_delay) {
                 current_state = next_state_after_message;
             }
             break;
        default: current_state = DISPLAY_MODE; break;
    }

    if (HAL_GetTick() - last_display_update > 250) {
        Update_LCD();
        last_display_update = HAL_GetTick();
    }
    
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) Error_Handler();
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK) Error_Handler();
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{
  __HAL_RCC_TIM2_CLK_ENABLE();
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK) Error_Handler();
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) Error_Handler();
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK) Error_Handler();
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) Error_Handler();
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK) Error_Handler();
  HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM2_IRQn);
}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK) Error_Handler();
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET); 

  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  * where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */


