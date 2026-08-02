#include "board.h"

#include "app_config.h"
#include "lcd_pins.h"
#include "main.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"

#include <string.h>

ADC_HandleTypeDef hadc1;
FDCAN_HandleTypeDef hfdcan1;
SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
UART_HandleTypeDef huart1;

volatile uint32_t g_fdcan_tx_error_count;
volatile uint32_t g_fdcan_bus_off_count;
volatile uint32_t g_fdcan_abort_error_count;

#define FDCAN_ABORT_TIMEOUT_MS 2U
#define FDCAN_ABORT_MAX_POLLS 200000UL

static bool fdcan_abort_unresolved;
static bool usb_device_started;

static uint32_t apb_timer_clock_hz(uint32_t pclk_hz)
{
    if (pclk_hz == HAL_RCC_GetHCLKFreq()) {
        return pclk_hz;
    }
    return pclk_hz * 2U;
}

static uint32_t tim1_clock_hz(void)
{
    return apb_timer_clock_hz(HAL_RCC_GetPCLK2Freq());
}

static uint32_t tim2_clock_hz(void)
{
    return apb_timer_clock_hz(HAL_RCC_GetPCLK1Freq());
}

static void system_clock_config(void)
{
    RCC_OscInitTypeDef oscillator = {0};
    RCC_ClkInitTypeDef clocks = {0};

    HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);
    while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {
    }

    oscillator.OscillatorType = RCC_OSCILLATORTYPE_HSE | RCC_OSCILLATORTYPE_HSI48;
    oscillator.HSEState = RCC_HSE_ON;
    oscillator.HSI48State = RCC_HSI48_ON;
    oscillator.PLL.PLLState = RCC_PLL_ON;
    oscillator.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    oscillator.PLL.PLLM = 2U;
    oscillator.PLL.PLLN = 40U;
    oscillator.PLL.PLLP = 1U;
    oscillator.PLL.PLLQ = 6U;
    oscillator.PLL.PLLR = 2U;
    oscillator.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
    oscillator.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
    oscillator.PLL.PLLFRACN = 0U;
    if (HAL_RCC_OscConfig(&oscillator) != HAL_OK) {
        Error_Handler();
    }

    clocks.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                       RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2 |
                       RCC_CLOCKTYPE_D3PCLK1 | RCC_CLOCKTYPE_D1PCLK1;
    clocks.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    clocks.SYSCLKDivider = RCC_SYSCLK_DIV1;
    clocks.AHBCLKDivider = RCC_HCLK_DIV2;
    clocks.APB3CLKDivider = RCC_APB3_DIV2;
    clocks.APB1CLKDivider = RCC_APB1_DIV2;
    clocks.APB2CLKDivider = RCC_APB2_DIV2;
    clocks.APB4CLKDivider = RCC_APB4_DIV2;
    if (HAL_RCC_ClockConfig(&clocks, FLASH_LATENCY_3) != HAL_OK) {
        Error_Handler();
    }
}

static void gpio_init(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    HAL_GPIO_WritePin(GPIOC, ACC_CS_Pin | GYRO_CS_Pin, GPIO_PIN_SET);
    gpio.Pin = ACC_CS_Pin | GYRO_CS_Pin;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOC, &gpio);

    HAL_GPIO_WritePin(PWM_5V_EN_GPIO_Port, PWM_5V_EN_Pin, GPIO_PIN_SET);
    gpio.Pin = PWM_5V_EN_Pin;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(PWM_5V_EN_GPIO_Port, &gpio);

    gpio.Pin = GPIO_PIN_15;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &gpio);
}

static void spi2_init(void)
{
    hspi2.Instance = SPI2;
    hspi2.Init.Mode = SPI_MODE_MASTER;
    hspi2.Init.Direction = SPI_DIRECTION_2LINES;
    hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
    hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
    hspi2.Init.NSS = SPI_NSS_SOFT;
    hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
    hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi2.Init.CRCPolynomial = 0U;
    hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
    hspi2.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
    hspi2.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
    hspi2.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
    hspi2.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
    hspi2.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
    hspi2.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
    hspi2.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
    hspi2.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
    hspi2.Init.IOSwap = SPI_IO_SWAP_DISABLE;
    if (HAL_SPI_Init(&hspi2) != HAL_OK) {
        Error_Handler();
    }
}

static void spi1_init(void)
{
    hspi1.Instance = SPI1;
    hspi1.Init.Mode = SPI_MODE_MASTER;
    hspi1.Init.Direction = SPI_DIRECTION_2LINES_TXONLY;
    hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
    hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi1.Init.NSS = SPI_NSS_SOFT;
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
    hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi1.Init.CRCPolynomial = 0x0U;
    hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
    hspi1.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
    hspi1.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
    hspi1.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
    hspi1.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
    hspi1.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
    hspi1.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
    hspi1.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
    hspi1.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
    hspi1.Init.IOSwap = SPI_IO_SWAP_DISABLE;
    if (HAL_SPI_Init(&hspi1) != HAL_OK) {
        Error_Handler();
    }
}

static void lcd_gpio_init(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();

    HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LCD_BLK_GPIO_Port, LCD_BLK_Pin | LCD_RES_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LCD_DC_GPIO_Port, LCD_DC_Pin, GPIO_PIN_RESET);

    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;

    gpio.Pin = LCD_CS_Pin;
    HAL_GPIO_Init(LCD_CS_GPIO_Port, &gpio);

    gpio.Pin = LCD_BLK_Pin | LCD_RES_Pin;
    HAL_GPIO_Init(LCD_BLK_GPIO_Port, &gpio);

    gpio.Pin = LCD_DC_Pin;
    HAL_GPIO_Init(LCD_DC_GPIO_Port, &gpio);
}

static void adc1_init(void)
{
    ADC_MultiModeTypeDef multimode = {0};
    ADC_ChannelConfTypeDef channel = {0};

    hadc1.Instance = ADC1;
    hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV64;
    hadc1.Init.Resolution = ADC_RESOLUTION_16B;
    hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
    hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    hadc1.Init.LowPowerAutoWait = DISABLE;
    hadc1.Init.ContinuousConvMode = ENABLE;
    hadc1.Init.NbrOfConversion = 1U;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
    hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
    hadc1.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
    hadc1.Init.OversamplingMode = DISABLE;
    if (HAL_ADC_Init(&hadc1) != HAL_OK) {
        Error_Handler();
    }

    multimode.Mode = ADC_MODE_INDEPENDENT;
    if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK) {
        Error_Handler();
    }

    channel.Channel = ADC_CHANNEL_4;
    channel.Rank = ADC_REGULAR_RANK_1;
    channel.SamplingTime = ADC_SAMPLETIME_32CYCLES_5;
    channel.SingleDiff = ADC_SINGLE_ENDED;
    channel.OffsetNumber = ADC_OFFSET_NONE;
    channel.Offset = 0U;
    if (HAL_ADC_ConfigChannel(&hadc1, &channel) != HAL_OK) {
        Error_Handler();
    }

    if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_ADC_Start(&hadc1) != HAL_OK) {
        Error_Handler();
    }
}

static uint32_t adc1_read_channel_raw(uint32_t channel_id)
{
    ADC_ChannelConfTypeDef channel = {0};
    uint32_t raw;

    (void)HAL_ADC_Stop(&hadc1);
    channel.Channel = channel_id;
    channel.Rank = ADC_REGULAR_RANK_1;
    channel.SamplingTime = ADC_SAMPLETIME_32CYCLES_5;
    channel.SingleDiff = ADC_SINGLE_ENDED;
    channel.OffsetNumber = ADC_OFFSET_NONE;
    channel.Offset = 0U;
    if (HAL_ADC_ConfigChannel(&hadc1, &channel) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_ADC_Start(&hadc1) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_ADC_PollForConversion(&hadc1, 2U) != HAL_OK) {
        return 0xFFFFFFFFUL;
    }
    raw = HAL_ADC_GetValue(&hadc1);
    return raw;
}

float board_read_bus_voltage(void)
{
    uint32_t raw = adc1_read_channel_raw(ADC_CHANNEL_4);

    if (raw > 65535UL) {
        raw = 0U;
    }

    return ((float)raw * 3.3f / 65535.0f) * 11.0f;
}

static uint8_t board_lcd_joystick_active(void)
{
    uint32_t raw = adc1_read_channel_raw(ADC_CHANNEL_19);

    if (raw > 65535UL) {
        return 0U;
    }
    /* Official CtrBoard-H7_KEY thresholds are 12-bit.  This project keeps
     * ADC1 at 16-bit for VBUS, so scale those windows by 16. */
    if (raw < 3200UL) {
        return 1U; /* middle press */
    }
    if (raw > 11200UL && raw < 16000UL) {
        return 1U; /* right */
    }
    if (raw > 24000UL && raw < 28800UL) {
        return 1U; /* left */
    }
    if (raw > 35200UL && raw < 40000UL) {
        return 1U; /* up */
    }
    if (raw > 44800UL && raw < 56000UL) {
        return 1U; /* down */
    }
    return 0U;
}

uint8_t board_user_start_pressed(void)
{
    typedef enum {
        USER_KEY_IDLE = 0,
        USER_KEY_DEBOUNCING,
        USER_KEY_LATCHED
    } user_key_state_t;
    static user_key_state_t state = USER_KEY_IDLE;
    static uint32_t pressed_since_ms;
    uint32_t now_ms = HAL_GetTick();
    uint8_t active = board_lcd_joystick_active();
    GPIO_PinState pin = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15);

    if (pin == GPIO_PIN_RESET) {
        active = 1U;
    }

    switch (state) {
    case USER_KEY_IDLE:
        if (active != 0U) {
            pressed_since_ms = now_ms;
            state = USER_KEY_DEBOUNCING;
        }
        break;
    case USER_KEY_DEBOUNCING:
        if (active == 0U) {
            state = USER_KEY_IDLE;
        } else if ((uint32_t)(now_ms - pressed_since_ms) >=
                   ROUTE_USER_KEY_DEBOUNCE_MS) {
            state = USER_KEY_LATCHED;
            return 1U;
        }
        break;
    case USER_KEY_LATCHED:
    default:
        if (active == 0U) {
            state = USER_KEY_IDLE;
        }
        break;
    }
    return 0U;
}

static void uart1_init(void)
{
    huart1.Instance = USART1;
    huart1.Init.BaudRate = 115200U;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
    huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    if (HAL_UART_Init(&huart1) != HAL_OK ||
        HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK ||
        HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK ||
        HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK) {
        Error_Handler();
    }
}

static void fdcan1_init(void)
{
    hfdcan1.Instance = FDCAN1;
    hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
    hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
    hfdcan1.Init.AutoRetransmission = ENABLE;
    hfdcan1.Init.TransmitPause = DISABLE;
    hfdcan1.Init.ProtocolException = DISABLE;
    hfdcan1.Init.NominalPrescaler = FDCAN_NOM_PRESCALER;
    hfdcan1.Init.NominalSyncJumpWidth = FDCAN_NOM_SJW;
    hfdcan1.Init.NominalTimeSeg1 = FDCAN_NOM_SEG1;
    hfdcan1.Init.NominalTimeSeg2 = FDCAN_NOM_SEG2;
    hfdcan1.Init.DataPrescaler = 1U;
    hfdcan1.Init.DataSyncJumpWidth = 1U;
    hfdcan1.Init.DataTimeSeg1 = 1U;
    hfdcan1.Init.DataTimeSeg2 = 1U;
    hfdcan1.Init.MessageRAMOffset = 0U;
    hfdcan1.Init.StdFiltersNbr = 0U;
    hfdcan1.Init.ExtFiltersNbr = 0U;
    hfdcan1.Init.RxFifo0ElmtsNbr = 16U;
    hfdcan1.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;
    hfdcan1.Init.RxFifo1ElmtsNbr = 0U;
    hfdcan1.Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_8;
    hfdcan1.Init.RxBuffersNbr = 0U;
    hfdcan1.Init.RxBufferSize = FDCAN_DATA_BYTES_8;
    hfdcan1.Init.TxEventsNbr = 0U;
    hfdcan1.Init.TxBuffersNbr = 0U;
    hfdcan1.Init.TxFifoQueueElmtsNbr = 8U;
    hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
    hfdcan1.Init.TxElmtSize = FDCAN_DATA_BYTES_8;
    if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK ||
        HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_REJECT,
                                     FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE) != HAL_OK ||
        HAL_FDCAN_Start(&hfdcan1) != HAL_OK) {
        Error_Handler();
    }
}

static void servo_pwm_init(void)
{
    GPIO_InitTypeDef gpio = {0};
    TIM_MasterConfigTypeDef master = {0};
    TIM_OC_InitTypeDef channel = {0};
    uint32_t tim1_clock = tim1_clock_hz();
    uint32_t tim2_clock = tim2_clock_hz();

    if (tim1_clock < SERVO_PWM_TIMER_HZ || tim2_clock < SERVO_PWM_TIMER_HZ) {
        Error_Handler();
    }

    __HAL_RCC_TIM1_CLK_ENABLE();
    __HAL_RCC_TIM2_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();

    htim1.Instance = TIM1;
    htim1.Init.Prescaler = (tim1_clock / SERVO_PWM_TIMER_HZ) - 1U;
    htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim1.Init.Period = SERVO_PWM_PERIOD_US - 1U;
    htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0U;
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) {
        Error_Handler();
    }

    htim2.Instance = TIM2;
    htim2.Init.Prescaler = (tim2_clock / SERVO_PWM_TIMER_HZ) - 1U;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = SERVO_PWM_PERIOD_US - 1U;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_PWM_Init(&htim2) != HAL_OK) {
        Error_Handler();
    }

    master.MasterOutputTrigger = TIM_TRGO_RESET;
    master.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &master) != HAL_OK ||
        HAL_TIMEx_MasterConfigSynchronization(&htim2, &master) != HAL_OK) {
        Error_Handler();
    }

    channel.OCMode = TIM_OCMODE_PWM1;
    channel.Pulse = 0U;
    channel.OCPolarity = TIM_OCPOLARITY_HIGH;
    channel.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    channel.OCIdleState = TIM_OCIDLESTATE_RESET;
    channel.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    channel.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim2, &channel, TIM_CHANNEL_1) != HAL_OK ||
        HAL_TIM_PWM_ConfigChannel(&htim2, &channel, TIM_CHANNEL_3) != HAL_OK ||
        HAL_TIM_PWM_ConfigChannel(&htim1, &channel, TIM_CHANNEL_1) != HAL_OK ||
        HAL_TIM_PWM_ConfigChannel(&htim1, &channel, TIM_CHANNEL_3) != HAL_OK) {
        Error_Handler();
    }

    gpio.Pin = GPIO_PIN_0 | GPIO_PIN_2;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    gpio.Alternate = GPIO_AF1_TIM2;
    HAL_GPIO_Init(GPIOA, &gpio);

    gpio.Pin = GPIO_PIN_9 | GPIO_PIN_13;
    gpio.Alternate = GPIO_AF1_TIM1;
    HAL_GPIO_Init(GPIOE, &gpio);

    if (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1) != HAL_OK ||
        HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3) != HAL_OK ||
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) != HAL_OK ||
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3) != HAL_OK) {
        Error_Handler();
    }
}

void board_init(void)
{
    SCB_EnableICache();
    SCB_EnableDCache();
    HAL_Init();
    system_clock_config();
    gpio_init();
    spi2_init();
    lcd_gpio_init();
    spi1_init();
    adc1_init();
    uart1_init();
    servo_pwm_init();
    MX_USB_DEVICE_Init();
    usb_device_started = true;
    fdcan_abort_unresolved = false;
    fdcan1_init();
}

void board_init_task_link_only(void)
{
    SCB_EnableICache();
    SCB_EnableDCache();
    HAL_Init();
    system_clock_config();
    uart1_init();
    MX_USB_DEVICE_Init();
    usb_device_started = true;
    fdcan_abort_unresolved = false;
}

static uint32_t servo_angle_to_pulse_us(float angle_deg)
{
    float normalized;
    const uint32_t pulse_range_us =
        SERVO_MG90S_MAX_PULSE_US - SERVO_MG90S_MIN_PULSE_US;

    if (angle_deg < 0.0f) {
        angle_deg = 0.0f;
    } else if (angle_deg > SERVO_MG90S_MAX_ANGLE_DEG) {
        angle_deg = SERVO_MG90S_MAX_ANGLE_DEG;
    }
    normalized = angle_deg / SERVO_MG90S_MAX_ANGLE_DEG;
    return SERVO_MG90S_MIN_PULSE_US +
           (uint32_t)(normalized * (float)pulse_range_us + 0.5f);
}

void board_servo_set_angle_deg_index(uint8_t servo_index, float angle_deg)
{
    uint32_t pulse_us = servo_angle_to_pulse_us(angle_deg);

    if (pulse_us > SERVO_PWM_PERIOD_US) {
        pulse_us = SERVO_PWM_PERIOD_US;
    }

    switch (servo_index) {
    case 0U:
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pulse_us);
        break;
    case 1U:
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, pulse_us);
        break;
    case 2U:
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pulse_us);
        break;
    case 3U:
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, pulse_us);
        break;
    default:
        break;
    }
}

void board_servo_disable_index(uint8_t servo_index)
{
    switch (servo_index) {
    case 0U:
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0U);
        break;
    case 1U:
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0U);
        break;
    case 2U:
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0U);
        break;
    case 3U:
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0U);
        break;
    default:
        break;
    }
}

void board_servo_set_angle_deg(float angle_deg)
{
    board_servo_set_angle_deg_index(0U, angle_deg);
}

void board_uart1_write_only(const char *text)
{
    size_t length;

    if (text == NULL) {
        return;
    }
    length = strlen(text);
    if (length > UINT16_MAX) {
        length = UINT16_MAX;
    }
    (void)HAL_UART_Transmit(&huart1, (const uint8_t *)text, (uint16_t)length, 1000U);
}

void board_uart1_write(const char *text)
{
    board_uart1_write_only(text);
    board_usb_write(text);
}

void board_usb_write(const char *text)
{
    static uint8_t tx_buffer[128];
    size_t length;
    size_t offset = 0U;

    if (!usb_device_started || text == NULL) {
        return;
    }
    length = strlen(text);
    while (offset < length) {
        uint32_t started_ms = HAL_GetTick();
        size_t chunk = length - offset;

        if (chunk > sizeof(tx_buffer)) {
            chunk = sizeof(tx_buffer);
        }
        memcpy(tx_buffer, text + offset, chunk);
        while (CDC_Transmit_HS(tx_buffer, (uint16_t)chunk) != USBD_OK) {
            if ((uint32_t)(HAL_GetTick() - started_ms) > 20U) {
                return;
            }
        }
        offset += chunk;
    }
}

void HAL_UART_MspInit(UART_HandleTypeDef *uart)
{
    GPIO_InitTypeDef gpio = {0};

    if (uart == NULL || uart->Instance != USART1) {
        return;
    }
    __HAL_RCC_USART1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    gpio.Pin = GPIO_PIN_9 | GPIO_PIN_10;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &gpio);
}

static bool fdcan_protocol_healthy(void)
{
    FDCAN_ProtocolStatusTypeDef status = {0};

    if (HAL_FDCAN_GetProtocolStatus(&hfdcan1, &status) != HAL_OK) {
        g_fdcan_tx_error_count++;
        return false;
    }
    if (status.BusOff != 0U) {
        g_fdcan_bus_off_count++;
        return false;
    }
    return true;
}

bool board_fdcan1_abort_all_pending(void)
{
    uint32_t pending = hfdcan1.Instance->TXBRP & FDCAN_TXBRP_TRP;
    uint32_t started_ms;
    uint32_t polls = 0U;

    if (pending == 0U) {
        fdcan_abort_unresolved = false;
        return true;
    }
    if (HAL_FDCAN_AbortTxRequest(&hfdcan1, pending) != HAL_OK) {
        g_fdcan_abort_error_count++;
        fdcan_abort_unresolved = true;
        return false;
    }
    started_ms = HAL_GetTick();
    while ((hfdcan1.Instance->TXBRP & pending) != 0U) {
        polls++;
        if ((uint32_t)(HAL_GetTick() - started_ms) >= FDCAN_ABORT_TIMEOUT_MS ||
            polls >= FDCAN_ABORT_MAX_POLLS) {
            g_fdcan_abort_error_count++;
            fdcan_abort_unresolved = true;
            return false;
        }
    }
    fdcan_abort_unresolved = false;
    return true;
}

bool board_fdcan1_wait_tx_fifo_free(uint32_t min_free, uint32_t timeout_ms)
{
    const uint32_t started_ms = HAL_GetTick();

    if (min_free == 0U || min_free > 8U) {
        return false;
    }
    while ((uint32_t)(HAL_GetTick() - started_ms) < timeout_ms) {
        if (!fdcan_abort_unresolved && fdcan_protocol_healthy() &&
            HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) >= min_free) {
            return true;
        }
    }
    g_fdcan_tx_error_count++;
    return false;
}

bool board_fdcan1_send_classic_std8_batch4(const board_can_frame_t frames[4])
{
    uint32_t request_mask = 0U;
    uint32_t i;

    if (frames == NULL || fdcan_abort_unresolved || !fdcan_protocol_healthy() ||
        !board_fdcan1_wait_tx_fifo_free(4U, 50U)) {
        g_fdcan_tx_error_count++;
        return false;
    }
    for (i = 0U; i < 4U; ++i) {
        if (frames[i].data == NULL || frames[i].standard_id > 0x7FFU) {
            g_fdcan_tx_error_count++;
            return false;
        }
    }
    for (i = 0U; i < 4U; ++i) {
        FDCAN_TxHeaderTypeDef header = {0};

        header.Identifier = frames[i].standard_id;
        header.IdType = FDCAN_STANDARD_ID;
        header.TxFrameType = FDCAN_DATA_FRAME;
        header.DataLength = FDCAN_DLC_BYTES_8;
        header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
        header.BitRateSwitch = FDCAN_BRS_OFF;
        header.FDFormat = FDCAN_CLASSIC_CAN;
        header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
        if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &header, frames[i].data) != HAL_OK) {
            g_fdcan_tx_error_count++;
            if (request_mask != 0U) {
                (void)HAL_FDCAN_AbortTxRequest(&hfdcan1, request_mask);
            }
            return false;
        }
        request_mask |= HAL_FDCAN_GetLatestTxFifoQRequestBuffer(&hfdcan1);
    }
    return true;
}

bool board_fdcan1_read_classic_std8(board_can_rx_frame_t *frame)
{
    FDCAN_RxHeaderTypeDef header = {0};

    if (frame == NULL) {
        return false;
    }
    if (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0) == 0U) {
        return false;
    }
    if (HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &header, frame->data) != HAL_OK) {
        g_fdcan_tx_error_count++;
        return false;
    }
    if (header.IdType != FDCAN_STANDARD_ID || header.FDFormat != FDCAN_CLASSIC_CAN ||
        header.DataLength != FDCAN_DLC_BYTES_8) {
        return false;
    }
    frame->standard_id = (uint16_t)header.Identifier;
    return true;
}

void HAL_SPI_MspInit(SPI_HandleTypeDef *spi)
{
    GPIO_InitTypeDef gpio = {0};
    RCC_PeriphCLKInitTypeDef clock = {0};

    if (spi == NULL) {
        return;
    }

    if (spi->Instance == SPI1) {
        /* LCD bus: SCK = PB3, MOSI = PD7, transmit only. */
        clock.PeriphClockSelection = RCC_PERIPHCLK_SPI1;
        clock.PLL2.PLL2M = 2U;
        clock.PLL2.PLL2N = 16U;
        clock.PLL2.PLL2P = 2U;
        clock.PLL2.PLL2Q = 2U;
        clock.PLL2.PLL2R = 2U;
        clock.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_3;
        clock.PLL2.PLL2VCOSEL = RCC_PLL2VCOWIDE;
        clock.PLL2.PLL2FRACN = 0U;
        clock.Spi123ClockSelection = RCC_SPI123CLKSOURCE_PLL2;
        if (HAL_RCCEx_PeriphCLKConfig(&clock) != HAL_OK) {
            Error_Handler();
        }
        __HAL_RCC_SPI1_CLK_ENABLE();
        __HAL_RCC_GPIOB_CLK_ENABLE();
        __HAL_RCC_GPIOD_CLK_ENABLE();

        gpio.Pin = GPIO_PIN_3;
        gpio.Mode = GPIO_MODE_AF_PP;
        gpio.Pull = GPIO_NOPULL;
        gpio.Speed = GPIO_SPEED_FREQ_LOW;
        gpio.Alternate = GPIO_AF5_SPI1;
        HAL_GPIO_Init(GPIOB, &gpio);

        gpio.Pin = GPIO_PIN_7;
        HAL_GPIO_Init(GPIOD, &gpio);
        return;
    }

    if (spi->Instance != SPI2) {
        return;
    }
    clock.PeriphClockSelection = RCC_PERIPHCLK_SPI2;
    clock.PLL2.PLL2M = 2U;
    clock.PLL2.PLL2N = 16U;
    clock.PLL2.PLL2P = 2U;
    clock.PLL2.PLL2Q = 2U;
    clock.PLL2.PLL2R = 2U;
    clock.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_3;
    clock.PLL2.PLL2VCOSEL = RCC_PLL2VCOWIDE;
    clock.PLL2.PLL2FRACN = 0U;
    clock.Spi123ClockSelection = RCC_SPI123CLKSOURCE_PLL2;
    if (HAL_RCCEx_PeriphCLKConfig(&clock) != HAL_OK) {
        Error_Handler();
    }
    __HAL_RCC_SPI2_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    gpio.Pin = GPIO_PIN_1 | GPIO_PIN_2;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init(GPIOC, &gpio);

    gpio.Pin = GPIO_PIN_13;
    HAL_GPIO_Init(GPIOB, &gpio);
}

void HAL_ADC_MspInit(ADC_HandleTypeDef *adc)
{
    GPIO_InitTypeDef gpio = {0};
    RCC_PeriphCLKInitTypeDef clock = {0};

    if (adc == NULL || adc->Instance != ADC1) {
        return;
    }
    clock.PeriphClockSelection = RCC_PERIPHCLK_ADC;
    clock.PLL2.PLL2M = 2U;
    clock.PLL2.PLL2N = 16U;
    clock.PLL2.PLL2P = 2U;
    clock.PLL2.PLL2Q = 2U;
    clock.PLL2.PLL2R = 2U;
    clock.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_3;
    clock.PLL2.PLL2VCOSEL = RCC_PLL2VCOWIDE;
    clock.PLL2.PLL2FRACN = 0U;
    clock.AdcClockSelection = RCC_ADCCLKSOURCE_PLL2;
    if (HAL_RCCEx_PeriphCLKConfig(&clock) != HAL_OK) {
        Error_Handler();
    }
    __HAL_RCC_ADC12_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    /* PA5 ------> ADC1_INP19, LCD joystick resistor ladder in the official
     * CtrBoard-H7_KEY example. */
    gpio.Pin = GPIO_PIN_5;
    gpio.Mode = GPIO_MODE_ANALOG;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &gpio);

    /* PC4 ------> ADC1_INP4, VBUS divider in the DM-MC02 reference project. */
    gpio.Pin = GPIO_PIN_4;
    gpio.Mode = GPIO_MODE_ANALOG;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &gpio);
}

void HAL_FDCAN_MspInit(FDCAN_HandleTypeDef *fdcan)
{
    GPIO_InitTypeDef gpio = {0};
    RCC_PeriphCLKInitTypeDef clock = {0};

    if (fdcan == NULL || fdcan->Instance != FDCAN1) {
        return;
    }
    clock.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
    clock.FdcanClockSelection = RCC_FDCANCLKSOURCE_PLL;
    if (HAL_RCCEx_PeriphCLKConfig(&clock) != HAL_OK) {
        Error_Handler();
    }
    __HAL_RCC_FDCAN_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    gpio.Pin = GPIO_PIN_0 | GPIO_PIN_1;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    gpio.Alternate = GPIO_AF9_FDCAN1;
    HAL_GPIO_Init(GPIOD, &gpio);
}

void HAL_MspInit(void)
{
    __HAL_RCC_SYSCFG_CLK_ENABLE();
}

void SysTick_Handler(void)
{
    HAL_IncTick();
}

extern PCD_HandleTypeDef hpcd_USB_OTG_HS;

void OTG_HS_IRQHandler(void)
{
    HAL_PCD_IRQHandler(&hpcd_USB_OTG_HS);
}
