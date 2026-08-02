#ifndef TEST1_BOARD_H
#define TEST1_BOARD_H

#include <stdbool.h>
#include <stdint.h>

#include "stm32h7xx_hal.h"

typedef struct {
    uint16_t standard_id;
    uint8_t *data;
} board_can_frame_t;

typedef struct {
    uint16_t standard_id;
    uint8_t data[8];
} board_can_rx_frame_t;

extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi2;
extern ADC_HandleTypeDef hadc1;
extern volatile uint32_t g_fdcan_tx_error_count;
extern volatile uint32_t g_fdcan_bus_off_count;
extern volatile uint32_t g_fdcan_abort_error_count;

void board_init(void);
void board_init_task_link_only(void);
void board_uart1_write_only(const char *text);
void board_uart1_write(const char *text);
void board_usb_write(const char *text);
void board_servo_set_angle_deg_index(uint8_t servo_index, float angle_deg);
void board_servo_disable_index(uint8_t servo_index);
void board_servo_set_angle_deg(float angle_deg);
float board_read_bus_voltage(void);
uint8_t board_user_start_pressed(void);
bool board_fdcan1_abort_all_pending(void);
bool board_fdcan1_wait_tx_fifo_free(uint32_t min_free, uint32_t timeout_ms);
bool board_fdcan1_send_classic_std8_batch4(const board_can_frame_t frames[4]);
bool board_fdcan1_read_classic_std8(board_can_rx_frame_t *frame);
void Error_Handler(void);

#endif
