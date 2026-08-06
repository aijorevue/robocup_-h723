#include "lcd_display.h"

#include "board.h"
#include "lcd.h"
#include "lcd_background.h"
#include "route_controller.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

/* Route controller telemetry published for status display. */
extern volatile float g_yaw_rad;
extern volatile float g_estimated_distance_m;

/* ---------------------------------------------------------------------------
 * Timing budget
 *
 * The panel is driven byte-by-byte over SPI1 (HAL_SPI_Transmit, 1 byte per
 * call), so repainting even a small numeric field costs a few milliseconds.
 * The control loop runs at 100 Hz (10 ms), therefore:
 *
 *   - VBUS is refreshed at LCD_DISPLAY_VBUS_PERIOD_MS so supply dips are easy
 *     to see during high-current movement;
 *   - auxiliary fields are repainted one at a time at a lower rate, which keeps
 *     the worst-case blocking time bounded during the 100 Hz control loop;
 *   - static labels / rules are painted once in lcd_display_init();
 *   - LCD_Fill over the full frame and LCD_ShowPicture are never called from
 *     the control loop.
 *
 * ------------------------------------------------------------------------- */
#define LCD_DISPLAY_VBUS_PERIOD_MS 100U
#define LCD_DISPLAY_AUX_PERIOD_MS 500U
#define LCD_DISPLAY_JOYSTICK_PERIOD_MS 20U

#define LCD_FONT_TITLE 24U
#define LCD_FONT_ROW   16U

#define LCD_LABEL_X 8U
#define LCD_VALUE_X 160U
#define LCD_UNIT_X  240U

#define LCD_ROW_VBUS_Y 46U
#define LCD_ROW_YAW_Y  72U
#define LCD_ROW_DIST_Y 98U
#define LCD_ROW_RK_Y   124U
#define LCD_ROW_START_Y 150U
#define LCD_ROW_JOY_Y  176U
#define LCD_ROW_FIELD_Y 202U

#define LCD_TEXT_BACKGROUND BLACK

#define LCD_FIELD_VBUS 0U
#define LCD_FIELD_YAW  1U
#define LCD_FIELD_DIST 2U
#define LCD_FIELD_RK   3U
#define LCD_FIELD_COUNT 4U

#define RAD_TO_DEG 57.29577951308232f

static bool lcd_display_ready;
static uint32_t lcd_display_last_vbus_ms;
static uint32_t lcd_display_last_aux_ms;
static uint32_t lcd_display_last_joystick_ms;
static uint8_t lcd_display_field;
static float lcd_display_vbus_v;
static board_lcd_joystick_direction_t lcd_display_last_joystick;
static board_field_t lcd_display_last_field;

static void lcd_display_draw_static(void)
{
    LCD_ShowPicture(0U, 0U, LCD_BACKGROUND_W, LCD_BACKGROUND_H,
                    g_lcd_background);

    LCD_ShowString(LCD_LABEL_X, 6U, (const uint8_t *)"DM-MC02", CYAN,
                   LCD_TEXT_BACKGROUND, LCD_FONT_TITLE, 0U);
    LCD_DrawLine(0U, 36U, LCD_W - 1U, 36U, GRAY);

    LCD_ShowString(LCD_LABEL_X, LCD_ROW_VBUS_Y, (const uint8_t *)"VBUS", WHITE,
                   LCD_TEXT_BACKGROUND, LCD_FONT_ROW, 0U);
    LCD_ShowString(LCD_UNIT_X, LCD_ROW_VBUS_Y, (const uint8_t *)"V", WHITE,
                   LCD_TEXT_BACKGROUND, LCD_FONT_ROW, 0U);

    LCD_ShowString(LCD_LABEL_X, LCD_ROW_YAW_Y, (const uint8_t *)"YAW", WHITE,
                   LCD_TEXT_BACKGROUND, LCD_FONT_ROW, 0U);
    LCD_ShowString(LCD_UNIT_X, LCD_ROW_YAW_Y, (const uint8_t *)"deg", WHITE,
                   LCD_TEXT_BACKGROUND, LCD_FONT_ROW, 0U);

    LCD_ShowString(LCD_LABEL_X, LCD_ROW_DIST_Y, (const uint8_t *)"DIST", WHITE,
                   LCD_TEXT_BACKGROUND, LCD_FONT_ROW, 0U);
    LCD_ShowString(LCD_UNIT_X, LCD_ROW_DIST_Y, (const uint8_t *)"m", WHITE,
                   LCD_TEXT_BACKGROUND, LCD_FONT_ROW, 0U);

    LCD_ShowString(LCD_LABEL_X, LCD_ROW_RK_Y, (const uint8_t *)"RK", WHITE,
                   LCD_TEXT_BACKGROUND, LCD_FONT_ROW, 0U);

    LCD_ShowString(LCD_LABEL_X, LCD_ROW_START_Y, (const uint8_t *)"START",
                   WHITE, LCD_TEXT_BACKGROUND, LCD_FONT_ROW, 0U);
    lcd_display_set_start_status("WAIT");

    LCD_ShowString(LCD_LABEL_X, LCD_ROW_JOY_Y, (const uint8_t *)"JOY",
                   WHITE, LCD_TEXT_BACKGROUND, LCD_FONT_ROW, 0U);
    LCD_ShowString(LCD_LABEL_X, LCD_ROW_FIELD_Y, (const uint8_t *)"FIELD",
                   WHITE, LCD_TEXT_BACKGROUND, LCD_FONT_ROW, 0U);
}

static void lcd_display_draw_vbus(void)
{
    float voltage = board_read_bus_voltage();

    /* Light smoothing so the last digit does not flicker. */
    lcd_display_vbus_v += 0.25f * (voltage - lcd_display_vbus_v);
    if (lcd_display_vbus_v < 0.0f) {
        lcd_display_vbus_v = 0.0f;
    } else if (lcd_display_vbus_v > 99.0f) {
        lcd_display_vbus_v = 99.0f;
    }

    LCD_ShowFloatNum1(LCD_VALUE_X, LCD_ROW_VBUS_Y, lcd_display_vbus_v, 3U, 2U,
                      (lcd_display_vbus_v < 22.0f) ? RED : GREEN,
                      LCD_TEXT_BACKGROUND, LCD_FONT_ROW);
}

static void lcd_display_draw_yaw(void)
{
    float yaw_deg = g_yaw_rad * RAD_TO_DEG;

    if (yaw_deg < -999.0f) {
        yaw_deg = -999.0f;
    } else if (yaw_deg > 999.0f) {
        yaw_deg = 999.0f;
    }
    LCD_ShowFloatNum(LCD_VALUE_X, LCD_ROW_YAW_Y, yaw_deg, 3U, 1U, YELLOW,
                     LCD_TEXT_BACKGROUND, LCD_FONT_ROW);
}

static void lcd_display_draw_dist(void)
{
    float distance_m = g_estimated_distance_m;

    if (distance_m < -99.0f) {
        distance_m = -99.0f;
    } else if (distance_m > 99.0f) {
        distance_m = 99.0f;
    }
    LCD_ShowFloatNum(LCD_VALUE_X, LCD_ROW_DIST_Y, distance_m, 2U, 2U, WHITE,
                     LCD_TEXT_BACKGROUND, LCD_FONT_ROW);
}

static void lcd_display_draw_rk(void)
{
    if (route_controller_rk_link_ready() != 0U) {
        LCD_ShowString(LCD_VALUE_X + 8U, LCD_ROW_RK_Y, (const uint8_t *)"OK",
                       GREEN, LCD_TEXT_BACKGROUND, LCD_FONT_ROW, 0U);
    } else {
        LCD_ShowString(LCD_VALUE_X + 8U, LCD_ROW_RK_Y, (const uint8_t *)"--",
                       RED, LCD_TEXT_BACKGROUND, LCD_FONT_ROW, 0U);
    }
}

static const char *lcd_display_joystick_name(
    board_lcd_joystick_direction_t direction)
{
    switch (direction) {
    case BOARD_LCD_JOYSTICK_UP:
        return "UP";
    case BOARD_LCD_JOYSTICK_DOWN:
        return "DOWN";
    case BOARD_LCD_JOYSTICK_LEFT:
        return "LEFT";
    case BOARD_LCD_JOYSTICK_RIGHT:
        return "RIGHT";
    case BOARD_LCD_JOYSTICK_PRESS:
        return "PRESS";
    case BOARD_LCD_JOYSTICK_NONE:
    default:
        return "NONE";
    }
}

static void lcd_display_draw_joystick(void)
{
    const char *name = lcd_display_joystick_name(board_lcd_joystick_direction());

    LCD_ShowString(LCD_VALUE_X + 8U, LCD_ROW_JOY_Y, (const uint8_t *)"      ",
                   WHITE, LCD_TEXT_BACKGROUND, LCD_FONT_ROW, 0U);
    LCD_ShowString(LCD_VALUE_X + 8U, LCD_ROW_JOY_Y, (const uint8_t *)name,
                   YELLOW, LCD_TEXT_BACKGROUND, LCD_FONT_ROW, 0U);
}

static void lcd_display_draw_field(void)
{
    const char *name = "--";
    uint16_t color = WHITE;

    switch (board_selected_field()) {
    case BOARD_FIELD_RED:
        name = "RED";
        color = RED;
        break;
    case BOARD_FIELD_BLUE:
        name = "BLUE";
        color = BLUE;
        break;
    case BOARD_FIELD_UNKNOWN:
    default:
        break;
    }

    LCD_ShowString(LCD_VALUE_X + 8U, LCD_ROW_FIELD_Y, (const uint8_t *)"      ",
                   WHITE, LCD_TEXT_BACKGROUND, LCD_FONT_ROW, 0U);
    LCD_ShowString(LCD_VALUE_X + 8U, LCD_ROW_FIELD_Y, (const uint8_t *)name,
                   color, LCD_TEXT_BACKGROUND, LCD_FONT_ROW, 0U);
}

/* Temporary bring-up self test.  While enabled the firmware only paints a
 * solid red screen after LCD_Init() and never draws telemetry. */
#define LCD_DISPLAY_SELFTEST 0

#if LCD_DISPLAY_SELFTEST
static void lcd_display_selftest(void)
{
    LCD_Fill(0U, 0U, LCD_W, LCD_H, RED);
}
#endif

void lcd_display_init(void)
{
    LCD_Init();
#if LCD_DISPLAY_SELFTEST
    lcd_display_selftest();
    lcd_display_ready = false;
    return;
#endif
    lcd_display_draw_static();

    lcd_display_vbus_v = board_read_bus_voltage();
    lcd_display_field = LCD_FIELD_YAW;
    lcd_display_last_vbus_ms = HAL_GetTick() - LCD_DISPLAY_VBUS_PERIOD_MS;
    lcd_display_last_aux_ms = HAL_GetTick();
    lcd_display_last_joystick_ms =
        HAL_GetTick() - LCD_DISPLAY_JOYSTICK_PERIOD_MS;
    lcd_display_last_joystick = (board_lcd_joystick_direction_t)0xFFU;
    lcd_display_last_field = (board_field_t)0xFFU;
    lcd_display_ready = true;
}

void lcd_display_update(void)
{
    uint32_t now_ms;
    board_lcd_joystick_direction_t joystick;
    board_field_t field;

    if (!lcd_display_ready) {
        return;
    }
    now_ms = HAL_GetTick();
    if ((uint32_t)(now_ms - lcd_display_last_vbus_ms) >=
        LCD_DISPLAY_VBUS_PERIOD_MS) {
        lcd_display_last_vbus_ms = now_ms;
        lcd_display_draw_vbus();
    }

    if ((uint32_t)(now_ms - lcd_display_last_joystick_ms) >=
        LCD_DISPLAY_JOYSTICK_PERIOD_MS) {
        lcd_display_last_joystick_ms = now_ms;
        joystick = board_lcd_joystick_direction();
        field = board_selected_field();
        if (joystick != lcd_display_last_joystick) {
            lcd_display_last_joystick = joystick;
            lcd_display_draw_joystick();
        }
        if (field != lcd_display_last_field) {
            lcd_display_last_field = field;
            lcd_display_draw_field();
        }
    }

    if ((uint32_t)(now_ms - lcd_display_last_aux_ms) <
        LCD_DISPLAY_AUX_PERIOD_MS) {
        return;
    }
    lcd_display_last_aux_ms = now_ms;

    /* Auxiliary values are intentionally slower than VBUS. */
    switch (lcd_display_field) {
    case LCD_FIELD_YAW:
        lcd_display_draw_yaw();
        break;
    case LCD_FIELD_DIST:
        lcd_display_draw_dist();
        break;
    case LCD_FIELD_RK:
        lcd_display_draw_rk();
        break;
    default:
        lcd_display_draw_yaw();
        break;
    }
    lcd_display_field++;
    if (lcd_display_field >= LCD_FIELD_COUNT) {
        lcd_display_field = LCD_FIELD_YAW;
    }
}

void lcd_display_set_start_status(const char *status)
{
    if (status == NULL) {
        status = "--";
    }
    LCD_ShowString(LCD_VALUE_X + 8U, LCD_ROW_START_Y, (const uint8_t *)"      ",
                   WHITE, LCD_TEXT_BACKGROUND, LCD_FONT_ROW, 0U);
    LCD_ShowString(LCD_VALUE_X + 8U, LCD_ROW_START_Y, (const uint8_t *)status,
                   GREEN, LCD_TEXT_BACKGROUND, LCD_FONT_ROW, 0U);
}

void lcd_display_refresh_input_status(void)
{
    if (!lcd_display_ready) {
        return;
    }

    lcd_display_last_joystick = board_lcd_joystick_direction();
    lcd_display_last_field = board_selected_field();
    lcd_display_draw_joystick();
    lcd_display_draw_field();
}
