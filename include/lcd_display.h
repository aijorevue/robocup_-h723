#ifndef TEST1_LCD_DISPLAY_H
#define TEST1_LCD_DISPLAY_H

/* Initialise the ST7789 panel and paint the static labels once. */
void lcd_display_init(void);

/* Refresh the dynamic values.  Safe to call from the 100 Hz control loop:
 * the function self-throttles to LCD_DISPLAY_PERIOD_MS and only repaints
 * the small value fields, never the whole frame. */
void lcd_display_update(void);
void lcd_display_set_start_status(const char *status);

#endif
