/*
 * pwm-sunxi.h
 *
 * (C) Copyright 2013
 * David H. Wilkins  <dwilkins@conecuh.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */


#define SUN4I_PWM_IOREG_MAX 10


/*
 * structure that defines the pwm control register
 */

typedef enum  {
  PRESCALE_INVALID = 0x00,
  PRESCALE_DIV120  = 0x00,
  PRESCALE_DIV180  = 0x01,
  PRESCALE_DIV240  = 0x02,
  PRESCALE_DIV360  = 0x03,
  PRESCALE_DIV480  = 0x04,
  PRESCALE_INVx05  = 0x05,
  PRESCALE_INVx06  = 0x06,
  PRESCALE_INVx07  = 0x07,
  PRESCALE_DIV12k  = 0x08,
  PRESCALE_DIV24k  = 0x09,
  PRESCALE_DIV36k  = 0x0a,
  PRESCALE_DIV48k  = 0x0b,
  PRESCALE_DIV72k  = 0x0c
} pwm_ctrl_prescale_t;




typedef struct {
  pwm_ctrl_prescale_t ch0_prescaler:4;
  unsigned int ch0_en:1;
  unsigned int ch0_act_state:1;
  unsigned int ch0_clk_gating:1;
  unsigned int ch0_mode:1;
  unsigned int ch0_pulse_start:1;
  unsigned int unused1:6;
  unsigned int ch1_prescaler:4;
  unsigned int ch1_en:1;
  unsigned int ch1_act_state:1;
  unsigned int ch1_clk_gating:1;
  unsigned int ch1_mode:1;
  unsigned int ch1_pulse_start:1;
/*  unsigned int unused:7; */
} sun4i_pwm_ctrl_t ;


#define A10CLK 24000000
#define MAX_CYCLES 0x0ffUL

#define NO_ENABLE_CHANGE 2

#define PWM_CTRL_ENABLE 1
#define PWM_CTRL_DISABLE 1

typedef struct {
  unsigned int pwm_active_cycles:8;
  unsigned int unused1:8;
  unsigned int pwm_entire_cycles:8;
  unsigned int unused2:8;
} sun4i_pwm_period_t;


typedef enum  {
  SELECT_INPUT      = 0x00,
  SELECT_OUTPUT     = 0x01,
  SELECT_PWM        = 0x02,
  SELECT_SPI2_CLK   = 0x02,
  SELECT_I2S_LRCK   = 0x02,
  SELECT_I2S_BCLK   = 0x02
} ioreg_pin_select_t;

typedef struct {
  ioreg_pin_select_t pin0_select:4;
  ioreg_pin_select_t pin1_select:4;
  ioreg_pin_select_t pin2_select:4;
  ioreg_pin_select_t pin3_select:4;
  ioreg_pin_select_t pin4_select:4;
  ioreg_pin_select_t pin5_select:4;
  ioreg_pin_select_t pin6_select:4;
  ioreg_pin_select_t pin7_select:4;
} ioreg_cfg0_t;


typedef struct {
  ioreg_pin_select_t pin8_select:4;
  ioreg_pin_select_t pin9_select:4;
  ioreg_pin_select_t pin10_select:4;
  ioreg_pin_select_t pin11_select:4;
  ioreg_pin_select_t pin12_select:4;
  ioreg_pin_select_t pin13_select:4;
  ioreg_pin_select_t pin14_select:4;
  ioreg_pin_select_t pin15_select:4;
} ioreg_cfg1_t;


typedef union {
  sun4i_pwm_ctrl_t s;
  unsigned int initializer;
} sun4i_pwm_ctrl_ut;

typedef union {
  sun4i_pwm_period_t s;
  unsigned int initializer;
} sun4i_pwm_period_ut;

typedef union {
  ioreg_cfg0_t s0;
  ioreg_cfg1_t s1;
  unsigned int initializer;
} ioreg_cfg_ut;


typedef struct {
  unsigned int *port_addr;
  ioreg_cfg_ut port;
} ioreg_backup_t;


typedef struct {
  void *ctrl_addr;
  void *pin_addr;
  void *period_reg_addr;
  unsigned int channel;
  unsigned long period;
  unsigned long duty;
  int duty_percent;
  pwm_ctrl_prescale_t prescale;
  sun4i_pwm_period_t period_reg;
  sun4i_pwm_ctrl_ut ctrl_backup;
  sun4i_pwm_ctrl_ut ctrl_mask;
  sun4i_pwm_ctrl_ut ctrl_current;
  ioreg_cfg_ut pin_backup;
  ioreg_cfg_ut pin_mask;
  ioreg_cfg_ut pin_current;
  char *pin_name;
  char *gpio_name;
} sun4i_pwm_available_channel_t;

typedef struct {
  char * suffix;
  unsigned long multiplier;
  bool freq;
} time_suffix_t;
