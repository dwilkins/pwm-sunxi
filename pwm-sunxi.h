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

enum pwm_ctrl_prescale {
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
};




struct sun4i_pwm_ctrl {
  enum pwm_ctrl_prescale ch0_prescaler:4;
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
};


#define A10CLK 24000000
#define MAX_CYCLES 0x0ffUL

#define NO_ENABLE_CHANGE 2

#define PWM_CTRL_ENABLE 1
#define PWM_CTRL_DISABLE 1

struct sun4i_pwm_period {
  unsigned int pwm_active_cycles:8;
  unsigned int unused1:8;
  unsigned int pwm_entire_cycles:8;
  unsigned int unused2:8;
};


enum ioreg_pin_select {
  SELECT_INPUT      = 0x00,
  SELECT_OUTPUT     = 0x01,
  SELECT_PWM        = 0x02,
  SELECT_SPI2_CLK   = 0x02,
  SELECT_I2S_LRCK   = 0x02,
  SELECT_I2S_BCLK   = 0x02
};

struct ioreg_cfg0 {
  enum ioreg_pin_select pin0_select:4;
  enum ioreg_pin_select pin1_select:4;
  enum ioreg_pin_select pin2_select:4;
  enum ioreg_pin_select pin3_select:4;
  enum ioreg_pin_select pin4_select:4;
  enum ioreg_pin_select pin5_select:4;
  enum ioreg_pin_select pin6_select:4;
  enum ioreg_pin_select pin7_select:4;
};


struct ioreg_cfg1 {
  enum  ioreg_pin_select pin8_select:4;
  enum  ioreg_pin_select pin9_select:4;
  enum  ioreg_pin_select pin10_select:4;
  enum  ioreg_pin_select pin11_select:4;
  enum  ioreg_pin_select pin12_select:4;
  enum  ioreg_pin_select pin13_select:4;
  enum  ioreg_pin_select pin14_select:4;
  enum  ioreg_pin_select pin15_select:4;
};


union sun4i_pwm_ctrl_u {
  struct sun4i_pwm_ctrl s;
  unsigned int initializer;
};

union sun4i_pwm_period_u {
  struct sun4i_pwm_period s;
  unsigned int initializer;
};

union ioreg_cfg_u {
  struct ioreg_cfg0 s0;
  struct ioreg_cfg1 s1;
  unsigned int initializer;
};


struct ioreg_backup {
  unsigned int *port_addr;
  union ioreg_cfg_u port;
};


struct sun4i_pwm_available_channel{
  void *ctrl_addr;
  void *pin_addr;
  void *period_reg_addr;
  unsigned int channel;
  unsigned long period;
  unsigned long duty;
  int duty_percent;
  enum pwm_ctrl_prescale prescale;
  union sun4i_pwm_period_u period_reg;
  union sun4i_pwm_ctrl_u ctrl_backup;
  union sun4i_pwm_ctrl_u ctrl_mask;
  union sun4i_pwm_ctrl_u ctrl_current;
  union ioreg_cfg_u pin_backup;
  union ioreg_cfg_u pin_mask;
  union ioreg_cfg_u pin_current;
  char *pin_name;
  char *gpio_name;
};

struct time_suffix {
  char * suffix;
  unsigned long multiplier;
  bool freq;
};
