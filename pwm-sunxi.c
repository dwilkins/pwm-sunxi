/*
 * pwm-sunxi.c
 *
 * pwm module for sun4i (and others) like cubieboard and pcduino
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

#include <linux/kobject.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/sysfs.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <asm/io.h>
#include <asm/delay.h>
#include <mach/platform.h>
#include <linux/pwm.h>
#include <pwm-sunxi.h>


void dump_pwm_ctrl_reg(sun4i_pwm_ctrl_t *pwm, sun4i_pwm_ctrl_t *pwm_compare,const char *name);
void dump_pwm_period_reg(sun4i_pwm_period_t *period, sun4i_pwm_period_t *period_compare,const char *name);
void dump_ioreg_cfg(ioreg_cfg0_t *cfg,ioreg_cfg0_t *cfg_compare, const char *name);
const ioreg_cfg_ut * save_ioreg(void *ioreg);
const sun4i_pwm_ctrl_ut * save_pwm_ctrl(int channel);
void release_pwm_sunxi(struct kobject *kobj);
void setup_available_channels(void );
ssize_t set_pwm_mode(unsigned int enable, sun4i_pwm_available_channel_t *chan);
pwm_ctrl_prescale_t  get_best_prescale(sun4i_pwm_available_channel_t *chan);
unsigned int get_entire_cycles(unsigned prescale,  sun4i_pwm_available_channel_t *chan);
unsigned int get_active_cycles(unsigned prescale,  sun4i_pwm_available_channel_t *chan);



static DEFINE_MUTEX(sysfs_lock);
static struct class pwm_class;

struct kobject * pwm0_kobj;
struct kobject * pwm1_kobj;

static sun4i_pwm_ctrl_ut gs_pwm_ctrl_backup;
static unsigned int      gs_saved_ioregs;
static ioreg_backup_t     gs_ioreg_backup[SUN4I_PWM_IOREG_MAX];
void * PWM_CTRL_REG_BASE = NULL;


static struct class_attribute pwm_class_attrs[] = {
  /* __ATTR(pwm0.gpio3, 0555, NULL, NULL), */
  /* __ATTR(pwm1.gpio5, 0555, NULL, NULL), */
  __ATTR_NULL
};


static struct class pwm_class = {
  .name =         "pwm-sunxi",
  .owner =        THIS_MODULE,
  .class_attrs =  pwm_class_attrs,
};


static ssize_t pwm_polarity_show(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t pwm_period_show(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t pwm_duty_show(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t pwm_run_show(struct device *dev,struct device_attribute *attr, char *buf);
static ssize_t pwm_request_show(struct device *dev,struct device_attribute *attr, char *buf);
static ssize_t pwm_duty_percent_show(struct device *dev,struct device_attribute *attr, char *buf);
static ssize_t pwm_pulse_show(struct device *dev,struct device_attribute *attr, char *buf);
static ssize_t pwm_pin_show(struct device *dev,struct device_attribute *attr, char *buf);
static ssize_t pwm_gpio_show(struct device *dev,struct device_attribute *attr, char *buf);

static ssize_t pwm_polarity_store(struct device *dev,struct device_attribute *attr, const char *buf, size_t size);
static ssize_t pwm_period_store(struct device *dev,struct device_attribute *attr, const char *buf, size_t size);
static ssize_t pwm_duty_store(struct device *dev,struct device_attribute *attr, const char *buf, size_t size);
static ssize_t pwm_run_store(struct device *dev,struct device_attribute *attr, const char *buf, size_t size);
static ssize_t pwm_request_store(struct device *dev,struct device_attribute *attr, const char *buf, size_t size);
static ssize_t pwm_duty_percent_store(struct device *dev,struct device_attribute *attr, const char *buf, size_t size);
static ssize_t pwm_pulse_store(struct device *dev,struct device_attribute *attr, const char *buf, size_t size);

static DEVICE_ATTR(polarity, 0666,pwm_polarity_show, pwm_polarity_store);
static DEVICE_ATTR(period, 0666, pwm_period_show, pwm_period_store);
static DEVICE_ATTR(duty, 0666,pwm_duty_show, pwm_duty_store);
static DEVICE_ATTR(run, 0666, pwm_run_show, pwm_run_store);
static DEVICE_ATTR(request, 0666, pwm_request_show, pwm_request_store);
static DEVICE_ATTR(duty_percent, 0666, pwm_duty_percent_show, pwm_duty_percent_store);
static DEVICE_ATTR(pulse, 0666, pwm_pulse_show, pwm_pulse_store);
static DEVICE_ATTR(pin, 0666, pwm_pin_show, NULL);
static DEVICE_ATTR(gpio, 0666, pwm_gpio_show, NULL);

static const struct attribute *pwm_attrs[] = {
  &dev_attr_polarity.attr,
  &dev_attr_period.attr,
  &dev_attr_duty.attr,
  &dev_attr_run.attr,
  &dev_attr_request.attr,
  &dev_attr_duty_percent.attr,
  &dev_attr_pulse.attr,
  &dev_attr_pin.attr,
  &dev_attr_gpio.attr,
  NULL,
};

static const struct attribute_group pwm_attr_group = {
  .attrs = (struct attribute **) pwm_attrs
};

struct device * pwm0_gpio3;
struct device * pwm1_gpio5;


static sun4i_pwm_available_channel_t pwm_available_chan[2];

int init_module(void)
{
  int return_val;
  setup_available_channels();


  return_val = class_register(&pwm_class);
  if(return_val) {
    printk(KERN_INFO "pwm-sunxi: return from sysfs_create_group(pwm0.gpio3) was %d",return_val);
    class_unregister(&pwm_class);
  } else {
    printk(KERN_INFO "pwm_class.dev_kobj = %p",pwm_class.dev_kobj);
  }

  pwm0_gpio3 = device_create(&pwm_class,NULL,MKDEV(0,0),&pwm_available_chan[0],"pwm0.gpio3");
  pwm1_gpio5 = device_create(&pwm_class,NULL,MKDEV(0,0),&pwm_available_chan[1],"pwm1.gpio5");
  /* pwm0_kobj = kobject_create_and_add("pwm0.gpio3",pwm_class.dev_kobj); */
  pwm0_kobj = &pwm0_gpio3->kobj;
  pwm1_kobj = &pwm1_gpio5->kobj;
  return_val = sysfs_create_group(pwm0_kobj,&pwm_attr_group);
  if(return_val) {
    printk(KERN_INFO "pwm-sunxi: return from sysfs_create_group(pwm0.gpio3) was %d",return_val);
  }
  /* pwm1_kobj = kobject_create_and_add("pwm1.gpio5",pwm_class.dev_kobj); */
  return_val = sysfs_create_group(pwm1_kobj,&pwm_attr_group);
  if(return_val) {
    printk(KERN_INFO "pwm-sunxi: return from sysfs_create_group(pwm1.gpio5) was %d",return_val);
  }

  printk(KERN_INFO "pwm-sunxi:chan0 ctrl_addr = %p, pin_addr = %p, period_addr = %p ",
         pwm_available_chan[0].ctrl_addr,
         pwm_available_chan[0].pin_addr,
         pwm_available_chan[0].period_reg_addr);

  printk(KERN_INFO "pwm-sunxi:chan1 ctrl_addr = %p, pin_addr = %p, period_addr = %p ",
         pwm_available_chan[1].ctrl_addr,
         pwm_available_chan[1].pin_addr,
         pwm_available_chan[1].period_reg_addr);


  printk(KERN_INFO "pwm-sunxi: Initialized...");
  return return_val;
}


void cleanup_module(void)
{
  void * timer_base = ioremap(SW_PA_TIMERC_IO_BASE, 0x400);
  void * PWM_CTRL_REG_BASE = timer_base + 0x200;

  device_destroy(&pwm_class,pwm0_gpio3->devt);
  device_destroy(&pwm_class,pwm1_gpio5->devt);
  writel(0, PWM_CTRL_REG_BASE + 0);

  class_unregister(&pwm_class);
  printk(KERN_INFO "pwm: cleanup_module() called - unregister returned\n");
}

/*
 * Functions to display the pwm variables currently set
 */


static ssize_t pwm_polarity_show(struct device *dev, struct device_attribute *attr, char *buf) {
  const sun4i_pwm_available_channel_t *chan = dev_get_drvdata(dev);
  ssize_t status;
  switch (chan->channel) {
  case 0:
    status = sprintf(buf,"%d",chan->ctrl_current.s.ch0_act_state);
    break;
  case 1:
    status = sprintf(buf,"%d",chan->ctrl_current.s.ch1_act_state);
    break;
  default:
    status = -EINVAL;
    break;
  }

  return status;
}
static ssize_t pwm_period_show(struct device *dev, struct device_attribute *attr, char *buf) {
  const sun4i_pwm_available_channel_t *chan = dev_get_drvdata(dev);
  ssize_t status;
  status = sprintf(buf,"%lu",chan->period);
  return status;
}
static ssize_t pwm_duty_show(struct device *dev, struct device_attribute *attr, char *buf) {
  const sun4i_pwm_available_channel_t *chan = dev_get_drvdata(dev);
  ssize_t status;
  status = sprintf(buf,"%lu",chan->duty);
  return status;
}
static ssize_t pwm_run_show(struct device *dev,struct device_attribute *attr, char *buf) {
  const sun4i_pwm_available_channel_t *chan = dev_get_drvdata(dev);
  ssize_t status;
  switch (chan->channel) {
  case 0:
    status = sprintf(buf,"%d",chan->ctrl_current.s.ch0_en);
    break;
  case 1:
    status = sprintf(buf,"%d",chan->ctrl_current.s.ch1_en);
    break;
  default:
    status = -EINVAL;
    break;
  }

  return status;
}
static ssize_t pwm_request_show(struct device *dev,struct device_attribute *attr, char *buf) {
  return 0;
}
static ssize_t pwm_duty_percent_show(struct device *dev,struct device_attribute *attr, char *buf) {
  return 0;
}
static ssize_t pwm_pulse_show(struct device *dev,struct device_attribute *attr, char *buf) {
  const sun4i_pwm_available_channel_t *chan = dev_get_drvdata(dev);
  ssize_t status;
  switch (chan->channel) {
  case 0:
    status = sprintf(buf,"%d",chan->ctrl_current.s.ch0_pulse_start);
    break;
  case 1:
    status = sprintf(buf,"%d",chan->ctrl_current.s.ch1_pulse_start);
    break;
  default:
    status = -EINVAL;
    break;
  }
  return status;
}

static ssize_t pwm_pin_show(struct device *dev,struct device_attribute *attr, char *buf) {
  const sun4i_pwm_available_channel_t *chan = dev_get_drvdata(dev);
  ssize_t status;
  status = sprintf(buf,"%s",chan->pin_name);

  return status;
}

static ssize_t pwm_gpio_show(struct device *dev,struct device_attribute *attr, char *buf) {
  const sun4i_pwm_available_channel_t *chan = dev_get_drvdata(dev);
  ssize_t status;
  status = sprintf(buf,"%s",chan->gpio_name);
  return status;
}

/*
 * Functions to store values for pwm
 */

static ssize_t pwm_polarity_store(struct device *dev,struct device_attribute *attr, const char *buf, size_t size) {
  sun4i_pwm_available_channel_t *chan = dev_get_drvdata(dev);
  ssize_t status = -EINVAL;
  int act_state = 0;

  printk(KERN_INFO "pwm: dev = %p, attr = %p, buf = %p, chan = %p,  size = %d\n",dev,attr,buf,chan,size);

  sscanf(buf,"%d",&act_state);
  if(act_state < 2) {
    switch (chan->channel) {
    case 0:
      chan->ctrl_current.s.ch0_act_state = act_state;
      break;
    case 1:
      chan->ctrl_current.s.ch0_act_state = act_state;
      break;
    default:
      status = -EINVAL;
      break;
    }
    status = size;
  }
  return status;
}
static ssize_t pwm_period_store(struct device *dev,struct device_attribute *attr, const char *buf, size_t size) {
  unsigned long long period = 0;
  sun4i_pwm_available_channel_t *chan = dev_get_drvdata(dev);

  printk(KERN_INFO "pwm: dev = %p, attr = %p, buf = %p, chan = %p,  size = %d\n",dev,attr,buf,chan,size);

  sscanf(buf,"%Lu",&period); /* L means long long pointer */
  period = period > 4000000000 ? 4000000000 : period;
  chan->period = period;
  return size;
}
static ssize_t pwm_duty_store(struct device *dev,struct device_attribute *attr, const char *buf, size_t size) {
  unsigned long long duty = 0;
  sun4i_pwm_available_channel_t *chan = dev_get_drvdata(dev);

  printk(KERN_INFO "pwm: dev = %p, attr = %p, buf = %p, chan = %p,  size = %d\n",dev,attr,buf,chan,size);

  sscanf(buf,"%Lu",&duty); /* L means long long pointer */
  duty = duty > 4000000000 ? 4000000000 : duty;

  chan->duty = duty;
  return size;
}

static const unsigned long prescale_divisor[13] = {120,
                                                  180,
                                                  240,
                                                  360,
                                                  480,
                                                  1,
                                                  1,
                                                  1,
                                                  12000,
                                                  24000,
                                                  36000,
                                                  48000,
                                                  72000};

pwm_ctrl_prescale_t  get_best_prescale(sun4i_pwm_available_channel_t *chan) {
  int i;
  const unsigned long long max_cycles = 0x0ff; /* 255 */
  pwm_ctrl_prescale_t best_prescale = 0;

  for(i = 0 ; i < 13 ; i++) {
    printk(KERN_INFO "pwm: checking prescale for %Lu\n",(unsigned long long)((max_cycles * 24L * prescale_divisor[i])));
    if((chan->period) < ((max_cycles * 24L * prescale_divisor[i]))) {
      best_prescale = i;
      break;
    }
  }
  printk(KERN_INFO "pwm: best_prescale is %u for %lu\n",best_prescale,chan->period);
  printk(KERN_INFO "pwm: divisor is  is %lu\n",prescale_divisor[best_prescale]);
  return best_prescale;
}

unsigned int get_entire_cycles(unsigned prescale,  sun4i_pwm_available_channel_t *chan) {
  unsigned long  period_us = chan->period;
  unsigned int entire_cycles = 0x0ff;
  entire_cycles = period_us / prescale_divisor[prescale] / 24;
  return entire_cycles;
}

unsigned int get_active_cycles(unsigned prescale,  sun4i_pwm_available_channel_t *chan) {
  unsigned long duty_us = chan->duty;
  unsigned int active_cycles = 0x0ff;
  active_cycles = (duty_us / prescale_divisor[prescale]) / 24;
  return active_cycles;
}


static ssize_t pwm_run_store(struct device *dev,struct device_attribute *attr, const char *buf, size_t size) {
  sun4i_pwm_available_channel_t *chan = dev_get_drvdata(dev);
  ssize_t status = -EINVAL;
  int enable = 0;

  printk(KERN_INFO "pwm: dev = %p, attr = %p, buf = %p, chan = %p,  size = %d\n",dev,attr,buf,chan,size);

  sscanf(buf,"%d",&enable);
  if(enable < 2) {
    status = set_pwm_mode(enable, chan);
  }
  return size;
}
static ssize_t pwm_request_store(struct device *dev,struct device_attribute *attr, const char *buf, size_t size) {
  return size;
}
static ssize_t pwm_duty_percent_store(struct device *dev,struct device_attribute *attr, const char *buf, size_t size) {
  unsigned int duty_percent = 0;
  sun4i_pwm_available_channel_t *chan = dev_get_drvdata(dev);

  sscanf(buf,"%u",&duty_percent);
  chan->duty_percent = duty_percent;
  return size;
}
static ssize_t pwm_pulse_store(struct device *dev,struct device_attribute *attr, const char *buf, size_t size) {
  sun4i_pwm_available_channel_t *chan = dev_get_drvdata(dev);
  ssize_t status = -EINVAL;
  int pulse = 0;

  printk(KERN_INFO "pwm: dev = %p, attr = %p, buf = %p, chan = %p,  size = %d\n",dev,attr,buf,chan,size);

  sscanf(buf,"%d",&pulse);
  if(pulse < 2) {
    switch (chan->channel) {
    case 0:
      chan->ctrl_current.s.ch0_pulse_start = pulse;
      break;
    case 1:
      chan->ctrl_current.s.ch0_pulse_start = pulse;
      break;
    default:
      status = -EINVAL;
      break;
    }
    status = size;
  }
  return status;
}

ssize_t set_pwm_mode(unsigned int enable, sun4i_pwm_available_channel_t *chan) {
  ssize_t status = 0;
  printk(KERN_INFO "pwm-sunxi(top):chan0 ctrl_addr = %p, pin_addr = %p, period_addr = %p ",
         chan->ctrl_addr,
         chan->pin_addr,
         chan->period_reg_addr);
  if(enable) {
    unsigned int best_prescale = 0;
    switch (chan->channel) {
    case 0:
      chan->ctrl_current.s.ch0_en = enable;
      chan->ctrl_current.s.ch0_clk_gating = 0;
      break;
    case 1:
      chan->ctrl_current.s.ch1_en = enable;
      chan->ctrl_current.s.ch1_clk_gating = 0;
      break;
    default:
      status = -EINVAL;
      break;
    }
    if(status) {
      return status;
    }
    if(chan->pin_mask.s0.pin0_select) {
      chan->pin_current.s0.pin0_select = SELECT_PWM;
    } else if(chan->pin_mask.s0.pin1_select) {
      chan->pin_current.s0.pin1_select = SELECT_PWM;
    } else if(chan->pin_mask.s0.pin2_select) {
      chan->pin_current.s0.pin2_select = SELECT_PWM;
    } else if(chan->pin_mask.s0.pin3_select) {
      chan->pin_current.s0.pin3_select = SELECT_PWM;
    } else if(chan->pin_mask.s0.pin4_select) {
      chan->pin_current.s0.pin4_select = SELECT_PWM;
    } else if(chan->pin_mask.s0.pin5_select) {
      chan->pin_current.s0.pin5_select = SELECT_PWM;
    } else if(chan->pin_mask.s0.pin6_select) {
      chan->pin_current.s0.pin6_select = SELECT_PWM;
    } else if(chan->pin_mask.s0.pin7_select) {
      chan->pin_current.s0.pin7_select = SELECT_PWM;
    }
    best_prescale = get_best_prescale(chan);
    chan->period_reg.pwm_entire_cycles = get_entire_cycles(best_prescale,chan);
    chan->period_reg.pwm_active_cycles = get_active_cycles(best_prescale,chan);
    writel(*(unsigned int *)&chan->period_reg, chan->period_reg_addr);
    if(chan->channel == 0) {
      chan->ctrl_current.s.ch0_prescaler = best_prescale;
    } else {
      chan->ctrl_current.s.ch1_prescaler = best_prescale;
    }
    dump_ioreg_cfg(&chan->pin_current.s0,NULL,"pin_current");
    dump_pwm_period_reg(&chan->period_reg,NULL,"period");
    writel(chan->pin_current.initializer,chan->pin_addr);
    writel(chan->ctrl_current.initializer,chan->ctrl_addr);
    switch (chan->channel) {
    case 0:
      chan->ctrl_current.s.ch0_clk_gating = 1;
      break;
    case 1:
      chan->ctrl_current.s.ch1_clk_gating = 1;
      break;
    }
    writel(chan->ctrl_current.initializer,chan->ctrl_addr);
    dump_pwm_ctrl_reg(&chan->ctrl_current.s,NULL,"pwm_ctrl");
  } else {
    switch (chan->channel) {
    case 0:
      chan->ctrl_current.s.ch0_en = enable;
      break;
    case 1:
      chan->ctrl_current.s.ch0_en = enable;
      break;
    default:
      status = -EINVAL;
      break;
    }
    if(status) {
      return status;
    }
    chan->pin_current.initializer &= ~chan->pin_mask.initializer;
    chan->pin_current.initializer |= readl(chan->pin_addr) & chan->pin_mask.initializer;
  }
  printk(KERN_INFO "pwm-sunxi(bottom):chan0 ctrl_addr = %p, pin_addr = %p, period_addr = %p ",
         chan->ctrl_addr,
         chan->pin_addr,
         chan->period_reg_addr);

  return 0;
}


const ioreg_cfg_ut * save_ioreg(void *ioreg) {
  int i;
  const ioreg_cfg_ut *return_val = NULL;
  for(i = 0; i < gs_saved_ioregs;i++) {
    if(gs_ioreg_backup[i].port_addr == ioreg) {return_val =  NULL;}
    if(!gs_ioreg_backup[i].port_addr) {break;}
  }
  if(i < SUN4I_PWM_IOREG_MAX) {
    gs_saved_ioregs = i;
    gs_ioreg_backup[gs_saved_ioregs].port_addr = ioreg;
    gs_ioreg_backup[gs_saved_ioregs].port.initializer = readl(ioreg);
    return_val =  (const ioreg_cfg_ut *)&gs_ioreg_backup[i].port;
  }
  return return_val;
}

const sun4i_pwm_ctrl_ut * save_pwm_ctrl(int channel) {
  const sun4i_pwm_ctrl_ut *return_val;
  sun4i_pwm_ctrl_ut saved_regs;
  saved_regs.initializer = readl(PWM_CTRL_REG_BASE);
  switch (channel) {
  case 0:
    gs_pwm_ctrl_backup.s.ch0_prescaler   = saved_regs.s.ch0_prescaler;
    gs_pwm_ctrl_backup.s.ch0_en          = saved_regs.s.ch0_en;
    gs_pwm_ctrl_backup.s.ch0_clk_gating  = saved_regs.s.ch0_clk_gating;
    gs_pwm_ctrl_backup.s.ch0_mode        = saved_regs.s.ch0_mode;
    gs_pwm_ctrl_backup.s.ch0_pulse_start = saved_regs.s.ch0_pulse_start;
    return_val = &gs_pwm_ctrl_backup;
    break;
  case 1:
    gs_pwm_ctrl_backup.s.ch1_prescaler   = saved_regs.s.ch1_prescaler;
    gs_pwm_ctrl_backup.s.ch1_en          = saved_regs.s.ch1_en;
    gs_pwm_ctrl_backup.s.ch1_clk_gating  = saved_regs.s.ch1_clk_gating;
    gs_pwm_ctrl_backup.s.ch1_mode        = saved_regs.s.ch1_mode;
    gs_pwm_ctrl_backup.s.ch1_pulse_start = saved_regs.s.ch1_pulse_start;
    return_val = &gs_pwm_ctrl_backup;
    break;
  default:
    return_val = NULL;
  }
  return return_val;
}


void dump_ioreg_cfg(ioreg_cfg0_t *cfg,
                    ioreg_cfg0_t *cfg_compare,
                    const char *name) {

  const char * cfg_pin0_select_diff = "";
  const char * cfg_pin1_select_diff = "";
  const char * cfg_pin2_select_diff = "";
  const char * cfg_pin3_select_diff = "";
  const char * cfg_pin4_select_diff = "";
  const char * cfg_pin5_select_diff = "";
  const char * cfg_pin6_select_diff = "";
  const char * cfg_pin7_select_diff = "";


  if(!cfg || !name) {
    return;
  }


  if(cfg && cfg_compare) {
    cfg_pin0_select_diff = (const char *)(cfg->pin0_select != cfg_compare->pin0_select ? " ******" : "");
    cfg_pin1_select_diff = (const char *)(cfg->pin1_select != cfg_compare->pin1_select ? " ******" : "");
    cfg_pin2_select_diff = (const char *)(cfg->pin2_select != cfg_compare->pin2_select ? " ******" : "");
    cfg_pin3_select_diff = (const char *)(cfg->pin3_select != cfg_compare->pin3_select ? " ******" : "");
    cfg_pin4_select_diff = (const char *)(cfg->pin4_select != cfg_compare->pin4_select ? " ******" : "");
    cfg_pin5_select_diff = (const char *)(cfg->pin5_select != cfg_compare->pin5_select ? " ******" : "");
    cfg_pin6_select_diff = (const char *)(cfg->pin6_select != cfg_compare->pin6_select ? " ******" : "");
    cfg_pin7_select_diff = (const char *)(cfg->pin7_select != cfg_compare->pin7_select ? " ******" : "");
  }

  printk(KERN_INFO "%s: pin0( 8)_select  :   0x%x %s",name,cfg->pin0_select,cfg_pin0_select_diff);
  printk(KERN_INFO "%s: pin1( 9)_select  :   0x%x %s",name,cfg->pin1_select,cfg_pin1_select_diff);
  printk(KERN_INFO "%s: pin2(10)_select  :   0x%x %s",name,cfg->pin2_select,cfg_pin2_select_diff);
  printk(KERN_INFO "%s: pin3(11)_select  :   0x%x %s",name,cfg->pin3_select,cfg_pin3_select_diff);
  printk(KERN_INFO "%s: pin4(12)_select  :   0x%x %s",name,cfg->pin4_select,cfg_pin4_select_diff);
  printk(KERN_INFO "%s: pin5(13)_select  :   0x%x %s",name,cfg->pin5_select,cfg_pin5_select_diff);
  printk(KERN_INFO "%s: pin6(14)_select  :   0x%x %s",name,cfg->pin6_select,cfg_pin6_select_diff);
  printk(KERN_INFO "%s: pin7(15)_select  :   0x%x %s",name,cfg->pin7_select,cfg_pin7_select_diff);
  printk(KERN_INFO "%s: pin_ctl: 0x%x",name,*(unsigned int *)cfg);

}



void dump_pwm_period_reg(sun4i_pwm_period_t *period,
                         sun4i_pwm_period_t *period_compare,const char *name) {

  const char * period_entire_cycles_diff = "";
  const char * period_active_cycles_diff = "";

  if(!period || !name) {
    return;
  }


  if(period && period_compare) {
    period_entire_cycles_diff  = (const char *)(period->pwm_entire_cycles != period_compare->pwm_entire_cycles ? " ******" : "");
    period_entire_cycles_diff  = (const char *)(period->pwm_active_cycles != period_compare->pwm_active_cycles ? " ******" : "");
  }

  printk(KERN_INFO "%s: entire_cycles:   0x%x %s",name,period->pwm_entire_cycles,period_entire_cycles_diff);
  printk(KERN_INFO "%s: active_cycles:   0x%x %s",name,period->pwm_active_cycles,period_active_cycles_diff);
  printk(KERN_INFO "%s: period_reg: 0x%x",name,*(unsigned int *)period);

}


void dump_pwm_ctrl_reg(sun4i_pwm_ctrl_t *pwm,
                       sun4i_pwm_ctrl_t *pwm_compare,const char *name) {
  const char * ch0_prescaler_diff = "";
  const char * ch0_en_diff = "";
  const char * ch0_act_state_diff = "";
  const char * ch0_clk_gating_diff = "";
  const char * ch0_mode_diff = "";
  const char * ch0_pulse_start_diff = "";

  const char * ch1_prescaler_diff = "";
  const char * ch1_en_diff = "";
  const char * ch1_act_state_diff = "";
  const char * ch1_clk_gating_diff = "";
  const char * ch1_mode_diff = "";
  const char * ch1_pulse_start_diff = "";

  if(!pwm || !name) {
    return;
  }


if(pwm && pwm_compare) {
  ch0_prescaler_diff =   (const char *)(pwm->ch0_prescaler != pwm_compare->ch0_prescaler ? " ******" : "");
  ch0_en_diff =          (const char *)(pwm->ch0_en != pwm_compare->ch0_en ? " ******" : "");
  ch0_act_state_diff =   (const char *)(pwm->ch0_act_state != pwm_compare->ch0_act_state ? " ******" : "");
  ch0_clk_gating_diff =  (const char *)(pwm->ch0_clk_gating != pwm_compare->ch0_clk_gating ? " ******" : "");
  ch0_mode_diff =        (const char *)(pwm->ch0_mode != pwm_compare->ch0_mode ? " ******" : "");
  ch0_pulse_start_diff = (const char *)(pwm->ch0_pulse_start != pwm_compare->ch0_pulse_start ? " ******" : "");

  ch1_prescaler_diff =   (const char *)(pwm->ch1_prescaler != pwm_compare->ch1_prescaler ? " ******" : "");
  ch1_en_diff =          (const char *)(pwm->ch1_en != pwm_compare->ch1_en ? " ******" : "");
  ch1_act_state_diff =   (const char *)(pwm->ch1_act_state != pwm_compare->ch1_act_state ? " ******" : "");
  ch1_clk_gating_diff =  (const char *)(pwm->ch1_clk_gating != pwm_compare->ch1_clk_gating ? " ******" : "");
  ch1_mode_diff =        (const char *)(pwm->ch1_mode != pwm_compare->ch1_mode ? " ******" : "");
  ch1_pulse_start_diff = (const char *)(pwm->ch1_pulse_start != pwm_compare->ch1_pulse_start ? " ******" : "");
 }


 printk(KERN_INFO "%s: ch0_prescaler:   0x%x %s",name,pwm->ch0_prescaler,ch0_prescaler_diff);
 printk(KERN_INFO "%s: ch0_en:          0x%x %s",name,pwm->ch0_en,ch0_en_diff);
 printk(KERN_INFO "%s: ch0_act_state:   0x%x %s",name,pwm->ch0_act_state,ch0_act_state_diff);
 printk(KERN_INFO "%s: ch0_clk_gating:  0x%x %s",name,pwm->ch0_clk_gating,ch0_clk_gating_diff);
 printk(KERN_INFO "%s: ch0_mode:        0x%x %s",name,pwm->ch0_mode,ch0_mode_diff);
 printk(KERN_INFO "%s: ch0_pulse_start: 0x%x %s",name,pwm->ch0_pulse_start,ch0_pulse_start_diff);


 printk(KERN_INFO "%s: ch1_prescaler:   0x%x %s",name,pwm->ch1_prescaler,ch1_prescaler_diff);
 printk(KERN_INFO "%s: ch1_en:          0x%x %s",name,pwm->ch1_en,ch1_en_diff);
 printk(KERN_INFO "%s: ch1_act_state:   0x%x %s",name,pwm->ch1_act_state,ch1_act_state_diff);
 printk(KERN_INFO "%s: ch1_clk_gating:  0x%x %s",name,pwm->ch1_clk_gating,ch1_clk_gating_diff);
 printk(KERN_INFO "%s: ch1_mode:        0x%x %s",name,pwm->ch1_mode,ch1_mode_diff);
 printk(KERN_INFO "%s: ch1_pulse_start: 0x%x %s",name,pwm->ch1_pulse_start,ch1_pulse_start_diff);


 printk(KERN_INFO "%s: ch1_ctrl_reg: 0x%x",name,*(unsigned int *)pwm);
 printk(KERN_INFO "%s: sizeof(*pwm) = %d",name,sizeof(*pwm));
}


void setup_available_channels( void ) {
  void * timer_base = ioremap(SW_PA_TIMERC_IO_BASE, 0x400);
  void * PWM_CTRL_REG_BASE = timer_base + 0x200;
  void * portc_io_base = ioremap(SW_PA_PORTC_IO_BASE,0x400);
  void * PB_CFG0_REG = (portc_io_base + 0x24); /* 0x24 */
  void * PI_CFG0_REG = (portc_io_base + 0x120);
  void * PH_CFG0_REG = (portc_io_base + 0xfc);
  void * PH_CFG1_REG = (portc_io_base + 0x100);

  pwm_available_chan[0].ctrl_addr = PWM_CTRL_REG_BASE;
  pwm_available_chan[0].pin_addr = PB_CFG0_REG;
  pwm_available_chan[0].period_reg_addr = pwm_available_chan[0].ctrl_addr + 0x04;
  pwm_available_chan[0].channel = 0;
  pwm_available_chan[0].ctrl_backup.initializer = readl(pwm_available_chan[0].ctrl_addr);
  pwm_available_chan[0].ctrl_mask.initializer = 0;
  pwm_available_chan[0].ctrl_mask.s.ch0_prescaler = 0x0f;
  pwm_available_chan[0].ctrl_mask.s.ch0_en = 0x01;
  pwm_available_chan[0].ctrl_mask.s.ch0_act_state = 0x01;
  pwm_available_chan[0].ctrl_mask.s.ch0_clk_gating = 0x01;
  pwm_available_chan[0].ctrl_mask.s.ch0_mode = 0x01;
  pwm_available_chan[0].ctrl_mask.s.ch0_pulse_start = 0x01;
  pwm_available_chan[0].ctrl_current.initializer = 0;
  pwm_available_chan[0].pin_backup.initializer = readl(pwm_available_chan[0].pin_addr);
  pwm_available_chan[0].pin_mask.initializer = 0;
  pwm_available_chan[0].pin_mask.s0.pin2_select = 0x07;
  pwm_available_chan[0].pin_current.s0.pin2_select = 0x02;
  pwm_available_chan[0].pin_name = "PB2";
  pwm_available_chan[0].gpio_name = "gpio3";


  pwm_available_chan[1].ctrl_addr = PWM_CTRL_REG_BASE;
  pwm_available_chan[1].pin_addr = PI_CFG0_REG;
  pwm_available_chan[1].period_reg_addr = pwm_available_chan[1].ctrl_addr + 0x08;
  pwm_available_chan[1].channel = 0;
  pwm_available_chan[1].ctrl_backup.initializer = readl(pwm_available_chan[1].ctrl_addr);
  pwm_available_chan[1].ctrl_mask.initializer = 0;
  pwm_available_chan[1].ctrl_mask.s.ch1_prescaler = 0x0f;
  pwm_available_chan[1].ctrl_mask.s.ch1_en = 0x01;
  pwm_available_chan[1].ctrl_mask.s.ch1_act_state = 0x01;
  pwm_available_chan[1].ctrl_mask.s.ch1_clk_gating = 0x01;
  pwm_available_chan[1].ctrl_mask.s.ch1_mode = 0x01;
  pwm_available_chan[1].ctrl_mask.s.ch1_pulse_start = 0x01;
  pwm_available_chan[1].ctrl_current.initializer = 0;
  pwm_available_chan[1].pin_backup.initializer = readl(pwm_available_chan[1].pin_addr);
  pwm_available_chan[1].pin_mask.initializer = 0;
  pwm_available_chan[1].pin_mask.s0.pin3_select = 0x07;
  pwm_available_chan[1].pin_current.s0.pin3_select = 0x02;
  pwm_available_chan[1].pin_name = "PI3";
  pwm_available_chan[1].gpio_name = "gpio5";
}


MODULE_LICENSE("GPL");
MODULE_AUTHOR("David H. Wilkins");