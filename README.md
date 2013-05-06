pwm-sunxi
=========

Kernel Module to support pwm for the sun4i (and others???)

Creates a sysfs structure like the following:

   -- pwm-sunxi
      |
      +----pwmX.gpioY
               |
               +---duty
               +---duty_percent
               +---gpio
               +---period
               +---polarity
               +---pulse
               +---pin
               +---run


* period (r/w)
  period that makes up a cycle.  Can be expressed as hz, khz, ms, or us.  Whole numbers only.Examples:

  echo 10hz > /sys/class/pwm-sunxi/pwm0.gpio3/period
  echo 1khz > /sys/class/pwm-sunxi/pwm0.gpio3/period
  echo 100ms > /sys/class/pwm-sunxi/pwm0.gpio3/period
  echo 100us > /sys/class/pwm-sunxi/pwm0.gpio3/period
  echo 150khz > /sys/class/pwm-sunxi/pwm0.gpio3/period

* duty (r/w)
  portion of the period above that is "active" or on.  Same units as above

* duty_percent (r/w)
  duty expressed as a percentage.  Whole numbers only

* polarity(r/w)
  polarity of the pin during the duty portion.
  1 = high, 0 = low during

* pulse (r/w)
  Output one pulse at the specified period and duty

* pin (ro)
  Name of the A10 pin this pwm outputs on.  This is hardwired and informational only.
  Example:  PB2

* gpio (ro)
  Name of the GPIO pin this pwm outputs on.  This is hardwired and informational only.
  Example: gpio3

* run (r/w)
  Enable the PWM with the previously set parameters.  Example:

  echo 1 > /sys/class/pwm-sunxi/pwm0.gpio3/run
  echo 0 > /sys/class/pwm-sunxi/pwm0.gpio3/run
