pwm-sunxi
=========

Kernel Module to support pwm for the sun4i (and others???)

Creates a sysfs structure like the following:

   -- pwm-sunxi
      |
      -----pwmX.gpioY
               |
               +---period
               +---duty
               +---duty_percent
               +---polarity
               +---pulse
               +---pin
               +---gpio


* period (r/w)
  number of microseconds that comprise a cycle.
* duty (r/w)
  number of microseconds that the pulse will have the polarity
  specified.
* duty_percent (r/w)
  percentage of period that the pulse will have the polarity
  specified
* polarity(r/w)
  polarity of the pin during the duty portion.
  1 = high, 0 = low during
* pin (r/w)
