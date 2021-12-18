/**
Fading LED example code.

This example increases and decreases the lighting level of a LED over and over again.

How to wire the LED: 
  - Place a 220ohm resistor in between the anode of the LED and GPIO19 and connect the cathode to ground.
  - The resistor limits the current that flows through the LED and the ESP32, increasing their life.
*/

import gpio.pwm as gpio
import gpio
import gpio.adc

/// The GPIO pin for the LED.
LED_GREEN ::= 4
LED_RED ::= 0
LED_BLUE ::= 2

/// The configured range of the PWM.
DUTY_MIN ::= 0.0
DUTY_MAX ::= 1.0
CHANGE_RATE ::= 0.01  // The amount of change every 5ms.

/// The PIN for the photo resistor.
PHOTO_RESISTOR_PIN ::= 32

/// Lighting threshold. Change the threshold to fit your lighting conditions.
THRESHOLD ::= 0.8

main:
  led_pwm ::= gpio.Pwm --frequency=100
  led_green := gpio.Pin LED_GREEN
  led_red := gpio.Pin LED_RED
  led_blue := gpio.Pin LED_BLUE
  fading_led_green := led_pwm.start led_green
  fading_led_red := led_pwm.start led_red
  fading_led_blue := led_pwm.start led_blue

  step := CHANGE_RATE
  duty := DUTY_MIN
  
  pin := gpio.Pin PHOTO_RESISTOR_PIN
  sensor := adc.Adc pin
  while true:

    on_off := sensor.get > THRESHOLD ? "on" : "off"

    if (sensor.get > THRESHOLD) and (duty >= DUTY_MIN):
      duty -= step
    else if (sensor.get <= THRESHOLD) and (duty <= DUTY_MAX):
      duty += step

    fading_led_green.set_duty_factor(duty)
    fading_led_red.set_duty_factor(duty)
    fading_led_blue.set_duty_factor(duty)

    sleep --ms=50


