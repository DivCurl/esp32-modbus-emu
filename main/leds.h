// TX and RX activity LEDs. Not thread safe: call everything from the Modbus task.
#pragma once

void leds_init(void);
void leds_pulse_tx(void);
void leds_pulse_rx(void);

// Turns off expired pulses. Call at least every few milliseconds.
void leds_service(void);
