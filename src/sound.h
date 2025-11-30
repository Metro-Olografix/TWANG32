/*
 *  This creates sound tones by outputting a square wave on a DAC pin. The
 *  volume of the tone is the level of the DAC pin.
 *
 *  The square wave is created by a timer. The timer runs at 2x the freq, because
 *  it needs to transition high and then low.
 *
 *  Updated for new ESP32 Arduino Core 3.0+
 */
#ifndef SOUND_H
#define SOUND_H

#include "esp32-hal-timer.h"

#define ESP32_F_CPU 80000000 // the speed of the processor
#define AUDIO_INTERRUPT_PRESCALER 80
#define SOUND_TIMER_NO 0
#define MIN_FREQ 20
#define MAX_FREQ 16000

hw_timer_t *sndTimer = NULL;

bool sound_on = true;
bool sound_wave_high = true; // this toggles to create the high/low transitions of the wave
uint8_t sound_volume = 0;

void sound_init(int pin);
bool sound(uint16_t freq, uint8_t volume);
void soundOff();

int dac_pin;

void IRAM_ATTR onSoundTimer()
{
	if (sound_on)
	{
		dacWrite(dac_pin, (sound_wave_high ? sound_volume : 0));
		sound_wave_high = !sound_wave_high;
	}
	else
		dacWrite(dac_pin, 0);
}

void sound_init(int pin)
{ // pin must be a DAC pin number !! (typically 25 or 26)
	dac_pin = pin;
	sound_on = false;
	pinMode(dac_pin, OUTPUT);
	sound_volume = 0;

	// New timer API for ESP32 Arduino Core 3.0+
	sndTimer = timerBegin(1000000); // 1MHz base frequency
	timerAttachInterrupt(sndTimer, &onSoundTimer);
	timerAlarm(sndTimer, 1000000 / MIN_FREQ / 2, true, 0); // Set initial frequency
}

void sound_pause() // this prevents the interrupt from firing ... use during eeprom write
{
	if (sndTimer != NULL)
		timerStop(sndTimer);
}

void sound_resume() // resume from pause ... after eeprom write
{
	if (sndTimer != NULL)
		timerRestart(sndTimer);
}

bool sound(uint16_t freq, uint8_t volume)
{
	if (volume == 0)
	{
		soundOff();
		return false;
	}
	if (freq < MIN_FREQ || freq > MAX_FREQ)
	{
		return false;
	}
	sound_on = true;
	sound_volume = volume;
	// Set timer frequency (divide by 2 for square wave)
	timerAlarm(sndTimer, 1000000 / (freq * 2), true, 0);
	return true;
}

void soundOff()
{
	sound_on = false;
	sound_volume = 0;
	timerAlarm(sndTimer, 1000000 / MIN_FREQ / 2, true, 0); // Reset to low frequency
}

#endif