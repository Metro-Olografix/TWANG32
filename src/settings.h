#ifndef SETTINGS_H
#define SETTINGS_H

#include <EEPROM.h>
#include "sound.h"

// Version 2 adds the number of LEDs
// change this whenever the saved settings are not compatible with a change
// It forces a reset from defaults.
#define SETTINGS_VERSION 2
#define EEPROM_SIZE 512

// LEDS
#define NUM_LEDS 144
#define MIN_LEDS 60

#define DEFAULT_BRIGHTNESS 150
#define MIN_BRIGHTNESS 10
#define MAX_BRIGHTNESS 255

// PLAYER
const uint8_t MAX_PLAYER_SPEED = 10; // Max move speed of the player
const uint8_t LIVES_PER_LEVEL = 3;	 // default lives per level
#define MIN_LIVES_PER_LEVEL 3
#define MAX_LIVES_PER_LEVEL 9

// JOYSTICK - kept for compatibility but uses button values
#define JOYSTICK_ORIENTATION 1		   // 0, 1 or 2 to set the axis of the joystick
#define JOYSTICK_DIRECTION 1		   // 0/1 to flip joystick direction
#define DEFAULT_ATTACK_THRESHOLD 30000 // The threshold that triggers an attack
#define MIN_ATTACK_THRESHOLD 20000
#define MAX_ATTACK_THRESHOLD 30000

#define DEFAULT_JOYSTICK_DEADZONE 8 // Angle to ignore
#define MIN_JOYSTICK_DEADZONE 3
#define MAX_JOYSTICK_DEADZONE 12

// AUDIO
#define DEFAULT_VOLUME 20 // 0 to 255
#define MIN_VOLUME 0
#define MAX_VOLUME 255
#define DAC_AUDIO_PIN 25 // should be 25 or 26 only

enum ErrorNums
{
	ERR_SETTING_NUM,
	ERR_SETTING_RANGE
};

long lastInputTime = 0;

// Function prototypes
void settings_init();
void settings_eeprom_write();
void settings_eeprom_read();
void reset_settings();

typedef struct
{
	uint8_t settings_version; // stores the settings format version

	uint16_t led_count;
	uint8_t led_brightness;

	uint8_t joystick_deadzone;
	uint16_t attack_threshold;

	uint8_t audio_volume;

	uint8_t lives_per_level;

	// saved statistics
	uint16_t games_played;
	uint32_t total_points;
	uint16_t high_score;
	uint16_t boss_kills;

} settings_t;

settings_t user_settings;

void settings_init()
{
	settings_eeprom_read();
	Serial.println("\r\n====== TWANG Settings ========");
	Serial.print("LED Count: ");
	Serial.println(user_settings.led_count);
	Serial.print("LED Brightness: ");
	Serial.println(user_settings.led_brightness);
	Serial.print("Sound Volume: ");
	Serial.println(user_settings.audio_volume);
	Serial.print("Lives per Level: ");
	Serial.println(user_settings.lives_per_level);
	Serial.print("Games played: ");
	Serial.println(user_settings.games_played);
	Serial.print("High Score: ");
	Serial.println(user_settings.high_score);
	Serial.print("Boss kills: ");
	Serial.println(user_settings.boss_kills);
}

void reset_settings()
{
	user_settings.settings_version = SETTINGS_VERSION;

	user_settings.led_count = NUM_LEDS;
	user_settings.led_brightness = DEFAULT_BRIGHTNESS;

	user_settings.joystick_deadzone = DEFAULT_JOYSTICK_DEADZONE;
	user_settings.attack_threshold = DEFAULT_ATTACK_THRESHOLD;

	user_settings.audio_volume = DEFAULT_VOLUME;

	user_settings.lives_per_level = LIVES_PER_LEVEL;

	user_settings.games_played = 0;
	user_settings.total_points = 0;
	user_settings.high_score = 0;
	user_settings.boss_kills = 0;

	Serial.println("Settings reset...");

	settings_eeprom_write();
}

void settings_eeprom_read()
{
	EEPROM.begin(EEPROM_SIZE);

	uint8_t ver = EEPROM.read(0);
	uint8_t temp[sizeof(user_settings)];

	if (ver != SETTINGS_VERSION)
	{
		Serial.print("Error: EEPROM settings read failed:");
		Serial.println(ver);
		Serial.println("Loading defaults...");
		reset_settings();
		return;
	}
	else
	{
		Serial.print("Settings version: ");
		Serial.println(ver);
	}

	for (int i = 0; i < sizeof(user_settings); i++)
	{
		temp[i] = EEPROM.read(i);
	}

	memcpy((char *)&user_settings, temp, sizeof(user_settings));
}

void settings_eeprom_write()
{
	sound_pause(); // prevent interrupt from causing crash

	EEPROM.begin(EEPROM_SIZE);

	uint8_t temp[sizeof(user_settings)];
	memcpy(temp, (uint8_t *)&user_settings, sizeof(user_settings));

	for (int i = 0; i < sizeof(user_settings); i++)
	{
		EEPROM.write(i, temp[i]);
	}

	EEPROM.commit();

	sound_resume(); // restore sound interrupt
}

#endif