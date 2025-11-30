/*
  TWANG32 - An ESP32 port of TWANG with Resistor Ladder Button Integration
  (c) B. Dring 3/2018
  License: Creative Commons 4.0 Attribution - Share Alike

  TWANG was originally created by Critters
  https://github.com/Critters/TWANG

  Modified to use ONLY resistor ladder button system with improved calibration
  Added LED visual feedback for calibration and changed to UP/DOWN movement
*/

#define VERSION "2024-12-ButtonLadder-LED-Feedback"

#include <Arduino.h>
#include <ArduinoJson.h>
#include <FastLED.h>
#include <Wire.h>
#include <EEPROM.h>
#include "RunningMedian.h"

// twang files
#include "config.h"
#include "Enemy.h"
#include "Particle.h"
#include "Spawner.h"
#include "Lava.h"
#include "Boss.h"
#include "Conveyor.h"
#include "iSin.h"
#include "sound.h"
#include "settings.h"

// ========== BUTTON LADDER CONFIGURATION ==========
#define BUTTON_LADDER_PIN 34 // GPIO34 - ADC1_CH6
#define BUTTON_EEPROM_SIZE 512
#define BUTTON_CALIBRATION_MAGIC 0xABCD

// EEPROM addresses for button calibration
#define BUTTON_ADDR_MAGIC 400
#define BUTTON_ADDR_BUTTON1 402
#define BUTTON_ADDR_BUTTON2 404
#define BUTTON_ADDR_BUTTON3 406
#define BUTTON_ADDR_NO_BUTTON 408

// Button state variables
int currentButtonPressed = 0;
unsigned long lastButtonTime = 0;
const unsigned long buttonDebounceDelay = 50;
const int buttonTolerance = 100;

// Calibrated button values
int button1Value = 0; // Up movement
int button2Value = 0; // Down movement
int button3Value = 0; // Attack
int noButtonValue = 0;

// Button press/release states for smooth gameplay
bool button1CurrentlyPressed = false;
bool button2CurrentlyPressed = false;
bool button3CurrentlyPressed = false;

unsigned long lastDataSend = 0;
const unsigned long DATA_SEND_INTERVAL = 100; // Send data every 100ms
bool dataLoggingEnabled = false;

// Function declarations for button system
bool isButtonCalibrated();
void performButtonCalibration();
void saveButtonCalibration();
void loadButtonCalibration();
void printButtonCalibrationValues();
int getButtonPressed(int adcValue);
int getStableButtonReading();
void waitForEnterButton();
void handleButtonInput();

// LED Visual Feedback Functions
void showCalibrationStep(int step, CRGB color);
void flashLEDs(CRGB color, int flashes, int delayMs);
void showWaitingPattern();
void showSuccessPattern();

// Function declarations
void FastLEDshowESP32();
void FastLEDshowTask(void *pvParameters);
void loadLevel();
void spawnBoss();
void moveBoss();
void spawnEnemy(int pos, int dir, int speed, int wobble);
void spawnLava(int left, int right, int ontime, int offtime, int offset, int state, float grow, float flow);
void spawnConveyor(int startPoint, int endPoint, int dir);
void cleanupLevel();
void levelComplete();
void nextLevel();
void gameOver();
void die();
void tickStartup(long mm);
void tickEnemies();
void tickBoss();
void drawPlayer();
void drawExit();
void tickSpawners();
void tickLava();
bool tickParticles();
void tickConveyors();
void tickComplete(long mm);
void tickBossKilled(long mm);
void tickDie(long mm);
void tickGameover(long mm);
void tickWin(long mm);
void drawLives();
void drawAttack();
int getLED(int pos);
bool inLava(int pos);
void updateLives();
void screenSaverTick();
void getInput();
void SFXFreqSweepWarble(int duration, int elapsedTime, int freqStart, int freqEnd, int warble);
void SFXFreqSweepNoise(int duration, int elapsedTime, int freqStart, int freqEnd, uint8_t noiseFactor);
void SFXtilt(int amount);
void SFXattacking();
void SFXdead();
void SFXgameover();
void SFXkill();
void SFXwin();
void SFXbosskilled();
void SFXcomplete();
long map_constrain(long x, long in_min, long in_max, long out_min, long out_max);
void Fire2012();
void random_LED_flashes();
void sinelon();
void juggle();
void sendGameData();
void sendCalibrationStatus(const char *message, int step = -1);
void handleSerialCommands();

#define DIRECTION 1
#define USE_GRAVITY 0  // 0/1 use gravity (LED strip going up wall)
#define BEND_POINT 550 // 0/1000 point at which the LED strip goes up the wall

// GAME
long previousMillis = 0; // Time of the last redraw
int levelNumber = 0;

#define TIMEOUT 20000 // time until screen saver in milliseconds

int joystickTilt = 0;   // Stores the angle of the joystick
int joystickWobble = 0; // Stores the max amount of acceleration (wobble)

// WOBBLE ATTACK
#define DEFAULT_ATTACK_WIDTH 70 // Width of the wobble attack, world is 1000 wide
int attack_width = DEFAULT_ATTACK_WIDTH;
#define ATTACK_DURATION 500 // Duration of a wobble attack (ms)
long attackMillis = 0;      // Time the attack started
bool attacking = 0;         // Is the attack in progress?
#define BOSS_WIDTH 40

// TODO all animation durations should be defined rather than literals
#define STARTUP_WIPEUP_DUR 200
#define STARTUP_SPARKLE_DUR 1300
#define STARTUP_FADE_DUR 1500

#define GAMEOVER_SPREAD_DURATION 1000
#define GAMEOVER_FADE_DURATION 1500

#define WIN_FILL_DURATION 500
#define WIN_CLEAR_DURATION 1000
#define WIN_OFF_DURATION 1200

CRGB leds[VIRTUAL_LED_COUNT];
iSin isin = iSin();

// POOLS
#define LIFE_LEDS 3
int lifeLEDs[LIFE_LEDS] = {7, 6, 5};

#define ENEMY_COUNT 10
Enemy enemyPool[ENEMY_COUNT] = {
    Enemy(), Enemy(), Enemy(), Enemy(), Enemy(), Enemy(), Enemy(), Enemy(), Enemy(), Enemy()};

#define PARTICLE_COUNT 100
Particle particlePool[PARTICLE_COUNT];

#define SPAWN_COUNT 5
Spawner spawnPool[SPAWN_COUNT] = {
    Spawner(), Spawner()};

#define LAVA_COUNT 5
Lava lavaPool[LAVA_COUNT] = {
    Lava(), Lava(), Lava(), Lava()};

#define CONVEYOR_COUNT 2
Conveyor conveyorPool[CONVEYOR_COUNT] = {
    Conveyor(), Conveyor()};

Boss boss = Boss();

enum stages
{
  STARTUP,
  PLAY,
  WIN,
  DEAD,
  SCREENSAVER,
  BOSS_KILLED,
  GAMEOVER
} stage;

long stageStartTime;        // Stores the time the stage changed for stages that are time based
int playerPosition;         // Stores the player position
int playerPositionModifier; // +/- adjustment to player position
bool playerAlive;
long killTime;
int lives = LIVES_PER_LEVEL;
bool lastLevel = false;

int score = 0;

#define FASTLED_SHOW_CORE 0 // -- The core to run FastLED.show()

// -- Task handles for use in the notifications
static TaskHandle_t FastLEDshowTaskHandle = 0;
static TaskHandle_t userTaskHandle = 0;

// ========== LED VISUAL FEEDBACK FUNCTIONS ==========

void showCalibrationStep(int step, CRGB color)
{
  FastLED.clear();
  int ledsPerStep = user_settings.led_count / 5; // 5 steps total

  // Light up LEDs for current step
  for (int i = 0; i < (step + 1) * ledsPerStep && i < user_settings.led_count; i++)
  {
    leds[i] = color;
  }
  FastLEDshowESP32();
}

void flashLEDs(CRGB color, int flashes, int delayMs)
{
  for (int i = 0; i < flashes; i++)
  {
    FastLED.clear();
    FastLEDshowESP32();
    delay(delayMs);

    fill_solid(leds, user_settings.led_count, color);
    FastLEDshowESP32();
    delay(delayMs);
  }
  FastLED.clear();
  FastLEDshowESP32();
}

void showWaitingPattern()
{
  static uint8_t hue = 0;
  FastLED.clear();

  // Breathing pattern
  int brightness = (sin(millis() / 500.0) + 1) * 127;
  fill_solid(leds, user_settings.led_count, CHSV(hue, 255, brightness));
  hue += 2;

  FastLEDshowESP32();
}

void showSuccessPattern()
{
  // Green wave from center outward
  FastLED.clear();
  int center = user_settings.led_count / 2;
  int wave = (millis() / 100) % (user_settings.led_count / 2);

  for (int i = 0; i < wave && i < user_settings.led_count / 2; i++)
  {
    if (center - i >= 0)
      leds[center - i] = CRGB::Green;
    if (center + i < user_settings.led_count)
      leds[center + i] = CRGB::Green;
  }
  FastLEDshowESP32();
}

// ========== BUTTON LADDER FUNCTIONS ==========

bool isButtonCalibrated()
{
  EEPROM.begin(BUTTON_EEPROM_SIZE);
  uint16_t magic;
  EEPROM.get(BUTTON_ADDR_MAGIC, magic);
  return (magic == BUTTON_CALIBRATION_MAGIC);
}

void performButtonCalibration()
{
  Serial.println("\nBUTTON LADDER CALIBRATION FOR TWANG");
  sendCalibrationStatus("Starting calibration", 0);

  // Wait for first input to start calibration
  Serial.println("Press any button or send any character to start calibration...");
  sendCalibrationStatus("Waiting for input to start", 0);

  // Show waiting pattern
  bool inputReceived = false;
  while (!inputReceived)
  {
    showWaitingPattern();

    // Check for serial input
    if (Serial.available())
    {
      Serial.readStringUntil('\n'); // Clear input
      inputReceived = true;
    }

    // Check for button press
    int buttonValue = analogRead(BUTTON_LADDER_PIN);
    if (abs(buttonValue - 4095) > 500)
    { // Any significant change from max value
      inputReceived = true;
    }

    delay(50);
  }

  // Step 0: Show calibration start
  flashLEDs(CRGB::Blue, 3, 300);
  showCalibrationStep(0, CRGB::Blue);
  delay(2000);

  Serial.println("Follow the instructions carefully:");
  Serial.println("Button 1 = UP, Button 2 = DOWN, Button 3 = ATTACK");
  Serial.println();

  // Calibrate no button pressed
  Serial.println("1. RELEASE ALL BUTTONS and press Enter (or wait 5 seconds)...");
  sendCalibrationStatus("Waiting for no button calibration", 1);

  showCalibrationStep(1, CRGB::Red);
  unsigned long startTime = millis();
  bool enterPressed = false;

  // Wait for Enter or timeout
  while (millis() - startTime < 5000 && !enterPressed)
  {
    showWaitingPattern();
    if (Serial.available())
    {
      String input = Serial.readStringUntil('\n');
      enterPressed = true;
    }
    delay(50);
  }

  noButtonValue = getStableButtonReading();
  Serial.println("   No button: " + String(noButtonValue));
  sendCalibrationStatus(("No button calibrated: " + String(noButtonValue)).c_str(), 1);
  showSuccessPattern();
  delay(1000);

  // Calibrate button 1 (UP)
  Serial.println("2. Press and HOLD BUTTON 1 (UP), then press Enter (or wait 5 seconds)...");
  sendCalibrationStatus("Waiting for button 1 (UP) calibration", 2);

  showCalibrationStep(2, CRGB::Yellow);
  startTime = millis();
  enterPressed = false;

  while (millis() - startTime < 5000 && !enterPressed)
  {
    showWaitingPattern();
    if (Serial.available())
    {
      String input = Serial.readStringUntil('\n');
      enterPressed = true;
    }
    delay(50);
  }

  button1Value = getStableButtonReading();
  Serial.println("   Button 1 (UP): " + String(button1Value));
  sendCalibrationStatus(("Button 1 calibrated: " + String(button1Value)).c_str(), 2);
  showSuccessPattern();
  delay(1000);

  // Calibrate button 2 (DOWN)
  Serial.println("3. Press and HOLD BUTTON 2 (DOWN), then press Enter (or wait 5 seconds)...");
  sendCalibrationStatus("Waiting for button 2 (DOWN) calibration", 3);

  showCalibrationStep(3, CRGB::Orange);
  startTime = millis();
  enterPressed = false;

  while (millis() - startTime < 5000 && !enterPressed)
  {
    showWaitingPattern();
    if (Serial.available())
    {
      String input = Serial.readStringUntil('\n');
      enterPressed = true;
    }
    delay(50);
  }

  button2Value = getStableButtonReading();
  Serial.println("   Button 2 (DOWN): " + String(button2Value));
  sendCalibrationStatus(("Button 2 calibrated: " + String(button2Value)).c_str(), 3);
  showSuccessPattern();
  delay(1000);

  // Calibrate button 3 (ATTACK)
  Serial.println("4. Press and HOLD BUTTON 3 (ATTACK), then press Enter (or wait 5 seconds)...");
  sendCalibrationStatus("Waiting for button 3 (ATTACK) calibration", 4);

  showCalibrationStep(4, CRGB::Purple);
  startTime = millis();
  enterPressed = false;

  while (millis() - startTime < 5000 && !enterPressed)
  {
    showWaitingPattern();
    if (Serial.available())
    {
      String input = Serial.readStringUntil('\n');
      enterPressed = true;
    }
    delay(50);
  }

  button3Value = getStableButtonReading();
  Serial.println("   Button 3 (ATTACK): " + String(button3Value));
  sendCalibrationStatus(("Button 3 calibrated: " + String(button3Value)).c_str(), 4);
  showSuccessPattern();
  delay(1000);

  // Save to EEPROM
  saveButtonCalibration();

  Serial.println("\nButton calibration complete and saved!");
  sendCalibrationStatus("Calibration complete", 5);
  printButtonCalibrationValues();

  // Final success indication
  flashLEDs(CRGB::Green, 5, 200);
  FastLED.clear();
  FastLEDshowESP32();
}

void handleSerialCommands()
{
  if (Serial.available())
  {
    String command = Serial.readStringUntil('\n');
    command.trim();

    if (command == "c" || command == "C")
    {
      Serial.println("\nStarting button recalibration...");
      sendCalibrationStatus("Manual recalibration started", 0);
      performButtonCalibration();
    }
    else if (command == "START_LOGGING")
    {
      dataLoggingEnabled = true;
      Serial.println("Data logging enabled");
    }
    else if (command == "STOP_LOGGING")
    {
      dataLoggingEnabled = false;
      Serial.println("Data logging disabled");
    }
    else if (command == "STATUS")
    {
      sendGameData();
    }
  }
}

void waitForEnterButton()
{
  Serial.flush();
  while (true)
  {
    if (Serial.available())
    {
      String input = Serial.readStringUntil('\n');
      break;
    }
    delay(10);
  }
}

int getStableButtonReading()
{
  const int numSamples = 50;
  long total = 0;

  for (int i = 0; i < numSamples; i++)
  {
    total += analogRead(BUTTON_LADDER_PIN);
    delay(10);
  }

  return total / numSamples;
}

void saveButtonCalibration()
{
  sound_pause(); // prevent interrupt from causing crash

  EEPROM.begin(BUTTON_EEPROM_SIZE);
  EEPROM.put(BUTTON_ADDR_MAGIC, BUTTON_CALIBRATION_MAGIC);
  EEPROM.put(BUTTON_ADDR_BUTTON1, button1Value);
  EEPROM.put(BUTTON_ADDR_BUTTON2, button2Value);
  EEPROM.put(BUTTON_ADDR_BUTTON3, button3Value);
  EEPROM.put(BUTTON_ADDR_NO_BUTTON, noButtonValue);
  EEPROM.commit();

  sound_resume(); // restore sound interrupt
}

void loadButtonCalibration()
{
  EEPROM.begin(BUTTON_EEPROM_SIZE);
  EEPROM.get(BUTTON_ADDR_BUTTON1, button1Value);
  EEPROM.get(BUTTON_ADDR_BUTTON2, button2Value);
  EEPROM.get(BUTTON_ADDR_BUTTON3, button3Value);
  EEPROM.get(BUTTON_ADDR_NO_BUTTON, noButtonValue);
}

void printButtonCalibrationValues()
{
  Serial.println("Button Calibration Values:");
  Serial.println("   No Button: " + String(noButtonValue));
  Serial.println("   Button 1 (UP):     " + String(button1Value));
  Serial.println("   Button 2 (DOWN):   " + String(button2Value));
  Serial.println("   Button 3 (ATTACK): " + String(button3Value));
  Serial.println("   Tolerance: ±" + String(buttonTolerance));
}

int getButtonPressed(int adcValue)
{
  // Check each button with tolerance
  if (abs(adcValue - button1Value) <= buttonTolerance)
  {
    return 1; // UP
  }
  else if (abs(adcValue - button2Value) <= buttonTolerance)
  {
    return 2; // DOWN
  }
  else if (abs(adcValue - button3Value) <= buttonTolerance)
  {
    return 3; // ATTACK
  }
  else if (abs(adcValue - noButtonValue) <= buttonTolerance)
  {
    return 0; // No button pressed
  }
  else
  {
    return 0; // Value doesn't match any calibrated button
  }
}

void handleButtonInput()
{
  static int lastButtonReading = 0;
  int buttonValue = analogRead(BUTTON_LADDER_PIN);
  currentButtonPressed = getButtonPressed(buttonValue);

  // Simple debouncing
  if (currentButtonPressed != lastButtonReading)
  {
    lastButtonTime = millis();
  }

  if ((millis() - lastButtonTime) > buttonDebounceDelay)
  {
    // Update button states
    button1CurrentlyPressed = (currentButtonPressed == 1);
    button2CurrentlyPressed = (currentButtonPressed == 2);
    button3CurrentlyPressed = (currentButtonPressed == 3);
  }

  lastButtonReading = currentButtonPressed;
}

/** show() for ESP32
 *  Call this function instead of FastLED.show(). It signals core 0 to issue a show,
 *  then waits for a notification that it is done.
 */
void FastLEDshowESP32()
{
  if (userTaskHandle == 0)
  {
    // -- Store the handle of the current task, so that the show task can
    //    notify it when it's done
    userTaskHandle = xTaskGetCurrentTaskHandle();

    // -- Trigger the show task
    xTaskNotifyGive(FastLEDshowTaskHandle);

    // -- Wait to be notified that it's done
    const TickType_t xMaxBlockTime = pdMS_TO_TICKS(200);
    ulTaskNotifyTake(pdTRUE, xMaxBlockTime);
    userTaskHandle = 0;
  }
}

/** show Task
 *  This function runs on core 0 and just waits for requests to call FastLED.show()
 */
void FastLEDshowTask(void *pvParameters)
{
  // -- Run forever...
  for (;;)
  {
    // -- Wait for the trigger
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    // -- Do the show (synchronously)
    FastLED.show();

    // -- Notify the calling task
    xTaskNotifyGive(userTaskHandle);
  }
}

void sendGameData()
{
  if (!dataLoggingEnabled)
    return;

  StaticJsonDocument<512> doc;

  // Game state
  doc["timestamp"] = millis();
  doc["stage"] = stage;
  doc["stageStartTime"] = stageStartTime;
  doc["levelNumber"] = levelNumber;
  doc["lives"] = lives;
  doc["score"] = score;
  doc["playerPosition"] = playerPosition;
  doc["playerAlive"] = playerAlive;
  doc["attacking"] = attacking;
  doc["attackMillis"] = attackMillis;

  // Button states
  JsonObject buttons = doc.createNestedObject("buttons");
  buttons["current"] = currentButtonPressed;
  buttons["button1"] = button1CurrentlyPressed;
  buttons["button2"] = button2CurrentlyPressed;
  buttons["button3"] = button3CurrentlyPressed;
  buttons["calibrated"] = isButtonCalibrated();

  // Joystick data (for compatibility)
  JsonObject joystick = doc.createNestedObject("joystick");
  joystick["tilt"] = joystickTilt;
  joystick["wobble"] = joystickWobble;

  // Boss data
  JsonObject bossData = doc.createNestedObject("boss");
  bossData["alive"] = boss.Alive();
  if (boss.Alive())
  {
    bossData["position"] = boss._pos;
    bossData["lives"] = boss._lives;
    bossData["ticks"] = boss._ticks;
  }

  // Enemies
  JsonArray enemies = doc.createNestedArray("enemies");
  for (int i = 0; i < ENEMY_COUNT; i++)
  {
    if (enemyPool[i].Alive())
    {
      JsonObject enemy = enemies.createNestedObject();
      enemy["id"] = i;
      enemy["position"] = enemyPool[i]._pos;
      enemy["wobble"] = enemyPool[i]._wobble;
      enemy["playerSide"] = enemyPool[i].playerSide;
    }
  }

  // Lava
  JsonArray lavaArray = doc.createNestedArray("lava");
  for (int i = 0; i < LAVA_COUNT; i++)
  {
    if (lavaPool[i].Alive())
    {
      JsonObject lava = lavaArray.createNestedObject();
      lava["id"] = i;
      lava["left"] = lavaPool[i]._left;
      lava["right"] = lavaPool[i]._right;
      lava["state"] = lavaPool[i]._state;
      lava["ontime"] = lavaPool[i]._ontime;
      lava["offtime"] = lavaPool[i]._offtime;
    }
  }

  // Settings
  JsonObject settings = doc.createNestedObject("settings");
  settings["ledCount"] = user_settings.led_count;
  settings["brightness"] = user_settings.led_brightness;
  settings["volume"] = user_settings.audio_volume;
  settings["gamesPlayed"] = user_settings.games_played;
  settings["highScore"] = user_settings.high_score;
  settings["bossKills"] = user_settings.boss_kills;

  // Send as JSON
  Serial.print("DATA:");
  serializeJson(doc, Serial);
  Serial.println();
}

void sendCalibrationStatus(const char *message, int step)
{
  StaticJsonDocument<256> doc;
  doc["timestamp"] = millis();
  doc["type"] = "calibration";
  doc["message"] = message;
  if (step >= 0)
    doc["step"] = step;
  doc["calibrated"] = isButtonCalibrated();

  if (isButtonCalibrated())
  {
    JsonObject values = doc.createNestedObject("values");
    values["noButton"] = noButtonValue;
    values["button1"] = button1Value;
    values["button2"] = button2Value;
    values["button3"] = button3Value;
  }

  Serial.print("CALIB:");
  serializeJson(doc, Serial);
  Serial.println();
}

void setup()
{
  Serial.begin(115200);
  Serial.print("\r\nTWANG32 VERSION: ");
  Serial.println(VERSION);
  Serial.println("Button Ladder Mode");

  // Initialize button ladder pin
  pinMode(BUTTON_LADDER_PIN, INPUT);

  // Initialize settings first to get LED count
  settings_init();

  // Initialize FastLED before calibration
#ifdef USE_NEOPIXEL
  Serial.print("\r\nCompiled for WS2812B (Neopixel) LEDs");
  FastLED.addLeds<LED_TYPE, DATA_PIN>(leds, MAX_LEDS);
#endif

#ifdef USE_APA102
  Serial.print("\r\nCompiled for APA102 (Dotstar) LEDs");
  FastLED.addLeds<LED_TYPE, DATA_PIN, CLOCK_PIN, LED_COLOR_ORDER>(leds, MAX_LEDS);
#endif

  FastLED.setBrightness(user_settings.led_brightness);
  FastLED.setDither(1);

  // Create the ESP32 FastLED show task BEFORE calibration
  xTaskCreatePinnedToCore(FastLEDshowTask, "FastLEDshowTask", 2048, NULL, 2, &FastLEDshowTaskHandle, FASTLED_SHOW_CORE);

  // Check button calibration (now FastLED is ready)
  if (!isButtonCalibrated())
  {
    Serial.println("No button calibration found.");
    Serial.println("Calibration will start when you provide input...");
    performButtonCalibration();
  }
  else
  {
    loadButtonCalibration();
    Serial.println("Button calibration loaded from EEPROM");
    printButtonCalibrationValues();
  }

  sound_init(DAC_AUDIO_PIN);

  stage = STARTUP;
  stageStartTime = millis();
  lives = user_settings.lives_per_level;

  Serial.println("\nTWANG Controls:");
  Serial.println("   Button 1 = UP movement");
  Serial.println("   Button 2 = DOWN movement");
  Serial.println("   Button 3 = ATTACK/Wobble");
  Serial.println("   Send 'c' to recalibrate buttons");
  Serial.println();
}

void loop()
{
  long mm = millis();
  int brightness = 0;

  // Check for button calibration command
  handleSerialCommands();

  if (stage == PLAY)
  {
    if (attacking)
    {
      SFXattacking();
    }
    else
    {
      SFXtilt(joystickTilt);
    }
  }
  else if (stage == DEAD)
  {
    SFXdead();
  }

  if (mm - previousMillis >= MIN_REDRAW_INTERVAL)
  {
    getInput();

    long frameTimer = mm;
    previousMillis = mm;

    if (abs(joystickTilt) > user_settings.joystick_deadzone)
    {
      lastInputTime = mm;
      if (stage == SCREENSAVER)
      {
        levelNumber = -1;
        stageStartTime = mm;
        stage = WIN;
      }
    }
    else
    {
      if (lastInputTime + TIMEOUT < mm)
      {
        stage = SCREENSAVER;
      }
    }

    if (stage == SCREENSAVER)
    {
      screenSaverTick();
    }
    else if (stage == STARTUP)
    {
      if (stageStartTime + STARTUP_FADE_DUR > mm)
      {
        tickStartup(mm);
      }
      else
      {
        SFXcomplete();
        levelNumber = 0;
        loadLevel();
      }
    }
    else if (stage == PLAY)
    {
      // PLAYING
      if (attacking && attackMillis + ATTACK_DURATION < mm)
        attacking = 0;

      // If not attacking, check if they should be
      if (!attacking && joystickWobble >= user_settings.attack_threshold)
      {
        attackMillis = mm;
        attacking = 1;
      }

      // If still not attacking, move!
      playerPosition += playerPositionModifier;
      if (!attacking)
      {
        int moveAmount = (joystickTilt / (6.0));
        if (DIRECTION)
          moveAmount = -moveAmount;
        moveAmount = constrain(moveAmount, -MAX_PLAYER_SPEED, MAX_PLAYER_SPEED);

        playerPosition -= moveAmount;
        if (playerPosition < 0)
          playerPosition = 0;

        // stop player from leaving if boss is alive
        if (boss.Alive() && playerPosition >= VIRTUAL_LED_COUNT)
          playerPosition = 999;

        if (playerPosition >= VIRTUAL_LED_COUNT && !boss.Alive())
        {
          // Reached exit!
          levelComplete();
          return;
        }
      }

      if (inLava(playerPosition))
      {
        die();
      }

      // Ticks and draw calls
      FastLED.clear();
      tickConveyors();
      tickSpawners();
      tickBoss();
      tickLava();
      tickEnemies();
      drawPlayer();
      drawAttack();
      drawExit();
    }
    else if (stage == DEAD)
    {
      // DEAD
      FastLED.clear();
      tickDie(mm);
      if (!tickParticles())
      {
        loadLevel();
      }
    }
    else if (stage == WIN)
    {
      // LEVEL COMPLETE
      tickWin(mm);
    }
    else if (stage == BOSS_KILLED)
    {
      tickBossKilled(mm);
    }
    else if (stage == GAMEOVER)
    {
      if (stageStartTime + GAMEOVER_FADE_DURATION > mm)
      {
        tickGameover(mm);
      }
      else
      {
        FastLED.clear();

        // restart from the beginning
        stage = STARTUP;
        stageStartTime = millis();
        lives = user_settings.lives_per_level;
      }
    }

    if (millis() - lastDataSend >= DATA_SEND_INTERVAL)
    {
      sendGameData();
      lastDataSend = millis();
    }

    FastLEDshowESP32();
  }
}

// ---------------------------------
// ----------- JOYSTICK ------------
// ---------------------------------
void getInput()
{
  // Handle button ladder input
  handleButtonInput();

  // Handle movement - UP/DOWN (fixed inversion)
  if (button1CurrentlyPressed && !button2CurrentlyPressed)
  {
    joystickTilt = 60; // UP movement (positive tilt moves up)
  }
  else if (button2CurrentlyPressed && !button1CurrentlyPressed)
  {
    joystickTilt = -60; // DOWN movement (negative tilt moves down)
  }
  else
  {
    joystickTilt = 0; // No movement if both or neither pressed
  }

  // Handle attack
  if (button3CurrentlyPressed)
  {
    joystickWobble = user_settings.attack_threshold + 1000; // Ensure it triggers
  }
  else
  {
    joystickWobble = 0;
  }
}

// ---------------------------------
// ------------ LEVELS -------------
// ---------------------------------
void loadLevel()
{
  // leave these alone
  FastLED.setBrightness(user_settings.led_brightness);
  updateLives();
  cleanupLevel();
  playerAlive = 1;
  lastLevel = false; // this gets changed on the boss level

  /// Defaults...OK to change the following items in the levels below
  attack_width = DEFAULT_ATTACK_WIDTH;
  playerPosition = 0;

  switch (levelNumber)
  {
  case 0: // basic introduction
    playerPosition = 200;
    spawnEnemy(1, 0, 0, 0);
    break;
  case 1:
    // Slow moving enemy
    spawnEnemy(900, 0, 1, 0);
    break;
  case 2:
    // Spawning enemies at exit every 2 seconds
    spawnPool[0].Spawn(1000, 3000, 2, 0, 0);
    break;
  case 3:
    // Lava intro
    spawnLava(400, 490, 2000, 2000, 0, Lava::OFF, 0, 0);
    spawnEnemy(350, 0, 1, 0);
    spawnPool[0].Spawn(1000, 5500, 3, 0, 0);
    break;
  case 4:
    // intro to moving lava (down)
    spawnLava(400, 490, 2000, 2000, 0, Lava::OFF, 0, -0.5);
    spawnEnemy(350, 0, 1, 0);
    spawnPool[0].Spawn(1000, 5500, 3, 0, 0);
    break;
  case 5:
    // lava spreading
    spawnLava(400, 450, 2000, 2000, 0, Lava::OFF, 0.25, 0);
    spawnEnemy(350, 0, 1, 0);
    spawnPool[0].Spawn(1000, 5500, 3, 0, 0);
    break;
  case 6:
    // Sin wave enemy
    spawnEnemy(700, 1, 7, 275);
    spawnEnemy(500, 1, 5, 250);
    break;
  case 7:
    // Sin enemy swarm
    spawnEnemy(700, 1, 7, 275);
    spawnEnemy(500, 1, 5, 250);
    spawnEnemy(600, 1, 7, 200);
    spawnEnemy(800, 1, 5, 350);
    spawnEnemy(400, 1, 7, 150);
    spawnEnemy(450, 1, 5, 400);
    break;
  case 8:
    // lava moving up
    playerPosition = 200;
    spawnLava(10, 180, 2000, 2000, 0, Lava::OFF, 0, 0.5);
    spawnEnemy(350, 0, 1, 0);
    spawnPool[0].Spawn(1000, 5500, 3, 0, 0);
    break;
  case 9:
    // Conveyor
    spawnConveyor(100, 600, -6);
    spawnEnemy(800, 0, 0, 0);
    break;
  case 10:
    // Conveyor of enemies
    spawnConveyor(50, 1000, 6);
    spawnEnemy(300, 0, 0, 0);
    spawnEnemy(400, 0, 0, 0);
    spawnEnemy(500, 0, 0, 0);
    spawnEnemy(600, 0, 0, 0);
    spawnEnemy(700, 0, 0, 0);
    spawnEnemy(800, 0, 0, 0);
    spawnEnemy(900, 0, 0, 0);
    break;
  case 11:
    // lava spread and fall
    spawnLava(400, 450, 2000, 2000, 0, Lava::OFF, 0.2, -0.5);
    spawnEnemy(350, 0, 1, 0);
    spawnPool[0].Spawn(1000, 5500, 3, 0, 0);
    break;
  case 12: // spawn train;
    spawnPool[0].Spawn(900, 1300, 2, 0, 0);
    break;
  case 13: // spawn train skinny attack width;
    attack_width = 32;
    spawnPool[0].Spawn(900, 1800, 2, 0, 0);
    break;
  case 14: // evil fast split spawner
    spawnPool[0].Spawn(550, 1500, 2, 0, 0);
    spawnPool[1].Spawn(550, 1500, 2, 1, 0);
    break;
  case 15: // split spawner with exit blocking lava
    spawnPool[0].Spawn(500, 1200, 2, 0, 0);
    spawnPool[1].Spawn(500, 1200, 2, 1, 0);
    spawnLava(900, 950, 2200, 800, 2000, Lava::OFF, 0, 0);
    break;
  case 16:
    // Lava run
    spawnLava(195, 300, 2000, 2000, 0, Lava::OFF, 0, 0);
    spawnLava(400, 500, 2000, 2000, 0, Lava::OFF, 0, 0);
    spawnLava(600, 700, 2000, 2000, 0, Lava::OFF, 0, 0);
    spawnPool[0].Spawn(1000, 3800, 4, 0, 0);
    break;
  case 17:
    // Sin enemy #2 practice (slow conveyor)
    spawnEnemy(700, 1, 7, 275);
    spawnEnemy(500, 1, 5, 250);
    spawnPool[0].Spawn(1000, 5500, 4, 0, 3000);
    spawnPool[1].Spawn(0, 5500, 5, 1, 10000);
    spawnConveyor(100, 900, -4);
    break;
  case 18:
    // Sin enemy #2 (fast conveyor)
    spawnEnemy(800, 1, 7, 275);
    spawnEnemy(700, 1, 7, 275);
    spawnEnemy(500, 1, 5, 250);
    spawnPool[0].Spawn(1000, 3000, 4, 0, 3000);
    spawnPool[1].Spawn(0, 5500, 5, 1, 10000);
    spawnConveyor(100, 900, -6);
    break;
  case 19: // (don't edit last level)
    // Boss this should always be the last level
    spawnBoss();
    break;
  }
  stageStartTime = millis();
  stage = PLAY;
}

void spawnBoss()
{
  lastLevel = true;
  boss.Spawn();
  moveBoss();
}

void moveBoss()
{
  int spawnSpeed = 1800;
  if (boss._lives == 2)
    spawnSpeed = 1600;
  if (boss._lives == 1)
    spawnSpeed = 1000;
  spawnPool[0].Spawn(boss._pos, spawnSpeed, 3, 0, 0);
  spawnPool[1].Spawn(boss._pos, spawnSpeed, 3, 1, 0);
}

void spawnEnemy(int pos, int dir, int speed, int wobble)
{
  for (int e = 0; e < ENEMY_COUNT; e++)
  {
    if (!enemyPool[e].Alive())
    {
      enemyPool[e].Spawn(pos, dir, speed, wobble);
      enemyPool[e].playerSide = pos > playerPosition ? 1 : -1;
      return;
    }
  }
}

void spawnLava(int left, int right, int ontime, int offtime, int offset, int state, float grow, float flow)
{
  for (int i = 0; i < LAVA_COUNT; i++)
  {
    if (!lavaPool[i].Alive())
    {
      lavaPool[i].Spawn(left, right, ontime, offtime, offset, state, grow, flow);
      return;
    }
  }
}

void spawnConveyor(int startPoint, int endPoint, int dir)
{
  for (int i = 0; i < CONVEYOR_COUNT; i++)
  {
    if (!conveyorPool[i]._alive)
    {
      conveyorPool[i].Spawn(startPoint, endPoint, dir);
      return;
    }
  }
}

void cleanupLevel()
{
  for (int i = 0; i < ENEMY_COUNT; i++)
  {
    enemyPool[i].Kill();
  }
  for (int i = 0; i < PARTICLE_COUNT; i++)
  {
    particlePool[i].Kill();
  }
  for (int i = 0; i < SPAWN_COUNT; i++)
  {
    spawnPool[i].Kill();
  }
  for (int i = 0; i < LAVA_COUNT; i++)
  {
    lavaPool[i].Kill();
  }
  for (int i = 0; i < CONVEYOR_COUNT; i++)
  {
    conveyorPool[i].Kill();
  }
  boss.Kill();
}

void levelComplete()
{
  stageStartTime = millis();
  stage = WIN;

  if (lastLevel)
  {
    stage = BOSS_KILLED;
  }
  if (levelNumber != 0) // no points for the first level
  {
    score = score + (lives * 10);
  }
}

void nextLevel()
{
  levelNumber++;

  if (lastLevel)
  {
    stage = STARTUP;
    stageStartTime = millis();
    lives = user_settings.lives_per_level;
  }
  else
  {
    lives = user_settings.lives_per_level;
    loadLevel();
  }
}

void gameOver()
{
  levelNumber = 0;
  loadLevel();
}

void die()
{
  playerAlive = 0;
  if (levelNumber > 0)
    lives--;

  if (lives == 0)
  {
    stage = GAMEOVER;
    stageStartTime = millis();
  }
  else
  {
    for (int p = 0; p < PARTICLE_COUNT; p++)
    {
      particlePool[p].Spawn(playerPosition);
    }
    stageStartTime = millis();
    stage = DEAD;
  }
  killTime = millis();
}

// ----------------------------------
// -------- TICKS & RENDERS ---------
// ----------------------------------
void tickStartup(long mm)
{
  FastLED.clear();
  if (stageStartTime + STARTUP_WIPEUP_DUR > mm)
  {
    int n = _min(map(((mm - stageStartTime)), 0, STARTUP_WIPEUP_DUR, 0, user_settings.led_count), user_settings.led_count);
    for (int i = 0; i <= n; i++)
    {
      leds[i] = CRGB(0, 255, 0);
    }
  }
  else if (stageStartTime + STARTUP_SPARKLE_DUR > mm)
  {
    for (int i = 0; i < user_settings.led_count; i++)
    {
      if (random8(30) < 28)
        leds[i] = CRGB(0, 255, 0);
      else
      {
        int flicker = random8(250);
        leds[i] = CRGB(flicker, 150, flicker);
      }
    }
  }
  else if (stageStartTime + STARTUP_FADE_DUR > mm)
  {
    int n = _max(map(((mm - stageStartTime)), STARTUP_SPARKLE_DUR, STARTUP_FADE_DUR, 0, user_settings.led_count), 0);
    int brightness = _max(map(((mm - stageStartTime)), STARTUP_SPARKLE_DUR, STARTUP_FADE_DUR, 255, 0), 0);
    for (int i = n; i < user_settings.led_count; i++)
    {
      leds[i] = CRGB(0, brightness, 0);
    }
  }
  SFXFreqSweepWarble(STARTUP_FADE_DUR, millis() - stageStartTime, 40, 400, 20);
}

void tickEnemies()
{
  for (int i = 0; i < ENEMY_COUNT; i++)
  {
    if (enemyPool[i].Alive())
    {
      enemyPool[i].Tick();
      // Hit attack?
      if (attacking)
      {
        if (enemyPool[i]._pos > playerPosition - (attack_width / 2) && enemyPool[i]._pos < playerPosition + (attack_width / 2))
        {
          enemyPool[i].Kill();
          SFXkill();
        }
      }
      if (inLava(enemyPool[i]._pos))
      {
        enemyPool[i].Kill();
        SFXkill();
      }
      // Draw (if still alive)
      if (enemyPool[i].Alive())
      {
        leds[getLED(enemyPool[i]._pos)] = CRGB(255, 0, 0);
      }
      // Hit player?
      if (
          (enemyPool[i].playerSide == 1 && enemyPool[i]._pos <= playerPosition) ||
          (enemyPool[i].playerSide == -1 && enemyPool[i]._pos >= playerPosition))
      {
        die();
        return;
      }
    }
  }
}

void tickBoss()
{
  // DRAW
  if (boss.Alive())
  {
    boss._ticks++;
    for (int i = getLED(boss._pos - BOSS_WIDTH / 2); i <= getLED(boss._pos + BOSS_WIDTH / 2); i++)
    {
      leds[i] = CRGB::DarkRed;
      leds[i] %= 100;
    }
    // CHECK COLLISION
    if (getLED(playerPosition) > getLED(boss._pos - BOSS_WIDTH / 2) && getLED(playerPosition) < getLED(boss._pos + BOSS_WIDTH))
    {
      die();
      return;
    }
    // CHECK FOR ATTACK
    if (attacking)
    {
      if (
          (getLED(playerPosition + (attack_width / 2)) >= getLED(boss._pos - BOSS_WIDTH / 2) && getLED(playerPosition + (attack_width / 2)) <= getLED(boss._pos + BOSS_WIDTH / 2)) ||
          (getLED(playerPosition - (attack_width / 2)) <= getLED(boss._pos + BOSS_WIDTH / 2) && getLED(playerPosition - (attack_width / 2)) >= getLED(boss._pos - BOSS_WIDTH / 2)))
      {
        boss.Hit();
        if (boss.Alive())
        {
          moveBoss();
        }
        else
        {
          spawnPool[0].Kill();
          spawnPool[1].Kill();
        }
      }
    }
  }
}

void drawPlayer()
{
  leds[getLED(playerPosition)] = CRGB(0, 255, 0);
}

void drawExit()
{
  if (!boss.Alive())
  {
    leds[user_settings.led_count - 1] = CRGB(0, 0, 255);
  }
}

void tickSpawners()
{
  long mm = millis();
  for (int s = 0; s < SPAWN_COUNT; s++)
  {
    if (spawnPool[s].Alive() && spawnPool[s]._activate < mm)
    {
      if (spawnPool[s]._lastSpawned + spawnPool[s]._rate < mm || spawnPool[s]._lastSpawned == 0)
      {
        spawnEnemy(spawnPool[s]._pos, spawnPool[s]._dir, spawnPool[s]._sp, 0);
        spawnPool[s]._lastSpawned = mm;
      }
    }
  }
}

void tickLava()
{
  int A, B, p, i, brightness, flicker;
  long mm = millis();

  Lava LP;
  for (i = 0; i < LAVA_COUNT; i++)
  {
    LP = lavaPool[i];
    if (LP.Alive())
    {
      LP.Update();
      A = getLED(LP._left);
      B = getLED(LP._right);
      if (LP._state == Lava::OFF)
      {
        if (LP._lastOn + LP._offtime < mm)
        {
          LP._state = Lava::ON;
          LP._lastOn = mm;
        }
        for (p = A; p <= B; p++)
        {
          flicker = random8(LAVA_OFF_BRIGHTNESS);
          leds[p] = CRGB(LAVA_OFF_BRIGHTNESS + flicker, (LAVA_OFF_BRIGHTNESS + flicker) / 1.5, 0);
        }
      }
      else if (LP._state == Lava::ON)
      {
        if (LP._lastOn + LP._ontime < mm)
        {
          LP._state = Lava::OFF;
          LP._lastOn = mm;
        }
        for (p = A; p <= B; p++)
        {
          if (random8(30) < 29)
            leds[p] = CRGB(150, 0, 0);
          else
            leds[p] = CRGB(180, 100, 0);
        }
      }
    }
    lavaPool[i] = LP;
  }
}

bool tickParticles()
{
  bool stillActive = false;
  uint8_t brightness;
  for (int p = 0; p < PARTICLE_COUNT; p++)
  {
    if (particlePool[p].Alive())
    {
      particlePool[p].Tick(USE_GRAVITY);

      if (particlePool[p]._power < 5)
      {
        brightness = (5 - particlePool[p]._power) * 10;
        leds[getLED(particlePool[p]._pos)] += CRGB(brightness, brightness / 2, brightness / 2);
      }
      else
        leds[getLED(particlePool[p]._pos)] += CRGB(particlePool[p]._power, 0, 0);

      stillActive = true;
    }
  }
  return stillActive;
}

void tickConveyors()
{
  int b, speed, n, i, ss, ee, led;
  long m = 10000 + millis();
  playerPositionModifier = 0;

  int levels = 5; // brightness levels in conveyor

  for (i = 0; i < CONVEYOR_COUNT; i++)
  {
    if (conveyorPool[i]._alive)
    {
      speed = constrain(conveyorPool[i]._speed, -MAX_PLAYER_SPEED + 1, MAX_PLAYER_SPEED - 1);
      ss = getLED(conveyorPool[i]._startPoint);
      ee = getLED(conveyorPool[i]._endPoint);
      for (led = ss; led < ee; led++)
      {
        n = (-led + (m / 100)) % levels;
        if (speed < 0)
          n = (led + (m / 100)) % levels;

        b = map(n, 5, 0, 0, CONVEYOR_BRIGHTNESS);
        if (b > 0)
          leds[led] = CRGB(0, 0, b);
      }

      if (playerPosition > conveyorPool[i]._startPoint && playerPosition < conveyorPool[i]._endPoint)
      {
        playerPositionModifier = speed;
      }
    }
  }
}

void tickComplete(long mm)
{
  int brightness = 0;
  FastLED.clear();
  SFXcomplete();
  if (stageStartTime + 500 > mm)
  {
    int n = _max(map(((mm - stageStartTime)), 0, 500, user_settings.led_count, 0), 0);
    for (int i = user_settings.led_count; i >= n; i--)
    {
      brightness = (sin(((i * 10) + mm) / 500.0) + 1) * 255;
      leds[i].setHSV(brightness, 255, 50);
    }
  }
  else if (stageStartTime + 5000 > mm)
  {
    for (int i = user_settings.led_count; i >= 0; i--)
    {
      brightness = (sin(((i * 10) + mm) / 500.0) + 1) * 255;
      leds[i].setHSV(brightness, 255, 50);
    }
  }
  else if (stageStartTime + 5500 > mm)
  {
    int n = _max(map(((mm - stageStartTime)), 5000, 5500, user_settings.led_count, 0), 0);
    for (int i = 0; i < n; i++)
    {
      brightness = (sin(((i * 10) + mm) / 500.0) + 1) * 255;
      leds[i].setHSV(brightness, 255, 50);
    }
  }
  else
  {
    nextLevel();
  }
}

void tickBossKilled(long mm)
{
  static uint8_t gHue = 0;
  FastLED.setBrightness(255);

  int brightness = 0;
  FastLED.clear();

  if (stageStartTime + 6500 > mm)
  {
    gHue++;
    fill_rainbow(leds, user_settings.led_count, gHue, 7);
    if (random8() < 200)
    {
      leds[random16(user_settings.led_count)] += CRGB::White;
    }
    SFXbosskilled();
  }
  else if (stageStartTime + 7000 > mm)
  {
    int n = _max(map(((mm - stageStartTime)), 5000, 5500, user_settings.led_count, 0), 0);
    for (int i = 0; i < n; i++)
    {
      brightness = (sin(((i * 10) + mm) / 500.0) + 1) * 255;
      leds[i].setHSV(brightness, 255, 50);
    }
    SFXcomplete();
  }
  else
  {
    nextLevel();
  }
}

void tickDie(long mm)
{
  const int duration = 200;
  const int width = 20;

  if (stageStartTime + duration > mm)
  {
    int brightness = map((mm - stageStartTime), 0, duration, 255, 150);

    // fill up
    int n = _max(map(((mm - stageStartTime)), 0, duration, getLED(playerPosition), getLED(playerPosition) + width), 0);
    for (int i = getLED(playerPosition); i <= n; i++)
    {
      leds[i] = CRGB(255, brightness, brightness);
    }

    // fill to down
    n = _max(map(((mm - stageStartTime)), 0, duration, getLED(playerPosition), getLED(playerPosition) - width), 0);
    for (int i = getLED(playerPosition); i >= n; i--)
    {
      leds[i] = CRGB(255, brightness, brightness);
    }
  }
}

void tickGameover(long mm)
{
  int brightness = 0;

  if (stageStartTime + GAMEOVER_SPREAD_DURATION > mm)
  {
    // fill to top
    int n = _max(map(((mm - stageStartTime)), 0, GAMEOVER_SPREAD_DURATION, getLED(playerPosition), user_settings.led_count), 0);
    for (int i = getLED(playerPosition); i <= n; i++)
    {
      leds[i] = CRGB(255, 0, 0);
    }
    // fill to bottom
    n = _max(map(((mm - stageStartTime)), 0, GAMEOVER_SPREAD_DURATION, getLED(playerPosition), 0), 0);
    for (int i = getLED(playerPosition); i >= n; i--)
    {
      leds[i] = CRGB(255, 0, 0);
    }
    SFXgameover();
  }
  else if (stageStartTime + GAMEOVER_FADE_DURATION > mm)
  {
    int n = _max(map(((mm - stageStartTime)), GAMEOVER_FADE_DURATION, GAMEOVER_SPREAD_DURATION, user_settings.led_count, 0), 0);
    brightness = map(((mm - stageStartTime)), GAMEOVER_SPREAD_DURATION, GAMEOVER_FADE_DURATION, 200, 0);

    for (int i = 0; i <= n; i++)
    {
      leds[i] = CRGB(brightness, 0, 0);
    }
    SFXcomplete();
  }
}

void tickWin(long mm)
{
  int brightness = 0;
  FastLED.clear();
  if (stageStartTime + WIN_FILL_DURATION > mm)
  {
    int n = _max(map(((mm - stageStartTime)), 0, WIN_FILL_DURATION, user_settings.led_count, 0), 0);
    for (int i = user_settings.led_count; i >= n; i--)
    {
      brightness = user_settings.led_brightness;
      leds[i] = CRGB(0, brightness, 0);
    }
    SFXwin();
  }
  else if (stageStartTime + WIN_CLEAR_DURATION > mm)
  {
    int n = _max(map(((mm - stageStartTime)), WIN_FILL_DURATION, WIN_CLEAR_DURATION, user_settings.led_count, 0), 0);
    for (int i = 0; i < n; i++)
    {
      brightness = user_settings.led_brightness;
      leds[i] = CRGB(0, brightness, 0);
    }
    SFXwin();
  }
  else if (stageStartTime + WIN_OFF_DURATION > mm)
  {
    leds[0] = CRGB(0, user_settings.led_brightness, 0);
  }
  else
  {
    nextLevel();
  }
}

void drawLives()
{
  SFXcomplete();
  FastLED.clear();

  int pos = 0;
  for (int i = 0; i < lives; i++)
  {
    for (int j = 0; j < 4; j++)
    {
      leds[pos++] = CRGB(0, 255, 0);
      FastLED.show();
    }
    leds[pos++] = CRGB(0, 0, 0);
    delay(20);
  }
  FastLED.show();
  delay(400);
  FastLED.clear();
}

void drawAttack()
{
  if (!attacking)
    return;
  int n = map(millis() - attackMillis, 0, ATTACK_DURATION, 100, 5);
  for (int i = getLED(playerPosition - (attack_width / 2)) + 1; i <= getLED(playerPosition + (attack_width / 2)) - 1; i++)
  {
    leds[i] = CRGB(0, 0, n);
  }
  if (n > 90)
  {
    n = 255;
    leds[getLED(playerPosition)] = CRGB(255, 255, 255);
  }
  else
  {
    n = 0;
    leds[getLED(playerPosition)] = CRGB(0, 255, 0);
  }
  leds[getLED(playerPosition - (attack_width / 2))] = CRGB(n, n, 255);
  leds[getLED(playerPosition + (attack_width / 2))] = CRGB(n, n, 255);
}

int getLED(int pos)
{
  return constrain((int)map(pos, 0, VIRTUAL_LED_COUNT, 0, user_settings.led_count - 1), 0, user_settings.led_count - 1);
}

bool inLava(int pos)
{
  int i;
  Lava LP;
  for (i = 0; i < LAVA_COUNT; i++)
  {
    LP = lavaPool[i];
    if (LP.Alive() && LP._state == Lava::ON)
    {
      if (LP._left <= pos && LP._right >= pos)
        return true;
    }
  }
  return false;
}

void updateLives()
{
  drawLives();
}

// ---------------------------------
// -------------- SFX --------------
// ---------------------------------

void SFXFreqSweepWarble(int duration, int elapsedTime, int freqStart, int freqEnd, int warble)
{
  int freq = map_constrain(elapsedTime, 0, duration, freqStart, freqEnd);
  if (warble)
    warble = map(sin(millis() / 20.0) * 1000.0, -1000, 1000, 0, warble);

  sound(freq + warble, user_settings.audio_volume);
}

void SFXFreqSweepNoise(int duration, int elapsedTime, int freqStart, int freqEnd, uint8_t noiseFactor)
{
  int freq;

  if (elapsedTime > duration)
    freq = freqEnd;
  else
    freq = map(elapsedTime, 0, duration, freqStart, freqEnd);

  if (noiseFactor)
    noiseFactor = noiseFactor - random8(noiseFactor / 2);

  sound(freq + noiseFactor, user_settings.audio_volume);
}

void SFXtilt(int amount)
{
  if (amount == 0)
  {
    SFXcomplete();
    return;
  }

  int f = map(abs(amount), 0, 90, 80, 900) + random8(100);
  if (playerPositionModifier < 0)
    f -= 500;
  if (playerPositionModifier > 0)
    f += 200;
  int vol = map(abs(amount), 0, 90, user_settings.audio_volume / 2, user_settings.audio_volume * 3 / 4);
  sound(f, vol);
}

void SFXattacking()
{
  int freq = map(sin(millis() / 2.0) * 1000.0, -1000, 1000, 500, 600);
  if (random8(5) == 0)
  {
    freq *= 3;
  }
  sound(freq, user_settings.audio_volume);
}

void SFXdead()
{
  SFXFreqSweepNoise(1000, millis() - killTime, 1000, 10, 200);
}

void SFXgameover()
{
  SFXFreqSweepWarble(GAMEOVER_SPREAD_DURATION, millis() - killTime, 440, 20, 60);
}

void SFXkill()
{
  sound(2000, user_settings.audio_volume);
}

void SFXwin()
{
  SFXFreqSweepWarble(WIN_OFF_DURATION, millis() - stageStartTime, 40, 400, 20);
}

void SFXbosskilled()
{
  SFXFreqSweepWarble(7000, millis() - stageStartTime, 75, 1100, 60);
}

void SFXcomplete()
{
  soundOff();
}

long map_constrain(long x, long in_min, long in_max, long out_min, long out_max)
{
  if (in_max > in_min)
  {
    x = constrain(x, in_min, in_max);
  }
  else
  {
    x = constrain(x, in_max, in_min);
  }
  return map(x, in_min, in_max, out_min, out_max);
}

// ---------------------------------
// --------- SCREENSAVER -----------
// ---------------------------------
void screenSaverTick()
{
  long mm = millis();
  int mode = (mm / 30000) % 4;

  SFXcomplete();

  if (mode == 0)
  {
    Fire2012();
  }
  else if (mode == 1)
    sinelon();
  else if (mode == 2)
    juggle();
  else
  {
    random_LED_flashes();
  }
}

// Fire2012 by Mark Kriegsman, July 2012
// as part of "Five Elements" shown here: http://youtu.be/knWiGsmgycY
#define COOLING 75
#define SPARKING 40

void Fire2012()
{
  static byte heat[VIRTUAL_LED_COUNT];
  bool gReverseDirection = false;

  // Step 1.  Cool down every cell a little
  for (int i = 0; i < user_settings.led_count; i++)
  {
    heat[i] = qsub8(heat[i], random8(0, ((COOLING * 10) / user_settings.led_count) + 2));
  }

  // Step 2.  Heat from each cell drifts 'up' and diffuses a little
  for (int k = user_settings.led_count - 1; k >= 2; k--)
  {
    heat[k] = (heat[k - 1] + heat[k - 2] + heat[k - 2]) / 3;
  }

  // Step 3.  Randomly ignite new 'sparks' of heat near the bottom
  if (random8() < SPARKING)
  {
    int y = random8(7);
    heat[y] = qadd8(heat[y], random8(160, 255));
  }

  // Step 4.  Map from heat cells to LED colors
  for (int j = 0; j < user_settings.led_count; j++)
  {
    CRGB color = HeatColor(heat[j]);
    int pixelnumber;
    if (gReverseDirection)
    {
      pixelnumber = (user_settings.led_count - 1) - j;
    }
    else
    {
      pixelnumber = j;
    }
    leds[pixelnumber] = color;
  }
}

void random_LED_flashes()
{
  long mm = millis();
  int i;

  for (i = 0; i < user_settings.led_count; i++)
  {
    leds[i].nscale8(250);
  }

  randomSeed(mm);
  for (i = 0; i < user_settings.led_count; i++)
  {
    if (random8(20) == 0)
    {
      leds[i] = CHSV(25, 255, 100);
    }
  }
}

void sinelon()
{
  static uint8_t gHue = 0;
  gHue++;

  fadeToBlackBy(leds, user_settings.led_count, 20);
  int pos = beatsin16(13, 0, user_settings.led_count);
  leds[pos] += CHSV(gHue, 255, 192);
}

void juggle()
{
  fadeToBlackBy(leds, user_settings.led_count, 20);
  byte dothue = 0;
  for (int i = 0; i < 4; i++)
  {
    leds[beatsin16(i + 7, 0, user_settings.led_count - 1)] |= CHSV(dothue, 200, 255);
    dothue += 64;
  }
}

void checkSerialInput()
{
  // Empty function - removed serial menu functionality
}