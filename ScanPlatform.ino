#include <avr/eeprom.h>
#include <Wire.h>
#include <string.h>
#include <AccelStepper.h>
#include <GroveEncoder.h> // https://github.com/dantler/GroveEncoder
#include <CRC.h>
#include "Button.h"

#include "ssd1306.h"
#include "ssd1306_console.h"

typedef void (*menu_callback)(int var);

// lcd
Ssd1306Console  lcd;

#define FONT_SIZE       (ssd1306xled_font8x16)

#if (FONT_SIZE == ssd1306xled_font8x16)
#define FONT_WIDTH      (8)
#define FONT_HEIGH      (16)
#elif (FONT_SIZE == ssd1306xled_font6x8)
#define FONT_WIDTH      (6)
#define FONT_HEIGH      (8)
#elif (FONT_SIZE == ssd1306xled_font5x6)
#define FONT_WIDTH      (5)
#define FONT_HEIGH      (7)
#else
#error "Unknown font type"
#endif

// On Uno/Nano only D2/D3 have interrupts: use ENCODER_CLK_PIN 2 and move shutter/button elsewhere.
#define ENCODER_BTN_PIN   (4)
#define ENCODER_CLK_PIN   (2)   // for Uno/Nano use 2 (uses 2 and 3)
GroveEncoder encoder(ENCODER_CLK_PIN, NULL);
Button enc_button(ENCODER_BTN_PIN);

// stepper motor
#define GEAR_RATIO                    (7)
#define MOTOR_STEPS_PER_REVOLUTION    (2038.0)
#define PLATFORM_STEPS_PER_REVOLUTION (MOTOR_STEPS_PER_REVOLUTION * GEAR_RATIO)
#define PLATFORM_ACCELERATION         (PLATFORM_STEPS_PER_REVOLUTION * 2)
#define STEPPER_IN1_PIN               (8)
#define STEPPER_IN2_PIN               (9)
#define STEPPER_IN3_PIN               (10)
#define STEPPER_IN4_PIN               (11)
#define PLATFORM_RPM_STEP             (0.05)
#define PLATFORM_RPM_MIN              (PLATFORM_RPM_STEP)
#define PLATFORM_RPM_MAX_RPM          (3.5)
#define PLATFORM_RPM_MAX_SETTING      ((int)((PLATFORM_RPM_MAX_RPM - PLATFORM_RPM_MIN) / PLATFORM_RPM_STEP + 0.5))
#define PLATFORM_RPM(rpm)             (PLATFORM_RPM_MIN + rpm * PLATFORM_RPM_STEP)

AccelStepper stepper(AccelStepper::FULL4WIRE, STEPPER_IN1_PIN, STEPPER_IN3_PIN, STEPPER_IN2_PIN, STEPPER_IN4_PIN);

#define SECONDS_PER_MINUTE (60)

const char* LOGO_STR    (" 3DScanPlatform");
const char* VERSION_STR ("  Version  1.0");

#define LOGO_DELAY          (1500)
#define SCAN_RESULT_DELAY   (1500)
#define RESET_MENU_DELAY    (20000)

enum
{
  ROTATION    = 0,
  START,
  MULTISHOOT,
  SETTINGS,
  NUM_MENU_ENTRIES
};

enum
{
  LANGUAGE = 0,
  SHOT_NUMBER,
  ROTATION_SPEED,
  SHUTTER_DELAY,
  SETTINGS_BACK,
  NUM_SETTINGS_ENTRIES
};

enum
{
  MSG_PUSH_STOP = 0,
  MSG_CANCELED,
  MSG_DONE,
  MSG_SPEED,
  NUM_MSGS
};

// English
const char* menu_entries_main_en[NUM_MENU_ENTRIES] = {
  "Rotation",
  "Scan",
  "Multishoot scan",
  "Settings",
};
const char* menu_entries_settings_en[NUM_SETTINGS_ENTRIES] = {
  "Language",
  "Number of shots",
  "Rotation speed",
  "Shutter delay",
  "Back",
};
// Polish (ASCII-safe for 8x16 font)
const char* menu_entries_main_pl[NUM_MENU_ENTRIES] = {
  "Obrot",
  "Skan",
  "Skan wielokrotny",
  "Ustawienia",
};
const char* menu_entries_settings_pl[NUM_SETTINGS_ENTRIES] = {
  "Jezyk",
  "Liczba zdjec",
  "Predkosc obr.",
  "Opozn. migawki",
  "Powrot",
};
const char** menu_entries_main;
const char** menu_entries_settings;

const char* messages_en[NUM_MSGS] = {
  "Push to stop",
  "Canceled",
  "Done!",
  "Speed:",
};

const char* messages_pl[NUM_MSGS] = {
  "Wcisnij stop",
  "Przerwano",
  "Gotowe!",
  "Predk:",
};

// menu handlers
void scan(void);
void multishoot_scan(void);
void rotate_continuous(void);
void set_language(void);
void set_number_of_shots(void);
void set_rpm(void);
void set_shutter_delay(void);
void show_main_menu_entry(byte entry);
void show_settings_menu_entry(byte entry);
void show_active_menu(void);
void (*handler[NUM_MENU_ENTRIES])(void) = {rotate_continuous, scan, multishoot_scan, NULL};
void (*settings_handler[NUM_SETTINGS_ENTRIES])(void) = {set_language, set_number_of_shots, set_rpm, set_shutter_delay, NULL};

#define MENU_LINE_POS    0, 20
#define SETTINGS_VAL_POS FONT_WIDTH*6, 40
#define ARROW_POS       FONT_WIDTH*3, 40
#define MENU_LINE_Y      (20)
#define SETTINGS_LINE_Y  (40)
#define LCD_COLS         (128 / FONT_WIDTH)
#define LCD_COLS_MAX     (26)

enum
{
  ALIGN_LEFT = 0,
  ALIGN_CENTER,
  ALIGN_RIGHT
};

// current menu entry, start with 0
int menu_entry = 0;
int settings_entry = 0;
bool in_settings_menu = false;

// default settings
#define SETTINGS_ADDRESS    (0)
#define ROTATION_SPEED_DEF  (10)   // range of 28BYJ-48 stepper is 0~17 rpm
#define NUMBER_OF_SHOTS_DEF (15)   // shots per rotation
#define SHUTTER_DELAY_DEF   (200)  // delay in ms
#define SHUTTER_PIN         (5)    // shutter pin

struct
{
  short shots;    // number of shots per rotation
  short rpm;      // rotation speed
  short delay;    // delay in ms
  short language; // 0=English, 1=Polish

  short crc16;  // CRC16 checksum
}settings;
  
const char* msg(byte id)
{
  return (settings.language == 1) ? messages_pl[id] : messages_en[id];
}

void lcd_clear_line(byte y)
{
  lcd.setCursor(0, y);
  for (byte i = 0; i < LCD_COLS; i++)
    lcd.print(" ");
}

#define CLEAR        (0x01)
#define ARROW        (0x02)
#define ALIGN_LEFT   (0x00)
#define ALIGN_CENTER (0x04)
#define ALIGN_RIGHT  (0x08)
#define ALIGN_MASK   (0x0C)

void lcd_print_text(byte y, const char* text, byte flags)
{
  int len = (int)strlen(text);
  if (len > LCD_COLS)
    len = LCD_COLS;

  byte start_col = 0;
  byte align = (flags & ALIGN_MASK);
  if (align == ALIGN_CENTER)
    start_col = (LCD_COLS - len) / 2;
  else if (align == ALIGN_RIGHT)
    start_col = LCD_COLS - len;

  char buf[LCD_COLS_MAX + 1];
  for (int i = 0; i < len; i++)
    buf[i] = text[i];
  buf[len] = '\0';

  if (flags & CLEAR)
    lcd_clear_line(y);
  if (flags & ARROW)
  {
    lcd.setCursor(ARROW_POS);
    lcd.print("-> ");
  }

  lcd.setCursor(start_col * FONT_WIDTH, y);
  lcd.print(buf);
}


void lcd_print_value_centered_int(int value)
{
  char buf[17];
  snprintf(buf, sizeof(buf), "%d", value);
  lcd_print_text(SETTINGS_LINE_Y, buf, ALIGN_CENTER | CLEAR);
}

void lcd_print_value_centered(const char* text)
{
  lcd_print_text(SETTINGS_LINE_Y, text, ALIGN_CENTER | CLEAR);
}

void lcd_print_value_centered_percent(int percent)
{
  char buf[17];
  snprintf(buf, sizeof(buf), "%d%%", percent);
  lcd_print_value_centered(buf);
}

void lcd_print_value_centered_float(float value, byte decimals)
{
  char buf[17];
  dtostrf(value, 0, decimals, buf);
  lcd_print_text(SETTINGS_LINE_Y, buf, ALIGN_CENTER | CLEAR);
}

void lcd_print_value_centered_edit(const char* text)
{
  lcd_print_text(SETTINGS_LINE_Y, text, ALIGN_CENTER | CLEAR | ARROW);
}

bool load_settings(void)
{
  // read EEPROM
  eeprom_read_block(&settings, SETTINGS_ADDRESS, sizeof (settings));
  
  // check CRC16
  if (settings.crc16 != crc16((uint8_t *)&settings, sizeof(settings) - sizeof(settings.crc16)))
  {
    // load defaults
    settings.shots = NUMBER_OF_SHOTS_DEF;
    settings.rpm = ROTATION_SPEED_DEF;
    settings.delay = SHUTTER_DELAY_DEF;
    settings.language = 0;
    save_settings();
    return false;
  }
  // ensure language is in valid range (in case of old EEPROM layout)
  if (settings.language < 0 || settings.language > 1)
    settings.language = 0;
 
  return true;
}

void save_settings(void)
{
  // calculate CRC16
  uint16_t new_crc16 = crc16((uint8_t *)&settings, sizeof(settings) - sizeof(settings.crc16));

  // save only if changed
  if (new_crc16 != settings.crc16)
  {
    settings.crc16 = new_crc16;
    // write to EEPROM
    eeprom_write_block(&settings, SETTINGS_ADDRESS, sizeof (settings));
  }
}

void menuitem_handler(short *value, const int _min, const int _max, const int _step, menu_callback cb)
{
  encoder.setValue(*value);
  
  lcd.setCursor(ARROW_POS);
  lcd.print("-> ");
  
  while (!enc_button.check())
  {
    int val = encoder.getValue();
    if (val < _min)
    {
      val = _min;
    }
    else if (val > _max)
    {
      val = _max;
    }

    if (*value != val)
    {
      if (val > *value)
      {
        *value += _step;
      }
      else
      {
        *value -= _step;
      }

      val = *value;

      if (cb != NULL)
      {
        (*cb)(val);
      }
    }

    encoder.setValue(val);
  }

  lcd.setCursor(NARROW_POS);
  lcd.print("   ");
  save_settings();
}

void scan(void)
{
  bool abort = false;
  long position = 0;
  byte idx = 0;

  lcd_print_text(MENU_LINE_Y, msg(MSG_PUSH_STOP), ALIGN_CENTER | CLEAR);

  // reset current position
  stepper.setCurrentPosition(position);
  stepper.enableOutputs();
  
  do
  {
    // print percentage
    int percent = (int)((idx * 100L + settings.shots - 1) / settings.shots);
    lcd_print_value_centered_percent(percent);
    
    // rotate
    position = PLATFORM_STEPS_PER_REVOLUTION * idx / settings.shots;

    stepper.moveTo(position);
    while(stepper.isRunning())
    {
      stepper.run();
      if (enc_button.check())
      {
        abort = true;
        break;
      }
    }
    
    // trigger shot
    digitalWrite(SHUTTER_PIN, LOW);
    delay(50);
    digitalWrite(SHUTTER_PIN, HIGH);
    // delay
    delay(200);
    
    idx++;
  } while (!abort && (idx < settings.shots));

  stepper.disableOutputs();

#if (SCAN_RESULT_DELAY > 0)
  // print result
  if (abort)
    lcd_print_text(SETTINGS_LINE_Y, msg(MSG_CANCELED), ALIGN_CENTER | CLEAR);
  else
    lcd_print_text(SETTINGS_LINE_Y, msg(MSG_DONE), ALIGN_CENTER | CLEAR);

  // delay
  delay(SCAN_RESULT_DELAY);
#endif

  show_active_menu();
}

void multishoot_scan(void)
{
  bool abort = false;

  lcd_print_text(MENU_LINE_Y, msg(MSG_PUSH_STOP), ALIGN_CENTER | CLEAR);

  // reset current position
  stepper.setCurrentPosition(0);
  stepper.enableOutputs();

  // trigger shot
  digitalWrite(SHUTTER_PIN, LOW);
  // delay
  delay(200);
  
  // rotate
  stepper.moveTo(PLATFORM_STEPS_PER_REVOLUTION);
  while(stepper.isRunning())
  {
    stepper.run();
    if (enc_button.check())
    {
      abort = true;
      break;
    }

  }

  digitalWrite(SHUTTER_PIN, HIGH);

  stepper.disableOutputs();

#if (SCAN_RESULT_DELAY > 0)
  // print result
  if (abort)
    lcd_print_text(SETTINGS_LINE_Y, msg(MSG_CANCELED), ALIGN_CENTER | CLEAR);
  else
    lcd_print_text(SETTINGS_LINE_Y, msg(MSG_DONE), ALIGN_CENTER | CLEAR);

  // delay
  delay(SCAN_RESULT_DELAY);
#endif

  show_active_menu();
}

void rotate_continuous(void)
{
  lcd_print_text(MENU_LINE_Y, msg(MSG_PUSH_STOP), ALIGN_CENTER | CLEAR);
  lcd_print_text(SETTINGS_LINE_Y, msg(MSG_SPEED), ALIGN_LEFT | CLEAR);
  lcd.setCursor(FONT_WIDTH*7, 40);
  lcd.print(PLATFORM_RPM(settings.rpm), 2);

  // rotate continuously at configured speed
  stepper.enableOutputs();
  float rpm_target = PLATFORM_RPM(settings.rpm);
  float rpm_current = rpm_target;
  stepper.setMaxSpeed(PLATFORM_STEPS_PER_REVOLUTION * rpm_current / SECONDS_PER_MINUTE);
  stepper.setSpeed(PLATFORM_STEPS_PER_REVOLUTION * rpm_current / SECONDS_PER_MINUTE);
  encoder.setValue(0);
  int last_encoder = encoder.getValue();
  unsigned long last_ramp = millis();
  while (!enc_button.check())
  {
    stepper.runSpeed();

    int enc = encoder.getValue();
    if (enc != last_encoder)
    {
      int delta = enc - last_encoder;
      last_encoder = enc;
      float new_rpm = rpm_target + (delta * 0.05f);
      if (new_rpm < PLATFORM_RPM(0))
        new_rpm = PLATFORM_RPM(0);
      else if (new_rpm > PLATFORM_RPM(PLATFORM_RPM_MAX_SETTING))
        new_rpm = PLATFORM_RPM(PLATFORM_RPM_MAX_SETTING);

      if (new_rpm != rpm_target)
      {
        rpm_target = new_rpm;
        lcd.setCursor(FONT_WIDTH*7, 40);
        lcd.print("     ");
        lcd.setCursor(FONT_WIDTH*7, 40);
        lcd.print(rpm_target, 2);
      }
    }

    unsigned long now = millis();
    if (now - last_ramp >= 30)
    {
      if (rpm_current < rpm_target)
        rpm_current += PLATFORM_RPM_STEP;
      else if (rpm_current > rpm_target)
        rpm_current -= PLATFORM_RPM_STEP;

      if (rpm_current < PLATFORM_RPM(0))
        rpm_current = PLATFORM_RPM(0);
      else if (rpm_current > PLATFORM_RPM(PLATFORM_RPM_MAX_SETTING))
        rpm_current = PLATFORM_RPM(PLATFORM_RPM_MAX_SETTING);

      stepper.setMaxSpeed(PLATFORM_STEPS_PER_REVOLUTION * rpm_current / SECONDS_PER_MINUTE);
      stepper.setSpeed(PLATFORM_STEPS_PER_REVOLUTION * rpm_current / SECONDS_PER_MINUTE);
      last_ramp = now;
    }
  }

  stepper.disableOutputs();
  lcd_clear_line(SETTINGS_LINE_Y);
  {
    float rpm_steps = (rpm_target - PLATFORM_RPM_MIN) / PLATFORM_RPM_STEP;
    int rpm_setting = (int)(rpm_steps + 0.5f);
    if (rpm_setting < 0) rpm_setting = 0;
    else if (rpm_setting > PLATFORM_RPM_MAX_SETTING) rpm_setting = PLATFORM_RPM_MAX_SETTING;
    settings.rpm = rpm_setting;
  }
  save_settings();

  show_active_menu();
}

void default_menuitem_cb(int value)
{
  char buf[17];
  snprintf(buf, sizeof(buf), "%d", value);
  lcd_print_value_centered_edit(buf);
}

void set_rpm_cb(int rpm)
{
  char buf[17];
  dtostrf(PLATFORM_RPM(rpm), 0, 2, buf);
  lcd_print_value_centered_edit(buf);
  
  // set speed
  stepper.setMaxSpeed(PLATFORM_STEPS_PER_REVOLUTION * PLATFORM_RPM(rpm) / SECONDS_PER_MINUTE);
}

void set_number_of_shots(void)
{
  menuitem_handler(&settings.shots, 5, 250, 1, default_menuitem_cb);
}

void set_rpm(void)
{
  // enable timer interrupt
  
  menuitem_handler(&settings.rpm, 0, PLATFORM_RPM_MAX_SETTING, 1, set_rpm_cb);

  // disable timer interrupt
}

void set_shutter_delay(void)
{ 
  menuitem_handler(&settings.delay, 1, 500, 10, default_menuitem_cb);
}

void set_menu_language_ptr(void)
{
  switch (settings.language)
  {
    case 1:
      menu_entries_main = menu_entries_main_pl;
      menu_entries_settings = menu_entries_settings_pl;
      break;
    default:
      menu_entries_main = menu_entries_main_en;
      menu_entries_settings = menu_entries_settings_en;
      break;
  }
}

void set_language_cb(int lang)
{
  lcd_print_value_centered_edit((lang == 0) ? "EN" : "PL");
}

void set_language(void)
{
  menuitem_handler(&settings.language, 0, 1, 1, set_language_cb);
  set_menu_language_ptr();
  show_active_menu();
}

void show_main_menu_entry(byte entry)
{
  lcd_clear_line(MENU_LINE_Y);
  lcd_clear_line(SETTINGS_LINE_Y);

  lcd_print_text(MENU_LINE_Y, menu_entries_main[entry], ALIGN_CENTER | CLEAR);
}

void show_settings_menu_entry(byte entry)
{
  lcd_clear_line(MENU_LINE_Y);
  lcd_clear_line(SETTINGS_LINE_Y);

  lcd_print_text(MENU_LINE_Y, menu_entries_settings[entry], ALIGN_CENTER | CLEAR);
  switch (entry)
  {
    case LANGUAGE:
      lcd_print_value_centered((settings.language == 0) ? "EN" : "PL");
      break;
    case ROTATION_SPEED:
      lcd_print_value_centered_float(PLATFORM_RPM(settings.rpm), 2);
      break;
    case SHOT_NUMBER:
      lcd_print_value_centered_int(settings.shots);
      break;
    case SHUTTER_DELAY:
      lcd_print_value_centered_int(settings.delay);
      break;
    case SETTINGS_BACK:
    default:
      break;
  }
}

void show_active_menu(void)
{
  if (in_settings_menu)
    show_settings_menu_entry(settings_entry);
  else
    show_main_menu_entry(menu_entry);
}

void button_handler(void)
{
  if (!in_settings_menu)
  {
    if (menu_entry == SETTINGS)
    {
      in_settings_menu = true;
      settings_entry = 0;
      encoder.setValue(settings_entry);
      show_settings_menu_entry(settings_entry);
      return;
    }

    if (*handler[menu_entry] != NULL)
    {
      (*handler[menu_entry])();

      // restore encoder value
      encoder.setValue(menu_entry);
    }
  }
  else
  {
    if (settings_entry == SETTINGS_BACK)
    {
      in_settings_menu = false;
      encoder.setValue(menu_entry);
      show_main_menu_entry(menu_entry);
      return;
    }

    if (*settings_handler[settings_entry] != NULL)
    {
      (*settings_handler[settings_entry])();
      encoder.setValue(settings_entry);
    }
  }
}


void setup()
{
  ssd1306_128x64_i2c_init();
  ssd1306_setFixedFont(FONT_SIZE);
  ssd1306_fillScreen(0);
  lcd.begin();

  load_settings();
  set_menu_language_ptr();

  stepper.disableOutputs();
  stepper.setAcceleration(PLATFORM_ACCELERATION);
  // set speed
  stepper.setMaxSpeed(PLATFORM_STEPS_PER_REVOLUTION * PLATFORM_RPM(settings.rpm) / SECONDS_PER_MINUTE);
  
#if (LOGO_DELAY > 0)
  lcd.setCursor(MENU_LINE_POS);
  lcd.print(LOGO_STR);
  lcd.setCursor(0, 40);
  lcd.print(VERSION_STR);
  delay(LOGO_DELAY);
  lcd.clear();
#endif

  digitalWrite(SHUTTER_PIN, HIGH);
  pinMode(SHUTTER_PIN, OUTPUT);
    
  show_active_menu();
}

void loop() 
{
  static unsigned long timestamp;

  int value = encoder.getValue();
  int max_entries = in_settings_menu ? NUM_SETTINGS_ENTRIES : NUM_MENU_ENTRIES;
  if (value < 0)
  {
    value = max_entries - 1;
    encoder.setValue(value);
  }
  
  value %= max_entries;

  if ((!in_settings_menu && (value != menu_entry)) ||
      (in_settings_menu && (value != settings_entry)))
  {
    if (in_settings_menu)
    {
      settings_entry = value;
      show_settings_menu_entry(settings_entry);
    }
    else
    {
      menu_entry = value;
      show_main_menu_entry(menu_entry);
    }
#if (RESET_MENU_DELAY > 0)    
    // save time of last menu action 
    timestamp = millis();
#endif
  }
#if (RESET_MENU_DELAY > 0)
  else if (value && (timestamp + RESET_MENU_DELAY < millis()))
  {
    encoder.resetValue();
  }
#endif

  if (enc_button.check())
  {
    button_handler();
#if (RESET_MENU_DELAY > 0)
    // update time of last menu action   
    timestamp = millis();
#endif
  }
}
