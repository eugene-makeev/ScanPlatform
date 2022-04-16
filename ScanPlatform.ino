#include <avr/eeprom.h>
#include <Wire.h>
#include <AccelStepper.h>
#include <GroveEncoder.h>
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

// encoder
#define ENCODER_BTN_PIN   (4)
#define ENCODER_CLK_PIN   (2)
GroveEncoder encoder(ENCODER_CLK_PIN, NULL);
Button enc_button(ENCODER_BTN_PIN);

// stepper motor
#define GEAR_RATIO                    (7)
#define MOTOR_STEPS_PER_REVOLUTION    (2038.0)
#define PLATFORM_STEPS_PER_REVOLUTION (MOTOR_STEPS_PER_REVOLUTION * GEAR_RATIO)
#define PLATFORM_ACCELERATION         (PLATFORM_STEPS_PER_REVOLUTION * 2)
#define STEPPER_IN1_PIN               (5)
#define STEPPER_IN2_PIN               (6)
#define STEPPER_IN3_PIN               (7)
#define STEPPER_IN4_PIN               (8)
#define PLATFORM_RPM(rpm)             (1.0 + rpm * 0.1)

AccelStepper stepper(AccelStepper::FULL4WIRE, STEPPER_IN1_PIN, STEPPER_IN3_PIN, STEPPER_IN2_PIN, STEPPER_IN4_PIN);

#define SECONDS_PER_MINUTE (60)

const char* LOGO_STR    (" 3DScanPlatform");
const char* VERSION_STR ("  Version  1.0");

#define LOGO_DELAY          (1500)
#define SCAN_RESULT_DELAY   (1500)
#define RESET_MENU_DELAY    (20000)

enum
{
  START       = 0,
  SHOT_NUMBER,
  ROTATION_SPEED,
  SHUTTER_DELAY,
  NUM_MENU_ENTRIES
};

char* menu_entries[NUM_MENU_ENTRIES] = \
{
  "   Start scan",
  " Number of shots",
  " Rotation speed",
  " Shutter delay",
};

// menu handlers
void scan(void);
void set_number_of_shots(void);
void set_rpm(void);
void set_shutter_delay(void);
void (*handler[NUM_MENU_ENTRIES])(void) = {scan, set_number_of_shots, set_rpm, set_shutter_delay};

#define MENU_LINE_POS    0, 20
#define SETTINGS_VAL_POS FONT_WIDTH*6, 40
#define NARROW_POS       FONT_WIDTH*3, 40

// current menu entry
int menu_entry = 0;

// settings
#define SETTINGS_ADDRESS    (0)
#define ROTATION_SPEED_DEF  (10)   // range of 28BYJ-48 stepper is 0~17 rpm
#define NUMBER_OF_SHOTS_DEF (15)   // shots per rotation
#define SHUTTER_DELAY_DEF   (200)  // delay in ms
#define SHUTTER_PIN         (9)    // shutter pin

union{
  struct
  {
    short crc16;  // CRC8 checksum
    short shots; // number of shots per rotation
    short rpm;   // rotation speed
    short delay; // delay in ms
  };
  
  short values[NUM_MENU_ENTRIES];
} settings;

bool load_settings(void)
{
  // read EEPROM
  eeprom_read_block(&settings, SETTINGS_ADDRESS, sizeof (settings));
  
  // check CRC16
  if (settings.crc16 != crc16((uint8_t *)&settings.values[1], sizeof(settings) - sizeof(settings.crc16)))
  {
    // load defaults
    settings.values[SHOT_NUMBER] = NUMBER_OF_SHOTS_DEF;
    settings.values[ROTATION_SPEED] = ROTATION_SPEED_DEF;
    settings.values[SHUTTER_DELAY] = SHUTTER_DELAY_DEF;
    save_settings();
    return false;
  }
 
  return true;
}

void save_settings(void)
{
  // calculate CRC16
  uint16_t new_crc16 = crc16((uint8_t *)&settings.values[1], sizeof(settings) - sizeof(settings.crc16));

  // save only if changed
  if (new_crc16 != settings.crc16)
  {
    settings.crc16 = new_crc16;
    // write to EEPROM
    eeprom_write_block(&settings, SETTINGS_ADDRESS, sizeof (settings));
  }
}

void menuitem_handler(const int menuitem, const int _min, const int _max, const int _step, menu_callback cb)
{
  encoder.setValue(settings.values[menu_entry]);
  
  lcd.setCursor(NARROW_POS);
  lcd.print("-> ");
  
  while (!enc_button.check())
  {
    int value = encoder.getValue();
    if (value < _min)
    {
      value = _min;
    }
    else if (value > _max)
    {
      value = _max;
    }

    if (settings.values[menuitem] != value)
    {
      if (value > settings.values[menuitem])
      {
        settings.values[menuitem] += _step;
      }
      else
      {
        settings.values[menuitem] -= _step;
      }

      value = settings.values[menuitem];

      if (cb != NULL)
      {
        (*cb)(value);
      }
    }

    encoder.setValue(value);
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

  lcd.setCursor(MENU_LINE_POS);
  lcd.print("  Push to stop");

  // reset current position
  stepper.setCurrentPosition(position);
  stepper.enableOutputs();
  
  do
  {
    // print percentage
    lcd.setCursor(SETTINGS_VAL_POS);
    lcd.print(100.0 * idx / settings.shots);
    lcd.print("%");
    
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
    delay(settings.delay);
    
    idx++;
  } while (!abort && (idx < settings.shots));

  stepper.disableOutputs();

#if (SCAN_RESULT_DELAY > 0)
  // print result
  lcd.setCursor(SETTINGS_VAL_POS);
  if (abort)
  {
    lcd.print("Canceled");
  }
  else
  {
    lcd.print("Done!  ");
  }

  // delay
  delay(SCAN_RESULT_DELAY);
#endif

  show_menu_entry(menu_entry);
}

void default_menuitem_cb(int value)
{
  lcd.setCursor(SETTINGS_VAL_POS);
  lcd.print("          ");
  lcd.setCursor(SETTINGS_VAL_POS);
  lcd.print(value);
}

void set_rpm_cb(int rpm)
{
  lcd.setCursor(SETTINGS_VAL_POS);
  lcd.print("          ");
  lcd.setCursor(SETTINGS_VAL_POS);
  lcd.print(PLATFORM_RPM(rpm));
  
  // set speed
  stepper.setMaxSpeed(PLATFORM_STEPS_PER_REVOLUTION * PLATFORM_RPM(rpm) / SECONDS_PER_MINUTE);
}

void set_number_of_shots(void)
{
  menuitem_handler(menu_entry, 5, 250, 1, default_menuitem_cb);
}

void set_rpm(void)
{
  // enable timer interrupt
  
  menuitem_handler(menu_entry, 0, 25, 1, set_rpm_cb);

  // disable timer interrupt
}

void set_shutter_delay(void)
{ 
  menuitem_handler(menu_entry, 1, 500, 10, default_menuitem_cb);
}

void show_menu_entry(byte menu_entry)
{
  lcd.setCursor(MENU_LINE_POS);
  lcd.print("                ");
  lcd.setCursor(SETTINGS_VAL_POS);
  lcd.print("          ");
  
  lcd.setCursor(MENU_LINE_POS);
  lcd.print(menu_entries[menu_entry]);  
  lcd.setCursor(SETTINGS_VAL_POS);
  switch (menu_entry)
  {
    case START:
      break;
    case ROTATION_SPEED:
      lcd.print(PLATFORM_RPM(settings.values[menu_entry]));
      break;
    default:
      lcd.print(settings.values[menu_entry]);
  }
}

void button_handler(void)
{
  (*handler[menu_entry])();
  // restore encoder value
  encoder.setValue(menu_entry);
}


void setup()
{
  ssd1306_128x64_i2c_init();
  ssd1306_setFixedFont(FONT_SIZE);
  ssd1306_fillScreen(0);
  lcd.begin();

  load_settings();

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
    
  show_menu_entry(menu_entry);
}

void loop() 
{
  static unsigned long timestamp;

  int value = encoder.getValue();
  if (value < 0)
  {
    value = NUM_MENU_ENTRIES - 1;
    encoder.setValue(value);
  }
  
  value %= NUM_MENU_ENTRIES;

  if (value != menu_entry)
  {
    menu_entry = value;
    show_menu_entry(menu_entry);
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
