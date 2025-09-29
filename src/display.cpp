#include "display.h"
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Wire.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
static const int OLED_RESET_PIN = -1;  // shared reset
static const uint8_t OLED_PRIMARY_ADDRESS = 0x3D;  // default for many Adafruit boards
static const uint8_t OLED_FALLBACK_ADDRESS = 0x3C;  // alternate address (also try 0x30 if needed)

static Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET_PIN);
static bool display_ready = false;

bool display_init() {
  Wire.begin();  // default pins SDA/SCL
  if (display.begin(SSD1306_SWITCHCAPVCC, OLED_PRIMARY_ADDRESS)) {
    display_ready = true;
  } else if (display.begin(SSD1306_SWITCHCAPVCC, OLED_FALLBACK_ADDRESS)) {
    display_ready = true;
  } else if (display.begin(SSD1306_SWITCHCAPVCC, 0x30)) {
    display_ready = true;
  } else {
    display_ready = false;
    Serial.println("OLED display not detected (addresses tried: 0x3D, 0x3C, 0x30)\r");
    return false;
  }

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(F("Frequency Counter"));
  display.println(F("Initializing..."));
  display.display();
  return true;
}

void display_update(DisplayStatus const& status) {
  if (!display_ready) {
    return;
  }

  display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  if (status.utc_valid) {
    char buffer[32];
    snprintf(buffer, sizeof(buffer), "%04u-%02u-%02u %02u:%02u:%02u",
             status.utc.year,
             status.utc.month,
             status.utc.day,
             status.utc.hour,
             status.utc.minute,
             status.utc.second);
    display.println(buffer);
  } else {
    display.println(F("UTC: --"));
  }

  display.print(F("Lock: "));
  display.println(status.pps_locked ? F("YES") : F("NO"));

  int16_t indicator_x = 3;
  int16_t indicator_y = SCREEN_HEIGHT-4;
  if (status.output_high) {
    display.fillCircle(indicator_x, indicator_y, 3, SSD1306_WHITE);
  } else {
    display.drawCircle(indicator_x, indicator_y, 3, SSD1306_WHITE);
  }

  if (status.calibrating) {
    // Show calibration status
    display.print(F("CALIBRATING P"));
    display.println(status.cal_phase);
    
    // Show countdown
    uint32_t minutes = status.cal_remaining_seconds / 60;
    uint32_t seconds = status.cal_remaining_seconds % 60;
    display.print(F("Time: "));
    display.print(minutes);
    display.print(F(":"));
    if (seconds < 10) display.print(F("0"));
    display.println(seconds);
    
    // Show current average in ppb
    display.print(F("Avg: "));
    display.print(status.cal_current_ppm * 1000.0, 1);
    display.println(F(" ppb"));
    
    display.print(F("Samples: "));
    display.println(status.sample_count);
  } else if (status.sample_count > 0 && status.pps_locked) {
    // Normal operation - show PPM measurements and calibration offset
    double ppb_inst = status.ppm_error * 1000.0;
    double ppb_avg = status.ppm_average * 1000.0;
    display.print(F("PPB Inst:"));
    display.println(ppb_inst, 2);
    display.print(F("PPB Avg :"));
    display.println(ppb_avg, 2);
    
    // Show sample count
    display.print(F("Samples: "));
    display.println(status.sample_count);
    
    // Show calibration offset in ppb
    display.print(F("CAL OFFSET: "));
    display.print(status.cal_offset_ppm * 1000.0, 1);
    display.println(F("ppb"));
  } else {
    display.println(F("Waiting for PPS"));
    
    // Show sample count even when waiting
    if (status.sample_count > 0) {
      display.print(F("Samples: "));
      display.println(status.sample_count);
    }
    
    // Show calibration offset even when waiting in ppb
    display.print(F("CAL OFFSET: "));
    display.print(status.cal_offset_ppm * 1000.0, 1);
    display.println(F("ppb"));
  }

  display.display();
}

bool display_available() {
  return display_ready;
}

