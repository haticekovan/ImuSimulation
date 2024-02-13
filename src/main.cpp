// oled de top hareket ettirme
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "imu.h"


#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

int centerX = SCREEN_WIDTH / 2;
int centerY = SCREEN_HEIGHT / 2;
int speed = 2;
int radius = 10; // Topun yarıçapı
int direction = -1;

IMU mu;
 
void setup() {

  mu.settings();

  Wire.begin();
  mu.mag();
  mu.calibrate();
  Serial.begin(9600);
  while (!Serial);

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;);
  }

  display.display();
  display.clearDisplay();
  Serial.begin(9600);

  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;);
  }

  delay(2000);
  display.clearDisplay();

  // Topu ekranın ortasına yerleştir
  centerX = SCREEN_WIDTH / 2;
  centerY = SCREEN_HEIGHT / 2;

  display.fillCircle(centerX, centerY, radius, SSD1306_WHITE);
}

void loop() {
    // Sensörlerden veri okuma işlemi
    mu.readData();

    centerX += mu.roll * speed;
    centerY += mu.pitch * speed;

    if (centerX  >= SCREEN_WIDTH ) {
        centerX  =SCREEN_WIDTH; // Yönü tersine çevir
    }
    if (centerX  < 0){
        centerX =0;
    }
    if (centerY >= SCREEN_HEIGHT ) {
        centerY = SCREEN_HEIGHT; // Yönü tersine çevir
    }
    if(centerY  <0){
        centerY =0;
    }

    display.clearDisplay();
    
    display.fillCircle(centerX, centerY, radius, SSD1306_WHITE);
    display.display();
    
    // Yaw, pitch, roll değerlerini yazdır
    Serial.print("Yaw: "); Serial.println(mu.yaw);
    Serial.print("Roll: "); Serial.println(mu.roll);
    Serial.print("Pitch: "); Serial.println(mu.pitch);
    Serial.println();

    delay(100);
}





