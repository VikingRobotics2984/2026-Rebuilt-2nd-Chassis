/*#include <Adafruit_NeoPixel.h>

#define LED_PIN 9
#define LED_COUNT 144//159
Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

void setup() {
  strip.begin();
  strip.setBrightness(50);
  Serial.begin(9600);
}

char pattern = 's';

void loop() {
  strip.show();
  delay(20);

  if (Serial.available() > 0) {
    pattern = Serial.read();
    switch(pattern){
      case 'w'://rainbow pattern
        patternRainbow();
        break;

      case 'e'://yellow/red error pattern
        patternError();
        break;

      case 's'://starting up code pattern
        patternStarting();
        break;

      case 'a'://yellow need apriltag
        for(int i=0; i<strip.numPixels(); i++)
          strip.setPixelColor(i, strip.Color(255, 160, 0));
        break;

      case 'b'://blue
        for(int i=0; i<strip.numPixels(); i++)
          strip.setPixelColor(i, strip.Color(0, 0, 255));
        break;
      
      case 'r'://red
        for(int i=0; i<strip.numPixels(); i++)
          strip.setPixelColor(i, strip.Color(255, 0, 0));
        break;
    }
    return;
  }

  switch(pattern){//update periodically
    case 'w':
      patternRainbow();
      break;
    case 'e':
      patternError();
      break;
    case 's':
      patternStarting();
      break;
    default:
      break;
  }
}

int cnt = 0;

void patternStarting() {
  cnt += 1;
  for(int i=0; i<strip.numPixels(); i++) {
    int num = ((cnt+i)%17)*15;
    strip.setPixelColor(i, strip.Color(num, num, 255));
  }
  if (cnt >= 17){
    cnt = 0;
  }
}

void patternError() {
  cnt += 1;
  for(int i=0; i<strip.numPixels(); i++) {
    int num = (cnt+i)%6;
    if (num < 3){
      strip.setPixelColor(i, strip.Color(255, 0, 0));
    }else{
      strip.setPixelColor(i, strip.Color(255, 200, 0));
    }
  }
  if (cnt >= 6){
    cnt = 0;
  }
}

void patternRainbow() {
  cnt += 8;
  for(int i=0; i<strip.numPixels(); i++) {
    int num = (cnt+i*8)%(200*6);
    int modnum = num%200;
    if (num < 200){
      strip.setPixelColor(i, strip.Color(modnum+56, 56, 56));
    }else if (num < 400){
      strip.setPixelColor(i, strip.Color(255, modnum+56, 56));
    }else if (num < 600){
      strip.setPixelColor(i, strip.Color(255, 255, modnum+56));
    }else if (num < 800){
      strip.setPixelColor(i, strip.Color(255-modnum, 255, 255));
    }else if (num < 1000){
      strip.setPixelColor(i, strip.Color(56, 255-modnum, 255));
    }else if (num < 1200){
      strip.setPixelColor(i, strip.Color(56, 56, 255-modnum));
    }
  }
  if (cnt >= 1200){
    cnt = 0;
  }
}*/