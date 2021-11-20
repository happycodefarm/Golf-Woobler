#include <Arduino.h>
#include <EEPROM.h>
#include "elapsedMillis/elapsedMillis.h"
#include <FastLED.h>
#include <Preferences.h>
#include "TM1637Display.h"
#include "slopes.h"
//#define SETUP

#define LEDSTRIP_PIN     5
#define COLOR_ORDER GRB
#define CHIPSET     WS2811
#define BUTTON    0
#define BRIGHTNESS  200
#define PIEZO 4
Preferences prefs;

float terrainScale = 200.0f;
float sensorScale = 2.0f;
float ballMass = 1.0f; // Kg // not used
float terrainFriction = 0.1f; // Coefficient of Friction (mu)
float worldGravity = 9.98f;
float ballInHoleVelocity = 1.0f;
float ballVelocity = 0.0f; // m/s
float ballAcceleration = 0.0; // m/s^2
float maxBallVelocity = 800.0;
unsigned int hyterisisLimit = 50;
unsigned long immobilityCounter = 0;
unsigned long hysterisisCounter = 0;
bool lastBallVelocityDirection = false;
int lastPositionIndex = 0;
int sensorSamplingStartTime = 0;
float peak = 0.0;
bool sensorSampling = false;
unsigned long lastPlay = 0;

// Module connection pins (Digital Pins)
#define CLK 18
#define DIO 19
TM1637Display display(CLK, DIO);

/* Animation Data - HGFEDCBA Map */
const uint8_t ANIMATION[5][4] = {
  { 0x08, 0x08, 0x08, 0x5c },  // Frame 0
  { 0x08, 0x08, 0x5c, 0x08 },  // Frame 1
  { 0x08, 0x5c, 0x08, 0x08 },  // Frame 2
  { 0x5c, 0x08, 0x08, 0x08 },  // Frame 3
  { 0x08, 0x08, 0x08, 0x08 }   // Frame 4
};

const uint8_t NONE[] = {};
const uint8_t PLAY[] = {B01110011, B00111000, B01011111, B01101110};
const uint8_t HITS[] = {0x76, 0x06, 0x78, 0x6d};
const uint8_t GOOD[] = {0x3d, 0x3f, 0x3f, 0x5e};
const uint8_t BAD[] =  {0x00, 0x7c, 0x5f, 0x5e};
const uint8_t LOST[] = {0x38, 0x3f, 0x6d, 0x78};
const uint8_t HOLE[] = {0x76, 0x3f, 0x38, 0x79};
const uint8_t GOLF[] = {0x3d, 0x3f, 0x38, 0x71};

float ballPosition = 0.0f;
int ballPositionIndex = 0;
int holeLedIndex = 0;

int holes[] = {125, 218,417};
int holeCounter = 0;
int maxHoles = 3;

bool playing = false; // playing state
int hits = 0; // hits counter

elapsedMillis worldTime;
int timeInterval = 2; // game timer

CRGB leds[NUM_LEDS];

void fadeall() { for(int i = 0; i < NUM_LEDS; i++) { leds[i].nscale8(250); } }


void showWin() {
  Serial.println("win");

  for(int i = 0; i < NUM_LEDS; i++) {
      (millis()/20 + i) % 20 < 10 ?  leds[i] = CRGB::Yellow : leds[i] = CRGB::Black;
      FastLED.show(); 
      if (millis()%2000<1000) display.setSegments(HITS);
      else display.showNumberDec(hits,false);
  }
}

void showSucess() {
    unsigned char hue = 0;
    Serial.println("success");
    display.setSegments(GOOD);
    for(int i = holes[holeCounter-1]; i < holes[holeCounter]; i++) {
      leds[i] = CHSV(hue++, 255, 255);
      FastLED.show(); 
      fadeall();
      
    }
}

void showLoose() {
    Serial.println("loose");
    display.setSegments(LOST);
    for(int i = 0; i < NUM_LEDS; i++) {
      (millis()/20 + i) % 20 < 10 ?  leds[i] = CRGB::Red : leds[i] = CRGB::Black;
      FastLED.show();
      if (millis()%2000<1000) display.setSegments(NONE);
      else display.showNumberDec(hits,false);
  }
}

void showMissed() {
  display.setSegments(BAD);
  Serial.println("missed");
  for(int i = 0; i < 100; i+=3) {
    FastLED.show(); 
    fadeall();
  }
}

void resetGame() {
  
  playing = false;
  ballPosition = 0;
  ballPositionIndex = 0;
  hysterisisCounter = 0;
  immobilityCounter = 0;
  hits = 0;
}

void replayHole() {
  playing = false;
  ballPosition = 0;
  ballPositionIndex = 0;
  hysterisisCounter = 0;
  immobilityCounter = 0;
}

void nextHole() {
  holeCounter++;
  holeLedIndex = holes[holeCounter];

  playing = false;
  ballPosition = 0;
  ballPositionIndex = 0;
  hysterisisCounter = 0;
  immobilityCounter = 0;

  if (holeCounter >= maxHoles) { // win !!
    showWin();
    resetGame();
  } else { // 
    if (hits == 8) { // restart game
      showLoose();
      resetGame();
    } else {
      showSucess();
    }
  }
}

void draw() {
  for (int led = 0; led < NUM_LEDS; led++) { // green terrain
    leds[led] =  led % 10 == 0 ? CRGB(2,5,0) : CRGB(0,5,0);//CHSV(terrainHeights[led],255,terrainHeights[led]/10.0);
  }

  // leds[millis()/200 % NUM_LEDS] = CRGB(3,5,0);
  
  // hole
  if (!playing) {
    if (millis()%500 < 250) {
      leds[holeLedIndex-1] = CRGB(255,20,0);
      leds[holeLedIndex] = CRGB::Red;
      leds[holeLedIndex+1] = CRGB(255,20,0);
    }
  } else {
    leds[holeLedIndex-1] = CRGB(255,20,0);
    leds[holeLedIndex] = CRGB::Red;
    leds[holeLedIndex+1] = CRGB(255,20,0);

  }

  // ball
  leds[ballPositionIndex] = CRGB::White;

  // if (playing) display.showNumberDec(ballPositionIndex, true);
  if (playing) display.setSegments(ANIMATION[millis()/200 % 5]); // 7 segments animation
  else { // print game info
      if (hits == 0) { // print "play golf"
        if (millis()%2000<1000) display.setSegments(PLAY);
        else display.setSegments(GOLF);
      } else { // print hits count
      int ticks = millis()%6000;
      if (ticks<1000) display.setSegments(PLAY);
      else if (ticks<2000) display.showNumberDec(hits,false);
      else display.setSegments(HITS);
    }
  }
  FastLED.show();
}

void setup() {
  Serial.begin(115200);

  prefs.begin("vGolf", false); 
  #ifdef SETUP
  prefs.putFloat("t_scale", terrainScale);
  prefs.putFloat("g_scale", sensorScale);
  prefs.putFloat("b_mass", ballMass);
  prefs.putFloat("t_friction", terrainFriction);
  prefs.putFloat("w_gravity", worldGravity);
  prefs.putFloat("b_hole", ballInHoleVelocity);
  #endif

  terrainScale = prefs.getFloat("t_scale");
  sensorScale = prefs.getFloat("g_scale");
  ballMass = prefs.getFloat("b_mass");
  terrainFriction = prefs.getFloat("t_friction");
  worldGravity = prefs.getFloat("w_gravity");
  ballInHoleVelocity = prefs.getFloat("b_hole");

  pinMode(BUTTON, INPUT_PULLUP);
  pinMode(LED_BUILTIN, OUTPUT);

  display.setBrightness(0x0a);
  holeLedIndex = holes[holeCounter];

  FastLED.addLeds<CHIPSET, LEDSTRIP_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection( TypicalLEDStrip );
  FastLED.setBrightness( BRIGHTNESS );

  display.setSegments(GOLF);

  draw();
  delay(2000);
}

void printPrefs() {
  Serial.printf("Setting\n\n----------\n(t)errain scale : %f \n", prefs.getFloat("t_scale"));
  Serial.printf("sensor (s)cale : %f \n", prefs.getFloat("g_scale"));
  Serial.printf("ball (m)ass : %f \n", prefs.getFloat("b_mass"));
  Serial.printf("ball in hole (v)elocity : %f \n", prefs.getFloat("b_hole"));
  Serial.printf("terrain (f)riction : %f \n", prefs.getFloat("t_friction"));
  Serial.printf("world (g)ravity : %f \n----------\n", prefs.getFloat("w_gravity"));
}

void idle(){
  if (millis() - lastPlay > 120000 ) { // 2 min reset.
    resetGame();
  }

  digitalWrite(LED_BUILTIN,LOW);
  float v = analogRead(PIEZO)/4095.0; // piezo reading
  if (v > 0.1 && !sensorSampling) { // valid hit ceil
    sensorSamplingStartTime = millis();
    sensorSampling = true;
    Serial.println("Sampling");
  }
  if (sensorSampling && millis()-sensorSamplingStartTime < 200) {
    peak += v;
  } else if (sensorSampling) {
    lastPlay = millis();
    ballPosition = 0;
    ballPositionIndex = 0;
    Serial.println("Sampling Done");

    ballVelocity = peak * sensorScale;
    playing = true;
    peak = 0;
    sensorSampling = false;
    Serial.printf("velocity = %f\n", ballVelocity);
    digitalWrite(LED_BUILTIN,HIGH);
    hits ++;
  }
}

void play() {
  digitalWrite(LED_BUILTIN,millis()%1000>500);
  worldTime = 0;

  float nReaction = (ballMass*worldGravity*sin(terrainSlopeAngles[ballPositionIndex]*PI/180));
  float nFriction = (ballMass*terrainFriction*worldGravity*cos(terrainSlopeAngles[ballPositionIndex]*PI/180));

  // acceleration
  lastBallVelocityDirection = ballVelocity >=0;
  ballVelocity -= nReaction;

  // friction 
  if (ballVelocity > 0.0f) ballVelocity -= nFriction;
  else if (ballVelocity < 0.0f ) ballVelocity += nFriction;

  // hysterisis check
  if (lastBallVelocityDirection != (ballVelocity >= 0)) hysterisisCounter ++;
  if (hysterisisCounter > hyterisisLimit) {
    Serial.println("hysterisis");
    ballVelocity = 0;
    //showMissed();
  }

  ballPosition += ballVelocity; // update ball position

  // position to led index with congruance
  ballPositionIndex = int(floor(ballPosition / terrainScale)) % NUM_LEDS;

  // immobility check
  if (lastPositionIndex == ballPositionIndex) immobilityCounter++; 
  else immobilityCounter = 0;
  lastPositionIndex = ballPositionIndex;

  if (immobilityCounter > 100) {
    showMissed();
    replayHole();
    return;
  }

  // ball over hole velocity reduction
  if (abs(ballPositionIndex - holeLedIndex) < 5) {
    ballVelocity *=0.8;
  }
  // ball in hole chek
  if (abs(ballPositionIndex - holeLedIndex) < 5 && abs(ballVelocity) < ballInHoleVelocity) {
    ballPositionIndex = holeLedIndex;
    Serial.println("HOLE");
    nextHole();
    return;
  
  } else if (ballPositionIndex <= 0 && ballVelocity < 0.0f) { // bounce
    ballVelocity = -ballVelocity * 0.8;
    ballPosition = 0;
    ballPositionIndex = 0;
  }
}

void loop() {
  if(!digitalRead(BUTTON)){ // debug button
    playing = false;
    ballPosition = 0;
    ballPositionIndex = 0;
  }

  if (Serial.available()) { // Serial command parsing
    uint8_t cmd = Serial.read();
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    if (cmd == 't') {
      terrainScale =  Serial.parseFloat();
      prefs.putFloat("t_scale", terrainScale);
      printPrefs();
    } else if (cmd == 's') {
      sensorScale =  Serial.parseFloat();
      prefs.putFloat("g_scale", sensorScale);
      printPrefs();
    } else if (cmd == 'm') {
      ballMass =  Serial.parseFloat();
      prefs.putFloat("b_mass", ballMass);
      printPrefs();
    }  else if (cmd == 'f') {
      terrainFriction =  Serial.parseFloat();
      prefs.putFloat("t_friction", terrainFriction);
      printPrefs();
    }  else if (cmd == 'g') {
      worldGravity =  Serial.parseFloat();
      prefs.putFloat("w_gravity", worldGravity);
      printPrefs();
    } else if (cmd == 'v') {
      ballInHoleVelocity =  Serial.parseFloat();
      prefs.putFloat("b_hole", ballInHoleVelocity);
      printPrefs();
    } else if (cmd == 'p') {
      float initialVelocity = Serial.parseFloat();
      ballVelocity = initialVelocity;
      playing = true;
    } else {
      printPrefs();
    }
  }

  if (!playing) {
    idle();
  //} else if (worldTime > timeInterval ) { // use game timer ?
  } else { // play simuation
    play();
  }
  draw();
}

