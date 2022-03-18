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
unsigned long hysterisisCounter = 0;
bool lastBallVelocityDirection = false;
int lastPositionIndex = 0;
int sensorSamplingStartTime = 0;
float peak = 0.0;
bool sensorSampling = false;
unsigned long lastPlay = 0;
float sensorCeil = 500;

volatile bool start = false;

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

const uint8_t NONE[] = {0x0, 0x0, 0x0, 0x0};

const uint8_t PLAY[] = {B01110011, B00111000, B01011111, B01101110};
const uint8_t HIT[] = {0x76, 0x30, 0x78, 0x00};
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
elapsedMillis immobilityTimer;
int timeInterval = 2; // game timer

CRGB leds[NUM_LEDS];

// fade all leds to black
void fadeall() { for(int i = 0; i < NUM_LEDS; i++) { leds[i].nscale8(250); } }

void showWin() {
  Serial.println("win");
  for(int i = 0; i < NUM_LEDS; i++) {
    (millis()/20 + i) % 20 < 10 ?  leds[i] = CRGB::Yellow : leds[i] = CRGB::Black;
    FastLED.show(); 
    if (millis()%2000<1000) display.setSegments(HIT);
    else display.showNumberDec(hits,false);
  }
}

void showSucess() {
  // Serial.println("success");
  unsigned char hue = 0;
  display.setSegments(GOOD);
  for(int i = holes[holeCounter-1]; i < holes[holeCounter]; i++) {
    leds[i] = CHSV(hue++, 255, 255);
    FastLED.show(); 
    fadeall();
  }
}

void showLoose() {
    // Serial.println("loose");
    display.setSegments(LOST);
    for(int i = 0; i < NUM_LEDS; i++) {
      (millis()/20 + i) % 20 < 10 ?  leds[i] = CRGB::Red : leds[i] = CRGB::Black;
      FastLED.show();
      if (millis()%2000<1000) display.setSegments(NONE);
      else display.showNumberDec(hits,false);
  }
}

void showMissed() {
  // Serial.println("missed");
  display.setSegments(BAD);
  delay(1000);
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
  holeCounter = 0;
  holeLedIndex = holes[holeCounter];
  hits = 0;
  start = false;
  sensorSampling = false;
}

void replayHole() {
  immobilityTimer = 0;
  playing = false;
  ballPosition = 0;
  ballPositionIndex = 0;
  hysterisisCounter = 0;
  start = false;
  sensorSampling = false;
  if (hits == 7) { // restart game
      showLoose();
      resetGame();
  }
}

void nextHole() {
  holeCounter++;
  playing = false;
  ballPosition = 0;
  ballPositionIndex = 0;
  hysterisisCounter = 0;
  start = false;
  sensorSampling = false;
  if (holeCounter >= maxHoles) { // win !!
    showWin();
    resetGame();
  } else { // 
    if (hits == 7) { // restart game
      showLoose();
      resetGame();
    } else {
      holeLedIndex = holes[holeCounter] + random(-20,20);
      showSucess();
    }
  }
}

void draw() {
  for (int led = 0; led < NUM_LEDS; led++) { // green terrain
    leds[led] =  led % 10 == 0 ? CRGB(2,5,0) : CRGB(0,5,0);//CHSV(terrainHeights[led],255,terrainHeights[led]/10.0);
  }

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

  if (playing) { // show ball animation
    display.setSegments(ANIMATION[(ballPositionIndex) % 5]);
  } else { // print game info
      if (hits == 0) { // print "play golf"
        if (millis()%2000<1000) display.setSegments(PLAY);
        else display.setSegments(GOLF);
      } else { // print hits count
      int ticks = millis()%3000;
      if (ticks<1000) display.setSegments(PLAY);
      else if (ticks<1500) display.setSegments(NONE);
      else if (ticks<2000) display.setSegments(HIT);
      else display.showNumberDec(hits,false);
    }
  }
  FastLED.show();
}

void startSampling() {
  start = true;
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
  prefs.putUInt("g_hysterisis", hyterisisLimit);
  prefs.putUInt("g_sensorCeil", sensorCeil);
  #endif

  terrainScale = prefs.getFloat("t_scale", 200.0);
  sensorScale = prefs.getFloat("g_scale", 2.0);
  ballMass = prefs.getFloat("b_mass", 1.0);
  terrainFriction = prefs.getFloat("t_friction", 0.1);
  worldGravity = prefs.getFloat("w_gravity", 9.98);
  ballInHoleVelocity = prefs.getFloat("b_hole", 1.0);
  hyterisisLimit = prefs.getUInt("g_hysterisis", 50);
  sensorCeil = prefs.getFloat("g_sensorCeil", 500.0);

  pinMode(BUTTON, INPUT_PULLUP);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(PIEZO, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(PIEZO), startSampling, CHANGE);


  display.setBrightness(0x0a);
  holeLedIndex = holes[holeCounter] + random(-20,20);

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
  Serial.printf("world (g)ravity : %f \n", prefs.getFloat("w_gravity"));
    Serial.printf("sensor (c)eil : %f \n", prefs.getFloat("g_sensorCeil"));

  Serial.printf("(h)Hysterisis limit : %i \n----------\n", prefs.getUInt("g_hysterisis"));
}

void idle(){
  static bool lastState = true;

  if (millis() - lastPlay > 120000 ) { // 2 min reset.
    resetGame();
  }

  digitalWrite(LED_BUILTIN,LOW);

  //bool v = digitalRead(PIEZO);
  // Serial.println(v);
  // return;

  if (start==true && !sensorSampling) { // valid hit ceil
    sensorSamplingStartTime = millis();
    sensorSampling = true;
    Serial.println("Sampling");
    peak = 0;
  }

  while(sensorSampling) {
    bool v = digitalRead(PIEZO);
    if (millis()-sensorSamplingStartTime < 500) {
      if (v != lastState){
        peak  += 1 ;//max(peak,v);
        lastState = v;
      }
    } else {
      Serial.print("peak ");
      Serial.println(peak);
      sensorSampling = false;
      
      lastPlay = millis();
      ballPosition = 0;
      ballPositionIndex = 0;
      Serial.println("Sampling Done");
      
      if (peak < 2) {
        start = false;
        return;
      }

      ballVelocity = peak * sensorScale;
      playing = true;
      peak = 0;
      sensorSampling = false;
      Serial.printf("velocity = %f\n", ballVelocity);
      digitalWrite(LED_BUILTIN,HIGH);

      immobilityTimer = 0;
      hits ++;
      start = false;
    }
  }

  
  

   //float v = analogRead(PIEZO);///4095.0; // piezo reading
  //  Serial.println(v);
  //  return;
  // if (v > sensorCeil && !sensorSampling) { // valid hit ceil
  //   sensorSamplingStartTime = millis();
  //   sensorSampling = true;
  //   Serial.println("sensor value :");
  //   Serial.println(v);
  //   Serial.println("Sampling");
  //   peak = 0;
  // }
  // if (sensorSampling && millis()-sensorSamplingStartTime < 250) {
  //   peak  += v ;//max(peak,v);
  // } else if (sensorSampling) {
  //   lastPlay = millis();
  //   ballPosition = 0;
  //   ballPositionIndex = 0;
  //   Serial.println("Sampling Done");

  //   ballVelocity = peak * sensorScale;
  //   playing = true;
  //   peak = 0;
  //   sensorSampling = false;
  //   Serial.printf("velocity = %f\n", ballVelocity);
  //   digitalWrite(LED_BUILTIN,HIGH);

  //   immobilityTimer = 0;
  //   hits ++;
  // }
}

void play() {
  digitalWrite(LED_BUILTIN,millis()%1000>500);
  worldTime = 0;


  // Newton's 2nd law of motion
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
    if (hysterisisCounter == 0) Serial.println("hysterisis");
    ballVelocity = 0;
  }

  ballPosition += ballVelocity; // update ball position

  // position to led index with congruance
  ballPositionIndex = int(floor(ballPosition / terrainScale)) % NUM_LEDS;

  // immobility check
  if (lastPositionIndex != ballPositionIndex) immobilityTimer = 0; 
  lastPositionIndex = ballPositionIndex;

  if (immobilityTimer > 2000) {
    showMissed();
    replayHole();
    return;
  }

  // ball over hole velocity reduction
  if (abs(ballPositionIndex - holeLedIndex) < 5) {
    ballVelocity *= 0.8;
  }
  // ball in hole chek
  if (abs(ballPositionIndex - holeLedIndex) < 5 && abs(ballVelocity) < ballInHoleVelocity) {
    ballPositionIndex = holeLedIndex;
    Serial.println("HOLE");
    draw();
    nextHole();
    return;
  
  } else if (ballPositionIndex <= 0 && ballVelocity < 0.0f) { // bounce
    ballVelocity = -ballVelocity * 0.8;
    ballPosition = 0;
    ballPositionIndex = 0;
  }
}

void loop() {


  //  float v = analogRead(PIEZO); // piezo reading
  //  Serial.println(v);
  //  delay(10);
  //  return;
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
    } else if (cmd == 'c') {
      sensorCeil = Serial.parseFloat();
       prefs.putFloat("g_sensorCeil", sensorCeil);
    } else if (cmd == 'h') {
      hyterisisLimit = Serial.parseInt();
      prefs.putUInt("g_hysterisis", hyterisisLimit);
      printPrefs();
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

