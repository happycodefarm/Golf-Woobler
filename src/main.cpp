#include <Arduino.h>
#include <EEPROM.h>
#include "elapsedMillis/elapsedMillis.h"
#include <FastLED.h>
#include <Adafruit_MPU6050.h>
#include "Kalman.h"
#include <Preferences.h>
#include "TM1637Display.h"

//#define SETUP

#define LEDSTRIP_PIN     5
#define COLOR_ORDER GRB
#define CHIPSET     WS2811
#define BUTTON    0
#define BRIGHTNESS  200

Preferences prefs;

float terrainScale = 200.0f;
float gyroScale = 2.0f;
float ballMass = 1.0f; // Kg // not used
float terrainFriction = 0.1f; // Coefficient of Friction (mu)
float worldGravity = 9.98f;
float ballInHoleVelocity = 1.0f;
float ballVelocity = 0.0f; // m/s
float ballAcceleration = 0.0; // m/s^2

// Module connection pins (Digital Pins)
#define CLK 18
#define DIO 19
TM1637Display display(CLK, DIO);

const uint8_t PLAY[] = {B01110011, B00111000, B01011111, B01101110};

const uint8_t DONE[] = {
  SEG_B | SEG_C | SEG_D | SEG_E | SEG_G,           // d
  SEG_A | SEG_B | SEG_C | SEG_D | SEG_E | SEG_F,   // O
  SEG_C | SEG_E | SEG_G,                           // n
  SEG_A | SEG_D | SEG_E | SEG_F | SEG_G            // E
};

const uint8_t LOST[] = {
  SEG_F | SEG_E | SEG_D ,           // d
  SEG_A | SEG_B | SEG_C | SEG_D | SEG_E | SEG_F,                           // n
  SEG_A | SEG_F | SEG_G | SEG_C | SEG_D ,   // O
  SEG_F | SEG_G | SEG_E | SEG_D            // E
};

#include "slopes.h"
//float terrainSlopeAngles[NUM_LEDS] = {-13.3625237457332, -13.089286550061559, -12.80715175903265, -12.515777133005827, -12.195728318258773, -11.844498962996965, -11.480089206489538, -11.101936145416403, -10.687301174574372, -10.232740532562957, -9.758855709324678, -9.264768059283654, -8.723360682669068, -8.129943156693685, -7.4797922559029075, -6.767114896132398, -5.986150583889298, -5.129893930616163, -4.19154876016421, -3.162970566655815, -2.036637031911141, -0.7618592187438367, 0.6767288164533625, 2.2942325584195373, 4.103909411488871, 6.114647971905356, 8.381355688660165, 10.897796446482687, 13.6270159899791, 16.55023473027842, 19.477274368304506, 22.204483756520744, 24.475735348240562, 25.972939631644522, 26.449303403042848, 25.743167437158093, 23.903859552507924, 21.168353415389106, 17.832501439512725, 14.263031907447044, 10.785824185221713, 7.511382171663286, 4.537539295947113, 1.9571639472842435, -0.29487117451441236, -2.2877067147084063, -3.9942681704114307, -5.448204131239038, -6.760007812507979, -7.941941657169707, -8.97257298796444, -9.870376430521787, -10.681896393442798, -11.415592734235304, -12.079629049086407, -12.680805215113821};

float ballPosition = 0.0f;
int ballPositionIndex = 0;
int holeLedIndex = 0;

bool playing = false;

Kalman kalman;
elapsedMillis worldTime;
int timeInterval = 2;

CRGB leds[NUM_LEDS];
Adafruit_MPU6050 mpu;

void draw() {
  for (int led = 0; led < NUM_LEDS; led++) { // green terrain
    leds[led] =  led % 10 == 0 ? CRGB(0,5,5) : CRGB(0,5,0);//CHSV(terrainHeights[led],255,terrainHeights[led]/10.0);
  }
  leds[millis()/200 % NUM_LEDS] = CRGB::Blue;

  display.showNumberDec(millis()/200 % NUM_LEDS, true);

  leds[holeLedIndex] = CRGB::Red;
  leds[ballPositionIndex] = CRGB::White;

  FastLED.show();
}

void setup() {
  Serial.begin(115200);

  prefs.begin("vGolf", false); 
  #ifdef SETUP
  prefs.putFloat("t_scale", terrainScale);
  prefs.putFloat("g_scale", gyroScale);
  prefs.putFloat("b_mass", ballMass);
  prefs.putFloat("t_friction", terrainFriction);
  prefs.putFloat("w_gravity", worldGravity);
  prefs.putFloat("b_hole", ballInHoleVelocity);
  #endif

  terrainScale = prefs.getFloat("t_scale");
  gyroScale = prefs.getFloat("g_scale");
  ballMass = prefs.getFloat("b_mass");
  terrainFriction = prefs.getFloat("t_friction");
  worldGravity = prefs.getFloat("w_gravity");
  ballInHoleVelocity = prefs.getFloat("b_hole");

  pinMode(BUTTON, INPUT_PULLUP);
  pinMode(LED_BUILTIN, OUTPUT);

  display.setBrightness(0x0a);
  //display.showNumberDec(0000, true);


  display.setSegments(LOST);

  holeLedIndex = random(NUM_LEDS);

  FastLED.addLeds<CHIPSET, LEDSTRIP_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection( TypicalLEDStrip );
  FastLED.setBrightness( BRIGHTNESS );

  if (!mpu.begin()) {
    Serial.println("Sensor init failed");
    while (1)
      yield();
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
  mpu.setGyroRange(MPU6050_RANGE_2000_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
  mpu.setCycleRate(MPU6050_CYCLE_40_HZ);
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  kalman.setAngle(g.gyro.z);
  draw();
}

void printPrefs() {
  Serial.printf("Setting\n\n----------\n(t)errain scale : %f \n", prefs.getFloat("t_scale"));
  Serial.printf("gyro (s)cale : %f \n", prefs.getFloat("g_scale"));
  Serial.printf("ball (m)ass : %f \n", prefs.getFloat("b_mass"));
  Serial.printf("ball in hole (v)elocity : %f \n", prefs.getFloat("b_hole"));
  Serial.printf("terrain (f)riction : %f \n", prefs.getFloat("t_friction"));
  Serial.printf("world (g)ravity : %f \n----------\n", prefs.getFloat("w_gravity"));
}

void loop() {
  static unsigned long immobilityCounter = 0;
  static int lastPositionIndex = 0;
  static int gyroSamplingStartTime = 0;
  static float peak = 0.0;
  static bool gyroSampling = true;

  if(!digitalRead(BUTTON)){
    playing = false;
    ballPosition = 0;
    ballPositionIndex = 0;
  }

  if (Serial.available()) {
    uint8_t cmd = Serial.read();
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    if (cmd == 't') {
      terrainScale =  Serial.parseFloat();
      prefs.putFloat("t_scale", terrainScale);
      printPrefs();
    } else if (cmd == 's') {
      gyroScale =  Serial.parseFloat();
      prefs.putFloat("g_scale", gyroScale);
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
    } else {
      printPrefs();
    }
  }

  if (!playing) {
    digitalWrite(LED_BUILTIN,LOW);
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    //a.acceleration
    //float acc = g.gyro.y; //fmax(abs(a.gyro.z ) , 0); //fmax(a.acceleration.z,0);
    float v =  abs(g.gyro.y); //kalman.getAngle(fabs(g.gyro.y), 0.01, 20);
    if (v > 3.0f && !gyroSampling) {
      gyroSamplingStartTime = millis();
      gyroSampling = true;
      // peak = fmax(v,peak);
      // Serial.print(v);
    }
    if (gyroSampling && millis()-gyroSamplingStartTime < 200) {
      peak += v;
    } else if (gyroSampling) {
      ballPosition = 0;
      ballPositionIndex = 0;

      ballVelocity = peak * gyroScale;
      playing = true;
      peak = 0;
      gyroSampling = false;
      digitalWrite(LED_BUILTIN,HIGH);

    }
    //Serial.println(peak);

  } else if (worldTime > timeInterval ) {
    digitalWrite(LED_BUILTIN,millis()%1000>500);

    worldTime = 0;

    /*
     nReaction = (mass*gravity*sin(slops[positionIndex])) 
    nFriction = (mass*friction*gravity*cos(slops[positionIndex]))
  
    velocity += nReaction

    if (velocity>0) velocity += nFriction
    else if (velocity<0) velocity -= nFriction
    

    position += velocity
    position %= (leds.length*scale)
    positionIndex = floor(position/scale)
    */

    float nReaction = (ballMass*worldGravity*sin(terrainSlopeAngles[ballPositionIndex]*PI/180));
    float nFriction = (ballMass*terrainFriction*worldGravity*cos(terrainSlopeAngles[ballPositionIndex]*PI/180));

    // acceleration
    ballVelocity -= nReaction;
  
    // friction 
    if (ballVelocity > 0.0f) ballVelocity -= nFriction;
    else if (ballVelocity < 0.0f ) ballVelocity += nFriction;

    //  if (ballVelocity > 0.0f) ballVelocity -= CONSTANT_FRICTION;
    // else if (ballVelocity < 0.0f ) ballVelocity += CONSTANT_FRICTION;

    //ballVelocity *= CONSTANT_FRICTION;
    // ball position 
    ballPosition += ballVelocity;

    // pposition to led index with congruance
    ballPositionIndex = int(floor(ballPosition / terrainScale)) % NUM_LEDS;

    // hole deceleration
    if (ballPositionIndex==holeLedIndex) {
      ballVelocity *= 0.8;
    }

    // immobility check
    if (lastPositionIndex == ballPositionIndex) immobilityCounter++; 
    else immobilityCounter = 0;
    lastPositionIndex = ballPositionIndex;

    if (immobilityCounter > 200) {
      playing = false;
      ballPosition = 0;
      ballPositionIndex = 0;
      holeLedIndex = random(NUM_LEDS);
      return;
    }

    // ball in hole check
    if (ballPositionIndex == holeLedIndex && abs(ballVelocity) < ballInHoleVelocity) {
      Serial.println("HOLE");
      playing = false;
      ballPosition = 0;
      ballPositionIndex = 0;
      holeLedIndex = random(NUM_LEDS);
      return;
    } else if (ballPositionIndex >= NUM_LEDS-1 && ballVelocity > 0.0f) { // bounce
      ballVelocity = -ballVelocity;
      ballPosition = NUM_LEDS-1;
      ballPositionIndex = NUM_LEDS-1;
    } else if (ballPositionIndex <= 0 && ballVelocity < 0.0f) { // bounce
      ballVelocity = -ballVelocity;
      ballPosition = 0;
      ballPositionIndex = 0;
    }
  }
 
  draw();
}

