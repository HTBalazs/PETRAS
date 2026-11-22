#include <Arduino.h>
#include <FastAccelStepper.h>

FastAccelStepperEngine engine = FastAccelStepperEngine();

class Calibrator {
private:
  int stepPin = 9;
  int dirPin = 4;
  int enablePin = 2;
  FastAccelStepper* stepper = NULL;
  const int microStep = 16;
  uint32_t motionSpeed = 1000; // Átváltva 32 bitre (uint32_t)
  const int stepsPerRevolution = 200;
public:
  Calibrator() {}
  
  void initialize() {
    engine.init();
    stepper = engine.stepperConnectToPin(stepPin);
    if (stepper) {
      stepper->setDirectionPin(dirPin);
      stepper->setEnablePin(enablePin, true);
      stepper->setAutoEnable(true);
      
      // Explicit kasztolás (uint32_t) a szorzásnál a biztonság kedvéért
      stepper->setSpeedInHz((uint32_t)motionSpeed * microStep);
      stepper->setAcceleration(10000);
    }
  }
  
  void calibrate(int const roundsPerSecond, uint32_t const duration) {
    // Lebegőpontos számítás, majd pontos konverzió 32 bites egésszé
    motionSpeed = (uint32_t)(roundsPerSecond * stepsPerRevolution);
    
    if (stepper) {
      uint32_t speedInHz = motionSpeed * (uint32_t)microStep;
      stepper->setSpeedInHz(speedInHz);
      stepper->setAcceleration(10000);
      stepper->runForward();
    }
    
    uint32_t startTime = millis();
    while (millis() - startTime < duration * 1000) {
      yield();
      delay(10);
    }
    
    // A kalibráció végén állítsuk meg a motort!
    if (stepper) {
      stepper->stopMove();
    }
    
    Serial.println("finished");
  }
};

Calibrator calibrator;
const int buttonPin = 5;

void setup() {
  Serial.begin(115200);
  delay(500);
  
  // INPUT_PULLUP: nem lebeg a pin. 
  // HA NINCS NYOMVA = HIGH, HA MEG VAN NYOMVA = LOW
  pinMode(buttonPin, INPUT_PULLUP); 
  
  calibrator.initialize();
  Serial.println("Rendszer kesz, gombra var...");
}

void loop() {
  // Mivel INPUT_PULLUP-ot használunk, a lenyomás LOW (false) lesz
  bool buttonPressed = (digitalRead(buttonPin) == HIGH);
  
  if (buttonPressed) {
    Serial.println("Gomb megnyomva, kalibracio indul...");
    calibrator.calibrate(3, 15);
    
    // Kényszerített várakozás, amíg el nem engeded a gombot, 
    // nehogy azonnal újrainduljon a ciklus
    while(digitalRead(buttonPin) == LOW) {
      delay(10); 
    }
    Serial.println("Gomb elengedve.");
  }
  delay(50); // Kis szünet a loop-nak
}