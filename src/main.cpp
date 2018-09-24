#include <Arduino.h>
#include <math.h>
// #include "Wire.h"
#include <SmartLeds.h>
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>

const int MPU_INT_PIN = 19;
const int MPU_PWR_PIN = 23;
MPU6050 mpu;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

const int LED_COUNT = 144;
const int RING_COUNT = 9;
const int RING_PIN = 18;
const int DATA_PIN = 13;
const int CHANNEL = 0;
const int CLK_PIN = 14;
const int LED_PAUSE_PIN = 18;
// APA102 -> SPI driver
Apa102 leds(LED_COUNT, CLK_PIN, DATA_PIN, DoubleBuffer);
// SmartLed ring( LED_WS2812, RING_COUNT, RING_PIN, CHANNEL, DoubleBuffer );


// Flower Drawing Settings
const int FLOWER_PETALS = 8;
const int PETAL_OFFET = 30;

float flowerLastRoll = 0;
float flowerCurrentOffset = 0;

uint8_t hue_offset = 0;

uint8_t random_hues[LED_COUNT] {0};

const int MODE_DISPLAY_TIME = 1500;
const int MODE_SWITCH_TIME = 200;
int lastModeMillis = 0;
uint8_t ledMode = 0;
int modeShowTime = 0;

void writeRGB(Rgb color) {
    for(int i = 0; i < LED_COUNT; i++){
        leds[i] = color;
    }
    leds.show();
}

void writeHSV(Hsv color) {
    for(int i = 0; i < LED_COUNT; i++){
        leds[i] = color;
    }
    leds.show();
}

void writeRedRoll(uint8_t pos) {
    for(int i = 0; i < LED_COUNT; i++) {
        leds[i] = Rgb{map(pos, 0, 255, 0, 30), 10, 10};
    }
    leds.show();
}

void writeRainbow(uint8_t pos) {
    for(int i = 0; i < LED_COUNT; i++) {
        leds[i] = Hsv{(pos + i) % 255, 255, 30};
    }
    leds.show();
}

void writeRandomRainbow(uint8_t pos) {
    for(int i = 0; i < LED_COUNT; i++){
        leds[i] = Hsv{(pos + random_hues[i]) % 255, 255, 30};
    }
    leds.show();
}

void writeBand(uint8_t pos, uint8_t hue) {
    for(int i = 0; i < LED_COUNT; i++){
        if(i > pos - 5 && i < pos + 5) {
            leds[i] = Hsv{(hue) % 255, 255, constrain(30 - (abs(pos - i) * 6), 1, 30)};
        } else {
            leds[i] = Rgb{0,0,0};
        }
    }
    leds.show();
}

void writeRainbowAccelZ() {
    uint8_t pos = map(aa.y, 0, 6000, 0, 144) + 72 % 144;
    uint8_t hue = (millis() >> 4) % 255;
    for(int i = 0; i < LED_COUNT; i++){
        if(i > pos - 5 && i < pos + 5) {
            leds[i] = Hsv{(hue) % 255, 255, constrain(30 - (abs(pos - i) * 6), 1, 30)};
        } else {
            leds[i] = Rgb{0,0,0};
        }
    }
    leds.show();
}

void writeFlowerOffset(float *ypr) {
    if(ypr[2] > 1 && flowerLastRoll < 1) {
        if(flowerCurrentOffset < 1.5) {
            flowerCurrentOffset += 0.1;
        } else {
            flowerCurrentOffset == 0;
        }
    }

    uint8_t pos = (uint8_t)(sin(((ypr[2] + flowerCurrentOffset) * FLOWER_PETALS) + HALF_PI) * 64) + 72;
    uint8_t hue = (uint8_t)((ypr[1] + HALF_PI) * (255.0 / PI));
    for(int i = 0; i < LED_COUNT; i++){
        if(i > pos - 5 && i < pos + 5) {
            leds[i] = Hsv{(hue) % 255, 255, constrain(30 - (abs(pos - i) * 6), 1, 30)};
        } else {
            leds[i] = Rgb{0,0,0};
        }
    }
    leds.show();
    flowerLastRoll = ypr[2];
}

void writeFlower(float *ypr) {
    uint8_t pos = (uint8_t)(sin((ypr[2] * FLOWER_PETALS) + HALF_PI) * 64) + 72;
    uint8_t hue = (uint8_t)((ypr[1] + HALF_PI) * (255.0 / PI));
    for(int i = 0; i < LED_COUNT; i++){
        if(i > pos - 5 && i < pos + 5) {
            leds[i] = Hsv{(hue) % 255, 255, constrain(30 - (abs(pos - i) * 6), 1, 30)};
        } else {
            leds[i] = Rgb{0,0,0};
        }
    }
    leds.show();
}

void writeRandomBand(uint8_t pos, uint8_t hue) {
    uint8_t timeHue = (millis() >> 3) % 255;

    for(int i = 0; i < LED_COUNT; i++){
        if(i > pos - 5 && i < pos + 5) {
            leds[i] = Hsv{timeHue, 255, constrain(abs(30 -(millis() >> 4) % 60), 1, 30) };
        }
    }
    leds.show();
}

void peteToy() {
    uint8_t pos = constrain((abs(aa.z) >> 3)  % 144, 1, 144);
    uint8_t hue = aa.y % 255;

    for(int i = 0; i < LED_COUNT; i++){
        if(i > pos - 2 && i < pos + 2) {
            leds[i] = Hsv{(hue) % 255, 255, constrain(30 - (abs(pos - i) * 6), 1, 30)};
        } else {
            leds[i] = Rgb{0,0,0};
        }
    }
    leds.show();
}

void writeTimeSine(int millis) {
    uint8_t pos = constrain((sin((((millis >> 5) % 288) / 288.0) * TWO_PI) * 72) + 72 + abs(aa.x >> 6 & 15), 1, 143);
    uint8_t hue = (millis >> 5) % 255;
    for(int i = 0; i < LED_COUNT; i++){
        if(i > pos - 5 && i < pos + 5) {
            leds[i] = Hsv{(hue) % 255, 255, constrain(30 - (abs(pos - i) * 6), 1, 30)};
        } else {
            leds[i] = Rgb{0,0,0};
        }
    }
    leds.show();

}

void writeMode(uint8_t mode) {
    for(int i = 0; i < LED_COUNT; i++) {
        if(i < 2 * mode && i % 2 == 0) {
            leds[i] = Hsv{i, 255, 10};
        } else {
            leds[i] = Rgb{0,0,0};
        }
    }
    leds.show();
    modeShowTime = millis();
}

void clearStrip() {
    writeRGB(Rgb{0,0,0});
}

void connectMPU() {
    digitalWrite(MPU_PWR_PIN, LOW);
    delay(100);
    digitalWrite(MPU_PWR_PIN, HIGH);
    delay(100);
    // join I2C bus (I2Cdev library doesn't do this automatically)
    Wire.begin();
    Wire.setClock(400000);

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // load and configure the DMP
    Serial.println(F("Initializing DMP.."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setFullScaleAccelRange(16);
    mpu.setRate(5);
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(MPU_INT_PIN, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;
        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
}


void setup() {
    for(int i = 0; i < LED_COUNT; i++) {
        random_hues[i] = rand() % 255;
    }
    pinMode(MPU_PWR_PIN, OUTPUT);

    pinMode(LED_PAUSE_PIN, INPUT_PULLUP);
    pinMode(0, INPUT_PULLUP);

    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    Serial.begin(115200);
    connectMPU();


    clearStrip();
    ledMode = 9;
}

void loop() {
    int loopMillis = millis();
    // if({
    //     connectMPU();
    // }
    // if programming failed, don't try to do anything
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
        // other program behavior stuff here
        // .
        // .
        // .
        // if you are really paranoid you can frequently test in between other
        // stuff to see if mpuInterrupt is true, and if so, "break;" from the
        // while() loop to immediately process the MPU data
        // .
        // .
        // .
    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if ((mpuIntStatus & 0x02) > 0) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;
    }
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetAccel(&aa, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    // writeHSV(Hsv{map(aaReal.z, -10000, 10000, -256, 256) % 256, 255, 64});
    // Serial.println(aaReal.z);
    // Serial.printf("%f Yaw, %f Pitch, %f Roll\n", ypr[0], ypr[1], ypr[2]);
    // writeRGB(Rgb{(uint8_t)(constrain(ypr[0] * 32, 2, 100)), (uint8_t)(constrain(ypr[1] * 32, 2, 100)), (uint8_t)(constrain(ypr[2] * 32, 2, 100))});
    uint8_t roll8 = (uint8_t)((ypr[2] + 1.37) * (512.0 / 2.78)) + 128 + hue_offset;
    uint8_t led_pos = (uint8_t)((ypr[2] + 1.37) * (144.0 / 2.78));
    // if(digitalRead(LED_PAUSE_PIN) && loopMillis > modeShowTime + MODE_DISPLAY_TIME) {
    if(digitalRead(LED_PAUSE_PIN)) {
        switch (ledMode) {
            case 0:
                writeRandomRainbow(roll8);
                break;
            case 1:
                writeHSV(Hsv{roll8, 255, 30});
                break;
            case 2:
                writeRainbow(roll8);
                break;
            case 3:
                writeBand(led_pos, roll8);
                break;
            case 4:
                writeFlower(ypr);
                break;
            case 5:
                writeRandomBand(led_pos, roll8);
                break;
            case 6:
                writeFlowerOffset(ypr);
                break;
            case 7:
                writeRainbowAccelZ();
                break;
            case 8:
                peteToy();
                break;
            case 9:
                writeTimeSine(loopMillis);
                break;
            default: 
                ledMode = 0;
    
        }
        if(!digitalRead(0)) {
            hue_offset++;
        }
    } else {
        if(!digitalRead(0) && loopMillis > (lastModeMillis + MODE_SWITCH_TIME)) {
            lastModeMillis = loopMillis;
            ledMode++;
            writeMode(ledMode);
        }
        if(loopMillis > modeShowTime + MODE_DISPLAY_TIME) {
            writeRGB(Rgb{0,0,0});
        }
    }
    // Serial.println(ypr[2]);
    // Serial.println(roll8);
}