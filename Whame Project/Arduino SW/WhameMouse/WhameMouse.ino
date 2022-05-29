#include <SoftwareSerial.h>
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

SoftwareSerial modBluetooth(1, 2);  //SoftwareSerial mySerial (rxPin, txPin);
MPU6050 mpu;
char body;
int sensibility;
//#define APP 1;

//aux variables
float horizontal;
float vertical;
float clicks;
float prev_clicks;
boolean click = false;

#define INTERRUPT_PIN 23

#define spamtimer(t) for (static uint32_t SpamTimer; (uint32_t)(millis() - SpamTimer) >= (t); SpamTimer = millis())

// Timer variables
unsigned long primero = 0;
unsigned long segundo = 0;

void mouse_event(byte b, int movi) {
  modBluetooth.write(b);
  modBluetooth.write(movi);
  modBluetooth.write(0x2C);
}
void mouse_event_hid(int buttons, float x, float y, int wheel) {
  modBluetooth.write(0xFD);
  modBluetooth.write(0x05);
  modBluetooth.write(0x02);
  modBluetooth.write(buttons);//
  modBluetooth.write(x);//(x);
  modBluetooth.write(y);//(y);
  modBluetooth.write(wheel);//(wheel);
}

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

void serialReadings() {
  Serial.print("ypr\t");
  Serial.print(ypr[0] * 180 / M_PI);
  Serial.print("\t");
  Serial.print(ypr[1] * 180 / M_PI);
  Serial.print("\t");
  Serial.print(ypr[2] * 180 / M_PI);
  Serial.println();
}

void initIMU() {
  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // wait for ready
  /*Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    while (Serial.available() && Serial.read()); // empty buffer
    while (!Serial.available());                 // wait for data
    while (Serial.available() && Serial.read()); // empty buffer again*/

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.PrintActiveOffsets();
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
    Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
    Serial.println(F(")..."));
    //attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
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

void updateIMU() {
  // read a packet from FIFO
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet

    // display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    float prev_yaw;
    bool deslizar = false;

    float yaw = ypr[0] * 180 / M_PI;
    float pitch = ypr[1] * 180 / M_PI;
    float roll = ypr[2] * 180 / M_PI;
    if (abs(prev_yaw - yaw) < 2) {
      yaw = prev_yaw;
    }
    prev_yaw = yaw;
    Serial.print("Yaw:\t");
    Serial.print(yaw);
    Serial.print(" Pitch:\t");
    Serial.print(pitch);
    Serial.print(" Roll:\t");
    Serial.println(roll);

    //This is to write on the bluetooth module
    prev_clicks = clicks;


    //if it is wore in the hand
    if (body == 'M') {
      horizontal = -pitch;
      vertical = roll;
      clicks = yaw;

    } else {
      //if it is wore in the head
      horizontal = -roll;
      vertical = pitch;
      clicks = yaw;
    }

    if (horizontal > sensibility || horizontal < -sensibility) {
      //modBluetooth.write((String)"Se esta moviendo el mouse en direccion horizontal: "+yaw);
      //Serial.println((String)"Se esta moviendo el mouse en direccion horizontal: "+horizontal);
      if (deslizar) {
      } else {
#ifdef APP
        mouse_event((byte)0x78, horizontal);
#else
        mouse_event_hid(0, horizontal, 0, 0);
#endif
      }


    }
    if (vertical >  sensibility || vertical < -sensibility) {
      //modBluetooth.write((String)"Se esta moviendo el mouse en direccion horizontal: "+yaw);
      //Serial.println((String)"Se esta moviendo el mouse en direccion vertical: " + vertical);
      if (deslizar) {

      } else {
#ifdef APP
        mouse_event((byte)0x79, vertical);
#else
        mouse_event_hid(0, 0, vertical, 0);
#endif
      }

    }

    if (abs(clicks - prev_clicks) > 10) {
      //modBluetooth.write((String)"Se esta moviendo el mouse en direccion horizontal: "+yaw);
      if (click == false) {
        primero = millis();
        click = true;
#ifdef APP
        mouse_event((byte)0x7A, clicks);
#else
        mouse_event_hid(9, 0, 0, 0);
        mouse_event_hid(0, 0, 0, 0);
#endif
      }
      else{
        segundo=millis();
        if((segundo - primero)<2000){
          
        }else{
          click = false;
        }
      }
      Serial.println(clicks);
    }
  }
}


void setup() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  Serial.begin(115200);     // set the serial monitor to 115200 bauds
  modBluetooth.begin(115200);  //set the serial bluetooth rate to 115200
  Serial.println("Esperando algo para empezar");

#ifdef APP
  while (!modBluetooth.available());
  char character;
  int i = 0;
  while (modBluetooth.available() > 0) {
    if (i == 0) {
      body = modBluetooth.read();
      i = 1;
    } else {
      character = modBluetooth.read();
      data.concat(character);
    }
  }
  sensibility = data.toInt();
  Serial.println("Empezando a calibrar, no te muevas...");

  initIMU();
#else
  while (Serial.available() && Serial.read());    // read serial monitor
  while (!Serial.available());        // wait if empty
  while (Serial.available() && Serial.read());    // read serial monitor
  body = 'H';   //'M' o 'C'
  sensibility = 15;
  initIMU();
#endif

}

void loop() {
  // if programming failed, don't try to do anything
  if (!dmpReady) return;

  spamtimer(100) {
    updateIMU();
  }

  //serialReadings();

}
