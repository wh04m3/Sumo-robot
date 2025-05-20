
#include <Bluepad32.h>

// PIN CONNECTIONS
int ledPin = 2;
int Ypin = 14; // motor 1 speed
int Bpin = 27; // motor 1 dir1
int Apin = 26; // motor 1 dir2
int Xpin = 25; // motor 2 dir1
int UpYostik = 33; // motor 2 dir2
int DownYostik = 32; // motor 2 speed
int LeftYostik = 12;
int RightYostik = 13;



HardwareSerial SerialArduino(2); // UART2



ControllerPtr myControllers[BP32_MAX_GAMEPADS];

// This callback gets called any time a new gamepad is connected.
// Up to 4 gamepads can be connected at the same time.

void onConnectedController(ControllerPtr ctl) {
  bool foundEmptySlot = false;
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == nullptr) {
      Serial.printf("CALLBACK: Controller is connected, index=%d\n", i);
      // Additionally, you can get certain gamepad properties like:
      // Model, VID, PID, BTAddr, flags, etc.
      ControllerProperties properties = ctl->getProperties();
      Serial.printf("Controller model: %s, VID=0x%04x, PID=0x%04x\n", ctl->getModelName().c_str(), properties.vendor_id, properties.product_id);
      myControllers[i] = ctl;
      foundEmptySlot = true;
      break;
      }
  }
  if (!foundEmptySlot) {
    Serial.println("CALLBACK: Controller connected, but could not found empty slot");
  }
}

void onDisconnectedController(ControllerPtr ctl) {
  bool foundController = false;

  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == ctl) {
      Serial.printf("CALLBACK: Controller disconnected from index=%d\n", i);
      myControllers[i] = nullptr;
      foundController = true;
      break;
    }
  }

  if (!foundController) {
    Serial.println("CALLBACK: Controller disconnected, but not found in myControllers");
  }
}

// == SEE CONTROLLER VALUES IN SERIAL MONITOR == //

void dumpGamepad(ControllerPtr ctl) {
  Serial.printf(
    "idx=%d, dpad: 0x%02x, buttons: 0x%04x, axis L: %4d, %4d, axis R: %4d, %4d, brake: %4d, throttle: %4d, "
    "misc: 0x%02x, gyro x:%6d y:%6d z:%6d, accel x:%6d y:%6d z:%6d\n",
    ctl->index(),        // Controller Index
    ctl->dpad(),         // D-pad
    ctl->buttons(),      // bitmask of pressed buttons
    ctl->axisX(),        // (-511 - 512) left X Axis
    ctl->axisY(),        // (-511 - 512) left Y axis
    ctl->axisRX(),       // (-511 - 512) right X axis
    ctl->axisRY(),       // (-511 - 512) right Y axis
    ctl->brake(),        // (0 - 1023): brake button
    ctl->throttle(),     // (0 - 1023): throttle (AKA gas) button
    ctl->miscButtons(),  // bitmask of pressed "misc" buttons
    ctl->gyroX(),        // Gyro X
    ctl->gyroY(),        // Gyro Y
    ctl->gyroZ(),        // Gyro Z
    ctl->accelX(),       // Accelerometer X
    ctl->accelY(),       // Accelerometer Y
    ctl->accelZ()        // Accelerometer Z
  );
}

// == GAME CONTROLLER ACTIONS SECTION == //

void processGamepad(ControllerPtr ctl) {
  // There are different ways to query whether a button is pressed.
  // By query each button individually:
  // a(), b(), x(), y(), l1(), etc...
 
  //== xbox A button = 0x0001 ==//
  if (ctl->buttons() == 0x0001) {
    //digitalWrite(ledPin, HIGH);
     digitalWrite(Apin, HIGH);

  }
  if (ctl->buttons() != 0x0001) {
    //digitalWrite(ledPin, LOW);
    digitalWrite(Apin, LOW);
  }


    //== xbox B button = 0x0001 ==//

    if (ctl->buttons() == 0x0002) {
    //digitalWrite(ledPin, HIGH);
    digitalWrite(Bpin, HIGH);
    // Enviar mensaje al otro Arduino
    
  

  }
  if (ctl->buttons() != 0x0002) {
   // digitalWrite(ledPin, LOW);
    digitalWrite(Bpin, LOW);
  }

      //== xbox X button = 0x0001 ==//

    if (ctl->buttons() == 0x0004) {
   // digitalWrite(ledPin, HIGH);
    digitalWrite(Xpin, HIGH);
    // Enviar mensaje al otro Arduino
    
  

  }
  if (ctl->buttons() != 0x0004) {
   // digitalWrite(ledPin, LOW);
    digitalWrite(Xpin, LOW);
  }

  //== xbox Y button = 0x0001 ==//

    if (ctl->buttons() == 0x0008) {
    //digitalWrite(ledPin, HIGH);
    digitalWrite(Ypin, HIGH);
    // Enviar mensaje al otro Arduino
    
  

  }
  if (ctl->buttons() != 0x0008) {
   // digitalWrite(ledPin, LOW);
    digitalWrite(Ypin, LOW);
  }




//**************************************************************

  //== xbox Abajo button = 0x0001 ==//
  if (ctl->dpad() == 0x02) {
    //digitalWrite(ledPin, HIGH);
     digitalWrite(DownYostik, HIGH);

  

  }
  if (ctl->dpad() != 0x02) {
    //digitalWrite(ledPin, LOW);
    digitalWrite(DownYostik, LOW);
  }


    //== xbox Arriba = 0x0001 ==//

    if (ctl->dpad() == 0x01) {
    //digitalWrite(ledPin, HIGH);
    digitalWrite(UpYostik, HIGH);
    // Enviar mensaje al otro Arduino
    
  

  }
  if (ctl->dpad() != 0x01) {
   // digitalWrite(ledPin, LOW);
    digitalWrite(UpYostik, LOW);
  }

      //== xbox left = 0x0001 ==//

    if (ctl->dpad() == 0x08) {
   // digitalWrite(ledPin, HIGH);
    digitalWrite(LeftYostik, HIGH);
    // Enviar mensaje al otro Arduino

 

    
  

  }
  if (ctl->dpad() != 0x08) {
   // digitalWrite(ledPin, LOW);
    digitalWrite(LeftYostik, LOW);
  }

  //== xbox rigth = 0x0001 ==//

    if (ctl->dpad() == 0x04) {
    //digitalWrite(ledPin, HIGH);
    digitalWrite(RightYostik, HIGH);
    // Enviar mensaje al otro Arduino
    
  

  }
  if (ctl->dpad() != 0x04) {
   // digitalWrite(ledPin, LOW);
    digitalWrite(RightYostik, LOW);
  }
  

  //== LEFT JOYSTICK - UP ==//
  // if (ctl->axisY() <= -25) {
  //   // map joystick values to motor speed
  //   //int motorSpeed = map(ctl->axisY(), -25, -508, 70, 255);
  //   // move motors/robot forward
  
  //   //analogWrite(ENApin, motorSpeed);
   
  //   //analogWrite(ENBpin, motorSpeed);
  //   digitalWrite(UpYostik, HIGH);

 

  // }

//   //== LEFT JOYSTICK - DOWN ==//
//   if (ctl->axisY() >= 25) {
//     // map joystick values to motor speed
//     //int motorSpeed = map(ctl->axisY(), 25, 512, 70, 255);
//     // move motors/robot in reverse
  
     
// digitalWrite(DownYostik, HIGH);

//   }

//   //== LEFT JOYSTICK - LEFT ==//
//   if (ctl->axisX() <= -25) {
//     // map joystick values to motor speed
//     //int motorSpeed = map(ctl->axisX(), -25, -508, 70, 255);
//     // turn robot left - move right motor forward, keep left motor still
//  digitalWrite(LeftYostik, HIGH);

 

  
//   }

//   //== LEFT JOYSTICK - RIGHT ==//
//   if (ctl->axisX() >= 25) {
//     // map joystick values to motor speed
//     //int motorSpeed = map(ctl->axisX(), 25, 512, 70, 255);
//     // turn robot right - move left motor forward, keep right motor still
//     digitalWrite(RightYostik, HIGH);
   
//   }

  // //== LEFT JOYSTICK DEADZONE ==//
  // if (ctl->axisY() > -25 && ctl->axisY() < 25 && ctl->axisX() > -25 && ctl->axisX() < 25) {
  //   // keep motors off
  //   // analogWrite(ENApin,0);
  //   // analogWrite(ENBpin, 0);

  //   digitalWrite(RightYostik, LOW);
  //   digitalWrite(LeftYostik, LOW);
  //   digitalWrite(DownYostik, LOW);
  //   digitalWrite(UpYostik, LOW);
  // }

  //== RIGHT JOYSTICK - X AXIS ==//
  if (ctl->axisRX()) {
    // map joystick values to servo position
   // int servoPos = map(ctl->axisRX(), -508, 512, 0, 180);
    // rotate xServo to the mapped position
    //xServo.write(servoPos);
  }

  //== RIGHT JOYSTICK - Y AXIS ==//
  if (ctl->axisRY()) {
    // map joystick values to servo position
    //int servoPos = map(ctl->axisRY(), -508, 512, 0, 180);
    // rotate yServo to the mapped position
   // yServo.write(servoPos);
  }
    dumpGamepad(ctl);
}

void processControllers() {
  for (auto myController : myControllers) {
    if (myController && myController->isConnected() && myController->hasData()) {
      if (myController->isGamepad()) {
        processGamepad(myController);
      }
      else {
        Serial.println("Unsupported controller");
      }
    }
  }
}

// Arduino setup function. Runs in CPU 1
void setup() {
  pinMode(ledPin, OUTPUT);
  pinMode(Ypin, OUTPUT);
  pinMode(Bpin, OUTPUT);
  pinMode(Apin, OUTPUT);
  pinMode(Xpin, OUTPUT);
  pinMode(UpYostik, OUTPUT);
  pinMode(DownYostik, OUTPUT);
  pinMode(LeftYostik, OUTPUT);
  pinMode(RightYostik, OUTPUT);



 // xServo.attach(xServoPin);
 // yServo.attach(yServoPin);

  Serial.begin(115200);
  Serial.printf("Firmware: %s\n", BP32.firmwareVersion());
  const uint8_t* addr = BP32.localBdAddress();
  Serial.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);



  // Setup the Bluepad32 callbacks
  BP32.setup(&onConnectedController, &onDisconnectedController);

  // "forgetBluetoothKeys()" should be called when the user performs
  // a "device factory reset", or similar.
  // Calling "forgetBluetoothKeys" in setup() just as an example.
  // Forgetting Bluetooth keys prevents "paired" gamepads to reconnect.
  // But it might also fix some connection / re-connection issues.
  BP32.forgetBluetoothKeys();

  // Enables mouse / touchpad support for gamepads that support them.
  // When enabled, controllers like DualSense and DualShock4 generate two connected devices:
  // - First one: the gamepad
  // - Second one, which is a "virtual device", is a mouse.
  // By default, it is disabled.
  BP32.enableVirtualDevice(false);
}

// Arduino loop function. Runs in CPU 1.
void loop() {
  // This call fetches all the controllers' data.
  // Call this function in your main loop.
  bool dataUpdated = BP32.update();
  if (dataUpdated)
    processControllers();

  // The main loop must have some kind of "yield to lower priority task" event.
  // Otherwise, the watchdog will get triggered.
  // If your main loop doesn't have one, just add a simple `vTaskDelay(1)`.
  // Detailed info here:
  // https://stackoverflow.com/questions/66278271/task-watchdog-got-triggered-the-tasks-did-not-reset-the-watchdog-in-time

  //     vTaskDelay(1);
  delay(150);
}