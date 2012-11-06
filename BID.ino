#include <OneWire.h>
#include <PID_v1.h>
#include <LiquidCrystal.h>

byte logo[8] = {
  B00100,
  B01010,
  B01010,
  B10001,
  B10111,
  B11001,
  B10000,
  B00000,
};

int runMode = 0;

// OneWire DS18S20, DS18B20, DS1822 Temperature Example
//
// http://www.pjrc.com/teensy/td_libs_OneWire.html
//
// The DallasTemperature library can do all this work for you!
// http://milesburton.com/Dallas_Temperature_Control_Library

// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(11, A3, 10, 9, 8, 7, 6, 5, 4, 3, 2);
//AnalogPin
int LCDDimPin = 1;
OneWire  ds(12);  // on pin 12
//AnalogPin
int potpin = 2;
int kocherPin = 13;
double Setpoint, Output;
double celsius;
byte addr[8];

//double Kp = 10000;
//double Ki = 10;
//double Kd = 100;
//PID myPID(&celsius, &Output, &Setpoint,Kp,Ki,Kd,DIRECT);  

//25 W
PID myPID(&celsius, &Output, &Setpoint,10000,10,100,DIRECT);
//3000 W
//PID myPID(&celsius, &Output, &Setpoint,100,0.5,10,DIRECT);
//PID myPID(&celsius, &Output, &Setpoint,20,0.1,1,DIRECT);


int WindowSize = 1000;
unsigned long windowStartTime;

void setup(void) {
  pinMode(A0, INPUT);

  lcd.createChar(1, logo);
  
    // set up the LCD's number of columns and rows: 
  lcd.begin(20, 2);
  analogWrite(A1, 0);
  //lcd.blink();
  //lcd.setCursor(0,0);
  lcd.clear();
  lcd.write(1);
  lcd.print(" Harry Kims Helm ");
  lcd.write(1);
  delay(2000);

  
  int i = 0;
  windowStartTime = millis();
  Setpoint = 50;
  myPID.SetOutputLimits(0, WindowSize);
  myPID.SetMode(AUTOMATIC);
  
  Serial.begin(9600);
  pinMode(kocherPin, OUTPUT);
  

  // Initial search  
  //
  while (! addr[0]) {
    lcd.clear();
    lcd.print("Searching Sensor...");
    //lcd.blink();
    ds.reset();
    ds.write(0x33);
    lcd.setCursor(0,1);
    lcd.print("Sensor: ");
    for ( i = 0; i < 8; i++ ) {
      addr[i] = ds.read();
      lcd.print(addr[i], HEX);
      lcd.print(" ");
    }
    if ( OneWire::crc8(addr, 7) != addr[7]) {
      lcd.print("CRC of addr is not valid!");
    }
    delay(2000);
  }
  
  // Start first data
  ds.reset();
  ds.select(addr);
  ds.write(0x44);
  //ds.reset();
    
}

void loop(void) {
  byte i;
  byte present = 0;
  byte data[12];
  int potValue;
 // double power;
  int dataReady = 0;
  
  // Read Pot
  potValue = analogRead(potpin);
  
  // Check if we should switch Modes
  if (digitalRead(A0) == HIGH) {
    // Debounce
    delay(10);
    if (digitalRead(A0) == HIGH) {
      if (runMode >= 2) {
        runMode = 0;
      } else {
        runMode += 1;
      }
      // Switch Modes
      lcd.clear();
      lcd.print("Mode: ");

      lcd.write(1);
      lcd.print(runMode, DEC);
      //Reset Starttime
      windowStartTime = millis();
      
      if (runMode == 0) {
        // 25W
        lcd.print(" 25 W");
        lcd.setCursor(0,1);
        myPID.SetTunings(10000,10,100);
        printTuning();
      } else if (runMode == 1) {
        // 3000W
        lcd.print(" 3000 W");
        lcd.setCursor(0,1);
        myPID.SetTunings(100,0.5,10);
        printTuning();
      } else if (runMode == 2) {
        // Manual
        lcd.print(" Manual");
        lcd.setCursor(0,1);
      }
    
      delay(5000);
      //return;
    }   
  }

  
  



  dataReady = ds.read();
  if (dataReady) {
    ds.reset();
    ds.select(addr);
 //   Serial.println("Yes we have data");
    ds.write(0xBE);
    for ( i = 0; i < 9; i++ ) {
      data[i] = ds.read();
      //Serial.print(data[i], HEX);
      //Serial.print(" ");
    }
    //Serial.println(" got data");
    convertData(data);
    int power = map(Output, 0, WindowSize, 0, 100);
    
    //Refresh LCD with new data
    lcd.clear();
    lcd.print("In ");
    lcd.print(celsius);
    lcd.print((char)223);
    lcd.print("C");
    if (runMode != 2) {
    //  lcd.print(power);
    //  lcd.print("%");
    //} else {
      lcd.print(" Set ");
      lcd.print(int(Setpoint));
      lcd.print((char)223);  
      lcd.print("C");
    }
    //lcd.print(" Celsius");
    lcd.setCursor(0,1);
    lcd.write(1);
    //String foo = String(runMode, DEC);
    lcd.print(runMode, DEC);
    lcd.print(" OUT = ");
    //lcd.rightToLeft();
    //lcd.setCursor(12,1);
    lcd.print(power);
    lcd.print("% ");
    lcd.setCursor(13,1);
    lcd.print(int(Output));
    lcd.print("ms");
    
    //Send command to get next data
    ds.reset();
    ds.select(addr);
//    ds.write(0xCC);
//    // Start first data
    ds.write(0x44);
 // } else {
 //   Serial.println("No still waiting");
  }

  if (! celsius) {
    return;
  }

  if (runMode == 2) {
    // Manual Mode
  // Convert Poti to ms in WindowSize
    double setting = map(potValue, 0, 1023, 0, WindowSize);
    if (setting != Output) {
      Output = setting;
    }
  } else {
    // PID MODE
    // Convert Poti setting to degree celcius
    double setting = map(potValue, 0, 1023, 0, 100);
    if (setting != Setpoint) {
      Setpoint = setting;
    }
  }


  // Next time frame / Window
  if ((millis() - windowStartTime) > WindowSize) {
    windowStartTime += WindowSize;
    // We have new data it makes sense to calculate control
    if (runMode != 2) {
      myPID.Compute();
    }
  }
  
  if (int(Output) < ( millis() - windowStartTime )) {
    //Serial.println("Heating");
    digitalWrite(kocherPin, LOW);
  } else {
    //Serial.println("Cooling");
    digitalWrite(kocherPin, HIGH);
  }

  
}

double convertData(byte data[12]) {
  byte type_s = 0;
  // convert the data to actual temperature

  unsigned int raw = (data[1] << 8) | data[0];
  if (type_s) {
    raw = raw << 3; // 9 bit resolution default
    if (data[7] == 0x10) {
      // count remain gives full 12 bit resolution
      raw = (raw & 0xFFF0) + 12 - data[6];
    }
  } else {
    byte cfg = (data[4] & 0x60);
    if (cfg == 0x00) raw = raw << 3;  // 9 bit resolution, 93.75 ms
    else if (cfg == 0x20) raw = raw << 2; // 10 bit res, 187.5 ms
    else if (cfg == 0x40) raw = raw << 1; // 11 bit res, 375 ms
    // default is 12 bit resolution, 750 ms conversion time
  }
  //celsius
  celsius = (float)raw / 16.0;
}

void printTuning(void) {
  lcd.print("P");
  lcd.print(int(myPID.GetKp()));
  lcd.print(" I");
  lcd.print(myPID.GetKi());
  lcd.print(" D");
  lcd.print(myPID.GetKd());
}
