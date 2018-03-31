//MILLIVOLT METER USING LTC2400
//24bit ADC CHIP
//Software version 3.00
//4.096 volt precision reference ADR4540
// LCD RS pin to digital pin 8
// LCD Enable pin to digital pin 7
// LCD D4 pin to digital pin 6
// LCD D5 pin to digital pin 5
// LCD D6 pin to digital pin 4
// LCD D7 pin to digital pin 3
// LCD R/W pin connect to ground

//LTC2400 SCK to digital pin 13
//LTC2400 SDO to digital pin 12
//LTC2400 CS to digital pin 10

//Calibration button to digital pin 2

#include <SPI.h>
#include <LiquidCrystal.h>

LiquidCrystal lcd(8, 7, 6, 5, 4, 3);
const int LTC_CS = 10
const int CalButton = 2;
long adcRead;
int CalSetup = 0;
float cal = 0;
float volt;
float vRef = 4.096;
String v;
float trim = 0;
const int sample = 4;
int ct = 190;
int d = 0;
int dV = 6;
int dmV = 2;
int duV = 0;

void setup(void) {

  pinMode(LTC_CS, OUTPUT);
  digitalWrite(LTC_CS, HIGH);

  SPI.begin();
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE0);
  SPI.setClockDivider(SPI_CLOCK_DIV16);
  lcd.begin(16, 2);
  lcd.setCursor(0, 0);
  lcd.print("   SCULLCOM");
  lcd.setCursor(0, 1);
  lcd.print("Hobby Electronics");
  
  delay(3000);
  lcd.clear();
  lcd.setCursor(0, 0);
  cal = (EEPROMreadlong(address));
  lcd.print("EEPROM Cal Level");
  lcd.setCursor(0, 1);
  lcd.print(cal);
  
  delay(3000);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Millivolt Meter");

  for (int i = 0; i < 5; i++) {
    Spi_Read();
    delay(ct);
  }
}

void loop(void) {

  CalSetup = digitalRead(CalButton);
  if (CalSetup == HIGH) {
    Cal_Adjust();
  } else {
    lcd.setCursor(0, 1);

    int i;
    long sum = 0;
    for (i=0; i<(samples); i++) {
      adcRead = Spi_Read();
      delay(ct);
      sum += adcRead;
    }

    sum = sum / samples;
    sum = sum >> 4;
    sum = sum - (cal - trim);
    volt = sum;
    volt = volt * 10;
    volt = volt * v_ref / (16777216);

    if (volt < 0.001) {
      volt = volt * 1000000;
      v = "uV";
      d = duV;
    } else if {
      volt = volt * 1000;
      v = "mV";
      d = dmV;
    } else {
      v = "V";
      d = dV;
    }

    lcd.setCursor(0, 1);
    lcd.print(volt, d);
    lcd.print(" ");
    lcd.print(v);
    lcd.print(" ");
  }
}

long Spi_Read(void) {

	long result = 0;
  long b;
  digitalWrite(LTC_CS, LOW);
	delayMicroseconds(1);

	if (!(PINB & (1 << 4))) {

		b = SPI.transfer(0xFF);
		b &= 0x0F;
		result = b;
    result <<= 8;
		b = SPI.transfer(0xFF);
    result |= b;
    result = result << 8;
		b = SPI.transfer(0xFF);
    result |= b;
    result = result << 8;
		b = SPI.transfer(0xFF);
    result |= b;

    digitalWrite(LTC_CS, HIGH);

		return(result);
	}
}

long Cal_Adjust(void) {

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Calibration");
  lcd.setCursor(0, 1);
  lcd.print("Short input lead");
  delay(2000);

  int i;
  long sum = 0;
  for (i=0; i < (samples); i++) {
    adcRead = Spi_Read();
    delay(ct);
    sum += adcRead;
  }

  sum = sum / samples;
  sum = sum >> 4;
  cal = sum;

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Adjust Factor");
  lcd.setCursor(0, 1);
  lcd.print(cal);
  delay(3000);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Millivolt Meter");
  
  EEPROMwritelong(address, cal);
}

//Routine to write a 4 byte (32 bit) long
//to EEPROM at specified addresses
void EEPROMwritelong(int address, long value) {

  //four = least significant byte
  byte four = (value & 0xFF);
  byte three = ((value >> 8) & 0xFF);
  byte two = ((value >> 16) & 0xFF);
  //one = most significant byte
  byte one = ((value >> 24) & 0xFF);

  //write the four bytes into EEPROM
  EEPROM.write(address, four);
  EEPROM.write(address+1, three);
  EEPROM.write(address+2, two);
  EEPROM.write(address+3, one);
}

//Routine to read back 4 bytes
//and return with (32 bit) long as value
long EEPROMreadlong(long address) {

	//read the 4 bytes from EEPROM
  long four = EEPROM.read(address);
  long three = EEPROM.read(address+1);
  long two = EEPROM.read(address+2);
  long one = EEPROM.read(address+3);

  return (four)+(three << 8) + (two << 16) + (one << 24);
}
