#include "I2Cdev.h"
#include "Wire.h"
#include "MPU6050_6Axis_MotionApps20.h"   // Obvezno vključiti za DMP, vsebuje firmware za inicializacijo MPU
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>
#include <EEPROM.h>
#include <Fonts/FreeMono9pt7b.h> // Pisava samo za menije

// SSD1306 OLED
const byte SCREEN_WIDTH = 128; // Širina OLED zaslona v slikovnih točkah
const byte SCREEN_HEIGHT = 64; // Višina OLED zaslona v slikovnih točkah
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// Definicije gumbov in menija
const byte MENU_BTN = 5;
const byte ENTER_BTN = 4;
unsigned long menuStartTime = 0;      // Za spremljanje časa začetka menija (avtomatski izhod)
byte menuItem = 0;     // Trenutno izbrani element menija
const int menuTimeout = 10000; // Časovna omejitev za avtomatski izhod iz menija
bool precisionMode = false; // Možnost prikaza ene decimalke namesto zaokroževanja

// Naslov MPU za I2C
byte devAddr = 0x68;
MPU6050 mpu(devAddr);

// MPU spremenljivke za nadzor/stanje (iz knjižnice JRowberg)
bool dmpReady = false;  // true, če je DMP inicializacija uspešna
byte mpuIntStatus;      // vsebuje dejanski statusni bajt prekinitve iz MPU
byte devStatus;         // vrne stanje po vsaki operaciji (0 = uspešno, !0 = napaka)
int packetSize;         // pričakovana velikost paketa DMP (običajno 42 bajtov)
int fifoCount;          // število bajtov v FIFO
byte fifoBuffer[64];    // FIFO pomnilnik za shranjevanje
Quaternion q;           // vsebnik kvaterniona [w, x, y, z]

// Prirejena verzija Adafruit BN0555 knjižnice za pretvorbo kvaterniona v Eulerjeve kote
VectorFloat QtoEulerAngle(Quaternion qt) {
  VectorFloat ret;
  double sqw = qt.w * qt.w;
  double sqx = qt.x * qt.x;
  double sqy = qt.y * qt.y;
  double sqz = qt.z * qt.z;

  ret.x = atan2(2.0 * (qt.x * qt.y + qt.z * qt.w), (sqx - sqy - sqz + sqw));
  ret.y = asin(2.0 * (qt.x * qt.z - qt.y * qt.w) / (sqx + sqy + sqz + sqw));  // Adafruit uporablja -2.0 * ...
  ret.z = atan2(2.0 * (qt.y * qt.z + qt.x * qt.w), (-sqx - sqy + sqz + sqw));

  // Pretvorba iz radianov v stopinje
  ret.x = ret.x * 180 / PI;
  ret.y = ret.y * 180 / PI;
  ret.z = ret.z * 180 / PI;
  return ret;
}

// Zapiši 2-bajtno vrednost v EEPROM, začne na podanem naslovu
void epromWriteWord(int startAddr, int value) {
  EEPROM.update(startAddr, value);
  EEPROM.update(startAddr + 1, value >> 8);
}

// Preberi 2-bajtno vrednost iz EEPROM z začetnega naslova
int epromReadWord(int startAddr) {
  int value = EEPROM.read(startAddr);
  value = value | (EEPROM.read(startAddr + 1) << 8);
  return value;
}

void getCalibration() {
  // Preberi shranjene kalibracijske vrednosti iz EEPROM in jih naloži v MPU
  // Naslovi vedno od 0 do 11, po 2 bajta za vsako os

  // V prihodnje preveri, ali obstajajo vrednosti (preveri, če so vsi bajti FF)
  // Če ni, predpostavi, da kalibracija še ni bila izvedena
  // Kasneje lahko uporabnika opozoriš, naj izvede kalibracijo!
  mpu.setXAccelOffset(epromReadWord(0));
  mpu.setYAccelOffset(epromReadWord(2));
  mpu.setZAccelOffset(epromReadWord(4));
  mpu.setXGyroOffset(epromReadWord(6));
  mpu.setYGyroOffset(epromReadWord(8));
  mpu.setZGyroOffset(epromReadWord(10)); // zadnji prebran naslov je 11 decimalno v EEPROM-u
}

void setCalibration() {
  // Zaženi samodejno DMP kalibracijo in shrani pridobljene vrednosti v EEPROM
  // To funkcijo kliči samo ob izbiri samodejne kalibracije
  // Za ohranjanje življenjske dobe EEPROM-a uporabi update namesto write

  // Izvedi samodejno kalibracijo 6-krat
  mpu.CalibrateAccel(6);
  mpu.CalibrateGyro(6);

  // Preberi končne vrednosti in jih shrani v EEPROM
  int Data[3];
  // Pospeškomer offseti
  I2Cdev::readWords(devAddr, 0x06, 3, (int *)Data);
  epromWriteWord(0, Data[0]);
  epromWriteWord(2, Data[1]);
  epromWriteWord(4, Data[2]);
  // Žiroskop offseti
  I2Cdev::readWords(devAddr, 0x13, 3, (int *)Data);
  epromWriteWord(6, Data[0]);
  epromWriteWord(8, Data[1]);
  epromWriteWord(10, Data[2]); // Zadnji zapisani bajt je naslov 11 decimalno v EEPROM-u
}

void setup() {
  // Iz primera JRowberg za nastavitev povezave MPU6050
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000); // 400 kHz I2C ura. Zakomentiraj to vrstico, če imaš težave pri prevajanju
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  Serial.begin(115200); // Potrebno samo za odpravljanje napak, sicer lahko zakomentiraš

  // Nastavi gumbe menija z notranjimi upori (pull-ups)
  pinMode(MENU_BTN, INPUT_PULLUP);
  pinMode(ENTER_BTN, INPUT_PULLUP);

  // Inicializacija OLED zaslona
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);

  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE); // Ni nujno za ta zaslon
  display.println();
  display.println(F("Starting!!"));
  display.display();

  // Inicializacija MPU6050
  mpu.initialize();
  devStatus = mpu.dmpInitialize();

  // Pridobi shranjene kalibracijske vrednosti iz EEPROM-a in jih nastavi v MPU
  // Če ni vrednosti, uporabi privzete vrednosti in prikaži potrebo po kalibraciji
  getCalibration();

  // Preveri, če je inicializacija uspešna (zaradi nalaganja firmware-a v DMP)
  // devStatus mora biti 0 za uspešno inicializacijo
  if (devStatus == 0) {
    mpu.setDMPEnabled(true);
    mpuIntStatus = mpu.getIntStatus();
    // Postavi zastavico, da lahko glavna zanka uporablja DMP
    dmpReady = true;

    // Pridobi pričakovano velikost DMP paketa za kasnejšo primerjavo
    packetSize = mpu.dmpGetFIFOPacketSize();

  } else {
    // Če inicializacija MPU-ja ni uspela, prikaži napako na OLED zaslonu
    display.clearDisplay();
    display.setCursor(0, 40);
    display.println("IMU FAIL");
    display.display();
  }
}

// Rutina za prikaz glavnega menija
void dispMenu(byte itemSelect) {
  display.clearDisplay();
  display.setRotation(0);
  display.setFont(&FreeMono9pt7b);
  display.setCursor(0, 14);
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.println(F("   Normalno"));
  display.println(F(" Precizno"));
  display.println(F(" Kalibracija"));

  if (itemSelect == 0) {
    display.drawRect(5, 2, 120, 17, SSD1306_WHITE); // Nariši okvir okoli prvega elementa
  }
  if (itemSelect == 1) {
    display.drawRect(5, 18, 120, 20, SSD1306_WHITE); // Nariši okvir okoli drugega elementa
  }
  if (itemSelect == 2) {
    display.drawRect(5, 36, 120, 20, SSD1306_WHITE); // Nariši okvir okoli tretjega elementa
  }

  // Prikaži vse
  display.display();
}

// Rutina za prikaz podmenija za kalibracijo
void dispCalibrate(byte itemSelect) {
  display.clearDisplay();
  display.setFont(&FreeMono9pt7b);
  display.setCursor(2, 14);
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.println(F("Postavi me"));
  display.println(F("  na hrbet"));
  display.setCursor(0, 55);
  display.println(F(" START   X"));

  if (itemSelect == 0) {
    display.drawRect(6, 41, 65, 20, SSD1306_WHITE); // Nariši okvir okoli START
  }
  else  {
    display.drawRect(95, 41, 20, 20, SSD1306_WHITE); // Nariši okvir okoli X
  }

  // Prikaži vse
  display.display();
}

// Upravljanje uporabniških interakcij z gumbi za glavne možnosti menija
void menuMainWait() {
  // Prikaži glavni meni na zaslonu
  menuItem = 0;
  dispMenu(menuItem);

  // Čakaj na pritisk gumba za izbiro elementa
  menuStartTime = millis(); // Ponastavi časovnik menija

  // Čakanje na uporabnikovo akcijo v meniju, če noben gumb ni pritisnjen
  // po preteku menuTimeout samodejno izstopi iz menija
  while (millis() - menuStartTime <= menuTimeout) {
    if (digitalRead(MENU_BTN) == LOW) {
      delay(200); // Debounce
      menuStartTime = millis(); // Ponastavi časovnik menija
      menuItem++;
      if (menuItem >= 3) {
        // Dosegli smo konec menija, vrni se na začetek
        menuItem = 0;
        dispMenu(menuItem);
      }
      else {
        // Nariši okvir okoli naslednjega elementa menija
        dispMenu(menuItem);
      }
    }
    if (digitalRead(ENTER_BTN) == LOW) {
      delay(200); // Debounce
      menuStartTime = millis(); // Ponastavi časovnik menija
      if (menuItem == 0) {
        // Normalni način, kot zaokrožen na celo število
        precisionMode = false;
        break;
      }
      if (menuItem == 1) {
        // Precizni način, kot zaokrožen na eno decimalno mesto
        precisionMode = true;
        break;
      }
      if (menuItem == 2) {
        // Prikaži podmeni za kalibracijo
        menuCalibrateWait();
        break;
      }
    }
  }
}

// Meni za samodejno kalibracijo, uporabi funkcijo samodejne kalibracije MPU
// in shrani vrednosti v EEPROM, ki se naložijo ob zagonu
// Kalibracijo je treba izvesti samo enkrat, vrednosti ostanejo
// točne za nadaljnjo uporabo.
// OPOMBA: Kalibracija mora biti izvedena na že znani ravni površini,
// ki je ravna spredaj-zadaj in levo-desno, npr. ravna miza z napravo obrnjeno navzgor

void menuCalibrateWait() {
  // Prikaži podmeni za kalibracijo
  menuItem = 1; // Privzeta izbira je IZSTOP
  dispCalibrate(menuItem);

  // Čakaj na naslednjo uporabnikovo akcijo, če noben gumb ni pritisnjen
  // po preteku menuTimeout samodejno izstopi iz menija
  menuStartTime = millis(); // Ponastavi časovnik menija
  while (millis() - menuStartTime <= menuTimeout) {
    if (digitalRead(MENU_BTN) == LOW) {
      delay(300); // Debounce
      if (menuItem == 1) {
        menuItem = 0;
        dispCalibrate(menuItem);
      }
      else {
        menuItem = 1;
        dispCalibrate(menuItem);
      }
    }
    if (digitalRead(ENTER_BTN) == LOW) {
      delay(200); // Debounce
      if (menuItem == 0) {
        // Začni kalibracijo, postopek traja nekaj sekund
        // Prikaži sporočilo uporabniku, da poteka kalibracija
        display.clearDisplay();
        display.setCursor(0, 40);
        display.println("KALIBRIRAM");
        display.display();

        setCalibration(); // Izvedi kalibracijo
        
        display.clearDisplay();
        display.setCursor(0, 40);
        display.println("KONČANO!");
        display.display();
        delay(500);  // Čas za prikaz sporočila o končani kalibraciji
        break;
      }
      if (menuItem == 1) {
        // Izbrana možnost X - Izstopi iz menija
        break;
      }
    }
  }
}

// Za OLED zaslon prikaži kot v pravilni orientaciji glede na
// položaj libele. Glede na izbran način preciznosti se spremeni
// prikaz decimalnih mest in velikost pisave, da se podatki pravilno prilegajo.
void formatDisplay(double angleVal) {

  display.clearDisplay();
  display.setCursor(0, 20);
  display.setFont();
  display.setRotation(0);

  // Nastavi velikost pisave in prikaži vrednost kota
  if (precisionMode == false) {
    display.setTextSize(3);
    display.setCursor(20, 20);
    display.println(round(angleVal));
  } else {
    display.setTextSize(2);
    display.setCursor(10, 30);
    display.println(angleVal, 1);
  }

  // Ponastavi barvo besedila
  display.setTextColor(WHITE, BLACK); 
  // Prikaži besedilo
  display.display();
}

// GLAVNA PROGRAMSKA ZANKA!!
void loop() {
  if (digitalRead(MENU_BTN) == LOW) {
    delay(500);
    menuMainWait();
  }
  // Če programiranje ni uspelo, ne izvajaj ničesar
  if (!dmpReady) return;

  // Pridobi vrednosti kvaterniona iz DMP medpomnilnika
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    mpu.dmpGetQuaternion(&q, fifoBuffer);

    // Izračunaj kote s pretvorbo iz kvaterniona v Eulerjeve kote
    VectorFloat ea = QtoEulerAngle(q);

    // SAMO ZA DEBUGGING – ZAKOMENTIRAJ, ČE NI POTREBNO
    /*  Serial.print("quat\t");
      Serial.print(ea.x);
      Serial.print("\t");
      Serial.print(ea.y);
      Serial.print("\t");
      Serial.print(ea.z);
      Serial.println("\t"); 
    */

    float angVal = ea.z;

    // Normaliziraj kot na območje 0-180
    if (angVal < 0) angVal += 360.0;
    if (angVal > 180) angVal = 360.0 - angVal;

    // Prikaži podatke na OLED zaslonu v pravilni obliki
    formatDisplay(angVal);
  }
}