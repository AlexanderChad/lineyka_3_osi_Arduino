#include <EEPROM.h>
float EEMEM Koef_LineikyX_addr;
float EEMEM Koef_LineikyY_addr;
float EEMEM Koef_LineikyZ_addr;
int32_t EEMEM KVX_addr;
int32_t EEMEM KVY_addr;
int32_t EEMEM KVZ_addr;
// NowPos / RealPos
float Koef_LineikyX = 0.05;
float Koef_LineikyY = 0.05;
float Koef_LineikyZ = 0.05;
int32_t KVX = 300;
int32_t KVY = 300;
int32_t KVZ = 300;

int32_t SetPoint[3] = {0, 0, 0};
int32_t UserPoints[5][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}};

uint16_t VvodUst = 0;

// Подключаем библиотеку программного UART
#include <SoftwareSerial.h>
// Определяем вывод RX
#define RX1 5
#define RX2 6
#define RX3 7
// Определяем вывод TX
#define TX 4
// Создаём объекты программного UART
SoftwareSerial Serial_1(RX1, TX);
SoftwareSerial Serial_2(RX2, TX);
SoftwareSerial Serial_3(RX3, TX);
#define Serial_Wait 100 //время на ответ 2313, мс

#define RTime_Display 300 //период обновления дисплея, мс
#define RTime_GetPos 100 //период обновления положения осей, мс
#define RTime_Dv 100 //период обновления данных для двигателя, мс
#define RL_DV_F_X 1 // 1 или 0, задает полярность управляющего сигнала
#define RL_DV_F_Y 1 // 1 или 0, задает полярность управляющего сигнала
#define RL_DV_F_Z 1 // 1 или 0, задает полярность управляющего сигнала
#define RL_DV_F_P 1 // 1 или 0, задает полярность управляющего сигнала

uint8_t ERR_GetPos = 0;

uint8_t Err_State = 0;

uint8_t ERR_KV = 0;
uint8_t Pcz_KV = 5; //допуск концевиков

uint8_t Pcz_X = 5; //допуск X
uint8_t Pcz_Y = 5; //допуск Y
uint8_t Pcz_Z = 5; //допуск Z

uint8_t Setting_Mode = 0;

uint8_t GoTo_Mode = 0;

uint8_t Run_Mode = 0;

uint8_t StateIO_X = 0b01100000;
uint8_t StateIO_Y = 0b10100000;
uint8_t StateIO_Z = 0b11100000;
#define X 0
#define Y 1
#define Z 2

#define SignalPin 3

const int P[] = {8, 9, A0, A1}; // пины строк
const int M[] = {A2, A3, A4, A5};   // пины столбцов
#define BT_Thr 1 //BT_Thr=time_ms/10, recommended BT_Thr=15
#define BT_Hold 50 //BT_Hold=time_ms/10, recommended BT_Hold=50

#define debug
#define debug_KV //концевики

//#include "LedControl.h"
/* Data is shifted out of this pin*/
int SPI_MOSI;
/* The clock is signaled on this pin */
int SPI_CLK;
/* This one is driven LOW for chip selectzion */
int SPI_CS;
//the opcodes for the MAX7221 and MAX7219
#define OP_NOOP   0
#define OP_DIGIT0 1
#define OP_DIGIT1 2
#define OP_DIGIT2 3
#define OP_DIGIT3 4
#define OP_DIGIT4 5
#define OP_DIGIT5 6
#define OP_DIGIT6 7
#define OP_DIGIT7 8
#define OP_DECODEMODE  9
#define OP_INTENSITY   10
#define OP_SCANLIMIT   11
#define OP_SHUTDOWN    12
#define OP_DISPLAYTEST 15

byte LED_stat[16];

/* The array for shifting the data to the devices */
uint8_t spidata[16];

void spiTransfer(int addr, volatile byte opcode, volatile uint8_t data) {
  //Create an array with the data to shift out
  int offset = addr * 2;
  for (int i = 0; i < 4; i++)
    spidata[i] = (byte)0;
  //put our device data into the array
  spidata[offset + 1] = opcode;
  spidata[offset] = data;
  //enable the line
  digitalWrite(SPI_CS, LOW);
  //Now shift out the data
  for (int i = 4; i > 0; i--)
    shiftOut(SPI_MOSI, SPI_CLK, MSBFIRST, spidata[i - 1]);
  //latch the data onto the display
  digitalWrite(SPI_CS, HIGH);
}
void setScanLimit(int addr, int limit) {
  if (limit >= 0 && limit < 8)
    spiTransfer(addr, OP_SCANLIMIT, limit);
}
void clearDisplay(int addr) {
  int offset = addr * 8;
  for (int i = 0; i < 8; i++) {
    LED_stat[offset + i] = 0;
    spiTransfer(addr, i + 1, LED_stat[offset + i]);
  }
}
void lc_shutdown(int addr, bool b) {
  if (b)
    spiTransfer(addr, OP_SHUTDOWN, 0);
  else
    spiTransfer(addr, OP_SHUTDOWN, 1);
}
void lc_setIntensity(int addr, int intensity) {
  if (intensity >= 0 && intensity < 16)
    spiTransfer(addr, OP_INTENSITY, intensity);
}
void LedControl_Init(int dataPin, int clkPin, int csPin) {
  SPI_MOSI = dataPin;
  SPI_CLK = clkPin;
  SPI_CS = csPin;
  pinMode(SPI_MOSI, OUTPUT);
  pinMode(SPI_CLK, OUTPUT);
  pinMode(SPI_CS, OUTPUT);
  digitalWrite(SPI_CS, HIGH);
  SPI_MOSI = dataPin;
  for (int i = 0; i < 16; i++)
    LED_stat[i] = 0x00;
  for (int i = 0; i < 2; i++) {
    spiTransfer(i, OP_DISPLAYTEST, 0);
    //scanlimit is set to max on startup
    setScanLimit(i, 7);
    //decode is done in source
    spiTransfer(i, OP_DECODEMODE, 0);
    clearDisplay(i);
    //we go into shutdown-mode on startup
    lc_shutdown(i, true);
  }
  lc_shutdown(0, false);
  lc_shutdown(1, false);
  lc_setIntensity(0, 8);
  lc_setIntensity(1, 8);
}

static const struct {
  char   ascii;
  char   segs;
} MAX7219_Font[] = {
  {'A', B01110111}, {'B', B01111111}, {'C', B01001110}, {'D', B01111110}, {'E', B01001111}, {'F', B01000111},
  {'G', B01011110}, {'H', B00110111}, {'I', B00110000}, {'J', B00111100}, {'L', B00001110}, {'N', B01110110},
  {'O', B01111110}, {'P', B01100111}, {'R', B00000101}, {'S', B01011011}, {'T', B00001111}, {'U', B00111110},
  {'Y', B00100111}, {'[', B01001110}, {']', B01111000}, {'_', B00001000}, {'a', B01110111}, {'b', B00011111},
  {'c', B00001101}, {'d', B00111101}, {'e', B01001111}, {'f', B01000111}, {'g', B01011110}, {'h', B00010111},
  {'i', B00010000}, {'j', B00111100}, {'l', B00001110}, {'n', B00010101}, {'o', B01111110}, {'p', B01100111},
  {'r', B00000101}, {'s', B01011011}, {'t', B00001111}, {'u', B00011100}, {'y', B00100111}, {'-', B00000001},
  {' ', B00000000}, {'0', B01111110}, {'1', B00110000}, {'2', B01101101}, {'3', B01111001}, {'4', B00110011},
  {'5', B01011011}, {'6', B01011111}, {'7', B01110000}, {'8', B01111111}, {'9', B01111011}, {'/0', B00000000},
};

int32_t NowPosX = 0;
int32_t NowPosY = 0;
int32_t NowPosZ = 0;

int32_t NullPosX = 0;
int32_t NullPosY = 0;
int32_t NullPosZ = 0;

void setup() {
#ifndef debug
  EEPROM.get((int)&Koef_LineikyX_addr, Koef_LineikyX);
  EEPROM.get((int)&Koef_LineikyY_addr, Koef_LineikyY);
  EEPROM.get((int)&Koef_LineikyZ_addr, Koef_LineikyZ);
  EEPROM.get((int)&KVX_addr, KVX);
  EEPROM.get((int)&KVY_addr, KVY);
  EEPROM.get((int)&KVZ_addr, KVZ);
#endif

  LedControl_Init(12, 10, 11);

  Serial_1.setTimeout(50);
  Serial_2.setTimeout(50);
  Serial_3.setTimeout(50);

  for (int i = 0; i <= 3; i++) {  // выставляем пины строк на выход, столбцов на вход
    pinMode(P[i], OUTPUT);
    pinMode(M[i], INPUT_PULLUP);
    digitalWrite(P[i], HIGH);
  }

#ifdef debug
  Serial.begin(9600);
  Serial.println("Lineika is begin");
#endif
}
void loop() {
  GetPosXYZ();
  Update_Display();
  Update_KB_4x4();
  Check_KV();
  Update_Dv();
  Err_Checker();
}
void Err_Checker() {
  Err_State = 0;
  if (ERR_GetPos) {
    tone(SignalPin, 1000, 500); //Если ошибка при получении координат хотя бы по одной из осей - подаем сигнал
    Err_State++;
  }
  if (ERR_KV) {
    tone(SignalPin, 350, 500); //Если вышла за пределы хотя бы одна из осей - подаем сигнал
    Err_State++;
  }
#ifdef debug
  if (Err_State) {
    Serial.println("Err_State!!!");
  }
#endif
}
void Update_Dv() {
  static uint32_t Dv_timer = 0;
  if (millis() - Dv_timer >= RTime_Dv) { //периодическое обновление данных для двигателя, если разрешено движение
    if (Err_State) {
      Run_Mode = 0;

      StateUpdate(X, 0, !RL_DV_F_X);
      StateUpdate(X, 1, !RL_DV_F_X);

      StateUpdate(Y, 0, !RL_DV_F_Y);
      StateUpdate(Y, 1, !RL_DV_F_Y);

      StateUpdate(Z, 0, !RL_DV_F_Z);
      StateUpdate(Z, 1, !RL_DV_F_Z);

      StateUpdate(Z, 3, !RL_DV_F_P);
      StateUpdate(Z, 4, !RL_DV_F_P);
    } else {
      if (Run_Mode) {
        uint8_t StateXYZ = 0;
        if (NowPosX < (SetPoint[0] - Pcz_X)) {
          StateUpdate(X, 0, RL_DV_F_X);
          StateUpdate(X, 1, !RL_DV_F_X);
        } else if (NowPosX > (SetPoint[0] + Pcz_X)) {
          StateUpdate(X, 0, !RL_DV_F_X);
          StateUpdate(X, 1, RL_DV_F_X);
        } else {
          StateUpdate(X, 0, !RL_DV_F_X);
          StateUpdate(X, 1, !RL_DV_F_X);
          StateXYZ++;
        }

        if (NowPosY < (SetPoint[1] - Pcz_Y)) {
          StateUpdate(Y, 0, RL_DV_F_Y);
          StateUpdate(Y, 1, !RL_DV_F_Y);
        } else if (NowPosY > (SetPoint[1] + Pcz_Y)) {
          StateUpdate(Y, 0, !RL_DV_F_Y);
          StateUpdate(Y, 1, RL_DV_F_Y);
        } else {
          StateUpdate(Y, 0, !RL_DV_F_Y);
          StateUpdate(Y, 1, !RL_DV_F_Y);
          StateXYZ++;
        }

        if (NowPosZ < (SetPoint[2] - Pcz_Z)) {
          StateUpdate(Z, 0, RL_DV_F_Z);
          StateUpdate(Z, 1, !RL_DV_F_Z);
        } else if (NowPosZ > (SetPoint[2] + Pcz_Z)) {
          StateUpdate(Z, 0, !RL_DV_F_Z);
          StateUpdate(Z, 1, RL_DV_F_Z);
        } else {
          StateUpdate(Z, 0, !RL_DV_F_Z);
          StateUpdate(Z, 1, !RL_DV_F_Z);
          StateXYZ++;
        }
        if (StateXYZ == 3) {
          Run_Mode = 0;
        }
      }
    }
    Dv_timer = millis();
  }
}
void Check_KV() {
  ERR_KV = 0;
  if (((KVX + Pcz_KV <= NowPosX)) | ((KVY + Pcz_KV) <= NowPosY) | ((KVZ + Pcz_KV) <= NowPosZ)) {
    ERR_KV = 0xFF;
#ifdef debug_KV
    Serial.print("ERR_KV");
    if ((KVX + Pcz_KV) <= NowPosX ) {
      Serial.print(", ERR_X");
    }
    if ((KVY + Pcz_KV) <= NowPosY) {
      Serial.print(", ERR_Y");
    }
    if ((KVZ + Pcz_KV) <= NowPosZ) {
      Serial.print(", ERR_Z");
    }
    Serial.println(".");
#endif
  }
}
void GetPosXYZ() {
  ERR_GetPos = 0;
  static uint32_t GetPos_timer = 0;
  if (millis() - GetPos_timer >= RTime_GetPos) { //периодическое обновление положения осей
    int32_t Pos;
    unsigned char int32_buffer[4];
    unsigned char *pos_b = (unsigned char *)&Pos;
    uint8_t i_p;
    uint32_t TTX;

    Serial_1.begin(9600);
    Serial_1.write(0b01000000);
    i_p = 0;
    TTX = millis();
    while (i_p < 4) {
      if (millis() - TTX > Serial_Wait) {
        i_p = 5;
      } else {
        if (Serial_1.available()) {
          pos_b[i_p] = Serial_1.read();
          i_p++;
        }
      }
    }
    Serial_1.end();
    if (i_p == 4) {
      NowPosX = Pos - NullPosX;
    } else {
      ERR_GetPos++;
#ifdef debug
      Serial.println("X: ERR.RX");
#endif
    }
#ifdef debug
    Serial.print("X: ");
    Serial.println(Pos);
#endif

    Serial_2.begin(9600);
    Serial_2.write(0b10000000);
    i_p = 0;
    TTX = millis();
    while (i_p < 4) {
      if (millis() - TTX > Serial_Wait) {
        i_p = 5;
      } else {
        if (Serial_2.available()) {
          pos_b[i_p] = Serial_2.read();
          i_p++;
        }
      }
    }
    Serial_2.end();
    if (i_p == 4) {
      NowPosY = Pos - NullPosY;
    } else {
      ERR_GetPos++;
#ifdef debug
      Serial.println("Y: ERR.RX");
#endif
    }
#ifdef debug
    Serial.print("Y: ");
    Serial.println(Pos);
#endif

    Serial_3.begin(9600);
    Serial_3.write(0b11000000);
    i_p = 0;
    TTX = millis();
    while (i_p < 4) {
      if (millis() - TTX > Serial_Wait) {
        i_p = 5;
      } else {
        if (Serial_3.available()) {
          pos_b[i_p] = Serial_3.read();
          i_p++;
        }
      }
    }
    Serial_3.end();
    if (i_p == 4) {
      NowPosZ = Pos - NullPosZ;
    } else {
      ERR_GetPos++;
#ifdef debug
      Serial.println("Z: ERR.RX");
#endif
    }
#ifdef debug
    Serial.print("Z: ");
    Serial.println(Pos);
#endif
    GetPos_timer = millis();
  }
}
void StateUpdate(uint8_t XYZ_Number, uint8_t PIN, uint8_t val) {
  switch (XYZ_Number) {
    case 0:
      if (val) {
        StateIO_X |= (1 << PIN);
      } else {
        StateIO_X &= ~(1 << PIN);
      }
      Serial_1.begin(9600);
      Serial_1.write(StateIO_X);
      //Serial_1.write(StateIO_X);
      delay(1);
      Serial_1.end();
      break;
    case 1:
      if (val) {
        StateIO_Y |= (1 << PIN);
      } else {
        StateIO_Y &= ~(1 << PIN);
      }
      Serial_2.begin(9600);
      Serial_2.write(StateIO_Y);
      //Serial_2.write(StateIO_Y);
      delay(1);
      Serial_2.end();
      break;
    case 2:
      if (val) {
        StateIO_Z |= (1 << PIN);
      } else {
        StateIO_Z &= ~(1 << PIN);
      }
      Serial_3.begin(9600);
      Serial_3.write(StateIO_Z);
      //Serial_3.write(StateIO_Z);
      delay(1);
      Serial_3.end();
      break;
  }
}
void Display_XYZ(int32_t X_p, int32_t Y_p, int32_t Z_p) {
  char StrDisplay[5];

  sprintf(StrDisplay, "%4d", (int)X_p);
  DisplayText(0, StrDisplay);
  sprintf(StrDisplay, "%4d", (int)Y_p);
  DisplayText(1, StrDisplay);
  sprintf(StrDisplay, "%4d", (int)Z_p);
  DisplayText(2, StrDisplay);
}

void Update_Display() {
  static uint32_t Disp_timer = 0;
  if (millis() - Disp_timer >= RTime_Display) { //периодическое обновление данных на дисплее
    if (Setting_Mode) {
      switch (GoTo_Mode) {
        case 1:
          Display_XYZ(VvodUst, round((float)SetPoint[1]*Koef_LineikyY), round((float)SetPoint[2]*Koef_LineikyZ));
          break;
        case 2:
          Display_XYZ(round((float)SetPoint[0]*Koef_LineikyX), VvodUst, round((float)SetPoint[2]*Koef_LineikyZ));
          break;
        case 3:
          Display_XYZ(round((float)SetPoint[0]*Koef_LineikyX), round((float)SetPoint[1]*Koef_LineikyY), VvodUst);
          break;
      }
    } else {
      Display_XYZ(round((float)NowPosX * Koef_LineikyX), round((float)NowPosY * Koef_LineikyY), round((float)NowPosZ * Koef_LineikyZ));
    }
    Disp_timer = millis();
  }
}

void DisplayText(uint8_t display_num, char *text) {
  int decimal[8];
  char trimStr[8] = "";
  int x, y = 0;
  int s;

  s = strlen(text);
  if (s > 8) s = 8;
  for (x = 0; x < s; x++) {
    if (text[x] == '.') {
      decimal[y - 1] = 1;
    }
    else {
      trimStr[y] = text[x];
      decimal[y] = 0;
      y++;
    }
  }
  if (y > 4) y = 4;
  for (x = 0; x < y; x++) {
    DisplayChar((int)(x), trimStr[x], decimal[x], display_num);
  }
}
void DisplayChar(int digit, char value, bool dp, uint8_t display_num) {
  int offset;
  uint8_t v = MAX7219_LookupCode(value);
  if (dp) v |= B10000000;
  uint8_t adr;
  switch (display_num) {
    case 0:
      adr = 0;
      break;
    case 1:
      adr = 0;
      digit += 4;
      break;
    case 2:
      adr = 1;
      break;
  }
  offset = adr * 8;
  LED_stat[offset + digit] = v;
  spiTransfer(adr, digit + 1, v);
}
uint8_t MAX7219_LookupCode (char character)
{
  int i;
  unsigned int d = 0;
  if (character >= 35 && character <= 44) {
    character += 13;
    d = 1;
  }
  for (i = 0; MAX7219_Font[i].ascii; i++)
    if (character == MAX7219_Font[i].ascii) {
      if (d) {
        d = MAX7219_Font[i].segs;
        d |= (1 << 7);
        return (d);
      }
      else {
        return MAX7219_Font[i].segs;
      }
    }
  return 0;
}

void Update_KB_4x4() {
  static uint16_t BT_St [4][4] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  static uint8_t  BT_Trig [4][4] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  static uint32_t KB_timer = 0;
  if (millis() - KB_timer >= 10) {
    for (byte p = 0; p <= 3; p++) {    // последовательно выставляем по одной строке в LOW
      digitalWrite(P[p], LOW);
      for (byte m = 0; m <= 3; m++) {  // и считываем столбцы вылавнивая где LOW происходит
        if (digitalRead(M[m])) {
          if ((BT_St[p][m] > BT_Thr) && (BT_St[p][m] < BT_Hold)) {
            switch (p) { //краткое нажатие
              case 0:
                switch (m) {
                  case 0:
                    if (Setting_Mode) {
                      VvodUst = VvodUst * 10 + 1;
                    }
                    //StateUpdate(X, 0, 0);
                    //StateUpdate(X, 1, 0);
                    break;
                  case 1:
                    if (Setting_Mode) {
                      VvodUst = VvodUst * 10 + 2;
                    }
                    //StateUpdate(X, 0, 0);
                    //StateUpdate(X, 1, 1);
                    break;
                  case 2:
                    if (Setting_Mode) {
                      VvodUst = VvodUst * 10 + 3;
                    }
                    //StateUpdate(X, 0, 1);
                    //StateUpdate(X, 1, 0);
                    break;
                  case 3:


                    break;
                }
                break;
              case 1:
                switch (m) {
                  case 0:
                    if (Setting_Mode) {
                      VvodUst = VvodUst * 10 + 4;
                    }
                    //StateUpdate(Y, 0, 0);
                    //StateUpdate(Y, 1, 0);
                    break;
                  case 1:
                    if (Setting_Mode) {
                      VvodUst = VvodUst * 10 + 5;
                    }
                    //StateUpdate(Y, 0, 0);
                    //StateUpdate(Y, 1, 1);
                    break;
                  case 2:
                    if (Setting_Mode) {
                      VvodUst = VvodUst * 10 + 6;
                    }
                    //StateUpdate(Y, 0, 1);
                    //StateUpdate(Y, 1, 0);
                    break;
                  case 3:
                    if (GoTo_Mode) {
                      SetPoint[GoTo_Mode - 1] = VvodUst / Koef_LineikyX;
                      if (GoTo_Mode < 3) {
                        GoTo_Mode++;
                      } else {
                        GoTo_Mode = 1;
                      }
                      VvodUst = SetPoint[GoTo_Mode - 1] * Koef_LineikyX;
                    } else {
                      SetPoint[0] = KVX - Pcz_KV;
                      SetPoint[1] = KVY - Pcz_KV;
                      SetPoint[2] = KVZ - Pcz_KV;
                      Run_Mode = 1;
                    }
                    break;
                }
                break;
              case 2:
                switch (m) {
                  case 0:
                    if (Setting_Mode) {
                      VvodUst = VvodUst * 10 + 7;
                    }
                    //StateUpdate(Z, 0, 0);
                    //StateUpdate(Z, 1, 0);
                    break;
                  case 1:
                    if (Setting_Mode) {
                      VvodUst = VvodUst * 10 + 8;
                    }
                    //StateUpdate(Z, 0, 0);
                    //StateUpdate(Z, 1, 1);
                    break;
                  case 2:
                    if (Setting_Mode) {
                      VvodUst = VvodUst * 10 + 9;
                    }
                    //StateUpdate(Z, 0, 1);
                    //StateUpdate(Z, 1, 0);
                    break;
                  case 3:

                    break;
                }
                break;
              case 3:
                switch (m) {
                  case 0:

                    break;
                  case 1:
                    if (Setting_Mode) {
                      VvodUst = VvodUst * 10;
                    }
                    break;
                  case 2:

                    break;
                  case 3:

                    break;
                }
                break;
            }
          }
          BT_St[p][m] = 0;
          BT_Trig[p][m] = 0;
        } else {
          BT_St[p][m]++;
          if ((BT_St[p][m] > BT_Hold) && (!BT_Trig[p][m])) {
            BT_Trig[p][m] = 1;
            switch (p) {//длительное нажатие
              case 0:
                switch (m) {
                  case 0:

                    break;
                  case 1:

                    break;
                  case 2:

                    break;
                  case 3:

                    break;
                }
                break;
              case 1:
                switch (m) {
                  case 0:

                    break;
                  case 1:

                    break;
                  case 2:

                    break;
                  case 3:
                    if (GoTo_Mode) {
                      switch (GoTo_Mode) {
                        case 1:
                          SetPoint[0] = VvodUst / Koef_LineikyX;
                          break;
                        case 2:
                          SetPoint[1] = VvodUst / Koef_LineikyY;
                          break;
                        case 3:
                          SetPoint[2] = VvodUst / Koef_LineikyZ;
                          break;
                      }
                      GoTo_Mode = 0;
                      Setting_Mode = 0;
#ifdef debug
                      Serial.println("GoTo_Mode OFF");
#endif
                      Run_Mode = 1;
                    } else {
                      GoTo_Mode = 1;
                      Setting_Mode = 1;
#ifdef debug
                      Serial.println("GoTo_Mode ON");
#endif
                    }
                    break;
                }
                break;
              case 2:
                switch (m) {
                  case 0:

                    break;
                  case 1:

                    break;
                  case 2:

                    break;
                  case 3:

                    break;
                }
                break;
              case 3:
                switch (m) {
                  case 0:

                    break;
                  case 1:
                    NullPosX = NowPosX;
                    NullPosY = NowPosY;
                    NullPosZ = NowPosZ;
                    break;
                  case 2:
                    KVX = NowPosX;
                    KVY = NowPosY;
                    KVZ = NowPosZ;
                    EEPROM.put((int)&KVX_addr, KVX);
                    EEPROM.put((int)&KVY_addr, KVY);
                    EEPROM.put((int)&KVZ_addr, KVZ);
#ifdef debug_KV
                    Serial.println("KV SAVED");
#endif
                    break;
                  case 3:

                    break;
                }
                break;
            }
          }
        }
      }
      digitalWrite(P[p], HIGH);       // возвращем строку в HIGH и крутим дальше
    }
    if (VvodUst > 9999) {
      VvodUst = 0;
    }
    KB_timer = millis();
  }
}
