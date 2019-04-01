// This program has been modified from BegOscopio by rogerio.bego@hotmail.com
// BegOscopio tutorial: https://www.instructables.com/id/Oscilloscope-Arduino-Processing/
// BegOscopio source code: https://github.com/rogeriobego/oscilloscope-arduino-processing
#define versao "v1.5"
#include <TimerOne.h>

/**************************************************************************
   The Adafruit_GFX and ST7735 TFT LCD code is from Adafruit library
  Written by Limor Fried/Ladyada for Adafruit Industries.
  MIT license, all text above must be included in any redistribution
 **************************************************************************/

#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7735.h> // Hardware-specific library for ST7735
#include <Adafruit_ST7789.h> // Hardware-specific library for ST7789
#include <SPI.h>

// CS (Chip Select) Pin
#define TFT_CS        10
// RST (Reset) Pin
#define TFT_RST        9 // Or set to -1 and connect to Arduino RESET pin
// D/C (Data/Command) Pin
#define TFT_DC         8
// delay in ms to display set of readings before clearing screen
#define DELAY_MS     1000
// create global-scopetft object for the Adafruit ST7735 TFT LCD
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);

// configure ADC prescaler
const unsigned char PS_16 = (1 << ADPS2);
const unsigned char PS_32 = (1 << ADPS2) | (1 << ADPS0);
const unsigned char PS_64 = (1 << ADPS2) | (1 << ADPS1);
const unsigned char PS_128 = (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);

// buffer to store the channel 0 and 1 readings
// channel 0 is stored in: vb[0-19]
// channel 1 is stored in: vb[20-39]
int vb[40];

int chi[] = {0, 20}; // channel init position on buffer vb[]
int chq = 2; // how many channels are ON
int q = 20; // number of readings
int qmax = 20; // qtd max allowed for q
int vtrigger = 0; // trigger voltage
// default 2 ms: good enough for 25 samples per channel of 50 Hz signal
unsigned int dt = 2; // 100us a 1000us(1ms) a 3000ms(3s)
char unidade = 'm'; // unidade: m=milisegundo, u=microsegundo

boolean varias = false; // v = varias ('several' in english)
boolean uma = false;  // u = uma ('an' in english)
// speed limited by serial 115200
unsigned long dtReal, tIni, tFim;  // End time counter for flow

// default is to trigger on channel 1 (AC transformer voltage)
char canalTrigger = '1';

// LED will turn on when current clamp (ch0) and AC transformer (ch1)
// are out of phase, and solar power is going back into the grid
int out_of_phase_led = 2;

// This threshold is the number of successive times that detecthInPhase()
// returns in_phase=1 before changing the state out_of_phase_led.
// This is to prevent the out_of_phase_led pin from "flapping".
// Before implementing this threshold, it was possible for out_of_phase_led
// to change state every 2 periods which is too much.
// Note: Each period is 20 ms since for 50 Hz signal. detecthInPhase() samples
// ch0 and ch1 over 2 periods which is 40 ms.
// The current setting of 25 results in 1 seconds (40 ms * 25)
int steady_state_phase_threshold = 25;

// count the number of successive periods
// where the phase did not change
int steady_state_in_phase_count = 0;
int steady_state_out_phase_count = 0;

// has the ch0 and ch1 been out of phase for the duration
// of period times steady_state_phase_threshold (40 ms * 25 = 1 sec)
int steady_state_out_of_phase = 0;

void setup() {

  pinMode(out_of_phase_led, OUTPUT);

  // Use this initializer if using a 1.8" TFT screen:
  tft.initR(INITR_BLACKTAB);      // Init ST7735S chip, black tab
  tft.fillScreen(ST77XX_WHITE);
  tft.setCursor(0, 0);
  tft.setTextColor(ST77XX_BLACK);
  tft.setTextWrap(true);
  tft.setTextSize(1);

  // configure ADC prescaler since our waveform is only 50 Hz
  ADCSRA &= ~PS_128;  //limpa configuração da biblioteca do arduino
  ADCSRA |= PS_16; // 16 prescaler

  Serial.begin(115200);
  Serial.println();
  Serial.print(">init="); Serial.println(versao); // print version
}

void loop() {

  // in_phase is 1 when ch0 and ch1 are in phase
  // in_phase is 0 when ch0 and ch1 are out of phase
  int in_phase = 0;

  in_phase = detectInPhase();
  if (in_phase == 1) {
    steady_state_in_phase_count++;
    steady_state_out_phase_count = 0;
  } else {
    steady_state_out_phase_count++;
    steady_state_in_phase_count = 0;
  }

  Serial.print("in_phase: ");
  Serial.print(in_phase);
  Serial.print("\t steady_state_in_phase_count: ");
  Serial.print(steady_state_in_phase_count);
  Serial.print("\t steady_state_out_phase_count: ");
  Serial.print(steady_state_out_phase_count);
  Serial.print("\t out > th: ");
  Serial.print(steady_state_out_phase_count >  steady_state_phase_threshold);
  Serial.print("\t in > th: ");
  Serial.print(steady_state_in_phase_count >  steady_state_phase_threshold);

  // TODO: combine this logic since we can never have both
  // a steady state of in-phase and out-of-phase
  if ( steady_state_out_phase_count >  steady_state_phase_threshold ) {
    digitalWrite(out_of_phase_led, HIGH);
  }

  if (steady_state_in_phase_count >  steady_state_phase_threshold) {
    digitalWrite(out_of_phase_led, LOW);
  }

  Serial.println();

  // delay DELAY_MS milliseconds and then clear the LCD
  // before printing the next set of buffered readings
  delay(DELAY_MS);
  tft.fillScreen(ST77XX_WHITE);
  tft.setCursor(0, 0);
}

unsigned long microsOuMillis() {
  if (unidade == 'u') {
    return micros();
  } else {
    return millis();
  }
}

// - search for voltage greater than zero on the Trigger channel ----
// - if UMA = true then it waits indefinitely
// - if UMA = false then it waits until the time tFIM (q * dt)
boolean trigger () {// variable channelTrigger indicates which channel will trigger: 0,1,2 or 3
  unsigned long tFim; // End Time Counter
  int v1 = 0, v2 = 0;
  //int c1=0, c2=0;
  boolean achou = false; // achou means found
  tFim = microsOuMillis() + q * dt;
  // triggers the value of vtrigger + 10
  // read the voltage while it is larger than vtrigger
  // And time less than tFim
  do {
    v1 = analogRead(canalTrigger - '0');

  } while (v1 > vtrigger && microsOuMillis() < tFim);

  if (v1 <= vtrigger) {
    tFim = microsOuMillis() + q * dt;
    // read the voltage while it is less than or equal to 10 + vtrigger
    // And time less than tFim
    do {
      v2 = analogRead(canalTrigger - '0');
    } while (v2 <= 10 + vtrigger && microsOuMillis() < tFim);
    if (v2 > 10 + vtrigger) {
      achou = true;
    }
  }
  return achou;
}


int detectInPhase() { /* ler = read, enviar = send */

  unsigned long tFim; // End Time Counter
  unsigned long tTotalReal; // Total reading time of the values.
  if (canalTrigger >= '0' && canalTrigger <= '3') {
    Serial.print("trigger="); Serial.println(trigger());
  }
  tTotalReal = microsOuMillis();

  for (int k = 0; k < q; k++) {
    tFim = microsOuMillis() + dt;
    // read current clamp voltage
    vb[chi[0] + k] = analogRead(A0);
    // read AC transformer voltage
    vb[chi[1] + k] = analogRead(A1);
    while (microsOuMillis() < tFim) {}
  }

  tTotalReal = microsOuMillis() - tTotalReal; // total de tempo para ler todas as amostras
  dtReal = tTotalReal / q; // calcular o tempo médio de cada leitura
  Serial.println();
  Serial.print(">q="); Serial.println(q);
  Serial.print(">dt="); Serial.print(dt); Serial.print(unidade); Serial.println("s");
  Serial.print(">dtReal="); Serial.print(dtReal); //  Serial.print(unidade); Serial.println("s");
  if (unidade == 'm') {
    Serial.println("e-3");
  } else if (unidade == 'u') {
    Serial.println("e-6");
  }

  // in_phase is 1 when ch0 and ch1 are in phase
  // in_phase is 0 when ch0 and ch1 are out of phase
  int in_phase = 0;
  // the number of successive zero readings from ch1 and ch2
  // to consider the signals in phase. Current value of
  // 4 is based on 50Hz signal sampled every 2ms
  int zero_threshold = 3;
  // the count of successive zero readings from ch1 and ch2
  int zero_count = 0;

  tft.println(" #  Ch0  Ch1 Z T Phas");
  // q is the number of readings in the buffer
  for (int k = 0; k < q; k++) {
    // print the readings from the buffer to serial port

    Serial.print(">v=");
    Serial.print(k);
    Serial.print("\t");
    Serial.print(vb[chi[0] + k]);
    Serial.print("\t");
    Serial.print(vb[chi[1] + k]);
    Serial.print("\t");
    Serial.print(zero_count);
    Serial.print("\t");
    Serial.print(zero_threshold);
    Serial.print("\t");
    Serial.println(in_phase);

    if ( (vb[chi[0] + k] == 0) && (vb[chi[1] + k] == 0) ) {
      zero_count++;
    } else {
      // not successive zeros so reset the count
      zero_count = 0;
    }

    if (zero_count >= zero_threshold) {
      in_phase = 1;
    }

    // create buffer to store the text that will be printed on the LCD
    char buf[30];
    sprintf(buf, "%2d %4d %4d %1d %1d  %s", k, vb[chi[0] + k], vb[chi[1] + k], zero_count, zero_threshold, (in_phase) ? " in" : "out");
    tft.println(buf);
  }

  Serial.print(">tTotalReal="); Serial.print(tTotalReal); //Serial.print(unidade); Serial.println("s");
  if (unidade == 'm') {
    Serial.println("e-3");
  } else if (unidade == 'u') {
    Serial.println("e-6");
  }

  return in_phase;
}
