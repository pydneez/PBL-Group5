#include "Encoder.h"

// Single-channel sensors: 
// a pulse just means "the wheel moved one slot", with no direction info. 
// These ISRs only ever count magnitude; 

// direction (if needed later) has to come from what was commanded
// e.g. encodersSetDirection() in Motors.cpp.

static volatile unsigned long tickCountLF = 0;
static volatile unsigned long tickCountLR = 0;
static volatile unsigned long tickCountRF = 0;
static volatile unsigned long tickCountRR = 0;

static void isrLF() { tickCountLF++; }
static void isrLR() { tickCountLR++; }
static void isrRF() { tickCountRF++; }
static void isrRR() { tickCountRR++; }

static void attachEncoderIfWired(int pin, void (*isr)()) {
  if (pin == PIN_NOT_WIRED) return;
  pinMode(pin, INPUT);
  attachInterrupt(digitalPinToInterrupt(pin), isr, RISING);
}

void encoderInit() {
  attachEncoderIfWired(ENCODER_LF_PIN, isrLF);
  attachEncoderIfWired(ENCODER_LR_PIN, isrLR);
  attachEncoderIfWired(ENCODER_RF_PIN, isrRF);
  attachEncoderIfWired(ENCODER_RR_PIN, isrRR);
  Serial.println("Encoder Setup Complete");
}

// Ticks are written from an ISR, so reading/clearing them must happen with
// interrupts briefly disabled -- otherwise a tick landing mid-read could tear
// the multi-byte value or get lost between the read and a reset.
static unsigned long readAtomic(volatile unsigned long& counter) {
  noInterrupts();
  unsigned long value = counter;
  interrupts();
  return value;
}

static unsigned long takeAndResetAtomic(volatile unsigned long& counter) {
  noInterrupts();
  unsigned long value = counter;
  counter = 0;
  interrupts();
  return value;
}

static volatile unsigned long& counterFor(WheelId wheel) {
  switch (wheel) {
    case WheelId::LEFT_FRONT:  return tickCountLF;
    case WheelId::LEFT_REAR:   return tickCountLR;
    case WheelId::RIGHT_FRONT: return tickCountRF;
    default:                   return tickCountRR;
  }
}

unsigned long encoderGetTicks(WheelId wheel) {
  return readAtomic(counterFor(wheel));
}

unsigned long encoderGetAndResetTicks(WheelId wheel) {
  return takeAndResetAtomic(counterFor(wheel));
}

void encoderResetAll() {
  takeAndResetAtomic(tickCountLF);
  takeAndResetAtomic(tickCountLR);
  takeAndResetAtomic(tickCountRF);
  takeAndResetAtomic(tickCountRR);
}
