#include "Globals.h"

// ============================================================
// Fault 锁存状态
// ============================================================

bool currentFaultA = false;
bool currentFaultB = false;
bool currentVonFault = false;

bool latchFaultA = false;
bool latchFaultB = false;
bool latchVonFault = false;

uint64_t A_fault_time_ms = 0;
uint64_t B_fault_time_ms = 0;
uint64_t Von_fault_time_ms = 0;

float Von_fault_value = 0.0f;
bool systemProtected = false;

float readVonVoltage() {
  uint16_t adcRaw = analogRead(PIN_VON_ADC);
  float adcVoltage = ((float)adcRaw / ADC_MAX_VALUE) * ADC_REF_VOLTAGE;
  return adcVoltage * VON_SCALE;
}


bool hasAnyFaultLatch() {
  return latchFaultA || latchFaultB || latchVonFault;
}


bool hasAnyCurrentFault() {
  return currentFaultA || currentFaultB || currentVonFault;
}


String getFaultBriefText() {
  if (!hasAnyFaultLatch() && !hasAnyCurrentFault()) {
    return "Perfect";
  }

  String msg = "";

  if (currentFaultA || latchFaultA) {
    msg += "A";
  }

  if (currentFaultB || latchFaultB) {
    msg += "B";
  }


  if (currentVonFault || latchVonFault) {
    msg += "VON";
  }

  if (msg.length() == 0) {
    msg = "Perfect";
  }

  return msg;
}


String getAxisFaultBriefText(char axis) {
  if (axis == 'A') {
    if (currentFaultA || latchFaultA) return "A";
  } else if (axis == 'B') {
    if (currentFaultB || latchFaultB) return "B";
  }

  if (currentVonFault || latchVonFault) {
    return "VON";
  }

  return "Perfect";
}


bool updateFaultLatch() {
#if ENABLE_FAULT_CHECK
  currentFaultA = digitalRead(PIN_A_FAULT) == LOW;
  currentFaultB = digitalRead(PIN_B_FAULT) == LOW;

  float von = readVonVoltage();
  currentVonFault = (von < VON_MIN || von > VON_MAX);

  uint64_t nowMs = getCorrectedTimeMs();

  if (currentFaultA && !latchFaultA) {
    latchFaultA = true;
    A_fault_time_ms = nowMs;
  }

  if (currentFaultB && !latchFaultB) {
    latchFaultB = true;
    B_fault_time_ms = nowMs;
  }


  if (currentVonFault && !latchVonFault) {
    latchVonFault = true;
    Von_fault_time_ms = nowMs;
    Von_fault_value = von;
  }

  return hasAnyCurrentFault();
#else
  currentFaultA = false;
  currentFaultB = false;
  currentVonFault = false;
  systemProtected = false;
  return false;
#endif
}


String getFaultInfo() {
  updateFaultLatch();

  String msg = "";

  msg += "Fault_latch=";
  if (!hasAnyFaultLatch()) {
    msg += "NONE";
  } else {
    if (latchFaultA) msg += "A";
    if (latchFaultB) msg += "B";
    if (latchVonFault) msg += "VON";
  }

  msg += ", Fault_current=";
  if (!hasAnyCurrentFault()) {
    msg += "NONE";
  } else {
    if (currentFaultA) msg += "A";
    if (currentFaultB) msg += "B";
    if (currentVonFault) msg += "VON";
  }

  msg += ", Protected=";
  msg += (systemProtected ? "YES" : "NO");

  msg += ", \nA_fault_time=";
  msg += msToTimestamp(A_fault_time_ms);

  msg += ", B_fault_time=";
  msg += msToTimestamp(B_fault_time_ms);


  msg += ", Von_fault_time=";
  msg += msToTimestamp(Von_fault_time_ms);

  msg += ", Von_fault_value=";
  msg += String(Von_fault_value, 2);

  msg += ", Von_current=";
  msg += String(readVonVoltage(), 2);

  return msg;
}


String clearFaultLatch() {
#if ENABLE_FAULT_CHECK
  updateFaultLatch();

  if (hasAnyCurrentFault()) {
    String msg = "";
    msg += "ERR, CLEARFAULT_DENIED, CURRENT_FAULT_EXISTS,\n";
    msg += "Fault_latch=";
    if (!hasAnyFaultLatch()) {
      msg += "NONE";
    } else {
      if (latchFaultA) msg += "A";
      if (latchFaultB) msg += "B";
      if (latchVonFault) msg += "VON";
    }

    msg += ", Fault_current=";
    if (!hasAnyCurrentFault()) {
      msg += "NONE";
    } else {
      if (currentFaultA) msg += "A";
      if (currentFaultB) msg += "B";
      if (currentVonFault) msg += "VON";
    }

    return msg;
  }
#endif

  latchFaultA = false;
  latchFaultB = false;
  latchVonFault = false;

  A_fault_time_ms = 0;
  B_fault_time_ms = 0;
  Von_fault_time_ms = 0;

  Von_fault_value = 0.0f;
  systemProtected = false;

  return "OK, CLEARFAULT";
}


bool canWakeNow() {
#if ENABLE_FAULT_CHECK
  updateFaultLatch();

  if (hasAnyCurrentFault()) {
    return false;
  }

  if (hasAnyFaultLatch()) {
    return false;
  }

  return true;
#else
  return true;
#endif
}