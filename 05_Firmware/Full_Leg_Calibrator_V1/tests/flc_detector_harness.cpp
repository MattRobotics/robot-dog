/*
 * Host harness for flc_contact_detector.h
 *
 * Compiles the SAME header the ESP32-S3 firmware compiles, so the offline
 * fault-injection suite exercises the real contact state machine rather than a
 * Python re-implementation that could drift.
 *
 * Build:
 *   g++ -std=c++17 -O0 -Wall -Wextra -Werror \
 *       -I../matdog_full_leg_calibrator_v1 flc_detector_harness.cpp -o flc_detector_harness
 *
 * Line protocol on stdin, one result line per OBS on stdout:
 *
 *   CONFIG <maxProgress> <maxVel> <targetTol> <minTravel> <persistence>
 *          <grace> <hardCurrent> <expTorqueLimit> <thermalC> <vmin> <vmax>
 *          <travelBudget> <timeBudgetMs> <probeSign>
 *   BASELINE <median> <mad> <valid>
 *   INIT <startPosition>
 *   OBS <telemetryValid> <driverError> <status> <torqueEnabled> <torqueLimit>
 *       <goal> <position> <velocity> <current> <temp> <voltage> <elapsedMs>
 *       <commandedTarget>
 *   REPEAT <firstTick> <secondTick> <toleranceTicks>
 *   DOMAIN <start> <ticks> <probeSign>
 *   DELTA <present> <reference>
 */

#include <cstdio>
#include <cstring>
#include <string>
#include <iostream>

#include "flc_contact_detector.h"

static const char *stateLabel(int s) {
  switch (s) {
    case FLC_FREE_MOTION: return "FREE_MOTION";
    case FLC_CONFIRMING: return "CONFIRMING";
    case FLC_CONTACT_CONFIRMED: return "CONTACT_CONFIRMED";
    case FLC_HARD_ABORT: return "HARD_ABORT";
    default: return "UNKNOWN";
  }
}

int main() {
  FlcContactConfig config{};
  FlcBaseline baseline{};
  FlcContactDetector detector{};
  bool initialized = false;

  std::string line;
  while (std::getline(std::cin, line)) {
    if (line.empty()) continue;

    char verb[32] = {0};
    if (sscanf(line.c_str(), "%31s", verb) != 1) continue;

    if (strcmp(verb, "CONFIG") == 0) {
      unsigned maxProgress, maxVel, targetTol, minTravel, persistence, grace;
      unsigned hardCurrent, expTorqueLimit, travelBudget, timeBudget;
      int thermalC, vmin, vmax, probeSign;
      if (sscanf(line.c_str(),
                 "CONFIG %u %u %u %u %u %u %u %u %d %d %d %u %u %d",
                 &maxProgress, &maxVel, &targetTol, &minTravel, &persistence,
                 &grace, &hardCurrent, &expTorqueLimit, &thermalC, &vmin, &vmax,
                 &travelBudget, &timeBudget, &probeSign) != 14) {
        printf("ERROR BAD_CONFIG\n");
        fflush(stdout);
        continue;
      }
      config.maxProgressTicks = (uint16_t)maxProgress;
      config.maxVelocityRaw = (uint16_t)maxVel;
      config.targetReachedToleranceTicks = (uint16_t)targetTol;
      config.minTravelTicks = (uint16_t)minTravel;
      config.persistenceSamples = (uint8_t)persistence;
      config.startupGraceSamples = (uint8_t)grace;
      config.hardCurrentAbortRaw = (uint16_t)hardCurrent;
      config.expectedTorqueLimit = (uint16_t)expTorqueLimit;
      config.thermalLimitC = thermalC;
      config.voltageMin = vmin;
      config.voltageMax = vmax;
      config.travelBudgetTicks = (uint16_t)travelBudget;
      config.timeBudgetMs = (uint32_t)timeBudget;
      config.probeSign = (int8_t)probeSign;
      printf("OK CONFIG\n");
      fflush(stdout);
      continue;
    }

    if (strcmp(verb, "BASELINE") == 0) {
      unsigned median, mad, valid;
      if (sscanf(line.c_str(), "BASELINE %u %u %u", &median, &mad, &valid) != 3) {
        printf("ERROR BAD_BASELINE\n");
        fflush(stdout);
        continue;
      }
      baseline.medianCurrent = (uint16_t)median;
      baseline.madCurrent = (uint16_t)mad;
      baseline.valid = (valid != 0);
      printf("OK BASELINE THRESHOLD=%u\n", flcBaselineContactThreshold(baseline));
      fflush(stdout);
      continue;
    }

    if (strcmp(verb, "INIT") == 0) {
      int startPosition;
      if (sscanf(line.c_str(), "INIT %d", &startPosition) != 1) {
        printf("ERROR BAD_INIT\n");
        fflush(stdout);
        continue;
      }
      flcDetectorInit(detector, config, baseline, startPosition);
      initialized = true;
      printf("OK INIT\n");
      fflush(stdout);
      continue;
    }

    if (strcmp(verb, "OBS") == 0) {
      if (!initialized) {
        printf("ERROR NOT_INITIALIZED\n");
        fflush(stdout);
        continue;
      }
      unsigned telemetryValid, driverError, status, torqueEnabled, torqueLimit;
      unsigned goal, position, current, temp, voltage, elapsedMs;
      int velocity, commandedTarget;
      if (sscanf(line.c_str(),
                 "OBS %u %u %u %u %u %u %u %d %u %u %u %u %d",
                 &telemetryValid, &driverError, &status, &torqueEnabled,
                 &torqueLimit, &goal, &position, &velocity, &current, &temp,
                 &voltage, &elapsedMs, &commandedTarget) != 13) {
        printf("ERROR BAD_OBS\n");
        fflush(stdout);
        continue;
      }
      FlcObservation o{};
      o.telemetryValid = (telemetryValid != 0);
      o.driverError = (driverError != 0);
      o.statusByte = (uint8_t)status;
      o.torqueEnabled = (torqueEnabled != 0);
      o.torqueLimit = (uint16_t)torqueLimit;
      o.goalPosition = (uint16_t)goal;
      o.position = (uint16_t)position;
      o.velocity = (int16_t)velocity;
      o.current = (uint16_t)current;
      o.temperature = (uint8_t)temp;
      o.voltage = (uint8_t)voltage;
      o.elapsedMs = (uint32_t)elapsedMs;

      int state = flcDetectorObserve(detector, o, commandedTarget);
      printf("STATE=%s REASON=%s CONTACT_TICK=%d CONFIRMING=%u PEAK_CURRENT=%u "
             "PEAK_TEMP=%u CURRENT_SUPPORTED=%d\n",
             stateLabel(state), flcAbortReasonLabel(detector.abortReason),
             detector.contactTick, (unsigned)detector.confirmingSamples,
             (unsigned)detector.peakCurrent, (unsigned)detector.peakTemperature,
             detector.currentSupportedContact ? 1 : 0);
      fflush(stdout);
      continue;
    }

    if (strcmp(verb, "REPEAT") == 0) {
      int first, second;
      unsigned tolerance;
      if (sscanf(line.c_str(), "REPEAT %d %d %u", &first, &second, &tolerance) != 3) {
        printf("ERROR BAD_REPEAT\n");
        fflush(stdout);
        continue;
      }
      FlcRepeatability r = flcEvaluateRepeatability(first, second, (uint16_t)tolerance);
      printf("SPREAD=%d CONTACT_TICK=%d ACCEPTED=%d\n", r.spreadTicks,
             r.contactTick, r.accepted ? 1 : 0);
      fflush(stdout);
      continue;
    }

    if (strcmp(verb, "DOMAIN") == 0) {
      int start, ticks, probeSign;
      if (sscanf(line.c_str(), "DOMAIN %d %d %d", &start, &ticks, &probeSign) != 3) {
        printf("ERROR BAD_DOMAIN\n");
        fflush(stdout);
        continue;
      }
      printf("IN_DOMAIN=%d\n", flcPlanStaysInDomain(start, ticks, probeSign) ? 1 : 0);
      fflush(stdout);
      continue;
    }

    if (strcmp(verb, "SUMMARY") == 0) {
      // SUMMARY <t0> <t1> ... — circular statistics over the given ticks.
      int ticks[512];
      int count = 0;
      const char *cursor = line.c_str() + strlen("SUMMARY");
      int value = 0;
      int consumed = 0;
      while (count < 512 && sscanf(cursor, " %d%n", &value, &consumed) == 1) {
        ticks[count++] = value;
        cursor += consumed;
      }
      FlcTickSummary s = flcSummarizeTicks(ticks, count);
      printf("VALID=%d COUNT=%d CENTRE=%d MIN=%d MAX=%d SPREAD=%d\n",
             s.valid ? 1 : 0, s.count, s.centreTick, s.minTick, s.maxTick,
             s.spreadTicks);
      fflush(stdout);
      continue;
    }

    if (strcmp(verb, "DELTA") == 0) {
      int present, reference;
      if (sscanf(line.c_str(), "DELTA %d %d", &present, &reference) != 2) {
        printf("ERROR BAD_DELTA\n");
        fflush(stdout);
        continue;
      }
      printf("DELTA=%d\n", flcSignedTickDelta(present, reference));
      fflush(stdout);
      continue;
    }

    printf("ERROR UNKNOWN_VERB\n");
    fflush(stdout);
  }
  return 0;
}
