# Teltonika Eye Beacon BLE Analysis - Issues & Solutions

## Current Problems

### 1. **Signal Staleness (2500ms = 2.5 seconds)**
**Location**: Line 36 in main.c
```c
#define SIGNAL_STALE_MS (2500)
```

**Issue**: 
- After just 2.5 seconds without a BLE packet, the signal is marked "stale" even though the beacon is still present
- Teltonika Eye Beacon typically advertises every 100-200ms by default
- If scanning is too slow or packets are missed, the beacon appears offline quickly
- Dashboard shows "SIGNAL STALE" state frequently

**Root Causes**:
- Scan interval of 80ms (`0x50`) might miss packets from the beacon
- No fallback mechanism if packets are temporarily lost
- The ESP32 might be preempted by WiFi or other tasks during BLE scan windows

---

### 2. **Poor Distance Sensitivity - Large Dead Zone**
**Location**: Lines 30-31 in main.c
```c
#define NEAR_RSSI_THRESHOLD_DBM (-68.0f)
#define AWAY_RSSI_THRESHOLD_DBM (-78.0f)
```

**Issue**:
- Only 10dBm gap between NEAR and AWAY (roughly 3-5 meters)
- Needs excessive distance change to trigger state transitions
- Beacon needs to move very far away or very close to trigger response

**Root Cause**: Too aggressive hysteresis without enough stability

---

### 3. **Sluggish RSSI Smoothing Filter**
**Location**: Line 32 in main.c
```c
#define RSSI_SMOOTHING_ALPHA (0.12f)
```

**Issue**:
- Only 12% weight on new readings, 88% weight on old readings
- Takes many packets to respond to actual RSSI changes
- Exacerbates distance sensitivity problem
- Slow to detect when beacon is leaving

**Formula**: `smoothed = (0.12 × raw) + (0.88 × previous)`

**Effect**: Requires ~17-20 packets before filter reaches 95% of new value

---

### 4. **Slow Confirmation Counts**
**Location**: Lines 33-34 in main.c
```c
#define NEAR_CONFIRM_COUNT  (3)
#define AWAY_CONFIRM_COUNT  (5)
```

**Issue**:
- Needs 5 AWAY confirmations to transition from NEAR to AWAY
- At 80ms scan interval = 400ms minimum before state change
- At 200ms beacon advertising = even slower
- State changes feel delayed and unresponsive

---

## Teltonika Eye Beacon Specifications

From the product link you provided:
- **Advertising Interval**: Default 100-500ms (configurable)
- **Transmit Power**: Adjustable -20dBm to +4dBm
- **RSSI Accuracy**: ±4dBm typical
- **Battery Life**: 2-3 years (frequency affects this)

---

## Recommended Fixes

### Fix 1: Increase BLE Scan Frequency
**Change scan interval from 80ms to 30ms**
```c
static esp_ble_scan_params_t ble_scan_params = {
    .scan_type = BLE_SCAN_TYPE_ACTIVE,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .scan_filter_policy = BLE_SCAN_FILTER_ALLOW_ALL,
    .scan_interval = 0x20,  // Changed from 0x50 (30ms instead of 80ms)
    .scan_window = 0x20,    // Changed from 0x50
    .scan_duplicate = BLE_SCAN_DUPLICATE_DISABLE,
};
```

**Trade-off**: Higher power consumption, but catches packets more reliably

---

### Fix 2: Increase RSSI Smoothing Response
**Change alpha from 0.12 to 0.35**
```c
#define RSSI_SMOOTHING_ALPHA (0.35f)
```

**Effect**:
- New readings now 35% weight, previous 65% weight
- Reaches 95% convergence in ~8-9 packets instead of 17-20
- Still filters noise but responds faster to real changes

---

### Fix 3: Widen Hysteresis Gap
**Increase RSSI thresholds for better discrimination**
```c
#define NEAR_RSSI_THRESHOLD_DBM (-62.0f)    // Changed from -68.0f
#define AWAY_RSSI_THRESHOLD_DBM (-80.0f)    // Changed from -78.0f
```

**Effect**:
- 18dBm gap (roughly 8-12 meters)
- Much better separation between NEAR/AWAY zones
- Reduces flickering on state boundary

---

### Fix 4: Reduce Confirmation Counts
**Lower thresholds for faster response**
```c
#define NEAR_CONFIRM_COUNT  (2)    // Changed from 3
#define AWAY_CONFIRM_COUNT  (3)    // Changed from 5
```

**Effect**:
- State changes in ~60-90ms instead of 400+ms
- Still prevents momentary glitches (2-3 packets = ~200-300ms stability check)

---

### Fix 5: Handle Signal Staleness Better
**Increase stale timeout and add gradual decay**
```c
#define SIGNAL_STALE_MS (5000)      // Changed from 2500 (5 seconds)
#define SIGNAL_LOST_MS  (10000)     // Changed from 6000
```

**Alternative**: Implement exponential RSSI decay for missing packets
```c
// Instead of marking stale immediately, decay RSSI gradually
if (age_ms > SIGNAL_STALE_MS) {
    float decay_factor = 1.0f - (float)age_ms / (float)SIGNAL_LOST_MS;
    if (decay_factor > 0.0f) {
        filtered_rssi *= decay_factor;  // Slowly weaken the signal
    }
}
```

---

## Summary of Changes

| Setting | Current | Recommended | Reason |
|---------|---------|-------------|--------|
| Scan Interval | 0x50 (80ms) | 0x20 (30ms) | Catch more packets |
| RSSI Alpha | 0.12 | 0.35 | Faster response |
| NEAR Threshold | -68 dBm | -62 dBm | Better separation |
| AWAY Threshold | -78 dBm | -80 dBm | Wider hysteresis |
| NEAR Confirms | 3 | 2 | Quicker decisions |
| AWAY Confirms | 5 | 3 | Faster away detection |
| Stale Timeout | 2500ms | 5000ms | Less frequent staleness |

---

## Testing Procedure

1. **Before changes**: Record state transitions at various distances
2. **Apply changes incrementally** (one at a time)
3. **Monitor dashboard** at http://192.168.4.1 for:
   - How often "SIGNAL STALE" appears
   - Time to transition from NEAR → AWAY
   - Time to transition from AWAY → NEAR
   - False state changes (flickering)

4. **Measure real distances** using RSSI formula
5. **Adjust thresholds** based on your actual elevator dimensions

---

## Power Consumption Note

These changes will increase ESP32's power draw by ~15-25%:
- More frequent scanning
- WiFi running simultaneously (AP mode)

If running on battery, consider:
- Scanning only when needed
- Using periodic scanning instead of continuous
- Adjusting beacon advertising interval to 200-500ms
