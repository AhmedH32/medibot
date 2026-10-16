# Subsystem C: Firebase & Mobile App Data Contract

*Privacy Invariant: Zero raw camera frames, audio recordings, or personal biometrics are ever transmitted to the cloud.*

---

## 1. Robot Heartbeat Document (`status/robot_state`)
- **Write Frequency:** Updated by Pi 5 every 300 seconds (5 minutes) and upon state transitions.
- **Dead-Man Switch Rule:** The mobile client fires an offline alert if `current_time - last_ping_epoch > 360` seconds.

```json
{
  "last_ping_epoch": 1760630400,
  "battery_pct": 84,
  "motor_stalled": false,
  "current_mode": "FOLLOWING",
  "user_visible": true,
  "sonar_min_cm": 140
}
```

---

## 2. Event Notifications & Logs (`events_log/` collection + FCM Push)
Dispatched for every conversation turn, medical triage query, medication verification, or emergency.

### Severity Tiers:
- `INFO`: Normal conversation summaries, routine status check, medication verified on time.
- `ADVISORY`: Minor medical situations handled locally via RAG (e.g., hydration, bandaid, resting), battery < 20%.
- `CRITICAL`: Fall detected (any angle), acute medical emergency reported (chest pain, severe breathlessness), medication missed alert.

```json
{
  "event_id": "evt_20261016_161022",
  "timestamp_epoch": 1760631022,
  "severity": "CRITICAL",
  "category": "FALL_DETECTED",
  "title": "EMERGENCY: Fall Detected",
  "summary": "Forward optical-axis fall detected (R_comp=1.18). Motors locked. Audio distress dialogue initiated.",
  "metadata": {
    "battery_pct": 82,
    "action_taken": "Hard motor lock engaged; awaiting verbal confirmation from user."
  }
}
```

---

## 3. Medication Alarm Schedules (`schedules/medications`)
Written by the family mobile application; polled and synced by the Pi 5 state engine.

```json
{
  "schedule_id": "med_01",
  "medication_name": "Blood Pressure Pill (Amlodipine 5mg)",
  "scheduled_time_24h": "18:00",
  "enabled": true,
  "verification_timeout_sec": 180
}
```