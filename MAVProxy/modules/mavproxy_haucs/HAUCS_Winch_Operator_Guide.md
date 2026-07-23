# HAUCS Winch Drone — Operator Guide

> **This guide covers the new winch mission format only.**
> Do not reuse old `.waypoints` files — the servo sequence has changed.

---

## Before You Fly — Checklist

| Item | Where to check | Required value |
|------|---------------|----------------|
| `SERVO8_FUNCTION` | Mission Planner → Full Params | `0` (Disabled) |
| `AUTO_OPTIONS` | Mission Planner → Full Params | `3` |
| `SCR_USER1` | Mission Planner → Full Params | Descent time (s) e.g. `20` |
| `SCR_USER2` | Mission Planner → Full Params | Pause at bottom (s) e.g. `2` |
| `SCR_USER3` | Mission Planner → Full Params | Retract time (s) e.g. `35` |
| GPS fix | Mission Planner status bar | 3D Fix (green) |
| Battery | Mission Planner HUD | ≥ 14.4V before arming |
| Pi script running | GCS console | `MAVLINK: connected` printed |

---

## Servo Mapping

| PWM | Meaning | When triggered |
|-----|---------|----------------|
| **1500** | Neutral / safe | Every takeoff |
| **1900** | Winch **DEPLOY** | After landing on water |
| **1200** | Data **UPLOAD** | After climbing to upload altitude |

---

## Generate the Mission

In the MAVProxy console:

```
haucs gen_mission_winch <points_file> <alt> <upload_alt> <stab_sec> <soak_sec>
```

**Example — Basler Farm, 2 ponds, 15m cruise:**
```
haucs gen_mission_winch basler_points 15 10 3 60
```

| Argument | Description | Example |
|----------|-------------|---------|
| `points_file` | CSV with pond lat/lon (no `.csv` needed) | `basler_points` |
| `alt` | Cruise altitude AGL (m) | `15` |
| `upload_alt` | Min altitude before data upload trigger (m) | `10` |
| `stab_sec` | Settle time after landing (s) | `3` |
| `soak_sec` | Time on water — must be ≥ SCR_USER1+2+3 | `60` |

> **Points CSV format:** must have `lat` and `lon` columns.

---

## Mission Sequence (per pond)

```
Takeoff → ch8=1500 (neutral)
Fly to pond
Land on water  ← drone waits here until touchdown confirmed
Wait 3s        ← stabilize
ch8=1900       ← winch deploys
Wait 60s       ← winch descends, samples, retracts
Takeoff
Climb to 10m   ← waits for altitude, not a fixed timer
ch8=1200       ← data uploads
          ↓ repeat for next pond
RTL
```

---

## Load and Upload the Mission

1. Mission Planner → **Flight Plan** tab
2. Click **Load WP File** → select the generated `.waypoints` file
3. **Verify the mission visually** on the map — confirm pond locations look correct
4. Click **Write WPs** to upload to drone
5. Switch drone to **AUTO** mode
6. Arm and the mission begins

---

## Tuning Winch Timing

Change `SCR_USER1/2/3` in Mission Planner at any time — the Pi script reads them **before each deployment**, so no restart needed.

| Parameter | Controls | Too low → | Too high → |
|-----------|----------|-----------|------------|
| `SCR_USER1` | Descent time | Sensor doesn't reach depth | Hits bottom / overruns |
| `SCR_USER2` | Pause at bottom | Sensor lifts before sampling | Wasted time |
| `SCR_USER3` | Retract time | Sensor not fully retracted | Wasted time |

> Keep `soak_sec` in the mission ≥ SCR_USER1 + SCR_USER2 + SCR_USER3.
> Default total: 20 + 2 + 35 = **57s** → use `soak_sec = 60`.

---

## If Something Goes Wrong

| Symptom | Likely cause | Fix |
|---------|-------------|-----|
| Winch stops almost instantly | Hall sensor bug in old script | Ensure `main_rc8_uart_parm.py` is the latest version |
| Winch deploys but no data | `soak_sec` too short | Increase `SCR_USER1` + `SCR_USER3`, increase `soak_sec` |
| ch8 never changes in AUTO | `SR1_EXTRA3` not set | Set `SR1_EXTRA3 = 10` in Mission Planner, reboot FC |
| Mission stalls after landing | `AUTO_OPTIONS ≠ 3` | Set `AUTO_OPTIONS = 3` |
| Pi script not detecting AUTO | Heartbeat filter bug | Ensure latest script version (compid fix applied) |
| SD card 4-blink error | Card loose or corrupted | Reseat card; reflash if needed |

---

## Safe Shutdown Procedure

1. Wait for mission to complete and drone to RTL
2. Disarm
3. **Reduce Pi load before cutting power** — toggle `SCR_USER5` in Mission Planner to trigger clean Pi shutdown
4. Wait ~10s for Pi to shut down
5. Cut drone power

> Never cut power while Pi is actively writing — risk of SD card corruption.
