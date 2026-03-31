#!/usr/bin/env python3
"""ICBM Speed Limit Daemon — looks up speed limits from OpenStreetMap.

Reads GPS coordinates from openpilot's gpsLocationExternal message,
queries the Overpass API for the nearest road's speed limit, and writes
the result to /data/openpilot/icbm_speed_limit for the ICBM controller.

Run on the Comma device:
  cd /data/openpilot && /usr/local/venv/bin/python3 icbm_speed_limit_daemon.py &
"""

import json
import time
import urllib.request

SPEED_LIMIT_FILE = "/data/openpilot/icbm_speed_limit"
QUERY_INTERVAL = 5  # seconds between OSM queries
GPS_POLL_TIMEOUT = 3000  # ms

# Overpass API endpoint (public, rate-limited)
OVERPASS_URL = "https://overpass-api.de/api/interpreter"

# Conversion factors
MPH_PER_KPH = 0.621371


def get_gps():
  """Get current GPS coordinates from openpilot."""
  import cereal.messaging as messaging
  sm = messaging.SubMaster(['gpsLocationExternal'])
  sm.update(GPS_POLL_TIMEOUT)
  if sm.alive['gpsLocationExternal']:
    msg = sm['gpsLocationExternal']
    return msg.latitude, msg.longitude
  return None, None


def query_speed_limit(lat, lon, radius=50):
  """Query Overpass API for the speed limit of the nearest road."""
  query = f"""
  [out:json][timeout:5];
  way(around:{radius},{lat},{lon})["highway"]["maxspeed"];
  out tags;
  """
  try:
    data = urllib.request.urlopen(
      urllib.request.Request(
        OVERPASS_URL,
        data=f"data={query}".encode(),
        method="POST",
      ),
      timeout=5,
    ).read()
    result = json.loads(data)
    elements = result.get("elements", [])
    if not elements:
      return None

    # Find the most relevant road (prefer primary/secondary/tertiary)
    best = None
    road_priority = {
      "motorway": 0, "trunk": 1, "primary": 2, "secondary": 3,
      "tertiary": 4, "residential": 5, "unclassified": 6,
    }
    for el in elements:
      tags = el.get("tags", {})
      maxspeed = tags.get("maxspeed", "")
      highway = tags.get("highway", "")
      priority = road_priority.get(highway, 99)
      if best is None or priority < best[0]:
        best = (priority, maxspeed)

    if best is None:
      return None

    return parse_speed(best[1])
  except Exception:
    return None


def parse_speed(maxspeed_str):
  """Parse OSM maxspeed tag to mph integer.

  Formats: '65 mph', '100', '50 km/h', 'none'
  Bare numbers are assumed km/h per OSM convention (except US roads often tag 'XX mph').
  """
  if not maxspeed_str or maxspeed_str in ("none", "signals", "variable"):
    return None

  s = maxspeed_str.strip().lower()
  if "mph" in s:
    try:
      return int(s.replace("mph", "").strip())
    except ValueError:
      return None
  else:
    # km/h (bare number or explicit)
    try:
      kph = int(s.replace("km/h", "").replace("kph", "").strip())
      return round(kph * MPH_PER_KPH)
    except ValueError:
      return None


def write_speed_limit(speed_mph):
  """Write speed limit to file for ICBM controller."""
  with open(SPEED_LIMIT_FILE, "w") as f:
    f.write(str(speed_mph))


def main():
  last_speed = 0
  consecutive_failures = 0

  print("ICBM Speed Limit Daemon starting...")

  while True:
    try:
      lat, lon = get_gps()
      if lat is None:
        time.sleep(QUERY_INTERVAL)
        continue

      speed = query_speed_limit(lat, lon)
      if speed is not None and speed > 0:
        if speed != last_speed:
          print(f"Speed limit changed: {last_speed} -> {speed} mph (GPS: {lat:.5f}, {lon:.5f})")
        last_speed = speed
        write_speed_limit(speed)
        consecutive_failures = 0
      else:
        consecutive_failures += 1
        # Widen search radius after consecutive failures
        if consecutive_failures >= 3:
          speed = query_speed_limit(lat, lon, radius=80)
          if speed is not None and speed > 0:
            if speed != last_speed:
              print(f"Speed limit (wide search): {last_speed} -> {speed} mph")
            last_speed = speed
            write_speed_limit(speed)
            consecutive_failures = 0

    except Exception as e:
      print(f"Error: {e}")

    time.sleep(QUERY_INTERVAL)


if __name__ == "__main__":
  main()
