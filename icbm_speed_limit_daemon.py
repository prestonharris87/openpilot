#!/usr/bin/env python3
"""ICBM Speed Limit Daemon — looks up speed limits from OpenStreetMap.

Reads GPS coordinates from openpilot's gpsLocationExternal message,
queries the Overpass API for the nearest road's speed limit, and writes
the result to /data/openpilot/icbm_speed_limit for the ICBM controller.

Uses GPS lookahead for speed DECREASES: queries a point ~500m ahead
of the current position so cruise speed starts dropping before reaching
the lower speed zone. Speed increases apply at the current position
(no need to slow down early for a higher limit).

Run on the Comma device:
  cd /data/openpilot && /usr/local/venv/bin/python3 icbm_speed_limit_daemon.py &
"""

import json
import math
import time
import urllib.request

SPEED_LIMIT_FILE = "/data/openpilot/icbm_speed_limit"
QUERY_INTERVAL = 3  # seconds between OSM queries
GPS_POLL_TIMEOUT = 3000  # ms

# Overpass API endpoint (public, rate-limited)
OVERPASS_URL = "https://overpass-api.de/api/interpreter"

# Lookahead distance in meters for detecting upcoming lower speed limits
LOOKAHEAD_METERS = 500

# Conversion factors
MPH_PER_KPH = 0.621371
EARTH_RADIUS_M = 6371000


def get_gps():
  """Get current GPS coordinates, speed, and bearing from openpilot."""
  import cereal.messaging as messaging
  sm = messaging.SubMaster(['gpsLocationExternal'])
  sm.update(GPS_POLL_TIMEOUT)
  if sm.alive['gpsLocationExternal']:
    msg = sm['gpsLocationExternal']
    return msg.latitude, msg.longitude, msg.speed, msg.bearingDeg
  return None, None, 0, 0


def project_point(lat, lon, bearing_deg, distance_m):
  """Project a GPS point forward by distance_m along bearing_deg."""
  lat_r = math.radians(lat)
  lon_r = math.radians(lon)
  bearing_r = math.radians(bearing_deg)
  d = distance_m / EARTH_RADIUS_M

  new_lat = math.asin(
    math.sin(lat_r) * math.cos(d) +
    math.cos(lat_r) * math.sin(d) * math.cos(bearing_r)
  )
  new_lon = lon_r + math.atan2(
    math.sin(bearing_r) * math.sin(d) * math.cos(lat_r),
    math.cos(d) - math.sin(lat_r) * math.sin(new_lat)
  )
  return math.degrees(new_lat), math.degrees(new_lon)


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

    # Find the most relevant road (prefer higher-class roads)
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
  """Parse OSM maxspeed tag to mph integer."""
  if not maxspeed_str or maxspeed_str in ("none", "signals", "variable"):
    return None

  s = maxspeed_str.strip().lower()
  if "mph" in s:
    try:
      return int(s.replace("mph", "").strip())
    except ValueError:
      return None
  else:
    try:
      kph = int(s.replace("km/h", "").replace("kph", "").strip())
      return round(kph * MPH_PER_KPH)
    except ValueError:
      return None


def write_speed_limit(speed_mph):
  with open(SPEED_LIMIT_FILE, "w") as f:
    f.write(str(speed_mph))


def main():
  current_limit = 0
  consecutive_failures = 0

  print("ICBM Speed Limit Daemon starting (OSM with lookahead)...", flush=True)

  while True:
    try:
      lat, lon, speed_ms, bearing = get_gps()
      if lat is None:
        time.sleep(QUERY_INTERVAL)
        continue

      # Query speed limit at current position
      here_limit = query_speed_limit(lat, lon)

      # Query speed limit at lookahead point (ahead along bearing)
      ahead_limit = None
      if speed_ms > 5 and bearing != 0:  # only lookahead if moving with valid bearing
        ahead_lat, ahead_lon = project_point(lat, lon, bearing, LOOKAHEAD_METERS)
        ahead_limit = query_speed_limit(ahead_lat, ahead_lon)

      # Determine effective speed limit:
      # - For decreases: use the LOWER of current and lookahead (anticipate the drop)
      # - For increases: use current position only (don't speed up early)
      if here_limit is not None and here_limit > 0:
        effective = here_limit

        if ahead_limit is not None and ahead_limit > 0 and ahead_limit < here_limit:
          # Lower limit ahead — start reducing now
          effective = ahead_limit
          if effective != current_limit:
            print(f"LOOKAHEAD: {here_limit}mph here, {ahead_limit}mph ahead "
                  f"({LOOKAHEAD_METERS}m) — using {effective}mph", flush=True)

        if effective != current_limit:
          if effective != ahead_limit:  # don't double-log lookahead
            print(f"Speed limit: {current_limit} -> {effective} mph "
                  f"(GPS: {lat:.5f}, {lon:.5f})", flush=True)
          current_limit = effective
          write_speed_limit(effective)
        consecutive_failures = 0

      else:
        consecutive_failures += 1
        if consecutive_failures >= 3:
          fallback = query_speed_limit(lat, lon, radius=80)
          if fallback is not None and fallback > 0:
            if fallback != current_limit:
              print(f"Speed limit (wide): {current_limit} -> {fallback} mph", flush=True)
            current_limit = fallback
            write_speed_limit(fallback)
            consecutive_failures = 0

    except Exception as e:
      print(f"Error: {e}", flush=True)

    time.sleep(QUERY_INTERVAL)


if __name__ == "__main__":
  main()
