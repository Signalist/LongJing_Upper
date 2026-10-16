# A-axis runtime position limits

`ALIMIT<lower>,<upper>` uses radians in the current PA coordinate frame.

Example:

```text
SLEEPA
ALIMIT-7.2,4.3
OK, ALIMIT SET, lower=-7.200000, upper=4.300000
LIMIT?
```

Both values must be finite, lower must be strictly less than upper, A must be
asleep with known sensor direction, and its current measured position must be
inside the requested range (inclusive). Invalid requests leave both limits
unchanged. Error replies identify FORMAT, RANGE, REQUIRES_SLEEPA,
SENSOR_DIRECTION_UNKNOWN, or CURRENT_POSITION_OUTSIDE.

Accepted updates immediately affect PA/DA clamping, active position guards,
REBASE validation, STATA?, and LIMIT?. A pending position target is clamped to
the new range. The command neither moves/enables the motor nor changes the
coordinate origin, PID, or B-axis limits. It is RAM-only: restart restores the
Config.h defaults [-7.2,4.3]. No hardware travel is certified by this command.
