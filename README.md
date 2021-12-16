# tracked

# Tracked Wrapper #

The wrapper is a Tracked object taking two parameters:
- `value` (the wrapped object, e. g. a number or string etc.)
- `location` (optional)
In addition, it has additional attributes `location_map` (no own class, just a dictionary), `rosslt_msg`.
To make coding with a Tracked object, several operatorshave been overridden: