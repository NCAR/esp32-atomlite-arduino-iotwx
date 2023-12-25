# IoTwx Core

This is the Arduino implementation of the IoTwx core library used by all nodes. It is optimized for M5Stack Atom Lite, but would likely work on any ESP32 chipset.

Use:

```C
#include "IoTwx.h"
```

to include this in the library.

The base class `IoTwx` implements two methods of use:

- `void establishCommunication()`
- `void publishMQTTMeasurement(const char* topic, const char* sensor, float m, long offset);`

The general flow in a node is :

1. take a measurement _m_
2. invoke `establishCommunication()` on your class instance
3. invoke `publishMQTTMeasurement()` with measurement _m_

See the [implementation](IoTwx.h) for more.

## Used in

The library is used in the following code bases:

- ESP32 IoTwx Atmos Node [https://github.com/NCAR/esp32-atomlite-arduino-atmos-node](https://github.com/NCAR/esp32-atomlite-arduino-atmos-node)
- ESP32 IoTwx Hydro Node [https://github.com/NCAR/esp32-atomlite-arduino-hydro-node](https://github.com/NCAR/esp32-atomlite-arduino-hydro-node)
- ESP32 IoTwx Aero Node [https://github.com/NCAR/esp32-atomlite-arduino-aero-node](https://github.com/NCAR/esp32-atomlite-arduino-aero-node)
