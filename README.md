# IoTwx Core

This is the Arduino implementation of the IoTwx core library used by all nodes.

Use:

```C
#include "IoTwx.h"
```

to include this in the library.

The base class `IoTwx` implements two methods of use:

* `void establishCommunication()`
* `void publishMQTTMeasurement(const char* topic, const char* sensor, float m, long offset);`

The general flow in a node is :

1. take a measurement _m_
2. invoke `establishCommunication()` on your class instance
3. invoke `publishMQTTMeasurement()` with measurement _m_

See the [implementation](libraries/src/IoTwx.h) for more.


