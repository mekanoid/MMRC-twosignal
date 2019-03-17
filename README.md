````java
 __  __ __  __ ___  ___
|  \/  |  \/  | _ \/ __|
| |\/| | |\/| |   / (__
|_|  |_|_|  |_|_|_\\___| - Two turnout
````

Modern Model Railroad Control - Software for ESP866 wifi capable Arduino and clones.

## Two turnout
To control two turnouts with separate buttons and LEDs to indicate normal or reverse track.


## Dependencies
This client i depending on the following Arduino libraries:

- ESP8266WiFi.h
- PubSubClient.h
- DNSServer.h
- ESP8266WebServer.h
- WiFiManager.h (hotfix branch)
- Servo.h

I got a compile error using the latest stable branch of WifiManager from the Library handler in Arduino IDE. So to use this software you have to manually download och install the **hotfix** version of the WifiManager library by tzapu: [WifiManager hotfix branch](https://github.com/tzapu/WiFiManager/tree/hotfixes).


*// Peter Kindström*