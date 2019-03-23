````
 __  __ __  __ ___  ___
|  \/  |  \/  | _ \/ __|
| |\/| | |\/| |   / (__
|_|  |_|_|  |_|_|_\\___| - Two turnout
````

Modern Model Railroad Control - Software for ESP866 wifi capable Arduino and clones.


## Two turnout
To control two signals from a separate turnouts. You have to set the signals topic in the turnout client settings.
Att the moment this client can only control ONE signal.

## Client pin use
This client uses the Wemos pins like this. If you have a signal with fewer lights, you just connect the lights you have and change settings SIGNAL01TYPE and SIGNAL02TYPE accordingly.

Signal 1, swedish 2, 3, 4 or 5-light signal

* D5 = Light 1 (top light)
* D6 = Light 2
* D7 = Light 3
* Tx = Light 4
* Rx = Light 5 (bottom light)

Signal 2, swedish 2, 3 or 4-light signal

- D1 - Not used att the moment
- D2 - Not used att the moment
- D3 - Not used att the moment
- D4 - Not used att the moment


## Dependencies
This client is depending on the following Arduino libraries:

- ESP8266WiFi.h
- PubSubClient.h
- DNSServer.h
- ESP8266WebServer.h
- WiFiManager.h (hotfix branch)

I got a compile error using the latest stable branch of WifiManager from the Library handler in Arduino IDE. So to use this software you have to manually download och install the **hotfix** version of the WifiManager library by tzapu: [WifiManager hotfix branch](https://github.com/tzapu/WiFiManager/tree/hotfixes).


*// Peter Kindström*