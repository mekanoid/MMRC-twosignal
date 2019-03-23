/**
  * Settings for this specific MMRC client
  */

// Device settings
#define BROKERIP "192.168.41.1"
#define BROKERPORT "1883"
#define DEVICEID "pkin-sj07"
#define DEVICENAME "Signalklient 01"

// Node one settings
#define NODE01ID "signal1"
#define NODE01NAME "Signal 1"
#define NODE01TYPE "signal"
#define NODE01PROP01 "main"
#define NODE01PROP01NAME "Huvudsignal"
#define NODE01PROP01DATATYPE "string"
#define NODE01PROP02 "slave"
#define NODE01PROP02NAME "Försignal"
#define NODE01PROP02DATATYPE "string"
#define SIGNAL01TYPE "5"
#define SIGNAL01LISTEN "mmrc/pkin-sj77/signal1/main"

// Node two settings
#define NODE02ID "signal2"
#define NODE02NAME "Signal 2"
#define NODE02TYPE "signal"
#define NODE02PROP01 "main"
#define NODE02PROP01NAME "Huvudsignal"
#define NODE02PROP01DATATYPE "string"
#define NODE02PROP02 "slave"
#define NODE02PROP02NAME "Försignal"
#define NODE02PROP02DATATYPE "string"
#define SIGNAL02TYPE "4"
#define SIGNAL02LISTEN "mmrc/pkin-sj77/signal2/main"

