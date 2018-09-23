#ifndef CONFIG
#define CONFIG

// receiver pulse length ranges from 8000 to 16000 centered around 12000
#define RECEIVER_CH_MIN 8000
#define RECEIVER_CH_CENTER 12000
#define RECEIVER_CH_MAX 16000
// how many ticks +/- RECEIVER_CENTER is part of the deadzone
#define RECEIVER_CH_DEADZONE 100

#endif