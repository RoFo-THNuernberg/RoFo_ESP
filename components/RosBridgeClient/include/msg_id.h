#pragma once

enum MSG_ID {
    INIT_ID = 0x01,
    ADVERTISE_ID,
    SUBSCRIBE_ID,
    KEEP_ALIVE_ID,
    PUBLISH_ID
};

#define RX_BUF_LEN 1024
#define MAX_KEEP_ALIVE_TIMOUT_MS 2000
#define KEEP_ALIVE_CHECK_PERIOD_MS 500
#define KEEP_ALIVE_SEND_PERIOD_MS 500

#define MAX_TOPIC_LENGTH 32
#define MAX_MSG_TYPE_LENGTH 32