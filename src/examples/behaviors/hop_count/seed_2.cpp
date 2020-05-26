#include "../../../../../SwarmTalk/build/Kilobot/swarmtalk.h"

#define LED_DURATION 200

typedef struct custom_message {
    int hop;
} custom_message_t;

Channel *channel_seed_2;
Publisher *publisher;
custom_message_t my_message;

void sent() {
    publisher->send((unsigned char *)&my_message, sizeof(my_message));
}

void loop() { LED_control->turn_on(1, 1, 1, LED_DURATION); }

void setup() {
    my_message.hop = 0;
    channel_seed_2 = swarmtalk.new_channel(2, 0, false);
    publisher = channel_seed_2->new_publisher(sent);
    publisher->send((unsigned char *)&my_message, sizeof(my_message));
    LED_control->turn_on(1, 1, 1, LED_DURATION);
}
