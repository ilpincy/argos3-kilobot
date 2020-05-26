#include "../../../../../SwarmTalk/build/Kilobot/swarmtalk.h"

#define MAX_HOP 100
#define LED_DURATION 200
#define POLL_PERIOD 50

typedef struct custom_message {
    int hop;
} custom_message_t;

Channel *channel_seed_1;
Channel *channel_seed_2;

Publisher *publisher_channel_1;
Publisher *publisher_channel_2;

Subscriber *subscriber_channel_1;
Subscriber *subscriber_channel_2;

custom_message_t my_message_channel_1;
custom_message_t my_message_channel_2;

int hop_from_seed_1;
int hop_from_seed_2;

int period_min_hop_from_seed_1;
int period_min_hop_from_seed_2;
int period_start;

int LED_flag;

void sent_callback_channel_1() {
    // after sending to channel 1 finished, start new send to channel 1
    publisher_channel_1->send((unsigned char *)&my_message_channel_1,
                              sizeof(my_message_channel_1));
}

void sent_callback_channel_2() {
    // after sending to channel 2 finished, start new send to channel 2
    publisher_channel_2->send((unsigned char *)&my_message_channel_2,
                              sizeof(my_message_channel_2));
}

void recv_callback_channel_1(unsigned char *msg, int size, int ttl,
                             Meta_t *meta) {
    custom_message_t *received_msg = (custom_message_t *)msg;
    if (received_msg->hop < period_min_hop_from_seed_1) {
        period_min_hop_from_seed_1 = received_msg->hop;
    }
}

void recv_callback_channel_2(unsigned char *msg, int size, int ttl,
                             Meta_t *meta) {
    custom_message_t *received_msg = (custom_message_t *)msg;
    if (received_msg->hop < period_min_hop_from_seed_2) {
        period_min_hop_from_seed_2 = received_msg->hop;
    }
}

void update_LED(int hop_1, int hop_2) {
    int hop = (LED_flag ? hop_1 : hop_2) % 4;
    LED_control->turn_on(hop % 2, hop / 2, 1, LED_DURATION);
}

void loop() {
    if (swarmtalk.sys.get_clock() - period_start > POLL_PERIOD) {
        if (period_min_hop_from_seed_1 < MAX_HOP) {
            hop_from_seed_1 = period_min_hop_from_seed_1 + 1;
            my_message_channel_1.hop = hop_from_seed_1;
        }
        if (period_min_hop_from_seed_2 < MAX_HOP) {
            hop_from_seed_2 = period_min_hop_from_seed_2 + 1;
            my_message_channel_2.hop = hop_from_seed_2;
        }
    }

    int r = hop_from_seed_1 % 2;
    int g = hop_from_seed_2 % 2;
    int b = (r == 0 && g == 0) ? 1 : 0;
    LED_control->turn_on(r * 255, g * 255, b * 255, LED_DURATION);
}

void setup() {
    hop_from_seed_1 = MAX_HOP;
    hop_from_seed_2 = MAX_HOP;
    period_min_hop_from_seed_1 = MAX_HOP;
    period_min_hop_from_seed_2 = MAX_HOP;
    period_start = swarmtalk.sys.get_clock();

    my_message_channel_1.hop = hop_from_seed_1;
    my_message_channel_2.hop = hop_from_seed_2;

    channel_seed_1 = swarmtalk.new_channel(1, 0, false);
    channel_seed_2 = swarmtalk.new_channel(2, 0, false);

    publisher_channel_1 =
        channel_seed_1->new_publisher(sent_callback_channel_1);
    publisher_channel_2 =
        channel_seed_2->new_publisher(sent_callback_channel_2);

    subscriber_channel_1 =
        channel_seed_1->new_subscriber(260, recv_callback_channel_1);
    subscriber_channel_2 =
        channel_seed_2->new_subscriber(260, recv_callback_channel_2);

    publisher_channel_1->send((unsigned char *)&my_message_channel_1,
                              sizeof(my_message_channel_1));
    publisher_channel_2->send((unsigned char *)&my_message_channel_2,
                              sizeof(my_message_channel_2));

    LED_control->turn_on(255, 255, 255, LED_DURATION);
    LED_flag = 0;
}
