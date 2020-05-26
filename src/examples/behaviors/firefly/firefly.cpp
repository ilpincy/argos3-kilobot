#include <math.h>

#include "../../../../../SwarmTalk/build/Kilobot/swarmtalk.h"

#define DEFAULT_PERIOD 120
#define LED_DURATION 5
#define TIMER_CHANGE_BOUND 300
#define NORMALIZE_FACTOR 0.15

typedef struct state {
    int period_length;
} state_t;

Channel *channel;
Publisher *publisher;
Subscriber *subscriber;
state_t my_state;
int last_rest;
float period;

void recv_callback(unsigned char *msg, int size, int ttl, Meta_t *meta) {
    int my_time = swarmtalk.sys.get_clock() - last_rest - meta->msg_delay;
    if (my_time < 0) {
        my_time = DEFAULT_PERIOD + my_time;
    }
    if (my_time < 0 || my_time > period) {
        return;
    }
    float step =
        NORMALIZE_FACTOR * (1 + (swarmtalk.sys.random_func() % 3) / 5.0);
    if (my_time < (my_state.period_length / 2))
        period = period + (my_time * step);
    else
        period = period - ((period - my_time) * step);
    if (period < DEFAULT_PERIOD - TIMER_CHANGE_BOUND) {
        period = period - TIMER_CHANGE_BOUND;
    }
    if (period > DEFAULT_PERIOD + TIMER_CHANGE_BOUND) {
        period = period + TIMER_CHANGE_BOUND;
    }
}

void loop() {
    if (swarmtalk.sys.get_clock() - last_rest > period) {
        LED_control->turn_on(255, 255, 255, LED_DURATION);
        publisher->send((unsigned char *)&my_state, sizeof(my_state));
        last_rest = swarmtalk.sys.get_clock();
        my_state.period_length = DEFAULT_PERIOD;
        period = DEFAULT_PERIOD;
    }
}

void setup() {
    channel = swarmtalk.new_channel(0, 0, true);
    publisher = channel->new_publisher(NULL);
    subscriber = channel->new_subscriber(150, recv_callback);

    my_state.period_length =
        swarmtalk.sys.random_func() % (DEFAULT_PERIOD + TIMER_CHANGE_BOUND);
    period = fmod(1.0 * swarmtalk.sys.random_func(), 1.0 * (DEFAULT_PERIOD));
    last_rest = swarmtalk.sys.get_clock();
}
