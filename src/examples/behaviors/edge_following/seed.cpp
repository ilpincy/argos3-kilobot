#include "../../../../../SwarmTalk/build/Kilobot/swarmtalk.h"

#define SEED_CHANNEL 0
#define FOLLOWER_CHANNEL 1

#define OCCUPY_TIME_OUT 100
#define MOTION_DURATION 5
#define LED_DURATION 20
#define MAX_DIST 200

typedef struct seed_state {
    int seed_id;
    int follower_id;
    bool occupied;
} seed_state_t;

typedef struct follower_state {
    int follower_id;
    int following;
    int dist;
} follower_state_t;

Channel *seed_channel;
Channel *follower_channel;
Publisher *seed_publisher;
Subscriber *follower_subscriber;
seed_state_t my_state;
Subscriber *seed_subscriber;

int last_occupied_time;

void sent_callback() {
    seed_publisher->send((unsigned char *)&my_state, sizeof(my_state));
}

void recv_callback(unsigned char *msg, int size, int ttl, Meta_t *meta) {
    follower_state_t *follower_state = (follower_state_t *)msg;
    if (follower_state->following == my_state.seed_id &&
        my_state.occupied == false) {
        my_state.occupied = true;
        my_state.follower_id = follower_state->follower_id;
        last_occupied_time = swarmtalk.sys.get_clock();
    } else if (follower_state->following == my_state.seed_id &&
               my_state.follower_id == follower_state->follower_id) {
        last_occupied_time = swarmtalk.sys.get_clock();
    }
}

void loop() {
    if (last_occupied_time + OCCUPY_TIME_OUT < swarmtalk.sys.get_clock()) {
        my_state.occupied = false;
        my_state.follower_id = -1;
    }
    if (my_state.occupied) {
        LED_control->turn_on(0, 0, 1, LED_DURATION);
    } else {
        LED_control->turn_on(1, 1, 1, LED_DURATION);
    }
}

void setup() {
    my_state.seed_id = swarmtalk.sys.random_func() % 255;
    my_state.occupied = false;
    my_state.follower_id = -1;
    last_occupied_time = swarmtalk.sys.get_clock();

    seed_channel = swarmtalk.new_channel(SEED_CHANNEL, 0, false);
    follower_channel = swarmtalk.new_channel(FOLLOWER_CHANNEL, 0, false);
    seed_publisher = seed_channel->new_publisher(sent_callback);
    follower_subscriber =
        follower_channel->new_subscriber(MAX_DIST, recv_callback);
    seed_publisher->send((unsigned char *)&my_state, sizeof(my_state));

    LED_control->turn_on(1, 1, 1, LED_DURATION);
}
