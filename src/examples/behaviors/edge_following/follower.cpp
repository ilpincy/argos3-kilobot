#include "../../../../../SwarmTalk/build/Kilobot/swarmtalk.h"

#define SEED_CHANNEL 0
#define FOLLOWER_CHANNEL 1

#define MOTION_STEP 10
#define LED_DURATION 200
#define MAX_DIST 200
#define SET_DISTANCE 60
#define ALLOW_NOISE 2

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
Publisher *follower_publisher;
Subscriber *seed_subscriber;
follower_state_t my_state;
int motion_state;  // 0 for stop, 1 for left, 2 for right, 3 for forward

void sent_callback() {
    follower_publisher->send((unsigned char *)&my_state, sizeof(my_state));
}

void PD_controller(int cur_dist, int prev_dist) {
    if (cur_dist > SET_DISTANCE + ALLOW_NOISE) {
        if (motion_state == 0) {
            if (cur_dist >= prev_dist) {
                motion_state = 2;
            } else {
                motion_state = 3;
            }
        }
    } else if (cur_dist < SET_DISTANCE - ALLOW_NOISE) {
        if (motion_state == 0) {
            if (cur_dist <= prev_dist) {
                motion_state = 1;
            } else {
                motion_state = 3;
            }
        }
    } else {
        if (motion_state == 0) {
            motion_state = 3;
        }
    }
}

void recv_callback(unsigned char *msg, int size, int ttl, Meta_t *meta) {
    seed_state_t *seed_state = (seed_state_t *)msg;
    if (seed_state->seed_id != my_state.following &&
        meta->dist < my_state.dist) {
        my_state.following = seed_state->seed_id;
        if (seed_state->follower_id !=
            my_state.follower_id) {  // wait for seed to be available
            motion_state = 0;
        }
    }

    if (seed_state->seed_id == my_state.following) {
        if (seed_state->follower_id != my_state.follower_id) {
            // keep waiting
            motion_state = 0;
        } else {
            // update dist
            PD_controller(meta->dist, my_state.dist);
            my_state.dist = meta->dist;
        }
    }
}

void loop() {
    if (motor_control->current_status() == Stop) {
        if (motion_state == 1) {
            motion_state = 0;
            motor_control->turn_left(MOTION_STEP);
            LED_control->turn_on(1, 0, 0, LED_DURATION);
        } else if (motion_state == 2) {
            motion_state = 0;
            motor_control->turn_right(MOTION_STEP);
            LED_control->turn_on(1, 1, 0, LED_DURATION);
        } else if (motion_state == 3) {
            motion_state = 0;
            motor_control->move_forward(MOTION_STEP);
            LED_control->turn_on(0, 1, 0, LED_DURATION);
        } else if (motion_state == 0) {
            motor_control->stop_motor();
        }
    }
    if (LED_control->current_status() == Off) {
        LED_control->turn_on(1, 1, 1, LED_DURATION);
    }
}

void setup() {
    my_state.follower_id = swarmtalk.sys.random_func() % 255;
    my_state.following = -1;
    my_state.dist = MAX_DIST;
    motion_state = 0;

    seed_channel = swarmtalk.new_channel(SEED_CHANNEL, 0, false);
    follower_channel = swarmtalk.new_channel(FOLLOWER_CHANNEL, 0, false);
    follower_publisher = follower_channel->new_publisher(sent_callback);
    seed_subscriber = seed_channel->new_subscriber(MAX_DIST, recv_callback);
    follower_publisher->send((unsigned char *)&my_state, sizeof(my_state));
}
