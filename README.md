# Kilobot + SwarmTalk on ARGoS Simulation

For more details about SwarmTalk: https://github.com/shzhangyihan/SwarmTalk
For more details about ARGoS: https://github.com/ilpincy/argos3

Please follow the ARGoS Kilobot plugin installation guide at https://github.com/ilpincy/argos3-kilobot
Make sure the Kilobot simulation runs without SwarmTalk before proceeding.

To build:

```shell
mkdir build
cd build
cmake ../src
make
cd ..
```

## Firefly synchronization

```shell
argos3 -c src/examples/experiments/firefly.argos
```

## Hop count with one seed

```shell
argos3 -c src/examples/experiments/hop_count_1_seed.argos
```

## Hop count with two seeds

```shell
argos3 -c src/examples/experiments/hop_count_2_seed.argos
```

## Edge following

```shell
argos3 -c src/examples/experiments/edge_following.argos
```
