# SwarmTalk + Kilobot on ARGoS Simulation

For more details about SwarmTalk: https://github.com/shzhangyihan/SwarmTalk

Make sure ARGoS and Kilobot plugin are installed correctly:

1. ARGoS Core: https://github.com/ilpincy/argos3
2. Kilobot plugin: https://github.com/ilpincy/argos3-kilobot

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

## Edge following

```shell
argos3 -c src/examples/experiments/edge_following.argos
```

## Hop count with one seed

```shell
argos3 -c src/examples/experiments/hop_count_1_seed.argos
```

## Hop count with two seeds

```shell
argos3 -c src/examples/experiments/hop_count_2_seed.argos
```
