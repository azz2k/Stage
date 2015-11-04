Stage - simServer
==========

This is a hacked version of [Stage](http://rtv.github.com/Stage) to act as a
simulation Server for simulation based internal model experiments.

It starts as a regular Stage instance but then waits until network packets
arrive at a specified port. These packets are simulation instructions with
initical conditions, robot controllers, and a simulation time. Upon completion
of the simulation the server returns a set of robot trajectories for the
simulated time.

The messages for the simulation server use
[protobuf](https://github.com/google/protobuf). As it is hacked, you have to
run the message generation yourself using `protoc --cpp_out . -I . epuck.proto`
in the protobuf directory.
