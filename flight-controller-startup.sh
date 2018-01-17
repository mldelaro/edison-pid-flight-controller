#!/bin/bash

set -e
cd ~/flight-controller/tcp-client
trap "trap - SIGTERM && kill -- -$$" SIGINT SIGTERM EXIT
./QuadrotorTcpRuntime & .././Flight_Controller &
wait