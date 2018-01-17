INTEL EDISON FLIGHT CONTROLLER PROJECT

This project works in parallel with a Quadrotor TCP Runtime that is responsible for receiving messages over a TCP socket connection. The flight controller also uses the following directory structure during runtime:

```
~/

~/logging/
~/logging/csv-stats/
~/logging/pid-controller/

~/flight-controller
~/flight-controller/Flight_Controller
~/flight-controller/flight-controller.properties
~/flight-controller/tcp-client
~/flight-controller/tcp-client/QuadrotorTcpRuntime
```

The `flight-controller-startup.sh` file can be placed in the `/etc/init.d` directory to be run on intel-edison startup.
