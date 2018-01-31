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

~/flight-controller/cloud/
```

The `flight-controller-startup.sh` file can be placed in the `/etc/init.d` directory to be run on intel-edison startup.
Please update the MRAA libs on the intel edison

AWS shell requires installation of:
pip install awscli --upgrade --user
( can be upgraded via: $ pip install awscli --upgrade --user )

- 'aws' command not found?
-- `~/.local/bin/aws` add to path or make link
-- chmod +x aws bin file

In order to use AWS CLI for cloud functions
Run `aws configure` to allow access to aws account: (See context-template.txt)