if ps | grep -v grep | grep FlightController; then
        exit 0
elif ps | grep -v grep | grep TestBed_UdpEchoServer; then
        exit 0
else
        /home/root/runtime/aws-module/scripts/s3-dir-sync.sh /home/root/runtime/aws-module/s3/
        exit 0
fi