mkdir /tmp/flight-controller
mkdir /tmp/flight-controller/logs
PATH=/usr/bin:/usr/local/bin:/usr/local/bin/aws
cronjob="* * * * * /home/root/runtime/aws-module/scripts/s3-dir-sync-driver.sh >> /tmp/flight-controller/logs/s3-cron-job.log 2>&1"
(crontab -l 2>/dev/null; echo "$cronjob") | crontab -