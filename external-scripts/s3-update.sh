#!/bin/bash

function indexFilesForSyncDirectory {
rm $STAGED_FILE_LIST
shopt -s globstar
for file in ${SYNC_DIR}/**
do
        if [ -f "$file" ]; then
                echo $file >> $STAGED_FILE_LIST
        fi
done
}

PERSISTANT_FILE_LIST=~/runtime/aws-module/tracked-files.txt
FILE_LIST=/tmp/flight-controller/s3/file-list
STAGED_FILE_LIST=/tmp/flight-controller/s3/file-list-staging
SYNC_DIR=$(realpath $1)
S3_BASE_DIR=s3://mldelarosa-thesis/edison-quadcopter
LOCAL_BASE_DIR=$SYNC_DIR

echo "Monitoring directory [${SYNC_DIR}] for changes"
echo "Copying to S3 Base Directory [${S3_BASE_DIR}]"
echo "Working from Local Directory [${LOCAL_BASE_DIR}]"

# try to open file_list in tmp, otherwise store existing files in new tmp file
if [ -e ${FILE_LIST} ]; then
        echo "Found file list"
        sort -u -o $FILE_LIST  $FILE_LIST #Sort and list unique entries
        cat $FILE_LIST
else
        echo "Creating file list"
        if [ ! -d "/tmp/flight-controller" ]; then
                mkdir /tmp/flight-controller
        fi
        if [ ! -d "/tmp/flight-controller/s3" ]; then
                mkdir /tmp/flight-controller/s3
        fi
        if [ -e ${PERSISTANT_FILE_LIST} ]; then
                echo "Found record of staged files ${PERSISTANT_FILE_LIST}"
                cp $PERSISTANT_FILE_LIST $FILE_LIST
        else
                echo "Creating persistant record of staged files..."
                ls -d $(find ${SYNC_DIR}) | grep '^-' > $FILE_LIST
        fi

fi

if [ -e $STAGED_FILE_LIST ]; then
        echo "Found staged file list..."
        sort -u -o $STAGED_FILE_LIST $STAGED_FILE_LIST #Sort and list unique entries
        cat $STAGED_FILE_LIST
else
        echo "Creating staged file list..."
        touch $STAGED_FILE_LIST
fi
echo "======================"

# Search directory for all files recursively and store them in STAGED FILES LIST
indexFilesForSyncDirectory
tmp_diff=$(diff $FILE_LIST $STAGED_FILE_LIST)
#echo "DIFF:"
#echo $tmp_diff

# Strip diff to just the file name
staged_files=$(echo $tmp_diff | sed 's/.*@@ //')
echo "STAGED_FILES:"
echo $staged_files

if [ $(diff -u <(cat ${FILE_LIST}) <(echo "$indexed_files")) != ""]
then
        did_update_s3=true
        echo "Uploading updates to S3..."
        for staged_file in $staged_files
        do
                if [[ ${staged_file:0:1} == "+" ]]; then #Diff shows a new file being added
                        if [ -f ${staged_file/+/} ]; then #File to be added is not a directory and exists
                                staged_file_path=${staged_file/+/}
                                echo "ADD $staged_file_path to ${staged_file_path/$SYNC_DIR/$S3_BASE_DIR}"
                                /usr/local/bin/aws s3 cp $staged_file_path ${staged_file_path/$SYNC_DIR/$S3_BASE_DIR}
                                if [ $? -eq 0 ]; then
                                        echo "OK"
                                        #Add line to FILE_LIST
                                        echo ${staged_file/+/} >> $FILE_LIST
                                        echo ${staged_file/+/} >> $PERSISTANT_FILE_LIST
                                else
                                        did_update_s3=false;
                                        echo "Failed to upload ${staged_file_path/$SYNC_DIR/$S3_BASE_DIR}"
                                fi
                        fi
                fi

                if [[ ${staged_file:0:1} == "-" ]]; then
                        # No need for check since file DNE
                        #if [ -f ${staged_file/-/} ]; then #File to be removed is not a directory and exists
                                staged_file_path=${staged_file/-/}
                                echo "DEL $staged_file_path from ${staged_file_path/$SYNC_DIR/$S3_BASE_DIR}"
                                /usr/local/bin/aws s3 rm ${staged_file_path/$SYNC_DIR/$S3_BASE_DIR}
                                if [ $? -eq 0 ]; then
                                        echo "OK"
                                        #Remove line from FILE_LIST
                                        temp_escaped_slashes=${staged_file//\//\\/}
                                        temp_trimmed=${temp_escaped_slashes:3:${#staged_file}}
                                        sed -i "/$temp_trimmed/d" $FILE_LIST
                                        sed -i "/$temp_trimeed/d" $PERSISTANT_FILE_LIST
                                else
                                        did_update_s3=false;
                                        echo "Failed to upload ${staged_file_path/$SYNC_DIR/$S3_BASE_DIR}"
                                fi
                        #fi
                fi
        done
        if [ "$did_update_s3" = true ]; then
                echo "Upload succeeded, updating persistant storage..."
                #cp $FILE_LIST $PERSISTANT_FILE_LIST
        else
                echo "Update to S3 FAILED"
        fi
        cp $FILE_LIST $PERSISTANT_FILE_LIST
echo "Finished updating S3"
fi

echo "DONE"
#shopt -s globstar # allow search by '*'
#for file in ${SYNC_DIR}/**
#do
#       if [ -f "$file" ]; then
#               echo "${file/$LOCAL_BASE_DIR/$S3_BASE_DIR}"
#       fi
#done
root@edison:~/runtime/aws-module/scripts# login as: root
root@edison:~/runtime# aw
-sh: aw: command not found
root@edison:~/runtime# cd aws-module/
root@edison:~/runtime/aws-module# ls
s3  scripts  tracked-files.txt
root@edison:~/runtime/aws-module# cd scripts/
root@edison:~/runtime/aws-module/scripts# ls
cron-driver-s3-sync.sh   s3-dir-sync-driver.sh  s3-dir-sync.sh~
cron-driver-s3-sync.sh~  s3-dir-sync.sh         setup-s3-env.sh
root@edison:~/runtime/aws-module/scripts# cat s3-dir-sync-driver.sh
if ps | grep -v grep | grep FlightController; then
        exit 0
elif ps | grep -v grep | grep TestBed_UdpEchoServer; then
        exit 0
else
        /home/root/runtime/aws-module/scripts/s3-dir-sync.sh /home/root/runtime/aws-module/s3/
        exit 0
fi