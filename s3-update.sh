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


FILE_LIST=/tmp/flight-controller/s3/file-list
STAGED_FILE_LIST=/tmp/flight-controller/s3/file-list-staging
SYNC_DIR=$(realpath $1)
S3_BASE_DIR=$2
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
        mkdir /tmp/flight-controller/s3
        ls -d $(find ${SYNC_DIR}) | grep '^-' > $FILE_LIST
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
echo "DIFF:"
echo $tmp_diff

# Strip diff to just the file name
staged_files=$(echo $tmp_diff | sed 's/.*@@ //')
echo "STAGED_FILES:"
echo $staged_files

if [ $(diff -u <(cat ${FILE_LIST}) <(echo "$indexed_files")) != ""]
then
        echo "Uploading updates to S3..."
        for staged_file in $staged_files
        do
                if [[ ${staged_file:0:1} == "+" ]]; then #Diff shows a new file being added
                        if [ -f ${staged_file/+/} ]; then #File to be added is not a directory and exists
                                echo "ADD ${staged_file/+/$S3_BASE_DIR}"

                                # Add line to FILE_LIST
                                echo ${staged_file/+/} >> $FILE_LIST
                        fi
                fi
                if [[ ${staged_file:0:1} == "-" ]]; then
                        #if [ -f ${staged_file/-/} ]; then #File to be removed is not a directory and exists
                              echo "DEL ${staged_file/-/$S3_BASE_DIR}"

                              # Remove line from FILE_LIST
                              #tes=${staged_file//\//\\/} #escape slashes
                              tes2=${staged_file:2:${#staged_file}}
                              sed -i "/$tes2/d" $FILE_LIST
                        #fi
                fi
        done
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
