#!/bin/bash
sudo gcc -o stard stard.c -lasound

retval=$?
if [ $retval -ne 0 ]; then
    echo "########################################"
    echo "######## Compilation Failed : $retval #########"
    echo "########################################"
else
    sudo ./stard
fi



echo "########################################"

exit $retval
