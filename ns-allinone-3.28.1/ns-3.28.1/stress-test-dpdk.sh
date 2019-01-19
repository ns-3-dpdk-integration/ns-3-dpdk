while (true) do
    ./test-dpdk.sh
    if [ $? -eq 0 ]
    then
        echo "*********************************";
        echo "***** Successfully executed *****";
        echo "*********************************";
    else
        echo "xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx";
        echo "xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx";
        echo "xxxxxxxxxxxxx Unuccessfully executed xxxxxxxxxxxxx";
        echo "xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx";
        echo "xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx";
        exit 1
    fi
done;