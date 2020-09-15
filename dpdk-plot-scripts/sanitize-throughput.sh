cat $1 | grep -ve ' 0.00-[^0]' | grep -oe '[0-9.]* Mbits/sec' | grep -oe '[0-9.]*'
