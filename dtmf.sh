make clean
make
./radioafsk > /dev/null 2>&1 &
rtl_fm -f 145600000 -s 22050 -M fm | multimon-ng -a DTMF -t raw - | ./DTMF.py
