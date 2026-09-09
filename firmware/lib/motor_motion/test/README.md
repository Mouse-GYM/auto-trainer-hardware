In the motor math code, there be dragons. I've spent over a hundred hours on it, and still finding errors. This script is designed to be run on a desktop to profile the code and see/diagnose the motion profiles.

Build
===
```sh
mkdir -p build
cd build
cmake ..
make
```

Use
===
See
```sh
build/motor_motion_test --help
```
for full options. Print output of short throw stepper motion from 0 to 50 revs, with max accel of 4/revs/sec/sec, max velocity of 50 revs/sec, usual timing constraints and 1/8th stepping:
```sh
build/motor_motion_test -p -t -a 4 -v 50 -x 0 -y 50 -i 0.000005 -n 0.125 -S 48 > out.csv
```
Use this helper script to graph (as long as the output file is `out.csv`):
```sh
python3 graph_stepper_or_servo.py
```

Notice that a great deal of information goes to `stderr` but the information to `stdout` is designed to be CSV-style data.

Print the same output from above but instead track number of iterations of Halley's method:
```sh
build/motor_motion_test -N -t -a 4 -v 50 -x 0 -y 50 -i 0.000005 -n 0.125 -S 48 > out.csv
```
Use this helper script to graph the output:
```sh
python3 graph_iters.py
```

Profile a servo movement across a 0-180 degree calibration:
```sh
./motor_motion_test -p -s -a 1000 -v 10 -x 0 -y 180 -m 0 -M 180 -u 1000 -o 2000 >out.csv
```
The two angle limits are calibration endpoints rather than ordered bounds, so an inverted calibration (the
minimum angle at the longer pulse) profiles the same way:
```sh
./motor_motion_test -p -s -a 1000 -v 10 -x 90 -y 10 -m 90 -M 10 -u 200 -o 1200 >out.csv
```
You may use the same script from above to graph it:
```sh
python3 graph_stepper_or_servo.py
```

Self test
===
Check the angle-to-PWM mapping, the calibration predicates and a whole generated move without a servo
attached. It prints each failing case to `stderr` and exits with the failure count:
```sh
build/motor_motion_test --selftest
```
