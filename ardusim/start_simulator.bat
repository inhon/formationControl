@echo off
cd copter1
start arducopter -w -S --model + --speedup 1 --defaults parameters/copter.parm -I0 --home 22.9048880,120.2719823,27.48,0
cd ../copter2
start arducopter -w -S --model + --speedup 1 --defaults parameters/copter.parm -I1 --home 22.9049103,120.2721190,27.48,0
cd ../copter3
start arducopter -w -S --model + --speedup 1 --defaults parameters/copter.parm -I2 --home 22.9049276,120.2722451,27.48,0
cd ../copter4
start arducopter -w -S --model + --speedup 1 --defaults parameters/copter.parm -I3 --home 22.9049374,120.2723417,27.48,0

:: start arducopter -w -S --model + --speedup 1 --defaults parameters/copter.parm -I1
:: start arducopter -w -S --model + --speedup 1 --defaults parameters/copter.parm -I2
:: start arducopter -w -S --model + --speedup 1 --defaults parameters/copter.parm -I3