#!/bin/bash

if [ $1 = "b" ]; then
  echo "build"
  cd /home/thinkpot/workspace/dr_v3/Debug/
  make all
fi
if [ $1 = "f" ]; then
  echo "flash"
  /home/thinkpot/Ac6/SystemWorkbench/plugins/fr.ac6.mcu.externaltools.openocd.linux64_1.8.0.201603291120/tools/openocd/bin/openocd -f dr_v3.cfg -s /home/thinkpot/workspace/dr_v3 -s /home/thinkpot/Ac6/SystemWorkbench/plugins/fr.ac6.mcu.debug_1.8.0.201603291114/resources/openocd/scripts -c "program Debug/dr_v3.elf verify exit"
fi
if [ $1 = "bf" ]; then
  echo "build"
  cd /home/thinkpot/workspace/dr_v3/Debug/
  make all
  /home/thinkpot/Ac6/SystemWorkbench/plugins/fr.ac6.mcu.externaltools.openocd.linux64_1.8.0.201603291120/tools/openocd/bin/openocd -f dr_v3.cfg -s /home/thinkpot/workspace/dr_v3 -s /home/thinkpot/Ac6/SystemWorkbench/plugins/fr.ac6.mcu.debug_1.8.0.201603291114/resources/openocd/scripts -c "program Debug/dr_v3.elf verify exit"
fi
exit 0
