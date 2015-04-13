#!/bin/bash
#rostopic echo move_base/status | grep -q 'status: 3' && while :;do echo 'GOAL REACHED!!!'; done
rostopic echo recognizer/output | grep -q 'data:' && dialog --title "BUD-E says" --msgbox 'GOAL REACHED' 6 20
