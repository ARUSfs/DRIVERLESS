import os
from eposhandle import EPOSHandle
import time


epos = EPOSHandle(3000, 1000, 4000)


epos.connect_to_device()
os.system('spd-say "Connected"')

time.sleep(1)

epos.enable()
os.system('spd-say "Enabled"')

time.sleep(1)

for i in range(5):
    os.system(f'spd-say "{5-i}"')
    time.sleep(1)

epos.move_to(35)
for i in range(7):
    os.system(f'spd-say "{7-i}"')
    time.sleep(1)
epos.move_to(0)

time.sleep(2)
epos.disable()
os.system('spd-say "Disabled"')

epos.disconnect_device()
