import os
from eposhandle import EPOSHandle
import time
import pynput


epos = EPOSHandle(3000, 1000, 4000)


epos.connect_to_device()
os.system('spd-say "Connected"')

time.sleep(1)

epos.enable()
os.system('spd-say "Enabled"')

time.sleep(1)

try:
    def on_click(x,y,button, pressed):
        if button == pynput.mouse.Button.left:
            if pressed:
                epos.move_to(35)
            else:
                epos.move_to(0)
        elif button == pynput.mouse.Button.right:
            if pressed:
                epos.move_to(-35)
            else:
                epos.move_to(0)

    listener = pynput.mouse.Listener(
        on_click = on_click);
    listener.start()
    listener.join()
except KeyboardInterrupt:
    pass
finally:
    time.sleep(2)
    epos.disable()
    os.system('spd-say "Disabled"')

    epos.disconnect_device()
