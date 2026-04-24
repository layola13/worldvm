import ctypes
from worldvm import WorldVM

vm = WorldVM()
# Entity 0 is apple. Let's see if we can get its flags if we had an export.
# Since we don't have an export for entity properties, let's try entity 5 (floor) 
# which is definitely static, and compare with apple.

vm.spawn(0, 16, 20, 16) # Apple
vm.run(1)
v = (ctypes.c_float * 3)()
vm.lib.get_instance_velocity(0, v)
print(f"Apple Velocity after 1 tick: {list(v)}")

vm.lib.apply_impulse(0, 0, 100, 0)
vm.lib.get_instance_velocity(0, v)
print(f"Apple Velocity after impulse: {list(v)}")

vm.close()
