import ctypes
import os
import sys

if sys.platform == "win32":
    lib_name = "worldvm.dll"
    lib_dir = os.path.join("zig-out", "bin")
else:
    lib_name = "libworldvm.so"
    lib_dir = os.path.join("zig-out", "lib")
lib_path = os.path.join(os.path.dirname(__file__), lib_dir, lib_name)

class TraceEntry(ctypes.Structure):
    _fields_ = [
        ("tick_id", ctypes.c_uint32),
        ("event_type", ctypes.c_char * 32),
        ("instance_id", ctypes.c_uint16),
        ("detail", ctypes.c_char * 64),
    ]

class WorldVM:
    def __init__(self):
        self.lib = ctypes.CDLL(lib_path)
        self.lib.init_kernel.restype = ctypes.c_int
        self.lib.spawn_instance.argtypes = [ctypes.c_uint16, ctypes.c_int32, ctypes.c_int32, ctypes.c_int32]
        self.lib.run_ticks.argtypes = [ctypes.c_uint32]
        self.lib.get_emotion_valence.restype = ctypes.c_int8
        self.lib.get_emotion_arousal.restype = ctypes.c_uint8
        self.lib.get_trace_count.restype = ctypes.c_uint32
        self.lib.get_trace_entry.argtypes = [ctypes.c_uint32]
        self.lib.get_trace_entry.restype = ctypes.POINTER(TraceEntry)
        if self.lib.init_kernel() < 0: raise RuntimeError("Init failed")

    def spawn(self, eid, x, y, z): self.lib.spawn_instance(eid, x, y, z)
    def run(self, t=1): return self.lib.run_ticks(t)
    def emotions(self): return {"v": self.lib.get_emotion_valence(), "a": self.lib.get_emotion_arousal()}
    def events(self):
        return [{"t": self.lib.get_trace_entry(i).contents.tick_id, 
                 "type": self.lib.get_trace_entry(i).contents.event_type.decode().rstrip('\0'),
                 "id": self.lib.get_trace_entry(i).contents.instance_id} 
                for i in range(self.lib.get_trace_count())]
    def close(self): self.lib.shutdown_kernel()

if __name__ == "__main__":
    vm = WorldVM()
    vm.spawn(5, 0, 0, 0)     # Floor
    vm.spawn(3, 10, 5, 15)   # Glass
    vm.spawn(2, 10, 7, 15)   # Hammer (Drop from y=7 to hit glass at y=5 quickly)
    
    print("Initial:", vm.emotions())
    for i in range(20):
        vm.run(1)
        evs = vm.events()
        if evs:
            print(f"Tick {i} Events:", evs)
            break
    print("Final:", vm.emotions())
    vm.close()
