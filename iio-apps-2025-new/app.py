from queue import Queue
from threading import Thread
from threading import Event
from pynput import keyboard
from pynput.keyboard import Controller
from time import sleep
import math
import iio
import time
import tkinter as tk
URI = "ip:10.76.84.233"
DEVICE = "iio_ad5592r_s"
TIMEOUT = 0.3
def init_device():
        ctx = iio.Context(URI)
        device = ctx.find_device(DEVICE)
        if device is None:
                raise ValueError("No device")
        return device
def get_data(device: iio.Device):
        channel_names = [f'voltage{i}' for i in range(6)]
        axes = ['x', 'y', 'z']
        axis_data = {
                'x': {'-': int, '+': int},
                'y': {'-': int, '+': int},
                'z': {'-': int, '+': int}
        }
        for i, channel_name in enumerate(channel_names):
                channel = device.find_channel(channel_name)
                if channel is None:
                        raise ValueError(f"Channel {channel_name} not found")
                raw_value = int(channel.attrs['raw'].value) #E ok logica de calcul?
                axis = axes[i // 2]
                sign = '+' if i % 2 == 0 else '-'
                axis_data[axis][sign] = raw_value
        # print("Axis data:", axis_data)
        return axis_data
def get_roll_pitch(data):
        x_accel = float(data['x']['+']) - float(data['x']['-'])
        y_accel = float(data['y']['+']) - float(data['y']['-'])
        z_accel = float(data['z']['+']) - float(data['z']['-'])
        roll = math.atan2(-x_accel, math.sqrt(y_accel**2 + z_accel**2)) * 180 / math.pi
        pitch = math.atan2(y_accel, z_accel) * 180 / math.pi
        return roll, pitch
def get_movement(roll, pitch):
        movement = {'left': 0, 'right': 0, 'front': 0, 'back': 0,}
        limit = 45
        if roll < 0:
                movement['left'] = min(1., abs(roll) / limit)
        else:
                movement['right'] = min(1., abs(roll) / limit)
        if pitch < 0:
                movement['front'] = min(1., abs(pitch) / limit)
        else:
                movement['back'] = min(1., abs(pitch) / limit)
        #print("Movement:", movement)
        return movement
def simulate_key(key, on_time):
    kb_controller = Controller()
    if on_time > 0:
        kb_controller.press(key)
        time.sleep(on_time)
        kb_controller.release(key)
def start_iio(device: iio.Device):
        start = time.time()
        data = get_data(device)
        roll, pitch = get_roll_pitch(data)
        movement = get_movement(roll, pitch)
        timer = time.time() - start
        for direction, key in [('front', 'w'), ('back', 's'), ('left', 'a'), ('right', 'd') ]:
                value = movement[direction]
                if value > 0.2:
                        on_time = TIMEOUT * value
                        key_thread = Thread(target = simulate_key, args=(key, on_time))
                        key_thread.daemon = True
                        key_thread.start()
        time.sleep(TIMEOUT - timer)
        return movement # AICI AM MODIFICAT
def create_gui(queue):
    root = tk.Tk()
    root.title("Input")

    canvas = tk.Canvas(root, width=400, height=400)
    canvas.pack()

    square_size = 100
    squares = {
        'left':   canvas.create_rectangle(50,150,50+square_size,150+square_size, fill='white'),
        'right':  canvas.create_rectangle(250,150,250+square_size,150+square_size, fill='white'),
        'front':  canvas.create_rectangle(150,50,150+square_size,50+square_size, fill='white'),
        'back':   canvas.create_rectangle(150,250,150+square_size,250+square_size, fill='white'),
    }

    def update_sq_color(canvas, sq_id, value):
        intensity = int(255 * value)
        color = f'#{intensity:02x}0000'  
        canvas.itemconfig(sq_id, fill=color)

    def update_gui():
        try:
            movement = queue.get_nowait()
            for direction in ['left','right','front','back']:
                update_sq_color(canvas, squares[direction], movement[direction])
        except:
            pass  

        root.after(int(TIMEOUT*1000), update_gui)

    update_gui()
    root.mainloop()

        # while not exit_event.is_set():
                # if start_event.is_set():
                        # mvm = queue.get()
                        # print("Current movement:", mvm)
                        # queue.put(mvm)
                        # sleep(1)
                # start_event.wait()
def simulate_movement(queue):
        device = init_device()
        while not exit_event.is_set():
                if start_event.is_set():
                        queue.put(start_iio(device))
                        # print("Simulating movement...")
                        # mvm = queue.get()
                        # mvm['left'] += 1
                        # queue.put(mvm)
                        # sleep(1)
                start_event.wait()
#
def on_keypress(key):
    if key == keyboard.Key.ctrl_l:
        if (start_event.is_set()):
            start_event.clear()
            print("Pause")
            return True
        else:
            start_event.set()
            print("Start")
            return True
    elif key == keyboard.Key.esc:
        exit_event.set()
        print("Exit")
        return False
if __name__ == "__main__":
        start_event = Event()
        exit_event = Event()
        init_movements = {'left': 0, 'right': 0, 'front': 0, 'back': 0 }
        movement_q = Queue()
        movement_q.put(init_movements)
        gui_thread = Thread(target=create_gui, args=(movement_q,))
        gui_thread.daemon = True
        gui_thread.start()
        iio_thread = Thread(target=simulate_movement, args=(movement_q,))
        iio_thread.daemon = True
        iio_thread.start()
        print("Press Left CTRL so start/stop application. Press ESC to exit")
        with keyboard.Listener(on_press=on_keypress) as listener:
                listener.join()