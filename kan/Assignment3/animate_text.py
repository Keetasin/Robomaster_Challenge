import time
import sys

def animate_loading(message, duration=5):
    spinner = ['.', '..', '...']
    start_time = time.time()
    while time.time() - start_time < duration:
        for dots in spinner:
            sys.stdout.write(f"\r{message}{dots}")
            sys.stdout.flush()
            time.sleep(0.5)
    print()  # ขึ้นบรรทัดใหม่หลังจากจบการแสดงผล



