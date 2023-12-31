#!/usr/bin/python3
import time
from src.io_connections.lever import Encoder

def main():
    try:
        # Settings for Right Encoder class
        CSX = 0
        CLK = 1000000
        BYTEMD = 4

        lever = Encoder(CSX, CLK, BYTEMD)

        while True:
            try:
                counter_value = lever.read_counter()
                print(f'Right Counter reads [{counter_value}].', end='\r', flush=True)
                time.sleep(0.1)
            except KeyboardInterrupt:
                break
    except Exception as e:
        print(f"An error occurred: {str(e)}")
    finally:
        if 'lever' in locals():
            lever.close()
        print('Goodbye!')

if __name__ == "__main__":
    main()
