import os
import sys
import subprocess

TESTS = {
	'solenoid': 'solenoid_test',
	'rfid': 'RFID_buffer_testing',
	'infrared': 'infrared_test',
	'encoder': 'encoder_test',
	'camera': 'camera_test',
	'motor': 'motor_test'
}


def run_test(part):
    module = f"src.tests.{TESTS[part]}"
    command = ["python3", "-m", module]
    try:
        subprocess.run(command, check=True)
    except subprocess.CalledProcessError as e:
        print(f"Error running {module}: {e}")


def run():
    module = "src.piPaw_pert"
    command = ["python3", "-m", module]
    try:
        subprocess.run(command, check=True)
    except subprocess.CalledProcessError as e:
        print(f"Error running {module}: {e}")


def main():
    if sys.argv[1] == "test":
        arg = sys.argv[2]
        run_test(arg)

    if sys.argv[1] == "run":
        run()


if __name__ == "__main__":
    main()
