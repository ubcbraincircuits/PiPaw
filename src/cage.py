import RPi.GPIO as GPIO
import multiprocessing as mp

from datetime import datetime
from time import time, sleep, mktime, strptime

from src.constants import *
from src.mouse import Mouse
from .io_connections import Camera, Infrared, Solenoid, RFID, Buzzer, Lever

MOUSED_DATA_FILE = "../data/mice.cfg"


class Cage:

    # =============================
    # Setup
    # =============================

    def __init__(self):
        # GPIO Setup
        self.gpio_setup()
        # IO Connections
        self.camera = Camera()
        self.infrared = Infrared()
        self.solenoid = Solenoid()
        self.rfid = RFID()
        self.buzzer = Buzzer()
        self.lever = Lever(CSX, CLK, BYTEMD)
        # Mouse Setup
        self.mice = []
        self.current_mouse = None
        self.data_loaded = False
        # Trial Stuff
        self.trial = {
            "ongoing": False,
            "time_mouse_gone": None,
            "datetime_mouse_last_detected": None,
        }
        self.trial_ongoing = False
        self.record_lever_position_process = None
        # If mouse leaves before trial is over
        self.trial_time_mouse_gone = None
        self.trial_datetime_mouse_last_detected = None
        # Read Mouse Config
        self.read_mouse_config()

    def gpio_setup(self):
        GPIO.setmode(GPIO.BCM)

    def read_mouse_config(self):
        """
        Reads the mouse configuration from the config file.
        """
        with open(MOUSED_DATA_FILE, "r", encoding='utf-8') as f:
            for line in f:
                mouse_data = line.replace('\n', '')
                mouse_data = mouse_data.split('\t')
                mouse = Mouse(
                    mouse_data[0],
                    name=mouse_data[1],
                    mouse_day=int(mouse_data[2]),
                    daily_water=int(mouse_data[3]),
                    reward_pulls=int(mouse_data[4]),
                    failed_pulls=int(mouse_data[5]),
                    tot_reward_pulls=int(mouse_data[6]),
                    tot_failed_pulls=int(mouse_data[7]),
                    tot_days=int(mouse_data[8]),
                    entrance_rewards=int(mouse_data[9]),
                    hold_time=float(mouse_data[10]),
                    ht_trials=int(mouse_data[11]),
                    pert_trials=int(mouse_data[12]),
                    phase=int(mouse_data[13]),
                    free_water_remained=int(mouse_data[14]),
                )
                self.mice.append(mouse)

    # =============================
    # Utility Functions
    # =============================

    def init_record_lever_position_process(self):
        """
        Initializes the record lever position process.
        """
        self.record_lever_position_process = mp.Process(target=self.record_lever_position)
        self.record_lever_position_process.start()

    def terminate_record_lever_position_process(self):
        """
        Terminates the record lever position process.
        """
        while self.record_lever_position_process.is_alive():
            self.record_lever_position_process.terminate()

    def record_lever_position(self):
        """
        Takes in an empty list and appends the current encoder position and time to it at a
        certain rate (based on specified sampling rate). Also checks if success range/fail
        range has been reached.
        """
        if self.current_mouse == None:
            return

        time_last = time()
        lever_position = self.lever.get_position()
        time_elapsed = 0
        self.lever.record_current_position(time_elapsed)

        if self.lever.get_position() <= THRESHOLD_POSITION_ZERO[self.current_mouse.phase]:
            while(time()-time_last < 1.0/SAMPLING_RATE):
                sleep(0.0001)
                time_last = time()

        trial_time_start = time()
        trial_time_range = None
        trial_time_grace = None
        self.lever.past_threshold_pos0 = True
        self.trial_ongoing = self.lever.past_threshold_pos0
        self.lever.ns.start_time = time()

        while self.trial_ongoing:
            time_last = time()
            time_elapsed = time() - trial_time_start
            lever_position = self.lever.record_current_position(time_elapsed)
            trial_time_range, trial_time_grace  = self.process_trial_phase(self.current_mouse.phase,
                                                                           lever_position,
                                                                           trial_time_range,
                                                                           trial_time_grace)

            if self.trial_ongoing:
                self.sleep_until_next_sample(time_last)

    def sleep_until_next_sample(self, last_sample_time):
        """
        Sleeps until it's time for the next sample.
        :param last_sample_time: Time when the last sample was taken.
        :param sampling_rate: The sampling rate in Hz.
        """
        while (time.time() - last_sample_time < 1.0 / SAMPLING_RATE):
            time.sleep(0.0001)

    def time_out(self, aht,rht):
        timeout = 5 - 4*(aht/(rht+0.000001))
        if timeout < 1:
            timeout = 0.5
        return timeout

    # =============================
    # Cage Lifecycle Handlers
    # =============================

    def cleanup(self):
        if self.current_mouse is not None:
            if self.lever.ns.past_threshold_pos0:
                self.terminate_record_lever_position_process()
            self.current_mouse.cleanup()
        GPIO.cleanup()
        self.buzzer.cleanup()
        self.lever.close()
        self.camera.camera.close()
        self.camera.stream.clear()

    # =============================
    # Trial Lifecycle Handlers
    # =============================

    def process_trial_phase(self, phase, position_reading, trial_time_range, trial_time_grace):
        """
        Processes the trial based on the current phase and updates the namespace accordingly.
        :param ns: Namespace shared among processes.
        :param position_reading: Current reading of the lever position.
        :param pos1: First position threshold.
        :param pos2: Second position threshold.
        :param phase: Current phase of the experiment.
        """
        if phase == 1:
            return self.handle_phase_one_conditions(position_reading,
                                                    trial_time_range,
                                                    trial_time_grace)
        elif phase == 2:
            return self.handle_phase_two_conditions(position_reading,
                                                    trial_time_range,
                                                    trial_time_grace)
        elif phase == 3:
            return self.handle_phase_three_conditions(position_reading,
                                                      trial_time_range,
                                                      trial_time_grace)

    def handle_phase_one_conditions(self, position_reading, trial_time_range, _):
        if not self.lever.ns.past_threshold_pos1 and position_reading > THRESHOLD_POSITION_ONE:
            trial_time_range = time()
            self.lever.ns.past_threshold_pos1 = True
        if self.lever.ns.past_threshold_pos1 and position_reading > THRESHOLD_POSITION_TWO:
            self.lever.ns.time_in_range = time() - trial_time_range
            self.lever.ns.past_threshold_pos2 = True
            self.lever.ns.trial_ongoing = False
        elif self.lever.ns.past_threshold_pos1 and position_reading <= THRESHOLD_POSITION_ONE:
            self.lever.ns.time_in_range = time() - trial_time_range
            self.lever.ns.trial_ongoing = False
        elif position_reading <= THRESHOLD_POSITION_ZERO[1]:
            self.lever.ns.time_in_range = 0.0
            self.lever.ns.trial_ongoing = False
        return trial_time_range, None

    def handle_phase_two_conditions(self, position_reading, trial_time_range, _):
        if not self.lever.ns.past_threshold_pos1 and position_reading > THRESHOLD_POSITION_ONE:
            trial_time_range = time()
            self.lever.ns.past_threshold_pos1 = True
        if self.lever.ns.past_threshold_pos1 and position_reading > THRESHOLD_POSITION_TWO:
            self.lever.ns.time_in_range = time() - trial_time_range
            self.lever.ns.past_threshold_pos2 = True
            self.lever.ns.trial_ongoing = False
        elif self.lever.ns.past_threshold_pos1 and position_reading <= THRESHOLD_POSITION_ONE:
            self.lever.ns.time_in_range = time() - trial_time_range
            self.lever.ns.trial_ongoing = False
        elif position_reading <= THRESHOLD_POSITION_ZERO[2]:
            self.lever.ns.time_in_range = 0.0
            self.lever.ns.trial_ongoing = False
        return trial_time_range, None

    def handle_phase_three_conditions(self, position_reading, trial_time_range, trial_time_grace):
        if not self.lever.ns.past_threshold_pos1 and not self.lever.ns.entered_range and position_reading > THRESHOLD_POSITION_ONE:
            self.lever.ns.entered_range = True
            trial_time_range = time()
            self.lever.ns.past_threshold_pos1 = True
        if self.lever.ns.past_threshold_pos1 and position_reading > THRESHOLD_POSITION_TWO:
            self.lever.ns.time_in_range = time() - trial_time_range
            if not self.lever.ns.grace_period and self.lever.ns.time_in_range >PERTURBATION_WAIT:
                self.lever.ns.grace_period = True
                trial_time_grace = time()
            if self.lever.ns.grace_period and time() - trial_time_grace > T_LEVGRACE:
                self.lever.ns.past_threshold_pos2 = True
                self.lever.ns.grace_period = False
                self.lever.ns.trial_ongoing = False
            if not self.lever.ns.grace_period and self.lever.ns.time_in_range < PERTURBATION_WAIT:
                self.lever.ns.past_threshold_pos2 = True
                self.lever.ns.grace_period = False
                self.lever.ns.trial_ongoing = False
        elif self.lever.ns.past_threshold_pos1 and position_reading <= THRESHOLD_POSITION_ONE:
            self.lever.ns.time_in_range = time() - trial_time_range
            if not self.lever.ns.grace_period and self.lever.ns.time_in_range > PERTURBATION_WAIT:
                self.lever.ns.grace_period = True
                trial_time_grace = time()
            if self.lever.ns.grace_period and time() - trial_time_grace > T_LEVGRACE:
                self.lever.ns.trial_ongoing = False
        elif position_reading <= THRESHOLD_POSITION_ZERO[3]:
            self.lever.ns.time_in_range = 0.0
            self.lever.ns.trial_ongoing = False
        return trial_time_range, trial_time_grace

    def start_trial(self, datetime_start, event, hold_time, trial_time, pert_force=0.0):
        if self.current_mouse.phase == 1:
            return

    def end_trial(self, datetime_start, event, hold_time, trial_time, outcome, pert_force=0.0):
        """
        Ends a trial when success or fail condition has been met. Returns lever to start position,
        records data and updates variables.
        - datetimeStart is a datetime object
        - trialTime is a float indicating the length of the trial
        - event is a string indicating the event code
        - outcome is a string indicating if trial was successful, failed or cancelled
        """
        trial_time_finished = time()
        self.lever.motor.ramp()
        self.current_mouse.record_data(datetime_start, event, trial_time, pert_force)

        self.camera.stop()

        if outcome == 'success':
            self.buzzer.play_tone('Hi')
            self.current_mouse.reward_pulls += 1
            self.current_mouse.tot_reward_pulls += 1
            self.current_mouse.daily_water += 1
            print(f"Successful trial: code {event}. {self.current_mouse.name} has \
                  {self.current_mouse.reward_pulls} successful trial(s) today, \
                  {self.current_mouse.tot_reward_pulls} in total ({datetime_start}).")
            self.camera.write_video(self.current_mouse.name, datetime_start, event)
            self.lever.write_position(self.current_mouse, datetime_start, event)

        if outcome == 'failed':
            self.buzzer.play_tone('Low', duration=0.5)
            self.current_mouse.failed_pulls += 1
            self.current_mouse.tot_failed_pulls += 1
            print(f"Failed trial: code {event}. {self.current_mouse.mouse_name} has \
                  {self.current_mouse.failed_pulls} failed trial(s) today, \
                  {self.current_mouse.tot_failed_pulls} in total ({datetime_start}).")
            self.camera.write_video(self.current_mouse.name, datetime_start, event)
            self.lever.write_position(self.current_mouse, datetime_start, event)

        # Also if outcome is cancelled
        self.camera.stream2.clear()
        self.camera.start_stream2()
        self.lever.complete_trial()

        # Checking if phase has changed
        if self.current_mouse.phase == 1 and self.current_mouse.tot_reward_pulls >= N_TRAINING:
            print(f"{self.current_mouse.name} has reached {self.current_mouse.tot_reward_pulls} \
                    successful trials. Changing to main stage.")
            self.current_mouse.update_and_analyze_hold_time(INITIAL_HT)
            self.current_mouse.phase = 2
            self.current_mouse.record_data(datetime.now(), '66')
        elif self.current_mouse.phase == 2:
            self.current_mouse.ht_trials += 1
            # if hold_time < MAX_HT and ht_trials >= N_SUCCESS_CHECK:
            print(f"{self.current_mouse.name} has {self.current_mouse.ht_trials} \
                    trials at {hold_time:.2f} second hold time.")

        print(f"Recorded trial data and reset variables in {time() - trial_time_finished} seconds.")

        if not self.rfid.out_of_range():
            self.trial_time_mouse_gone = time()
            self.trial_datetime_mouse_last_detected = datetime.now()

    def get_mouse_with_tag(self, tag):
        for mouse in self.mice:
            if mouse.tag == tag:
                return mouse
        return None

    # =============================
    # Mouse handlers
    # =============================
    def enters_mouse(self):
        tag_id = self.rfid.get_serial()
        self.has_mouse_changed(tag_id)
        self.lever.motor.set_high_duty_cycle()
        self.lever.motor.enable()
        self.current_mouse = self.get_mouse_with_tag(tag_id)
        self.current_mouse.update()
        self.current_mouse.record_data(datetime.now(), '00')
        self.camera.start()
        print("Data loaded and motor enabled. Waiting for nose poke...")

    def switch_mouse(self, tag_id):
        self.camera.stop()
        self.camera.stream.clear()
        self.current_mouse.record_data(
            self.trial_datetime_mouse_last_detected, '99', self.current_mouse.hold_time) # 99 is the mouse exit code
        self.current_mouse.save()
        self.current_mouse = self.get_mouse_with_tag(tag_id)
        self.current_mouse.update()
        self.current_mouse.record_data(datetime.now(), '00') # 00 is the mouse entrance code
        print("Waiting for nose poke...")
        self.camera.start()

    def wait_for_mouse_to_exit_cage(self):
        self.lever.motor.enable()
        self.lever.motor.set_high_duty_cycle()
        print("Mouse in chamber during program start. Waiting for mouse to exit...")
        time_start = time()
        while self.is_mouse_in_chamber():
            sleep(0.1)
        self.lever.motor.set_low_duty_cycle()
        sleep(0.1)
        self.lever.motor.enable(False)
        print(f"Chamber cleared in {time() - time_start:.2f} seconds.")

    # =============================
    # Checks
    # =============================

    def is_mouse_in_chamber(self):
        return self.rfid.rfid_detected() or self.infrared.beam_broken()

    def is_mouse_nose_poking(self):
        return self.infrared.beam_broken()

    def is_same_mouse_tag(self, tag_id):
        return tag_id == self.current_mouse.tag

    def is_same_mouse(self, mouse):
        """
        Checks to see if the current mouse is the same as the previous ones.
        """
        return self.current_mouse is mouse

    def has_entrance_reward(self):
        """
        Checks if animal has reached interval to get another entrance reward (if in training 1)
        """
        with open(self.current_mouse.data_file, 'r', encoding='utf-8') as file:
            events = file.readlines()

        for x in range(len(events)-1, 0, -1):
            event = events[x]
            event = event.replace('\n', '')
            event = event.split('\t')

            if event[1] == '01':
                last_drop_time = mktime(strptime(event[0], "%Y-%m-%d %H:%M:%S.%f"))
                interval = time() - last_drop_time
                if interval > T_ENTRANCE_REWARD_INTERVAL:
                    print(f"{int(interval)} seconds since last entrance reward given. \
                        Delivering entrance reward.")
                    return True
                print(f"Insufficient time has passed since last entrance reward \
                    ({int(interval)} seconds)")
                return False
        print("Delivering first entrance reward.")
        return True

    def has_mouse_changed(self, tag):
        for mouse in self.mice:
            if mouse.tag == tag:
                if self.is_same_mouse(mouse):
                    print(
                        f'Mouse has changed! {mouse.name}:{mouse.tag} entered the chamber ({datetime.now()}).')
                    return True
                else:
                    print("Mouse has not changed.")
                    return False

            # If data is not loaded, we are just checking to make sure mouse name is in
            # mice.cfg, don't need to return anything
            print(
                f'{mouse.name}:{mouse.tag} entered the chamber ({datetime.now()}).')
            return

        # If has_mouse_changed gets through the full mice.cfg file without finding the
        # mouse name, should raise a value error
        print("Mouse ID = " + tag)
        raise ValueError("Mouse name not found in mice.cfg.")
