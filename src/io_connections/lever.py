import multiprocessing as mp

from time import time, sleep

from src.constants import *
from src.io_connections.encoder import Encoder
from src.io_connections.motor import Motor

class Lever(Encoder):
    def __init__(self, csx, clk, btmd):
        self.motor = Motor()
        super().__init__(csx, clk, btmd)
        self.motor.disable()

        # Set up namespace
        self.manager = mp.Manager()
        self.ns = self.manager.Namespace()
        self.ns.position_list = self.manager.list()
        self.ns.past_threshold_pos0 = False
        self.ns.past_threshold_pos1 = False
        self.ns.past_threshold_pos2 = False
        self.ns.completed = False
        self.ns.pert = 0
        self.ns.time_in_range = time()
        self.ns.grace_period = False
        self.ns.entered_range = False
        self.ns.pert_force = 0
        self.ns.start_time = None

    def get_position(self):
        """
        Returns the current position of the lever.
        """
        return self.read_counter()

    def record_current_position(self, _time):
        """
        Records the current position of the lever.
        :params time: The time (elapsed since the start of the trial) at which the position was recorded.
        """
        position = self.get_position()
        self.ns.position_list.append([max(_time, 0), position, self.ns.pert_force])
        return position

    def write_position(self, mouse, event_time, event):
        """
        Writes lever position data from a trial to the lever position file for the current mouse.

        Args:
        event_time (datetime): The time of the event.
        lever_positions (list): List of lever positions recorded by record_lever_pos function.
        event (str): String containing the event code.
        """
        if mouse is None:
            return

        with open(mouse.lever_position_file, 'a', encoding='utf-8') as file:
            file.write(f'{event_time}\t{event}\n')
            first_index = 0
            
            for i, lever_position in enumerate(self.ns.position_list):
                if lever_position[1] > THRESHOLD_POSITION_ZERO[mouse.phase]:
                    if i > 100:
                        first_index = i - 100
                    else:
                        first_index = 0
                    break

            # Write position data to file
            for i in range(first_index, len(self.ns.position_list)):
                line = self.ns.position_list[i]
                file.write(f'{str(line[0])}\t{str(line[1])}\t{str(line[2])}\n')

        print("Lever position saved to file.")


    def complete_trial(self):
        print("Call to Complete Trial")
        self.motor.set_high_duty_cycle()
        self.clear_counter()
        self.ns.position_list = self.manager.list()
        self.ns.past_threshold_pos0 = False
        self.ns.past_threshold_pos1 = False
        self.ns.past_threshold_pos2 = False
        self.ns.completed = False
        self.ns.pert = 0
        self.ns.time_in_range = time()
        self.ns.grace_period = False
        self.ns.entered_range = False
        self.ns.pert_force = 0
        self.ns.start_time = None

    def set_starting_position(self):
        self.motor.set_low_duty_cycle()
        self.motor.enable()
        sleep(0.5)
        self.clear_counter()
        self.motor.enable(False)
