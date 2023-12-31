import os
import numpy as np

from datetime import datetime, timedelta
from time import time, sleep, mktime, strptime

TASK_SUCCESS_CODES = ['02', '03', '04']
TASK_FAIL_CODES = ['53', '54']

MOUSED_DATA_FILE = "../data/mice.cfg"

# Maximum hold time mice can advance to
MAX_HT = 1.0
REQ_SUCCESS_RATE = 0.5
VALID_PHASES = [1, 2, 3]

THRESHOLD_POSITION = {
    1: 34,
    2: 66,
    3: 66
}

class Mouse:
    def __init__(self, tag, **kwargs):
        # Mouse Properties
        self.tag = tag
        self.name = kwargs.get('name')
        self.mouse_day = kwargs.get('mouse_day')
        self.daily_water = kwargs.get('daily_water')
        self.reward_pulls = kwargs.get('reward_pulls')
        self.failed_pulls = kwargs.get('failed_pulls')
        self.tot_reward_pulls = kwargs.get('tot_reward_pulls')
        self.tot_failed_pulls = kwargs.get('tot_failed_pulls')
        self.tot_days = kwargs.get('tot_days')
        self.entrance_rewards = kwargs.get('entrance_rewards')
        self.hold_time = kwargs.get('hold_time')
        self.ht_trials = kwargs.get('ht_trials')
        self.pert_trials = kwargs.get('pert_trials')
        self.phase = kwargs.get('phase')
        self.free_water_remained = kwargs.get('free_water_remained')

        # Make relevant data directories
        if not os.path.exists(f'../data/{self.name}'):
            os.makedirs(f'../data/{self.name}')

    def cleanup(self):
        self.record_data(datetime.now(), '99')
        self.save()
        print("Saving mouse data before closing")

    def record_daily_report(self, mean_ht, median_ht, q3ht, old_ht, free_water_remained, phase):
        date = datetime.today() - timedelta(days=1)
        with open(self.daily_report_file, 'a', encoding='utf-8') as f:
            f.write((f'{date.date()}\t{self.reward_pulls}\t{self.failed_pulls}\t{mean_ht}\t'
                     f'{median_ht}\t{q3ht}\t{old_ht}\t{free_water_remained}\t{phase}\n'))

    def record_data(self, timestamp, event, dt=0.0, pert_force=0.0):
        """
        Records event data for mouse trials to a file, including timestamps, event codes,
        hold times, durations, and perturbation forces.

        Parameters:
        - timestamp (datetime): The time at which the event occurred.
        - event (str): Event code representing different trial and entrance events:
            00: Mouse entered
            99: Mouse exited
            01: Mouse given entrance reward
            02: Successful trial (training phase)
            03: Successful trial - exited forwards
            04: Successful trial - exited backwards
            51: Cancelled trial - nose removed from port mid-trial (note that this
                does not count as a failed trial for success rate purposesand
                lever position/video are not recorded)
            52: Cancelled trial - did not reach success range (note that this
                does not count as a failed trial for success rate purposes and
                lever position/video are not recorded)
            53: Unsuccessful trial - not held long enough in range, exited forwards
            54: Unsuccessful trial - not held long enough in range, exited backwards
            66: Mouse switched to acquisition stage
            77: Mouse advanced up in hold-time
            88: Mouse switched to perturbation stage
        Each event code corresponds to a specific action or trial outcome.
        - ht (float): Required hold time for the event (0 for training).
        - dt (float, optional): Duration of the event, default is 0.0. Used for trial events.
        - pert_force (float, optional): Perturbation force applied, default is 0.0.

        Note:
        - The function appends the recorded data to a file specified in `self.data_file`.
        - Data format per line: 'timestamp\t event\t hold_time\t duration\t perturbation_force\n'.
        """
        with open(self.data_file, 'a', encoding='utf-8') as file:
            data = f"{timestamp}\t{event}\t{self.hold_time}\t{round(dt, 4)}\t{pert_force}\n"
            file.write(data)


    def save(self):
        with open('../data/mice.cfg', 'r', encoding='utf-8') as f:
            data = f.readlines()

        for idx, row in enumerate(data):
            data[idx] = row.replace('\n', '').split('\t')

            if data[idx][1] == self.name:
                data[idx][2] = self.mouse_day
                data[idx][3] = self.daily_water
                data[idx][4] = self.reward_pulls
                data[idx][5] = self.failed_pulls
                data[idx][6] = self.tot_reward_pulls
                data[idx][7] = self.tot_failed_pulls
                data[idx][8] = self.tot_days
                data[idx][9] = self.entrance_rewards
                data[idx][10] = self.hold_time
                data[idx][11] = self.ht_trials
                data[idx][12] = self.pert_trials
                data[idx][13] = self.phase
                data[idx][14] = self.free_water_remained

        with open('../data/mice.cfg', 'w', encoding='utf-8') as f:
            for row in data:
                newLine = '\t'.join(map(str, row))
                f.write(newLine + '\n')

    def get_prev_hold_times(self):
        hold_time_list = []
        fail_list = []

        with open(self.data_file, 'r', encoding='utf-8') as f:
            hold_times = f.readlines()

        for x in range(len(hold_times)-1, 0, -1):
            hold_time = hold_times[x].replace('\n', '').split('\t')
            mkt = mktime(strptime(hold_time[0], '%Y-%m-%d %H:%M:%S.%f'))
            if (time() - mkt)/86400 < 1:
                if hold_time[1] in TASK_SUCCESS_CODES:
                    hold_time_list.append(float(hold_time[2]))
                elif hold_time[1] in TASK_FAIL_CODES:
                    fail_list.append(0)
            else:
                break

        return hold_time_list, fail_list

    def update_and_analyze_hold_time(self, prev_hold_time):
        """
        Analyzes trial data to determine an updated hold time and calculates key statistical
        metrics of past trials. The function also considers the current experimental phase
        and success rates in its calculations.

        Parameters:
        - new_hold_time (float): The proposed hold time to evaluate for potential updating.

        Returns:
        - tuple: (median_ht, mean_ht, q3ht), where median_ht is the median hold time,
        mean_ht is the mean hold time, and q3ht is the 75th percentile of the hold times.
        Returns (0, 0, 0) if there are no successful trials.

        Note:
        - Updates the hold time of the experiment based on specified conditions.
        - If the phase or other conditions warrant, the hold time may be adjusted to
        reflect new experimental needs.
        """
        if self.phase not in VALID_PHASES:
            print("Minimum daily water consumption was less than 1 mL.")
            print("The Hold time is not changed.")
            return

        hold_time_list, fail_list = self.get_prev_hold_times()

        if len(hold_time_list) == 0:
            print("No Successful Trials in the last 24 hours.")
            return 0, 0, 0

        total_trials = len(hold_time_list) + len(fail_list)
        success_rate = len(hold_time_list) / total_trials

        median_ht = round(np.percentile(hold_time_list, 50), 2)
        q3ht = round(np.percentile(hold_time_list + fail_list, 75), 2)
        mean_ht = round(np.mean(hold_time_list + fail_list), 2)

        if prev_hold_time == MAX_HT and success_rate > REQ_SUCCESS_RATE:
            pass
        elif q3ht > MAX_HT:
            self.hold_time = MAX_HT
            print("Maximum Hold time reached.")
        elif prev_hold_time > q3ht:
            self.hold_time = q3ht
            print(f"75th percentile ({q3ht}) is less than current hold time. The hold_time is not changed.")
        elif success_rate > 0.3 and q3ht > prev_hold_time + 0.3:
            self.hold_time = prev_hold_time + 0.1
            print(f'75th percentile ({q3ht}) is more than 0.3s longer than current HT. The hold_time is increased by 0.1s.')
        elif success_rate > 0.3:
            self.hold_time = q3ht
            print(f'Hold time set to {self.hold_time}')

        self.hold_time = prev_hold_time
        return median_ht, mean_ht, q3ht

    def update_phase(self, prev_hold_time):
        hold_time_list, fail_list = self.get_prev_hold_times()

        if len(hold_time_list) == 0:
            print("No Successful Trials in the last 24 hours.")
            return 0, 0, 0

        total_trials = len(hold_time_list) + len(fail_list)
        success_rate = len(hold_time_list) / total_trials

        # NOTE: if perturbation trials are added, phase 3 should be updated here

    def update(self):
        now = datetime.now()

        if now.day() == self.mouse_day:
            return

        print("Rollover time reached. Resetting daily variables...")
        prev_hold_time = self.hold_time

        if self.phase in VALID_PHASES:
            median_ht, mean_ht, q3ht = self.update_and_analyze_hold_time(prev_hold_time)
            self.update_phase(prev_hold_time)
            self.ht_trials = 0

        free_water_remained = max(0, 100 - self.daily_water)

        self.record_daily_report(
            mean_ht,
            median_ht,
            q3ht,
            prev_hold_time,
            free_water_remained,
            self.phase
        )

        self.daily_water = 0
        self.reward_pulls = 0
        self.failed_pulls = 0
        self.tot_days += 1
        self.mouse_day = now.day()

    # NOTE We need to update hold time?

    @property
    def daily_report_file(self):
        return f'../data/{self.name}/{self.name}_dailyreport.txt'

    @property
    def data_file(self):
        return f'../data/{self.name}/{self.name}_data.txt'

    @property
    def lever_position_file(self):
        return f'../data/{self.name}/{self.name}_lever_position.txt'