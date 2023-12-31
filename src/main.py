import random

from datetime import datetime
from time import time, sleep

from .cage import Cage
from .constants import *

def time_out(aht,rht):
    timeout = 5 - 4*(aht/(rht+0.000001))
    if timeout < 1:
        timeout = 0.5
    return timeout

def main():
    cage = Cage()
    nose_poke = False
    infrared_restored = False

    try:
        if cage.is_mouse_in_chamber():
            cage.wait_for_mouse_to_exit_cage()
            cage.lever.set_starting_position()
            print("Done Init! Waiting for mouse!")

        trial_time_start = None
        trial_time_ir_broken = None
        trial_time_cancel = None
        trial_timeout = None

        while True:
            if cage.is_mouse_in_chamber():
                # Following is executed if an RFID has come into range for the first time since
                # the grace period last expired
                if cage.current_mouse is None:
                    cage.enters_mouse()
                elif cage.rfid.interrupted:
                    cage.rfid.interrupted = False
                    print("RFID signal restored. Double checking animal identification.")
                    tag_id = cage.rfid.get_serial()
                    if cage.has_mouse_changed(tag_id):
                        cage.switch_mouse(tag_id)
                        trial_timeout = T_TIMEOUT_CANCELLED
                    nose_poke = False

            elif not cage.is_mouse_in_chamber():

                # Evaluates True when a previously detected mouse goes out of range:
                if cage.current_mouse and cage.rfid.interrupted:
                    print(f"RFID signal for {cage.current_mouse.name} went out of range.")
                    cage.rfid.interrupted = True
                    cage.trial_time_mouse_gone = time()
                    cage.trial_datetime_mouse_last_detected = datetime.now()

                # Evaluates True once grace period for mouse detection is over and
                # mouse is assumed to have left the chamber:
                elif cage.rfid.interrupted and \
                    time() - cage.trial_time_mouse_gone > T_RFID_GRACE and not nose_poke:
                    print(f"{cage.current_mouse.name} not detected for more than {T_RFID_GRACE} seconds. \
                          Assuming chamber is empty ({datetime.now()}).")
                    cage.current_mouse.record_data(cage.trial_datetime_mouse_last_detected, '99')
                    cage.current_mouse.save()
                    cage.camera.stop()
                    cage.camera.clear_streams()
                    trial_timeout = T_TIMEOUT_CANCELLED
                    current_time = time()
                    while cage.lever.read_counter() > 6:
                        if (time() - current_time) > 2.0:
                            print(f"Possible Error, lever counter value prior to \
                                  clearing is {cage.lever.read_counter()}.")
                            break

                    cage.lever.clear_counter()
                    cage.lever.motor.set_low_duty_cycle()
                    sleep(0.1)
                    cage.lever.motor.enable(False)
                    cage.lever.motor.set_high_duty_cycle()
                    cage.current_mouse = None
                    cage.rfid.interrupted = False
                    nose_poke = False
                else:
                    sleep(0.01)

            if not nose_poke:
                if cage.is_mouse_in_chamber():
                    # Nose poke is detected before the RFID was identified
                    if cage.current_mouse == None:
                        print("Infrared broken before RFID was detected. \
                              Waiting until ID can be determined.")
                        current_time = time()
                        while not cage.is_mouse_in_chamber():
                            if (time() - current_time) > T_IRGRACE:
                                raise ValueError("Infrared broken but RFID could \
                                                 not be detected within 10 seconds.")
                            sleep(0.01)
                        continue
                    print("Infrared broken. Waiting to begin trial.")
                    nose_poke = True
                    trial_time_ir_broken = time()
                    # Delivers a free water drop for the first nose poke if required
                    if cage.current_mouse.phase == 1 or cage.current_mouse.free_water_remained != 0:
                        cage.solenoid.dispense_water(T_SOLENOID)
                        cage.buzzer.play_tone('Hi')
                        cage.current_mouse.daily_water += 1
                        cage.current_mouse.entrance_rewards += 1
                        cage.camera.stop()

                        if cage.current_mouse.phae != 1:
                            cage.current_mouse.free_water_remained -= 1

                        # 01 - Nose Poke reward code
                        cage.current_mouse.record_data(datetime.now(), '01')

                        # If mouse given nose poke drop, wait 5 seconds
                        # until the first trial (otherwise can start immediately)
                        trial_time_start = time()
                        cage.camera.write_video(cage.current_mouse.name, datetime.now(), '01')
                        cage.camera.camera.start_recording(cage.camera.stream2, format='h264')

            while nose_poke:
                # Animal moved head which broke beam
                if not cage.infrared.beam_broken():
                    if not infrared_restored:
                        trial_time_cancel = time()
                        infrared_restored = True
                    elif time() - trial_time_cancel > T_IR_GRACE:
                        if cage.lever.ns.past_threshold_pos0 and not cage.trial_ongoing:
                            cage.terminate_record_lever_position_process()
                            cage.camera.camera.stop_preview()
                            cage.camera.camera.stop_recording()
                            cage.camera.stream2.clear()
                            cage.camera.write_video(cage.current_mouse.name, datetime.now(), '50')
                            cage.camera.camera.start_recording(cage.camera.stream2, 'h264')
                            cage.lever.motor.set_high_duty_cycle()
                            cage.trial_ongoing = False
                            print("Head removed from position before trial was initiated.")
                        elif cage.trial_ongoing:
                            cage.terminate_record_lever_position_process()
                            print("Head removed before trial could be completed. \
                                  Trial cancelled (Code 51).")
                            cage.end_trial(
                                trial_time_start,
                                0.0,
                                '51',
                                'cancelled'
                            ) # 51 is trial code for head being removed mid-trial
                            trial_timeout = T_TIMEOUT_CANCELLED
                        else:
                            print("Head removed from position .")
                        nose_poke = False
                        infrared_restored = False
                        break
                elif cage.infrared.beam_broken() and infrared_restored:
                    infrared_restored = False
                    trial_time_ir_broken = time()

                if not cage.trial_ongoing:
                    if (time() - trial_time_start) > trial_timeout and \
                       (time() - trial_time_ir_broken) > T_IR_WAIT:
                        cage.trial_ongoing = True
                        cage.lever.ns.pert_force = random.choice(PERT_LIST)
                        print("Timeout is up, motor switched to low torque.")
                        cage.camera.camera.start_preview()
                        cage.camera.stream2.clear()
                        cage.camera.camera.split_recording(cage.camera.stream2)
                        cage.init_record_lever_position_process()
                        cage.lever.motor.set_low_duty_cycle()
                        cage.buzzer.play_tone('Med')

                elif cage.lever.ns.past_threshold_pos0 and not cage.trial_ongoing:
                    trial_time_start = datetime.now()
                    cage.trial_ongoing = True

                    if cage.phase == 1:
                        print("Trial initiated - mouse is in training stage.")
                    elif cage.phase == 2:
                        print("Trial initiated - mouse is in main stage of testing.")

                elif cage.trial_ongoing and cage.lever.ns.past_threshold_pos0:
                    cage.terminate_record_lever_position_process()
                    trial_timeout = time_out(cage.lever.ns.time_in_range, cage.current_mouse.hold_time)
                    if cage.phase == 1:
                        # 02 is success code for Training
                        cage.end_trial(trial_time_start, cage.lever.ns.time_in_range, '02', 'success', pert_force=0.0)
                    elif cage.phase == 2 or cage.phase == 3:
                        if cage.lever.ns.pos1:
                            if cage.lever.ns.pos2: # Lever exited range towards back of range
                                if cage.lever.ns.time_in_range >= cage.current_mouse.hold_time:
                                    # 04 is success code for Acquisition
                                    # (exited towards back of range)
                                    cage.end_trial(
                                        trial_time_start,
                                        cage.lever.ns.time_in_range,
                                        '04',
                                        'success',
                                    )
                                else:
                                    # 54 is fail code for Acquisition
                                    # (exited towards back of range)
                                    cage.end_trial(
                                        trial_time_start,
                                        cage.lever.ns.time_in_range,
                                        '54',
                                        'failed',
                                    )
                            else: # Lever exited range towards front of range
                                if cage.lever.ns.time_in_range >= cage.mouse.hold_time:
                                    # 03 is success code for Acquisition
                                    # (exited towards front of range)
                                    cage.end_trial(
                                        trial_time_start,
                                        cage.lever.ns.time_in_range,
                                        '03',
                                        'success',
                                    )
                                else:
                                    # 53 is fail code for Acquisition
                                    # (exited towards front of range)
                                    cage.end_trial(
                                        trial_time_start,
                                        cage.lever.ns.time_in_range,
                                        '53',
                                        'failed',
                                    )
                        # Lever did not reach goal range - in this case
                        # trial is cancelled and recorded in mouse file, but
                        # lever position not saved etc.
                        else:
                            print("Lever did not reach goal range. Trial cancelled (Code 52).")
                            # 52 is code for trial did not reach goal range (trial cancelled)
                            cage.end_trial(trial_time_start, 0.0, '52', 'cancelled')
                            trial_timeout = T_TIMEOUT_CANCELLED
                    else:
                        trial_timeout = T_TIMEOUT_CANCELLED
                        raise ValueError("Testing phase not found.")

                    cage.lever.ns.pos1 = False
                    cage.lever.ns.pos2 = False
                    cage.lever.ns.entered_range = False
                    cage.lever.ns.grace_period = False
                    break

            # Mouse pulls the lever past threshold at any time outside of a trial
            if not cage.lever.ns.past_threshold_pos0 and cage.lever.read_counter() > THRESHOLD_POSITION_ZERO:
                if cage.current_mouse is None:
                    cage.buzzer.play_tone('Low',duration=0.5)
                    raise ValueError("Lever moved past threshold before any RFID detected.")
                else:
                    print("Lever moved past threshold during timeout or while head out \
                          of position.")
                    time_mark = time()
                    while cage.lever.read_counter() > THRESHOLD_POSITION_ZERO:
                        sleep(0.01)
                    print(f"Lever returned to threshold range in \
                          {time() - time_mark:.2f} seconds. \
                          Restarting time until trial can begin.")
                    trial_time_start = time()

    except KeyboardInterrupt:
        print("Keyboard Interrupt")

    except ValueError as err:
        print(f"ValueError: {err}")
        print("Exit Time: ", datetime.now())
        exit(1)

    finally:
        if cage.current_mouse is not None:
            trial_timeout = T_TIMEOUT_CANCELLED
        cage.cleanup()
