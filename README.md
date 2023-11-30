# PiPaw

## Overview

PiPaw is an automated home-cage system for assessing motor learning and movement kinematics in mice.

## Table of Contents

1. [Overview](#overview)
   - Introduction to PiPaw
2. [Files](#files)
   - List of Key Files and Directories
3. [Getting Started with the Program](#getting-started-with-the-program)
   - [1. Raspberry Pi Setup](#1-raspberry-pi-setup)
   - [2. Configuring Mouse Data in `mice.cfg`](#2-configuring-mouse-data-in-micecfg)
   - [3. Creating Mouse-specific Folders](#3-creating-mouse-specific-folders)
   - [4. Preparing Mouse Data Files](#4-preparing-mouse-data-files)
   - [5. Organizing Video Data](#5-organizing-video-data)
   - [6. Running the Program](#6-running-the-program)
4. [Git Branch Structure and Workflow](#git-branch-structure-and-workflow)
   - Explanation of Repository Branching Strategy

## Files

```
.gitignore                # Excludes specific files and directories from Git version control
camera_test.py            # Script for testing the camera functionality
encoder_test.py           # Script for testing the encoder performance
infrared_test.py          # Script for assessing the infrared sensor operation
motor_test.py             # Script dedicated to testing motor functionalities
solenoid_test.py          # Script for verifying solenoid operation
RFID_buffer_testing.py    # Script for testing the RFID reader's data buffering capabilities
classReadEncoder.py       # Class file for managing and controlling encoder actions
pwm.py                    # Interface script for working with the Linux PWM driver via sysfs
piPaw_pert.py             # Main executable program for the project
mice.cfg                  # Configuration file containing data specific to each mouse
monitor.sh                # Script to initiate and run the main program
LICENSE                   # Contains the licensing agreement for this repository
README.md                 # This file, providing an overview and instructions for the repository
data/                     # Directory storing outputted results from the cage experiments

```

## Getting Started with the Program

This section guides you through setting up and configuring the program. Follow these steps carefully to ensure everything is correctly established.

### 1. Raspberry Pi Setup

**Initial Configuration:**

1. **Preparing the SD Card:**
   - Download the latest NOOBS zip file from the [Raspberry Pi Downloads](https://www.raspberrypi.org/downloads/noobs/) page.
   - Format a microSD card (minimum 16GB) to FAT32 and extract the NOOBS files onto it.
   - Insert the SD card into your Raspberry Pi.
   - _Note_: If using a pre-loaded NOOBS SD card, it may have an older kernel version. Consider reformatting it and installing the latest NOOBS files to ensure the latest kernel (4.9 or later) and Raspbian version.

**Installation and Configuration:** 2. **Installing Raspbian:**

- Power up the Raspberry Pi and install Raspbian Stretch when prompted.

3. **Enabling Hardware PWM:**
   - Add `dtoverlay=pwm-2chan` to `/boot/config.txt`. Refer to [Using the Raspberry Pi Hardware PWM timers](http://www.jumpnowtek.com/rpi/Using-the-Raspberry-Pi-Hardware-PWM-timers.html) for more details.
4. **Setting Up Project Files:**
   - Transfer all cage files, test files, and additional classes (like pwm, classReadEncoder) to a new folder named `piPaw` in the Documents directory.

### 2. Configuring Mouse Data in `mice.cfg`

**Data Entry:**

- Edit the `mice.cfg` file located in the program directory.
- For each mouse, enter the following information in a new row:
  - **RFID Tag Number:** The unique identifier for the mouse.
  - **Mouse Name:** The name assigned to the mouse.
  - **Day:** The current date's day (e.g., "10th" for October 10th).
  - **Water:** The number of water drops the mouse has consumed.
  - **rwdP (Reward Pulls):** The count of successful trials completed by the mouse today.
  - **Fail:** The number of failed trials today.
  - **totRw:** Total successful trials they found from the beginning of the phase.
  - **totFai (Total Fails):** The total number of failed trials since the beginning of the study.
  - **totDay (Total Day):** The total number of days the mouse has been under the current hold time (`ht`). Resets when `ht` increases.
  - **entRw (Free Water Rewards):** Entrance rewards the mouse has (number of free water drops they have from phase 1).
  - **ht (Hold Time):** The current required hold time for the mouse.
  - **httri:** The number of trials completed with the current hold time today.
  - **prt (Perturbation Trials):** The count of perturbation trials. Perturbations affect 30% of mice in phase 3, increasing with each exposure.
  - **pha:** Phase number.
  - **frW (Free Water):** The number of free water drops the mouse will receive until midnight. If the total is below 100, it indicates less than 1 mL of water consumption. Free water is given every 15 minutes.

### 3. Creating Mouse-specific Folders

**Organization:**

- In the `data` directory:
  - Create a new folder for each mouse named after its Mouse Name as listed in `mice.cfg`.

### 4. Preparing Mouse Data Files

**File Setup:**

- Within each mouse's folder:
  - Create a text file named `{your_mouse_name}_data.txt` (replace `{your_mouse_name}` with the mouse's name).
  - This file will store specific data and logs for each mouse.

### 5. Organizing Video Data

**Video Storage:**

- In each mouse's folder:
  - Create a subfolder named `Videos`.

**Example Structure:**

```
data/
    TEST/
         Videos/
         TEST_data.txt
```

### 6. Running the Program

**Execution and Monitoring:**

- Assign execute permissions to `monitor.sh` (use `chmod 744 monitor.sh`). Logs will be output to `log.txt` in the `piPaw` folder.
- Reboot the Raspberry Pi and check for an internet connection (`ifconfig`).
- Set the Pi to log in to the CLI via the Raspberry Pi Configuration menu for efficiency.
- Power down the Pi.
- Set `precisePaw` to launch on boot using crontab (as root):
  - Enter `sudo crontab –e`, choose nano, then add `@reboot /home/pi/piPaw/monitor.sh` to the crontab file.
  - Save and exit (CTRL-X).

These steps are crucial for the successful setup and operation of the program. Ensure each step is followed accurately for optimal performance.

## Git Branch Structure and Workflow

In this repository, we use a branching strategy that is tailored to ensure efficient and reliable code development and testing. Here is an overview of each branch and its role in the workflow:

1. **Master Branch**

   - **Name:** `master`
   - **Purpose:** The master branch contains code that is designated for Long-Term Support (LTS). It represents the most stable and tested version of the code, having passed through the rigorous testing processes of the other branches. Code in the master branch is ready for production use.

2. **Development Branch**

   - **Name:** `dev`
   - **Purpose:** The development branch (`dev`) is the primary branch for integrating tested code. It contains code that has been developed, tested, and is ready for integration into the next LTS release. This branch does not receive direct merges from the beta branch; instead, it integrates tested and finalized code from feature branches.

3. **Beta Branch**

   - **Name:** `beta`
   - **Purpose:** The beta branch is utilized for bulk testing of multiple feature branches simultaneously. It is a testing ground for combinations of new features or major changes before they are considered for integration into the dev branch. This branch allows for comprehensive multi-feature testing without affecting the more stable dev branch.

4. **Feature Branches**
   - **Name Pattern:** `feat/*`
   - **Purpose:** Feature branches, named with the `feat/` prefix, are used for the development of individual new features or specific changes. These branches are where the initial development and preliminary testing occur. Upon satisfactory testing at this level, code from these branches may be merged either directly into the dev branch or into the beta branch for combined feature testing.

#### Workflow Overview

- **Feature Development:** New features are developed in their respective `feat/` branches.
- **Beta Testing:** For large-scale or multi-feature testing, feature branches are merged into the `beta` branch. This allows for testing how various features interact with each other in a controlled environment.
- **Development Integration:** Features that have passed individual testing (and optionally beta testing) are then merged into the `dev` branch. This branch serves as the final integration point before code is considered for LTS.
- **Release:** After thorough integration and testing in the dev branch, code is merged into the `master` branch for Long-Term Support.

This branching strategy ensures a robust and multi-tiered testing environment, allowing for both individual feature evaluation and combined feature interaction testing, leading up to a stable and reliable master branch for production releases.
