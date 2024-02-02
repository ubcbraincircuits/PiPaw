"""
GPIO Configuration
"""
G_INFRARED = 21

# Specific GPIO for this should be between 1-8 as these
# are pulled High by default on boot, otherwise valve will open
G_SOLENOID = 4

G_MOTOR_ENABLE = 23
G_MOTOR_DIRECTION = 24
G_RFID = 16

"""
Trial Configuration
"""
# Threshold to determine  when lever moved enough to signal the start of a trial
THRESHOLD_POSITION_ZERO = {
    1: 34,
    2: 66,
    3: 66
}
THRESHOLD_POSITION_ONE = 68  # 6 degrees
THRESHOLD_POSITION_TWO = 274  # 24 degrees

# Number of successful trials in training stage
N_TRAINING = 200

# Size of the bin in which trials are checked to determine if animal advances to next hold time
N_SUCCESS_CHECK = 50

# Success ratio required for increase in hold time
SUCCESS_REQ = 0.75

# Initial hold time at beginning of main phase
INITIAL_HT = 0.05

# Amount hold time is increased by each time
HT_INCREASE = 0.1

# Maximum hold time mice can advance to
MAX_HT = 1.0

# Length of time solenoid opens for entrance rewards and successful trials
T_SOLENOID = 0.25

# Minimum duration between two trials
T_TIMEOUT = 5.0

# Minimum duration between two successful trials
T_TIMEOUT_SUCCESS = 1.0

# Minimum duration between two cancelled trials
T_TIMEOUT_CANCELLED = 5.0

# Minimum duration between failed trials
T_TIMEOUT_FAILED = 5.0

# This raises the gain a bit on the sensitivity of the IR breakbeam
T_IR_GRACE = 0.2

# Duration after last RFID signal detection that animal is considered to have left the chamber
T_RFID_GRACE = 30.0

# Minimum duration IR breakbeam must be broken for before trial initiates
T_IR_WAIT = 1.0

# Fixed interval for delivery of entrance reward in seconds (only for training 1 phase)
T_ENTRANCE_REWARD_INTERVAL = 900

# Time to wait before starting the perturbation
PERTURBATION_WAIT = 0.5

# % of max force for perturbation
PERT_LIST = [-1, -0.5, 0, 0.5, 1, 2]

# Duration of the force perturbation
PERT_DURATION = 1.5

MAX_HT_TRIALS = 100
REQ_SUCCESS_RATE = 0.5

# This margin accounts for brief fluctuations in lever position
T_LEVGRACE = 0.2


"""
Encoder Configuration
"""
CSX = 0 # Pin CS0
CLK = 10000000 # 10Mhz clock freq.
BYTEMD = 4 # Bytes that will be sent to you.
SAMPLING_RATE = 410 # Rate of lever sampling in Hz. 205 gets ~200 Hz rate in practice
