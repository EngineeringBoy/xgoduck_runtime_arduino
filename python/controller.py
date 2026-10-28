"""100Hz receive/filter callback and independent 50Hz ONNX loop."""
import collections
import math
import threading
import time
from policy import Policy
from recovery import Recovery
from proc import SignalFilter
from rl_core import MOTOR_IDS, IMU_BIAS_DEG, MOUTH_INDEX, MOUTH_MAX_DEG
from wire import *

EXCLUSIVE_MODES = ('calibrate', 'servo_debug')
PICK_PERIOD_S = 4.0
PICK_MOUTH_CLOSE_PHI = 0.4
ROULADE_S = 1.9
ROULADE_ALPHA = 0.15
CONTROL_PERIOD = 0.02

class Controller:
    def __init__(self, bridge):
        self.bridge = bridge
        self.lock = threading.RLock()
        self.servo_op_lock = threading.Lock()
        self.cal_op_lock = threading.Lock()
        self.policy_lock = threading.Lock()
        self.policy = Policy()
        ok, message = self.policy.load(use_model_home=True)
        if not ok:
            raise RuntimeError(message)
        self.model_message = message
        self.walk_policy = self.policy
        self.roller_policy = Policy()
        ok, message = self.roller_policy.load('xgoduck_roller.onnx', use_model_home=True)
        if not ok:
            raise RuntimeError(message)
        self.roller_policy.action_scale = 0.7
        self.roller_policy.set_action_alphas(leg=0.4, head=0.5)
        self.model_message += '; ' + message
        self.roller_mode = False
        self.getup_policy = Policy()
        ok, message = self.getup_policy.load('xgoduck_getup.onnx', use_model_home=True)
        if not ok:
            raise RuntimeError(message)
        self.model_message += '; ' + message
        self.pick_policy = Policy()
        ok, message = self.pick_policy.load('xgoduck_pick.onnx', use_model_home=True)
        if not ok:
            raise RuntimeError(message)
        self.model_message += '; ' + message
        self.roulade_policy = Policy()
        ok, message = self.roulade_policy.load('xgoduck_roulade.onnx', use_model_home=True)
        if not ok:
            raise RuntimeError(message)
        self.model_message += '; ' + message
