"""Named joint poses and constants shared by the Handy example scripts (radians)."""

ARM_JOINTS = [f"joint_{i}" for i in range(1, 8)]
HAND_JOINT = "joint_8"  # joint_9 mirrors it

# Bent pose away from limits/singularities. Outside the +/-0.3 test window -> needs limits:=full.
READY = {
    "joint_1": 0.0,
    "joint_2": -0.8,
    "joint_3": 0.2,
    "joint_4": 0.8,
    "joint_5": 0.0,
    "joint_6": 0.8,
    "joint_7": 0.0,
}

# Arm reaching forward (gripper ~47 cm out, 54 cm up). Chosen by grid search for a low Jacobian
# condition number (14.7 vs 25.8 at READY; Servo slows at 17, halts at 30), so Cartesian Servo
# motions around it stay smooth. Used by servo_wave. Needs limits:=full.
WAVE = {
    "joint_1": 0.0,
    "joint_2": -1.5,
    "joint_3": 0.0,
    "joint_4": 1.5,
    "joint_5": 0.0,
    "joint_6": -1.0,
    "joint_7": 0.0,
}

HAND = {"open": -0.514, "close": 0.125, "neutral": 0.0}

# /servo_node/status codes (MoveIt Servo, Humble)
SERVO_STATUS = {0: "OK", 1: "slowing: singularity", 2: "HALT: singularity",
                3: "slowing: collision", 4: "HALT: collision", 5: "HALT: joint limit"}
