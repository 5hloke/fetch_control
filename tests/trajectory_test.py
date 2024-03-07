from fetch_core.arm import Arm 
from fetch_core.torso import Torso



if __name__ == "__main__":
    joint_states = np.load("../export_0304/1_folded_to_straight/output_traj.npy")
    arm = Arm()
    torso = Torso()

