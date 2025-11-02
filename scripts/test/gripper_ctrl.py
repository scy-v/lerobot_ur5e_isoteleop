from pyDHgripper import PGE

def main():
    gripper = PGE("/dev/ur5e_left_gripper")
    gripper.init_feedback()
    gripper.set_force(20)
    while True:
        val = input("enter: ")
        gripper.set_pos(val=int(val), blocking=False)

