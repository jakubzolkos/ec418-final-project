from utils import PyTux
from argparse import ArgumentParser
import pystk
import math

def normalize(x, l, r):
    return max(l, min(r, x))

def velocity_curvature_factor(current_vel, target_vel, aim_point):
    turn_sharpness = abs(aim_point[0])
    factor = int(2 * (turn_sharpness * current_vel)**2 / target_vel)
    return factor

def set_acceleration(current_vel, target_vel, aim_point):
    turn_sharpness = abs(aim_point[0])
    factor = velocity_curvature_factor(current_vel, target_vel, aim_point)
    return (1 - turn_sharpness)**(factor)

def set_braking_strategy(current_vel, aim_point):
    turn_sharpness = abs(aim_point[0])
    return turn_sharpness > 0.4 and current_vel > 10

def set_drifting(skid_thresh, current_vel, target_vel, aim_point):
    turn_sharpness = abs(aim_point[0])

    if turn_sharpness > 0.6 and current_vel > 15:  
        return True
    elif turn_sharpness > skid_thresh and current_vel > 15: 
        return (turn_sharpness > skid_thresh)
    return False

def normalize(x, l, r):
    return max(l, min(r, x))

def set_steer_gain(steer_gain, current_vel, target_vel, aim_point):
    turn_sharpness = aim_point[0]      
    return normalize(steer_gain * turn_sharpness, -1, 1)

def set_nitro_usage(current_vel, target_vel, aim_point):
    turn_sharpness = abs(aim_point[0])
    return True if (turn_sharpness < 0.2 and current_vel <= target_vel) else False



def control(aim_point, current_vel, steer_gain=8, skid_thresh=0.20, target_vel=25):
    action = pystk.Action()

    action.steer = set_steer_gain(steer_gain, current_vel, target_vel, aim_point)
    action.acceleration = set_acceleration(current_vel, target_vel, aim_point)
    action.brake = set_braking_strategy(current_vel, aim_point)
    action.nitro = set_nitro_usage(current_vel, target_vel, aim_point)
    action.drift = set_drifting(skid_thresh, current_vel, target_vel, aim_point)

    return action


if __name__ == '__main__':

    def test_controller(args):
        pytux = PyTux()
        for t in args.track:
            steps, how_far = pytux.rollout(t, control, max_frames=1000, verbose=args.verbose)
            print(steps, how_far)
        pytux.close()


    parser = ArgumentParser()
    parser.add_argument('track', nargs='+')
    parser.add_argument('-v', '--verbose', action='store_true')
    args = parser.parse_args()
    test_controller(args)
