from utils import PyTux
from argparse import ArgumentParser
import pystk


def set_target_velocity(aim_point, current_vel, target_vel):
    turn_sharpness = abs(aim_point[0])
    return (1 - turn_sharpness) * (target_vel - 0.4 * current_vel)

def set_acceleration(current_vel, target_vel):
    return min(1, 1.3 * (target_vel - current_vel) / target_vel)

def set_steer_gain(steer_gain, current_vel, aim_point):
    turn_sharpness = abs(aim_point[0])
    if current_vel > 30 and turn_sharpness > 0.5:
        return min(steer_gain * 1.5, 12)
    elif current_vel > 30:
        return max(steer_gain * 0.8, 6)
    return steer_gain

def set_braking_strategy(current_vel, aim_point, target_vel):
    turn_sharpness = abs(aim_point[0])
    return turn_sharpness * (target_vel - current_vel) / target_vel > 0.5

def set_drifting(aim_point, current_vel, skid_thresh, target_vel):
    turn_sharpness = abs(aim_point[0])
    high_speed = current_vel > target_vel * 1.5
    if turn_sharpness > 0.7:
        return True
    elif turn_sharpness > skid_thresh:
        return not high_speed or (turn_sharpness > skid_thresh * 0.5 and high_speed)
    return False

def set_nitro_usage(current_vel, aim_point, should_drift, target_vel):
    turn_sharpness = abs(aim_point[0])
    return True if (turn_sharpness < 0.2 and current_vel < target_vel and not should_drift) else False



def control(aim_point, current_vel, steer_gain=8, skid_thresh=0.15, target_vel=45):
    action = pystk.Action()

    adjusted_target_vel = set_target_velocity(aim_point, current_vel, target_vel)
    adaptive_steer_gain = set_steer_gain(steer_gain, current_vel, aim_point)

    action.steer = max(-1, min(1, adaptive_steer_gain * aim_point[0]))
    action.acceleration = set_acceleration(current_vel, adjusted_target_vel)
    action.brake = set_braking_strategy(current_vel, aim_point, adjusted_target_vel)
    action.drift = set_drifting(aim_point, current_vel, skid_thresh, target_vel)
    action.nitro = set_nitro_usage(current_vel, aim_point, action.drift, target_vel)

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
