import os.path
from matplotlib import pyplot as plt
from matplotlib.font_manager import FontProperties
import matplotlib.patches as patches
from scipy.interpolate import splrep, splev
import numpy as np
import math
import json



dt = 0.1
HMI_LAST_TIME = 2
PET_THRESHOLD = 1


def parse_input_data(row, agents_history, durations):
    EHMIRequest = json.loads(row)
    durations.append(EHMIRequest['0'])
    for PawnInfo in EHMIRequest['1']:
        if PawnInfo['n'] not in agents_history:
            agents_history[PawnInfo['n']] = []
        agents_history[PawnInfo['n']].append([PawnInfo['x'], PawnInfo['y']])

def smooth_ployline(cv_init, point_num=1000):
    cv = cv_init
    if np.size(cv, 0) <= 3:
        interpolated_cv = []
        for n in range(np.size(cv, 0) - 1):
            for i in range(5):
                interpolated_cv.append((2 - i) / 2 * cv[n,] +
                                       i / 2 * cv[n + 1,])
        cv = np.array(interpolated_cv)
        # print(interpolated_cv)

    list_x = cv[:, 0]
    list_y = cv[:, 1]
    if type(cv) is not np.ndarray:
        cv = np.array(cv)
    delta_cv = cv[1:, ] - cv[:-1, ]
    s_cv = np.linalg.norm(delta_cv, axis=1)

    s_cv = np.array([0] + list(s_cv))
    s_cv = np.cumsum(s_cv)

    bspl_x = splrep(s_cv, list_x, s=0.1)
    bspl_y = splrep(s_cv, list_y, s=0.1)
    # values for the x-axis
    s_smooth = np.linspace(0, max(s_cv), point_num)
    # get y values from interpolated curve
    x_smooth = splev(s_smooth, bspl_x)
    y_smooth = splev(s_smooth, bspl_y)
    new_cv = np.array([x_smooth, y_smooth]).T

    delta_new_cv = new_cv[1:, ] - new_cv[:-1, ]
    s_accumulated = np.cumsum(np.linalg.norm(delta_new_cv, axis=1))
    s_accumulated = np.concatenate(([0], s_accumulated), axis=0)
    return new_cv, s_accumulated


def estimate_speed(trj):
    # Calculate speed based on the last two points
    dx = trj[-1, 0] - trj[-2, 0]
    dy = trj[-1, 1] - trj[-2, 1]
    dist = np.sqrt(dx ** 2 + dy ** 2)
    speed = dist / dt  # Assuming one time unit between points
    return speed


def calculate_intersection(trj1, trj2):
    # Extract positions
    x1, y1 = trj1[-1]
    x2, y2 = trj2[-1]

    # Estimate velocities
    v1x = (trj1[-1, 0] - trj1[-2, 0]) / dt
    v1y = (trj1[-1, 1] - trj1[-2, 1]) / dt
    v2x = (trj2[-1, 0] - trj2[-2, 0]) / dt
    v2y = (trj2[-1, 1] - trj2[-2, 1]) / dt

    # Calculate direction
    if v1x == 0 or v2x == 0:
        return None
    k1 = v1y / v1x
    k2 = v2y / v2x

    # Calculate intersection point
    ix = (k1 * x1 - k2 * x2 + y2 - y1) / (k1 - k2)
    iy = k1 * (ix - x1) + y1

    t1 = (ix - x1) / v1x
    t2 = (ix - x2) / v2x
    if t1 < 0 or t2 < 0:
        return None

    return (ix, iy)


def calculate_pet(trj1, trj2, bound_offset=None):
    if bound_offset is None:
        bound_offset = [0, 0]
    intersection = calculate_intersection(trj1, trj2)
    if intersection is None:
        return None  # No intersection, cannot calculate PET

    # Calculate distances to the intersection point
    dist1 = np.sqrt((trj1[-1, 0] - intersection[0]) ** 2 + (trj1[-1, 1] - intersection[1]) ** 2) - bound_offset[0]
    dist2 = np.sqrt((trj2[-1, 0] - intersection[0]) ** 2 + (trj2[-1, 1] - intersection[1]) ** 2) - bound_offset[1]

    # Estimate speeds
    speed1 = estimate_speed(trj1)
    speed2 = estimate_speed(trj2)

    # Calculate times to intersection
    time1 = dist1 / speed1 if speed1 != 0 else float('inf')
    time2 = dist2 / speed2 if speed2 != 0 else float('inf')

    # PET is the absolute difference in times to intersection
    pet = abs(time1 - time2)
    return pet


# Function to calculate heading from positions
def calculate_heading(pos1, pos2):
    delta_x = pos2[0] - pos1[0]
    delta_y = pos2[1] - pos1[1]
    heading_rad = math.atan2(delta_y, delta_x)
    heading_deg = math.degrees(heading_rad)
    return heading_deg


def generate_hmi_info(agents_history, time_duration, history_info, safety_index_history, need_plot=False):
    """
    Calculate HMI and eHMI plan from safety index.
    :param agents_history: trajectories of agent of interest
    :param time_duration: time point list of data receiving
    :param history_info: last_ehmi_info, last_hmi_info, t_hmi_remaining (>0 if hmi was on)
    :param safety_index_history: recorded safety index values from previous steps
    :param need_plot: if plot the trajectory
    :return: ehmi and hmi info. lists
    """

    control_mode = 'PET'

    # recall hmi and ehmi history
    last_ehmi_info, last_hmi_info, t_hmi_remaining = history_info

    # keep the hmi info. in last step by default
    ehmi = last_ehmi_info
    hmi = last_hmi_info

    # Reference line  TODO: hardcoded from records
    ref_ego = [[326585 - 3 * (326585 - 335178), 418035 - 3 * (418035 - 428500)],
               [335178, 428500], [326585, 418035],
               [326585 + 3 * (326585 - 335178), 418035 + 3 * (418035 - 428500)]]
    ref_ego_left = [[328696 - 3 * (328696 - 335133), 421261 - 3 * (421261 - 428996)],
                    [335133, 428996], [328696, 421261],
                    [328696 + 3 * (328696 - 335133), 421261 + 3 * (421261 - 428996)]]
    ref_ped = [[330926 - 3 * (330926 - 331744), 424124 - 3 * (424124 - 423539)],
               [331744, 423539], [330926, 424124],
               [330926 + 3 * (330926 - 331744), 424124 + 3 * (424124 - 423539)]]

    " ==== Calculate Conflict Index ==== "
    # history trajectories ([x, y]) in 10-by-2 arrays
    ego_trj = np.array(agents_history['ego'][-10:])
    ego_left_trj = np.array(agents_history['veh1'][-10:])
    ped_trj = np.array(agents_history['veh2'][-10:])

    # reference paths in 4-by-2 arrays
    # ref_ego = np.array(ref_ego)
    # ref_ego_left = np.array(ref_ego_left)
    # ref_ped = np.array(ref_ped)

    # Calculate Post-Encroachment Time between any two of agents (ego, ego_left, ped) assuming they travel with
    # their current heading and speed
    pet_ego_ego_left = calculate_pet(ego_trj, ego_left_trj, bound_offset=[100 / 2, 100 / 2])
    pet_ego_ped = calculate_pet(ego_trj, ped_trj, bound_offset=[200 / 2, 50 / 2])
    # pet_ego_left_ped = calculate_pet(ego_left_trj, ped_trj, bound_offset=[200 / 2, 200 / 2])

    safety_index = [pet_ego_ego_left, pet_ego_ped]

    " ==== Set HMI and eHMI ==== "

    caution_messages = []
    if len(safety_index_history) > 10:
        # change HMI according to PET
        if not hmi[0] or t_hmi_remaining <= 0:
            hmi[0] = False
            if control_mode == 'PET' and safety_index_history[-1][0] and safety_index_history[-2][0]:
                if pet_ego_ego_left:
                    r_pos_ego_ego_left = ego_trj[-1, :] - ego_left_trj[-1, :]
                    if (pet_ego_ego_left < PET_THRESHOLD
                            and np.hypot(r_pos_ego_ego_left[0], r_pos_ego_ego_left[1]) < 5000):
                        if safety_index_history[-1][0] < PET_THRESHOLD and safety_index_history[-2][0] < PET_THRESHOLD:
                            hmi[0] = True
                            t_hmi_remaining = HMI_LAST_TIME

                            hmi[1] = 0  # Caution: Left Vehicle Approaching
                            hmi[2] = 0
                            caution_messages.append('Caution: Left Vehicle Approaching!')
                            caution_messages.append('注意左侧来车！')
                            print("PET between ego and ego_left:", pet_ego_ego_left)

                if pet_ego_ped and safety_index_history[-1][1] and safety_index_history[-2][1]:
                    r_pos_ego_ped = ego_trj[-1, :] - ped_trj[-1, :]
                    if (pet_ego_ped < PET_THRESHOLD
                            and np.hypot(r_pos_ego_ped[0], r_pos_ego_ped[1]) < 5000):
                        if safety_index_history[-1][1] < PET_THRESHOLD and safety_index_history[-2][1] < PET_THRESHOLD:
                            t_hmi_remaining = HMI_LAST_TIME
                            if hmi[0]:
                                hmi[1] = 2
                                hmi[2] = 2
                            else:
                                hmi[0] = True
                                hmi[1] = 1  # Caution: Approaching Pedestrian
                                hmi[2] = 1
                            caution_messages.append('Caution: Approaching Pedestrian!')
                            caution_messages.append('注意前方行人！')
                            print("PET between ego and ped:", pet_ego_ped)

            """
                ehmi = {'ID': 'ego',
                    'IsShowEHMI': True,
                    'C2CFront': True,
                    'C2CLight': True,
                    'C2CInform': 0,
                    'C2PRoof': True,
                    'C2PCopilot': True,
                    'C2PLeft': True,
                    'C2PRight': True,
                    'C2PInform': 0,
                    'TextRGBA': [99, 99, 99, 0.7],
                    'LightRGBA': [99, 99, 99, 0.7]
                    }
                hmi = {
                    'ID': 'ego',  # 标识符
                    'IsShowHMI': True,  # 是否显示HMI界面
                    'HMIInform': 0,  # HMI界面信息提示, 其值为:
                                     # 0 - 注意左侧车辆！
                                     # 1 - 注意前方行人！
                                     # 2 - 注意左侧车辆和前方行人！
                    'AudioInform': 0  # 音频信息提示, 与HMIInform对应, 用于播放相应的提示音, 其值为:
                                      # 0 - 播放"注意左侧车辆"的提示音
                                      # 1 - 播放"注意前方行人"的提示音
                                      # 2 - 播放"注意左侧车辆和前方行人"的提示音
                                      # 3 - 无提示音
                                      注：目前的0，1，2的提示音都为相同的提示铃声，但先预留语音提示的接口
        }
                """
        else:  # HMI is on and is within the lasting duration
            t_hmi_remaining -= (time_duration[-1] - time_duration[-2])  # update lasting time
            hmi[2] = 3  # TODO: allocate value 3 for 'AudioInform' for not playing audio again during HMI lasting

            # retrieve message for plotting
            if hmi[1] == 0:
                caution_messages.append('Caution: Left Vehicle Approaching!')
                caution_messages.append('注意左侧来车！')
            elif hmi[1] == 1:
                caution_messages.append('Caution: Approaching Pedestrian!')
                caution_messages.append('注意前方行人！')
            elif hmi[1] == 2:
                caution_messages.append('Caution: Left Vehicle Approaching!')
                caution_messages.append('注意左侧来车！')
                caution_messages.append('Caution: Approaching Pedestrian!')
                caution_messages.append('注意前方行人！')

    if need_plot:
        # Define the font properties
        font = FontProperties(fname='C:/Windows/Fonts/simhei.ttf', size=14)

        plt.cla()
        #plt.style.use('seaborn-v0_8-whitegrid')

        # Use a colormap to differentiate the trajectories
        colors = plt.cm.viridis(np.linspace(0, 1, len(agents_history)))

        plt.plot([x for x, _ in ref_ego], [y for _, y in ref_ego], color='gray', alpha=0.3)  # ego
        plt.plot([x for x, _ in ref_ego_left], [y for _, y in ref_ego_left], color='gray', alpha=0.3)  # ego left
        plt.plot([x for x, _ in ref_ped], [y for _, y in ref_ped], color='gray', alpha=0.3)  # pedestrian

        # Assuming 'ego' has at least one position recorded
        if len(agents_history['ego']) > 0:
            ego_current_position = agents_history['ego'][-1]  # Get the current (last) position of 'ego'

            # Define the window size around the 'ego' position
            window_size = 8000  # unit: cm

            # Set the view limits around the 'ego' position
            plt.xlim(ego_current_position[0] - window_size / 2, ego_current_position[0] + window_size / 2)
            plt.ylim(ego_current_position[1] - window_size / 2, ego_current_position[1] + window_size / 2)

        for idx, (role, trj) in enumerate(agents_history.items()):
            if len(trj) > 9:  # Check if trajectory is not empty
                trj = np.array(trj[-10:])
                for i in range(1, 10):  # Consider the last 10 positions
                    size = [200, 450]
                    if role == 'ped':
                        size = [50, 50]
                    pos1 = trj[i - 1, :]
                    pos2 = trj[i, :]
                    heading = calculate_heading(pos1, pos2)

                    # Create a rectangle patch
                    rect = patches.Rectangle((pos1[0] + size[0] / 2, pos1[1] + size[1] / 2), size[0], size[1],
                                             angle=heading + 90, linewidth=1, edgecolor='r',
                                             facecolor=colors[idx % len(colors)],
                                             alpha=0.7 * (i / 10))
                    ax.add_patch(rect)

        # Define HMI window properties
        position_offset = 800
        hmi_window_position = (ego_trj[-1, 0] + position_offset, ego_trj[-1, 1])
        hmi_window_size = (1000, 500)  # Width, Height in plot units
        hmi_background_color = 'white'
        hmi_text_color = 'red'
        hmi_alpha = 0.7  # Transparency of the background

        # Create the HMI window rectangle
        hmi_rect = patches.Rectangle(hmi_window_position, *hmi_window_size, linewidth=1,
                                     edgecolor='white', facecolor=hmi_background_color, alpha=hmi_alpha)
        ax.add_patch(hmi_rect)

        # Display caution messages within the HMI window
        text_x, text_y = hmi_window_position[0] + 50, hmi_window_position[1] + hmi_window_size[1] - 100
        for message in caution_messages:
            plt.text(text_x, text_y, message, fontsize=10, color=hmi_text_color, ha='left', va='top',
                     fontproperties=font)
            text_y -= 250  # Adjust for next line of text

        # Enhancements for the plot
        ax.set_aspect('equal')
        plt.title('Agent Trajectories Centered on Ego', fontsize=16)
        plt.xlabel('X Position', fontsize=14)
        plt.ylabel('Y Position', fontsize=14)
        plt.grid(False)
        plt.pause(0.1)
        # plt.savefig(f'{output_path}/{t}.png')

    return ehmi, hmi, t_hmi_remaining, safety_index


if __name__ == '__main__':

    for file_name in ['requirement']:

        " ===== Get recorded data ===== "
        file_path = f'{file_name}.txt'
        output_path = f'./figures/{file_name}/'
        if not os.path.exists(output_path):
            os.makedirs(output_path)
        fig, ax = plt.subplots()
        fig.set_size_inches(10, 6)

        " ===== Initialization ===== "
        t = 0  # time index for data reading
        durations = []
        agents_history = {}
        t_hmi_remaining = 0  # set hmi lasting duration (2 second at least)
        ehmi = [False, False, False, 0, False, False, False, False, 0, [99, 99, 99, 0.7], [99, 99, 99, 0.7]]
        hmi = [False, 0, 0]
        outputs_history = [ehmi, hmi, 0]
        safety_index_history = []  # record data for robust hmi control

        " ===== Start loop ===== "
        for row_from_records in open(file_path,'r').readlines():
            parse_input_data(row_from_records, agents_history, durations)
            # print(f"current time: {durations[-1]} s")

            # Generate ehmi and hmi given enough observation
            if len(durations) > 3 and durations[-1] > 12:
                try:
                    ehmi_info, hmi_info, t_hmi_remaining, safety_index = generate_hmi_info(agents_history, durations,
                                                                                        outputs_history,
                                                                                        safety_index_history,
                                                                                        need_plot=True)
                    # save outputs for reference
                    outputs_history = [ehmi_info, hmi_info, t_hmi_remaining]
                    safety_index_history.append(safety_index)
                except:
                    pass
            t += 1
