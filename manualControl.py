import sys
import os

# Predefined options for joints and positions
joint_options = [
    "jnt_baseArm", "jnt_upperArmY", "jnt_foreArmY", 
    "jnt_wrist1Y", "jnt_wrist2X", "jnt_wrist3Z", 
    "jaw", "jnt_combo"
]

position_options = ["desGripperTipPos", "desJntGripperPos", "desJntWristPos"]

# Current values and delta defaults
joint_ids = [31, 41, 32, 51, 52, 53, 54]
current_values = {joint: (joint_ids[i], 0) for i, joint in enumerate(joint_options) if joint != "jnt_combo"}
current_values["jnt_combo"] = None  # No ID associated with jnt_combo
current_values.update({pos: [0, 0, 0] for pos in position_options})
delta_defaults = {joint: 1 for joint in joint_options}
delta_defaults.update({pos: [1, 1, 1] for pos in position_options})
saved_values = {
    "jnt_combo": {
        "A": [(51, -40), (52, 225), (53, 180), (54,280) ], 
        "B": [(51, 45), (52, 225), (53, 180), (54, -720)], 
        "C": [(51, 45), (52, 260), (53, 100), (54, 280)],
        "D": [(3, 70), (31, 80)]
    },
    "positions": {
        "a1": [0, 0, 0],
        "a2": [0, 0, 0],
        "a3": [0, 0, 0],
    }
}

def input_with_default(prompt, default):
    return input(f"{prompt} (default: {default}): ") or str(default)

def joint_loop():
    while True:
        print("\nJoint Loop: Select a joint:")
        for i, joint in enumerate(joint_options):
            print(f"{i+1}. {joint}")
        selection = input("> ").strip()
        if selection == 'q': return

        try:
            selected_joint = joint_options[int(selection) - 1]
        except (ValueError, IndexError):
            print("Invalid selection.")
            continue

        if selected_joint == "jnt_combo":
            while True:
                print("Select Option A, B, C or create new in New (n):")
                combo_selection = input("> ").strip()
                if combo_selection == 'q': return
                if combo_selection in saved_values["jnt_combo"]:
                    combo_data = saved_values["jnt_combo"][combo_selection]
                    size_value = len(combo_data) * 2
                    data_values = [item for pair in combo_data for item in pair]
                    print(f'ros2 topic pub -1 /motorCnt std_msgs/msg/Float64MultiArray '
                          f'"{{layout: {{dim: [{{label: \'ID\', size: {size_value//2}}}, '
                          f'{{label: \'Position\', size: {size_value//2}}}] }}, '
                          f'data: {data_values}}}"')
                    os.system(f'ros2 topic pub -1 /motorCnt std_msgs/msg/Float64MultiArray '
                          f'"{{layout: {{dim: [{{label: \'ID\', size: {size_value//2}}}, '
                          f'{{label: \'Position\', size: {size_value//2}}}] }}, '
                          f'data: {data_values}}}"')
                elif combo_selection == 'n':
                    new_set = input("Enter new set of ID-value pairs (comma-separated, e.g., '31,10,41,20'): ")
                    id_values = list(map(int, new_set.split(',')))
                    new_combo = [(id_values[i], id_values[i+1]) for i in range(0, len(id_values), 2)]
                    saved_values["jnt_combo"]["New"] = new_combo
                    size_value = len(new_combo) * 2
                    data_values = [item for pair in new_combo for item in pair]
                    print(f"New set saved: {saved_values['jnt_combo']['New']}")
                    print(f'ros2 topic pub -1 /motorCnt std_msgs/msg/Float64MultiArray '
                          f'"{{layout: {{dim: [{{label: \'ID\', size: {size_value//2}}}, '
                          f'{{label: \'Position\', size: {size_value//2}}}] }}, '
                          f'data: {data_values}}}"')
                    os.system(f'ros2 topic pub -1 /motorCnt std_msgs/msg/Float64MultiArray '
                          f'"{{layout: {{dim: [{{label: \'ID\', size: {size_value//2}}}, '
                          f'{{label: \'Position\', size: {size_value//2}}}] }}, '
                          f'data: {data_values}}}"')
                    
                else:
                    print("Invalid selection.")
        else:
            while True:
                print(f"Selected joint: {selected_joint}")
                current_id, current_value = current_values[selected_joint]
                print(f"Current value: {current_value} (ID: {current_id})")
                mode = input("Choose 'v' for value or 'd' for delta (q to go back): ").strip()
                if mode == 'q': break

                if mode == 'v':
                    while True:
                        value = input("Enter new value: ").strip()
                        if value == 'q': break
                        try:
                            current_values[selected_joint] = (current_id, int(value))
                            print(f'ros2 topic pub -1 /motorCnt std_msgs/msg/Float64MultiArray '
                                  f'"{{layout: {{dim: [{{label: \'ID\', size: 2}}, '
                                  f'{{label: \'Position\', size: 2}}] }}, '
                                  f'data: [{current_id}, {current_values[selected_joint][1]}]}}"')

                            os.system(f'ros2 topic pub -1 /motorCnt std_msgs/msg/Float64MultiArray '
                                  f'"{{layout: {{dim: [{{label: \'ID\', size: 2}}, '
                                  f'{{label: \'Position\', size: 2}}] }}, '
                                  f'data: [{current_id}, {current_values[selected_joint][1]}]}}"')
                        except ValueError:
                            print("Invalid value.")
                elif mode == 'd':
                    while True:
                        delta = input_with_default("Enter delta", delta_defaults[selected_joint])
                        if delta == 'q': break
                        try:
                            delta = int(delta)
                            current_values[selected_joint] = (current_id, current_value + delta)
                            current_value = current_values[selected_joint][1]  # Update current_value with the new value
                            print(f"New value: {current_value}")
                            delta_defaults[selected_joint] = delta
                            print(f'ros2 topic pub -1 /motorCnt std_msgs/msg/Float64MultiArray '
                                  f'"{{layout: {{dim: [{{label: \'ID\', size: 2}}, '
                                  f'{{label: \'Position\', size: 2}}] }}, '
                                  f'data: [{current_id}, {current_value}]}}"')
                            os.system(f'ros2 topic pub -1 /motorCnt std_msgs/msg/Float64MultiArray '
                                  f'"{{layout: {{dim: [{{label: \'ID\', size: 2}}, '
                                  f'{{label: \'Position\', size: 2}}] }}, '
                                  f'data: [{current_id}, {current_value}]}}"')
                        except ValueError:
                            print("Invalid delta.")
                else:
                    print("Invalid mode.")

def position_loop():
    while True:
        print("\nPosition Loop: Select a position:")
        for i, pos in enumerate(position_options):
            print(f"{i+1}. {pos}")
        selection = input("> ").strip()
        if selection == 'q': return

        try:
            selected_pos = position_options[int(selection) - 1]
        except (ValueError, IndexError):
            print("Invalid selection.")
            continue

        while True:
            print(f"Selected position: {selected_pos}")
            print(f"Current value: {current_values[selected_pos]}")
            mode = input("Choose 'v' for value, 'd' for delta, or 'r' to recall saved value (q to go back): ").strip()
            if mode == 'q': break

            if mode == 'v':
                while True:
                    value = input("Enter new x y z values (space-separated): ").strip()
                    if value == 'q': break
                    try:
                        new_values = list(map(int, value.split()))
                        if len(new_values) != 3:
                            raise ValueError("You must provide exactly three values for x, y, z.")
                        current_values[selected_pos] = new_values
                        print(f'ros2 topic pub /{selected_pos} geometry_msgs/msg/Point '
                              f'"{{ x: {new_values[0]}, y: {new_values[1]}, z: {new_values[2]} }}" --once')
                        os.system(f'ros2 topic pub /{selected_pos} geometry_msgs/msg/Point '
                              f'"{{ x: {new_values[0]}, y: {new_values[1]}, z: {new_values[2]} }}" --once')
                    except ValueError:
                        print("Invalid input. Please enter exactly three integers.")
            elif mode == 'd':
                while True:
                    delta = input_with_default("Enter delta (x y z)", " ".join(map(str, delta_defaults[selected_pos])))
                    if delta == 'q': break
                    try:
                        delta = list(map(int, delta.split()))
                        if len(delta) != 3:
                            raise ValueError("You must provide exactly three delta values for x, y, z.")
                        current_values[selected_pos] = [current_values[selected_pos][i] + delta[i] for i in range(3)]
                        print(f"New value: {current_values[selected_pos]}")
                        delta_defaults[selected_pos] = delta
                        print(f'ros2 topic pub /{selected_pos} geometry_msgs/msg/Point '
                              f'"{{ x: {current_values[selected_pos][0]}, y: {current_values[selected_pos][1]}, z: {current_values[selected_pos][2]} }}" --once')
                        os.system(f'ros2 topic pub /{selected_pos} geometry_msgs/msg/Point '
                              f'"{{ x: {current_values[selected_pos][0]}, y: {current_values[selected_pos][1]}, z: {current_values[selected_pos][2]} }}" --once')
                    except ValueError:
                        print("Invalid delta. Please enter exactly three integers.")
            elif mode == 'r':
                while True:
                    recall_option = input("Press 's' to save a new value (a1, a2, a3), or 'r' to recall: ").strip()
                    if recall_option == 'q': break
                    elif recall_option == 's':
                        save_name = input("Choose a save name (a1, a2, a3): ").strip()
                        if save_name in saved_values["positions"]:
                            new_value = input("Enter new x y z values to save (space-separated): ").strip()
                            if all(v.isdigit() for v in new_value.split()):
                                saved_values["positions"][save_name] = list(map(int, new_value.split()))
                                print(f"New value {new_value} saved as {save_name}.")
                            else:
                                print("Invalid value.")
                        else:
                            print("Invalid save name.")
                    elif recall_option == 'r':
                        recall_name = input("Choose a saved value to recall (a1, a2, a3): ").strip()
                        if recall_name in saved_values["positions"]:
                            recalled_value = saved_values["positions"][recall_name]
                            current_values[selected_pos] = recalled_value
                            print(f"Recalled value: {recalled_value}")
                            print(f'ros2 topic pub /{selected_pos} geometry_msgs/msg/Point '
                                  f'"{{ x: {recalled_value[0]}, y: {recalled_value[1]}, z: {recalled_value[2]} }}" --once')
                            os.system(f'ros2 topic pub /{selected_pos} geometry_msgs/msg/Point '
                                  f'"{{ x: {recalled_value[0]}, y: {recalled_value[1]}, z: {recalled_value[2]} }}" --once')
                        else:
                            print("No saved value found or invalid selection.")
            else:
                print("Invalid mode.")

def main_loop():
    while True:
        print("\nMain Loop: Select an option:")
        print("1. Joint")
        print("2. Position")
        selection = input("> ").strip()
        if selection == 'q': break

        if selection == '1':
            joint_loop()
        elif selection == '2':
            position_loop()
        else:
            print("Invalid selection.")

if __name__ == "__main__":
    main_loop()
    print("Program exited.")

