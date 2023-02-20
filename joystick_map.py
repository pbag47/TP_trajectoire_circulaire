# from typing import List


def joystick_map(device_name: str):
    mode = 2
    if device_name == '5-Axis,12-Button with POV ':
        axis_names = {
            0x00: 'roll',
            0x01: 'pitch',
            0x02: 'height',
            0x05: '/yaw (broken)',
            0x06: 'throttle',
        }

        button_names = {
            0x120: 'Stop',  # 1 / trigger
            0x121: 'Takeoff / Land',  # 2
            0x122: 'Standby',  # 3
            0x123: 'Manual flight',  # 4
            0x124: 'Yaw-',  # 5
            0x125: 'Yaw+',  # 6
            0x126: 'Point of interest',  # 7
            0x127: '8',  # 8
            0x128: 'Circle with tangent x axis',  # 9
            0x129: '10',  # 10
            0x12a: 'Circle',  # 11
            0x12b: '12',  # 12
        }

    elif device_name == 'Logitech Logitech Extreme 3D':
        axis_names = {
            0x00: 'roll',
            0x01: 'pitch',
            0x05: '/yaw',
            0x06: 'height',
            0x10: 'hat0x',
            0x11: 'hat0y'
        }

        button_names = {
            0x120: 'Stop',  # 1 / trigger
            0x121: 'Takeoff / Land',  # 2
            0x122: 'Standby',  # 3
            0x123: 'Manual flight',  # 4
            0x124: 'Yaw-',  # 5
            0x125: 'Yaw+',  # 6
            0x126: 'Point of interest',  # 7
            0x127: '8',  # 8
            0x128: 'Circle with tangent x axis',  # 9
            0x129: '10',  # 10
            0x12a: 'Circle',  # 11
            0x12b: '12',  # 12
        }

    elif device_name == 'Microsoft X-Box One S pad' and mode == 1:
        axis_names = {
            0: 'roll',
            1: 'pitch',
            2: 'yaw-',
            3: 'yaw',
            4: 'height2',
            5: 'yaw+',
        }

        button_names = {
            0: 'Takeoff / Land',  # a
            1: 'Point of interest',  # b
            2: 'Circle',  # x
            3: 'Circle with tangent x axis',  # y
            4: 'Standby',  # lx
            5: 'Manual flight',  # rx
            6: 'select',  # select
            7: 'Stop',  # start
            8: 'thumb_left',  # thumb left
            9: 'thumb_right',  # thumb right
            # 0x13c: 'mode',  # mode
        }

    elif device_name == 'Microsoft X-Box One S pad' and mode == 2:
        axis_names = {
            0x00: 'yaw',
            0x01: 'height2',
            0x02: 'yaw-',
            0x03: 'roll',
            0x04: 'pitch',
            0x05: 'yaw+',
            0x10: 'hat0x',
            0x11: 'hat0y',
        }

        button_names = {
            0: 'Takeoff / Land',  # a
            1: 'Point of interest',  # b
            2: 'Circle',  # x
            3: 'Circle with tangent x axis',  # y
            4: 'Standby',  # lx
            5: 'Manual flight',  # rx
            6: 'select',  # select
            7: 'Stop',  # start
            8: 'thumb_left',  # thumb left
            9: 'thumb_right',  # thumb right
            # 0x13c: 'mode',  # mode
        }

    else:
        print('Warning :', device_name, 'not configured, using standard buttons and axis settings')
        axis_names = {
            0x00: 'roll',
            0x01: 'pitch',
            0x02: 'height',
            0x03: 'rx',
            0x04: 'ry',
            0x05: 'yaw',
            0x06: 'throttle',
            0x07: 'rudder',
            0x08: 'wheel',
            0x09: 'gas',
            0x0a: 'brake',
            0x10: 'hat0x',
            0x11: 'hat0y',
            0x12: 'hat1x',
            0x13: 'hat1y',
            0x14: 'hat2x',
            0x15: 'hat2y',
            0x16: 'hat3x',
            0x17: 'hat3y',
            0x18: 'pressure',
            0x19: 'distance',
            0x1a: 'tilt_x',
            0x1b: 'tilt_y',
            0x1c: 'tool_width',
            0x20: 'volume',
            0x21: 'misc',
        }

        button_names = {
            0x120: 'trigger',
            0x121: '2',
            0x122: '3',
            0x123: '4',
            0x124: '5',
            0x125: '6',
            0x126: '7',
            0x127: '8',
            0x128: '9',
            0x129: '10',
            0x12a: '11',
            0x12b: '12',
            0x12f: 'dead',
            0x130: 'a',
            0x131: 'b',
            0x132: 'c',
            0x133: 'x',
            0x134: 'y',
            0x135: 'z',
            0x136: 'tl',
            0x137: 'tr',
            0x138: 'tl2',
            0x139: 'tr2',
            0x13a: 'select',
            0x13b: 'start',
            0x13c: 'mode',
            0x13d: 'thumb_left',
            0x13e: 'thumb_right',

            0x220: 'dpad_up',
            0x221: 'dpad_down',
            0x222: 'dpad_left',
            0x223: 'dpad_right',

            # xBox 360 controller uses these codes.
            0x2c0: 'dpad_left',
            0x2c1: 'dpad_right',
            0x2c2: 'dpad_up',
            0x2c3: 'dpad_down'
        }
    return button_names, axis_names
