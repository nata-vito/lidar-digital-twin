"""
Developed by: BRAIN
--------------------------------------------------------------
Developers: Natanael Vitorino
---
e-mail: natanael.vitorino@facens.br
---
BRAIN, Sorocaba, Brazil, 2024
----------------
CAN DATA Request
"""
import cantools as dbc
import can

def load_dbc(dbc_path):
    """
    To load CAN database

    ---
    Args:
        dbc_path: dbc file path

    Returns:
        Car data msg received.
    """

    # Create dict to save data here
    car_data = {
        "YawRate": None,
        "EgoVelocity": None,
        "LongAcc": None,
        "LatAcc": None,
        "GasPosition": None,
        "TurnLeftLight":None
    }
    print(car_data)
    # Loading DBC file "CAN msg dict"
    car_dbc = dbc.db.load_file(dbc_path)
    car_vel = car_dbc.get_message_by_name('BRAKE1')
    car_turn_light = car_dbc.get_message_by_name('BCM_COMMAND')
    car_yaw = car_dbc.get_message_by_name('BSM_YRS_DATA')
    car_pedal_position = car_dbc.get_message_by_name('ENGINE1')

    # Setting up CAN bus
    can_bus = can.Bus(channel='can0', interface='socketcan')
    can_bus.set_filters([
        {"can_id":car_vel.frame_id, "can_mask":0xFFF},
        {"can_id":car_turn_light.frame_id, "can_mask":0xFFF}
    ])

    # Data processing and extration
    car_data_received = get_can_data(car_data, can_bus, car_vel, car_turn_light, car_yaw, car_pedal_position)

    print(f'{car_data_received}\n')

    return car_data_received


def get_can_data(car_data, can_bus, car_vel, car_turn_light, car_yaw, car_pedal_position):
    """
    To get specific CAN message data

    ---
    Args:
        car_data: data to extract from CAN Bus
        can_bus: data resource
        car_vel: car ego velocity CAN message dict
        car_turn_light: car turn light CAN message dict
        car_yaw: car yaw CAN message dict
        
    Returns:
        Car data dict    
    """

    while True:

        # Get msg from CAN Bus
        can_msg = can_bus.recv()

        # save ego velocity received into data dict created here
        if can_msg.arbitration_id == car_vel.frame_id:
            car_vel_received = car_vel.decode(can_msg.data)
            car_data["EgoVelocity"] = car_vel_received["VehicleSpeedVSOSig"]

        # save car turn light data received into data dict created here
        elif can_msg.arbitration_id == car_turn_light.frame_id:
            car_turn_light_received = car_turn_light.decode(can_msg.data)
            car_data["TurnLeftLight"] = car_turn_light_received["TurnIndicatorSts"]

        # save car turn light data received into data dict created here
        elif can_msg.arbitration_id == car_yaw.frame_id:
            car_yaw_received = car_yaw.decode(can_msg.data)
            car_data["YawRate"] = car_yaw_received["YawRate_BSM"]
            car_data["LongAcc"] = car_yaw_received["LongAcceleration_BSM"]
            car_data["LatAcc"] = car_yaw_received["LatAcceleration_BSM"]
            
        # save car turn light data received into data dict created here
        elif can_msg.arbitration_id == car_pedal_position.frame_id:
            car_pedal_position_received = car_pedal_position.decode(can_msg.data)
            car_data["GasPosition"] = car_pedal_position_received["GasPedalPosition"]
          

    return car_data

if __name__ == '__main__':
    load_dbc('./data/DBC2.dbc')