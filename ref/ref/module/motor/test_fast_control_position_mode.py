from fi_fsa import fi_fsa_v2
import time
import math
server_ip_list = ['192.168.137.101','192.168.137.102','192.168.137.103',
                  '192.168.137.104','192.168.137.105','192.168.137.106',
                  '192.168.137.107','192.168.137.108','192.168.137.109',
                  '192.168.137.110','192.168.137.111','192.168.137.112'] 
def main():
    server_ip_list = fi_fsa_v2.broadcast_func_with_filter(filter_type="Actuator")
    if server_ip_list:
        # enable all the motors
        for i in range(len(server_ip_list)):
            fi_fsa_v2.fast_set_enable(server_ip_list[i])

        # set work at position control mode
        for i in range(len(server_ip_list)):
            fi_fsa_v2.fast_set_mode_of_operation(
                server_ip_list[i], fi_fsa_v2.FSAModeOfOperation.POSITION_CONTROL
            )

        # change motors into target position
        for i in range(len(server_ip_list)):
            set_position = 0 # [deg]
            fi_fsa_v2.fast_set_position_control(server_ip_list[i], set_position)


if __name__ == "__main__":
    main()
