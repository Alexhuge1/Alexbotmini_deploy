from fi_fsa import fi_fsa_v2
import time
from utils import clear_screen
server_ip_list = ['192.168.137.101', '192.168.137.102', '192.168.137.103',
                  '192.168.137.104', '192.168.137.105', '192.168.137.106',
                  '192.168.137.107', '192.168.137.108', '192.168.137.109',
                  '192.168.137.110', '192.168.137.111', '192.168.137.112']
motors_num = 12

def main():
    server_ip_list = fi_fsa_v2.broadcast_func_with_filter(filter_type="Actuator")
    clear_screen()     
    if server_ip_list:
        motors_num = len(server_ip_list)
        if motors_num == 12:
            server_ip_list = ['192.168.137.101', '192.168.137.102', '192.168.137.103',
                  '192.168.137.104', '192.168.137.105', '192.168.137.106',
                  '192.168.137.107', '192.168.137.108', '192.168.137.109',
                  '192.168.137.110', '192.168.137.111', '192.168.137.112']
            print('find 12 motors, and pvc is:')
            for i in range(len(server_ip_list)):
                position, velocity, current = fi_fsa_v2.fast_get_pvc(server_ip_list[i])
                print(
                    "Position = %f, Velocity = %f, Current = %.4f"
                    % (position, velocity, current)
                )
        else:
            raise ValueError('Lost connection of motors. The number of motors is not 12.')
                

if __name__ == "__main__":
    main()
