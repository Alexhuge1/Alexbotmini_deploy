from fi_fsa import fi_fsa_v2

server_ip_list = []
motors_num = 0

def main():
    global server_ip_list, motors_num
    server_ip_list= fi_fsa_v2.broadcast_func_with_filter(filter_type="Actuator")
    if server_ip_list:
        motors_num = len(server_ip_list)

if __name__ == "__main__":
    main()
    print("Server IP List:", server_ip_list)
    print("Motors Num:", motors_num)
