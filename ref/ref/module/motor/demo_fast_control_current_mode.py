from fi_fsa import fi_fsa_v2
import time

server_ip_list = ['192.168.137.101', '192.168.137.102', '192.168.137.103',
                  '192.168.137.104', '192.168.137.105', '192.168.137.106',
                  '192.168.137.107', '192.168.137.108', '192.168.137.109',
                  '192.168.137.110', '192.168.137.111', '192.168.137.112']

def main():
    # 使能所有电机
    for i in range(len(server_ip_list)):
        fi_fsa_v2.fast_set_enable(server_ip_list[i])

    # 设置所有电机为电流控制模式
    for i in range(len(server_ip_list)):
        fi_fsa_v2.fast_set_mode_of_operation(
            server_ip_list[i], fi_fsa_v2.FSAModeOfOperation.CURRENT_CONTROL
        )

    # 设置电机电流为0.0
    for i in range(len(server_ip_list)):
        fi_fsa_v2.fast_set_current_control(server_ip_list[i], 0.0)
        time.sleep(1)

    # 设置电机电流为0.3（持续一段时间，这里简化为2秒，每秒10次设置）
    for _ in range(500):
        for i in range(len(server_ip_list)):
            fi_fsa_v2.fast_set_current_control(server_ip_list[i], 0.8)
        time.sleep(0.1)

    # 最后将电机电流设置为0.0
    for i in range(len(server_ip_list)):
        fi_fsa_v2.fast_set_current_control(server_ip_list[i], 0.0)
        time.sleep(1)

    # 关闭所有电机
    for i in range(len(server_ip_list)):
        fi_fsa_v2.fast_set_disable(server_ip_list[i])


if __name__ == "__main__":
    main()
