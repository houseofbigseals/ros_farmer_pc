# import glob
import time


# def read_temp_raw(name):
#     device_file = '/sys/bus/w1/devices/{}/w1_slave'.format(name)
#     # device_folder = glob.glob(base_dir + '28-0301a279ab3b')[0]
#     # device_file = device_folder + '/w1_slave'
#
#     f = open(device_file, 'r')
#     lines = f.readlines()
#     f.close()
#     return lines


def ds18b20_read_temp(devname, restart_count=10, error_code=-65536):

    device_file = '/sys/bus/w1/devices/{}/w1_slave'.format(devname)
    f = open(device_file, 'r')
    lines = f.readlines()
    f.close()
    err_count = 0
    while lines[0].strip()[-3:] != 'YES' and err_count <= restart_count:
        # we want to see in this file something like this:
        # 09 02 55 05 7f a5 a5 66 5c : crc=5c YES
        # 09 02 55 05 7f a5 a5 66 5c t=32562

        # we will read from file until we don`t get YES in first line
        time.sleep(0.2)
        f = open(device_file, 'r')
        lines = f.readlines()
        f.close()
        err_count += 1
    if err_count <= restart_count:
        equals_pos = lines[1].find('t=')
        if equals_pos != -1:
            temp_string = lines[1][equals_pos + 2:]
            temp_c = float(temp_string) / 1000.0
            return temp_c
        else:
            return error_code
    else:
        return error_code


if __name__ == "__main__":
    while True:
        print(ds18b20_read_temp(devname='28-0301a279ab3b'))
        time.sleep(1)


