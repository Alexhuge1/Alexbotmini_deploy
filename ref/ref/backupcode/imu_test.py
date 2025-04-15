import sys
import os
import click
from module.imu.utils import check_python_version
from module.imu.commands.cmd_list import cmd_list
from module.imu.commands.read_data import cmd_read
from module.imu.commands.cmd_send import cmd_send

@click.group()
def cli():
    """HiPNUC Python Example"""
    check_python_version()

cli.add_command(cmd_list)
cli.add_command(cmd_read)
cli.add_command(cmd_send)

if __name__ == "__main__":
    cli()
# first ask: which python
# test code: sudo /home/alexhuge/Documents/github/sim2real/sim2real-main/.venv/bin/python imu_t.py read -p /dev/ttyUSB0 -b 115200