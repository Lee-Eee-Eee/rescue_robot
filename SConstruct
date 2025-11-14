#coding=utf-8
import os
import sys
from SCons.Script import *
import json

DEVEL_BOARDS_DICT = {
    "MFM1": {
        "chip": "stm32",
        "type": "STM32F407VET6",
        "description": "Competition kit."
        },
    }

with open('boards.json', 'r') as file:
    DEVEL_BOARDS_DICT = json.load(file)

if os.environ.get("BOARD_NAME"):
    BOARD = os.environ.get("BOARD_NAME")
    CHIP = DEVEL_BOARDS_DICT[BOARD]["chip"].lower()
else:
    try:
        with open('.config', 'r') as config_file:
            config_content = config_file.readlines()
    except:
        config_content = None
        print("Open .config file failed.")

    default_board = 0
    print("Please select development board:")
    for index, board in enumerate(DEVEL_BOARDS_DICT.keys()):

        board_info = "{:2}: {}, Adopted chip {}, {}" \
            .format(index, board, DEVEL_BOARDS_DICT[board]["type"], DEVEL_BOARDS_DICT[board]["description"])
        if config_content and any(board in line for line in config_content):
            default_board = index
            board_info += "(*)"
        print(board_info)

    try:
        user_input = input("Choose a development board to use? [{}]: ".format(default_board))
    except Exception as e:
        user_input = default_board
    else:
        user_input = int(user_input) if user_input else default_board
    BOARD = list(DEVEL_BOARDS_DICT.keys())[user_input]
    os.environ["BOARD_NAME"] = BOARD
    CHIP = DEVEL_BOARDS_DICT[BOARD]["chip"].lower()
    
BOARD_PATH = os.path.join(os.getcwd(), 'boards', CHIP, BOARD)
RTT_PATH = os.path.join(os.getcwd(), 'rt-thread')
KCONFIG_PATH = os.path.join(os.getcwd(), ".config")

os.environ["BSP_ROOT"] = BOARD_PATH
os.environ["RTT_ROOT"] = RTT_PATH
os.environ["KCONFIG_CONFIG"] = KCONFIG_PATH

sys.path.insert(0, os.getenv("BSP_ROOT"))

os.chdir(os.getenv("BSP_ROOT"))

print("{} firmware building...".format(BOARD))

import build

