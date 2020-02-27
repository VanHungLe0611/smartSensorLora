#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# File            : mv_cam_boards.py
# Author          : Duy Anh Pham <duyanh.y4n.pham@gmail.com>
# Date            : 22.11.2019
# Last Modified By: Duy Anh Pham <duyanh.y4n.pham@gmail.com>
import os
boards_dir = os.path.dirname(os.path.abspath(__file__))

boards_properties = {
        'stm32f767ZITx': {
            'MCU':'STM32F767xx',
            'MCU_SERIES':'STM32F7xx',
            'CPU':'cortex-m7',
            'FPU':'fpv5-sp-d16',
            'STM_HAL_DIR':'STM32F7xx_HAL_Driver',
            'CMSIS_DIR':'CMSIS/Device/ST/STM32F7xx',
            # 'ARM_MATH':'ARM_MATH_CM7',
            'HAL_INC':'stm32f7xx_hal.h',
            # 'CFLAGS_MCU':'MCU_SERIES_F7',
            'STARTUP':'startup_stm32f767xx.s',
            'LINKER':'STM32F767ZITx_FLASH.ld',
            'FreeRTOS_DIR' : 'ARM_CM7/r0p1'},
         'stm32l496RGTx': {
            'MCU':'STM32L496xx',
            'MCU_SERIES':'STM32L4xx',
            'CPU':'cortex-m7',
            'FPU':'fpv5-sp-d16',
            'STM_HAL_DIR':'STM32L4xx_HAL_Driver',
            'CMSIS_DIR':'CMSIS/Device/ST/STM32L4xx',
            # 'ARM_MATH':'ARM_MATH_CM7',
            'HAL_INC':'stm32f7xx_hal.h',
            # 'CFLAGS_MCU':'MCU_SERIES_L4',
            'STARTUP':'startup_stm32l496xx.s',
            'LINKER':'STM32L496RGTx_FLASH.ld',
            'FreeRTOS_DIR' : 'ARM_CM7/r0p1'},
        'stm32f746ZGTx': {
            'MCU':'STM32F746xx',
            'MCU_SERIES':'STM32F7xx',
            'CPU':'cortex-m7',
            'FPU':'fpv5-sp-d16',
            'STM_HAL_DIR':'STM32F7xx_HAL_Driver',
            'CMSIS_DIR':'CMSIS/Device/ST/STM32F7xx',
            # 'ARM_MATH':'ARM_MATH_CM7',
            'HAL_INC':'stm32f7xx_hal.h',
            # 'CFLAGS_MCU':'MCU_SERIES_F7',
            'STARTUP':'startup_stm32f746xx.s',
            'LINKER':'STM32F746ZGTx_FLASH.ld',
            'FreeRTOS_DIR':'ARM_CM7/r0p1',
            },
        'stm32h743IITx': {
            'MCU':'STM32H743xx',
            'MCU_SERIES':'STM32H7xx',
            'CPU':'cortex-m7',
            'FPU':'fpv5-sp-d16',
            'STM_HAL_DIR':'STM32H7xx_HAL_Driver',
            'CMSIS_DIR':'CMSIS/Device/ST/STM32H7xx',
            # 'ARM_MATH':'ARM_MATH_CM7',
            'HAL_INC':'stm32h7xx_hal.h',
            # 'CFLAGS_MCU':'MCU_SERIES_H7',
            'STARTUP':'startup_stm32h743xx.s',
            'LINKER':'STM32H743IITx_FLASH.ld',
            'FreeRTOS_DIR':'ARM_CM4F',
            },
        }


def moveBoardFiles(src, board_files):
    global boards_dir
    board_file_to_be_moved = ['LINKER', 'STARTUP']
    board_file_dst_path = ['linker', 'startup']
    for _dir in board_file_dst_path:
        if _dir not in os.listdir(boards_dir):
            print(_dir + ' folder not existed -> create folder:' + dir)
            os.mkdir(os.path.join(boards_dir, _dir))
    for file_to_be_moved, file_dst_path in zip(board_file_to_be_moved, board_file_dst_path):
        for board in boards_properties.values():
            if board[file_to_be_moved] in board_files and not board[file_to_be_moved] in os.listdir(os.path.join(boards_dir, file_dst_path)):
                src_path = os.path.abspath(board[file_to_be_moved])
                dst_path = os.path.join(boards_dir, file_dst_path, board[file_to_be_moved])
                print('move: ' + src_path)
                print('->to: ' + dst_path)
                os.rename(src_path, dst_path)
            elif board[file_to_be_moved] in board_files and board[file_to_be_moved] in os.listdir(os.path.join(boards_dir, file_dst_path)):
                promt = input(board[file_to_be_moved] + ' existed in mv_cam_boards/. Overwrite ' + '?(y/N)')
                if promt == 'y':
                    src_path = os.path.abspath(board[file_to_be_moved])
                    dst_path = os.path.join(boards_dir, file_dst_path, board[file_to_be_moved])
                    print('move: ' + src_path)
                    print('->to: ' + dst_path)
                    os.rename(src_path, dst_path)
                else:
                    promt = input('Remove ' + board[file_to_be_moved] + ' in project folder?(y/N)')
                    if promt == 'y':
                        print('->rm: ' + os.path.join(src, board[file_to_be_moved]))
                        os.remove(os.path.join(src, board[file_to_be_moved]))


def manageBoardsFolder(project_dir='./'):
    global boards_properties, boards_dir
    if project_dir == boards_dir:
        raise Exception('Board dir must be subfolder of project_dir')
    board_files = [_file for _file in os.listdir(project_dir) if ('.s' in _file or '.ld' in _file) and 'stm32' in _file.lower()]
    if len(board_files) > 0:
        print(board_files)
        moveBoardFiles(project_dir, board_files)
    else:
        print('No new generated board files(inker, startup ...) found')


def main():
    manageBoardsFolder(os.getcwd())


if __name__ == "__main__":
    main()
