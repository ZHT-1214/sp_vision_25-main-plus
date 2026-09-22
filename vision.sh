#!/bin/bash
# 使用无密码的sudo或直接使用root权限
chmod 666 /dev/ttyACM* 2>/dev/null || true

# Start vision
cd /home/rm/桌面/sp_vision_25-main-plus
exec ./build/auto_aim_debug_mpc
