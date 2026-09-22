#!/usr/bin/env bash
set -euo pipefail

BASHRC="$HOME/.bashrc"
MARKER_START="# >>> wust_vision dev >>>"
MARKER_END="# <<< wust_vision dev <<<"
BACKUP="$BASHRC.bak.$(date +%Y%m%d_%H%M%S)"

# 备份
cp "$BASHRC" "$BACKUP"
echo "💾 已备份 $BASHRC 到 $BACKUP"

# 临时文件
TMP=$(mktemp)

# 写入 bashrc 除去旧 block
awk -v start="$MARKER_START" -v end="$MARKER_END" '
  $0 == start {inside=1; next}
  inside && $0 == end {inside=0; next}
  !inside {print}
' "$BASHRC" > "$TMP"

# 使用 cat <<EOF 方式追加 block，保证特殊字符安全
cat <<'EOF' >> "$TMP"
# >>> wust_vision dev >>>
py() { 
    source ~/anaconda3/etc/profile.d/conda.sh
    echo "已激活 conda, conda activate激活环境，conda deactivate退出环境， conda info --envs查看环境"
}

buildme() { 
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
}

builddebug() { 
    colcon build --packages-select "$1" --cmake-args -DCMAKE_BUILD_TYPE=Debug
}

killros() { 
    pkill -f ros && ros2 daemon stop && ros2 daemon start
}

format() { 
    find . -path ./build -prune -o \
    -type f \( -name '*.h' -o -name '*.hpp' -o -name '*.c' -o -name '*.cu' -o -name '*.cpp' \) \
    -exec clang-format -i {} +
}

s() { 
    source install/setup.bash
}

hik() { 
    export MVCAM_SDK_PATH=/opt/MVS
    export MVCAM_COMMON_RUNENV=/opt/MVS/lib
    export MVCAM_GENICAM_CLPROTOCOL=/opt/MVS/lib/CLProtocol
    export ALLUSERSPROFILE=/opt/MVS/MVFG
    export LD_LIBRARY_PATH=/opt/MVS/lib/64:/opt/MVS/lib/32:$LD_LIBRARY_PATH
}
# <<< wust_vision dev <<<
EOF

# 原子替换
mv "$TMP" "$BASHRC"
echo "✅ 已安全替换或追加 block 到 $BASHRC"
