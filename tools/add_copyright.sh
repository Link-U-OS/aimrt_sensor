#!/bin/bash
set -e

# ===============================
# 脚本位置：<repo_root>/tools/
# 目标目录：<repo_root>/*
# ===============================

# 获取脚本自身所在目录
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 仓库根目录（tools 的上一级）
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

# ===== 固定跳过目录（相对于仓库根目录）=====
SKIP_DIRS=(
    "$REPO_ROOT/bazel-aima_sensor"
    "$REPO_ROOT/bazel-bin"
    "$REPO_ROOT/bazel-out"
    "$REPO_ROOT/bazel-testlogs"
    "$REPO_ROOT/src/thirdparty"
    "$REPO_ROOT/src/mid360_lidar/rs_driver"
)

COPYRIGHT_LINE1="// Copyright (c) 2025, AgiBot Inc."
COPYRIGHT_LINE2="// All rights reserved."

# ===== 构造 find 排除规则 =====
EXCLUDE_ARGS=()
for dir in "${SKIP_DIRS[@]}"; do
    if [ -d "$dir" ]; then
        EXCLUDE_ARGS+=( -path "$dir" -prune -o )
    fi
done

# ===== 执行遍历 =====
find "$REPO_ROOT" \
    "${EXCLUDE_ARGS[@]}" \
    -type f \( -name "*.cpp" -o -name "*.hpp" \) -print | \
while read -r file; do

    # 防止重复插入
    if head -n 2 "$file" | grep -qF "$COPYRIGHT_LINE1"; then
        echo "Skip: $file"
        continue
    fi

    tmpfile=$(mktemp)
    {
        echo "$COPYRIGHT_LINE1"
        echo "$COPYRIGHT_LINE2"
        echo
        cat "$file"
    } > "$tmpfile"

    mv "$tmpfile" "$file"
    echo "Updated: $file"
done
