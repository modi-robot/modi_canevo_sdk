#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "$SCRIPT_DIR/.." && pwd)"
LOG_DIR="$REPO_ROOT/build/logs"
KEEP=20
SIZE="10M"
CAN_IF="${1:-can0}"
CLEAN_PID=""

mkdir -p "$LOG_DIR"

if ! command -v candump >/dev/null 2>&1; then
  echo "错误: 找不到 candump，请先安装 can-utils" >&2
  exit 1
fi

if ! command -v rotatelogs >/dev/null 2>&1; then
  echo "错误: 找不到 rotatelogs，请先安装 apache2-utils/apache2-bin" >&2
  exit 1
fi

if ! ip link show "$CAN_IF" >/dev/null 2>&1; then
  echo "错误: CAN 接口不存在: $CAN_IF" >&2
  exit 1
fi

if ! ip link show "$CAN_IF" | grep -q "state UP"; then
  echo "警告: $CAN_IF 可能没有 UP，candump 可能会立即退出" >&2
fi

cleanup_old_logs() {
  while true; do
    ls -1t "$LOG_DIR"/canevo_can.*.log 2>/dev/null |
        tail -n +"$((KEEP + 1))" |
        xargs -r rm --
    sleep 10
  done
}

cleanup() {
  trap - INT TERM EXIT
  if [[ -n "$CLEAN_PID" ]]; then
    kill "$CLEAN_PID" 2>/dev/null || true
    wait "$CLEAN_PID" 2>/dev/null || true
  fi
}

trap cleanup INT TERM EXIT

cleanup_old_logs &
CLEAN_PID=$!

echo "开始抓取 $CAN_IF，日志目录: $LOG_DIR"
echo "单文件大小: $SIZE，最多保留: $KEEP 个文件"
echo "按 Ctrl+C 停止"

set +e
candump -tz -L "$CAN_IF" |
    tee >(rotatelogs -l "$LOG_DIR/canevo_can.%Y%m%d_%H%M%S.log" "$SIZE")
STATUS=$?
set -e

echo "candump 已退出, status=$STATUS"
exit "$STATUS"
