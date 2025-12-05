#!/usr/bin/env bash
set -e

###########################
# 配置区：只改这里就行
###########################

# 原始 PCD 地图文件名（必须放在 map/ 目录下）
MAP_NAME="xw.pcd"

# 体素下采样尺寸（单位：米），三个方向统一使用这个值
VOXEL_LEAF_SIZE=0.05    # 例如 0.05 表示 5cm

# 地图点云发布持续时间（单位：秒），到时间后自动关闭 pcd_to_pointcloud
MAP_PUBLISH_DURATION=3  # 比如 3 秒

# 每次发布的时间间隔（秒），例如 1.0 表示 1 秒发布一次
PCD_PUBLISH_INTERVAL=1.0

###########################
# 自动推导目录
###########################
BASE_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
MAP_DIR="$BASE_DIR/map"

# 原始 PCD 路径
PCD_PATH="$MAP_DIR/$MAP_NAME"

# 下采样后 PCD 名称：前缀 + "_down" + .pcd
# 例如 xw.pcd -> xw_down.pcd
PCD_BASENAME="${MAP_NAME%.*}"
DOWNSAMPLED_NAME="${PCD_BASENAME}_down.pcd"
DOWNSAMPLED_PATH="$MAP_DIR/$DOWNSAMPLED_NAME"

RVIZ_CONFIG="$MAP_DIR/relocalization.rviz"

# 如果想在脚本里顺便 source 自己的 workspace，可以打开这一行：
# source "$BASE_DIR/devel/setup.bash"

###########################
# 简单检查
###########################
if [ ! -f "$PCD_PATH" ]; then
  echo "原始 PCD 地图不存在: $PCD_PATH"
  exit 1
fi

if [ ! -f "$RVIZ_CONFIG" ]; then
  echo "RViz 配置文件不存在: $RVIZ_CONFIG"
  exit 1
fi

if ! command -v pcl_voxel_grid >/dev/null 2>&1; then
  echo "未找到 pcl_voxel_grid 命令，请先安装 pcl-tools，例如："
  echo "  sudo apt-get install pcl-tools"
  exit 1
fi

###########################
# 1. 对 PCD 地图做一次下采样
###########################
echo "原始地图: $PCD_PATH"
echo "下采样后地图: $DOWNSAMPLED_PATH"
echo "体素尺寸: ${VOXEL_LEAF_SIZE} m"

# 如果已存在下采样文件，这里选择覆盖重生成（你也可以改成跳过）
if [ -f "$DOWNSAMPLED_PATH" ]; then
  echo "检测到已存在下采样地图，将覆盖重新生成..."
fi

pcl_voxel_grid "$PCD_PATH" "$DOWNSAMPLED_PATH" -leaf \
  ${VOXEL_LEAF_SIZE},${VOXEL_LEAF_SIZE},${VOXEL_LEAF_SIZE}

echo "下采样完成。"

###########################
# 2. 启动 pcd_to_pointcloud 发布下采样的地图
###########################
echo "使用下采样地图发布: $DOWNSAMPLED_PATH"
echo "地图发布间隔: ${PCD_PUBLISH_INTERVAL}s, 持续约 ${MAP_PUBLISH_DURATION}s"

rosrun pcl_ros pcd_to_pointcloud "$DOWNSAMPLED_PATH" "$PCD_PUBLISH_INTERVAL" _frame_id:=map &
PCD_PID=$!

# 定时关闭地图发布节点（到时间自动 kill）
(
  sleep "$MAP_PUBLISH_DURATION"
  if ps -p $PCD_PID > /dev/null 2>&1; then
    echo
    echo "已到地图发布持续时间 ${MAP_PUBLISH_DURATION}s，自动关闭 pcd_to_pointcloud (PID=$PCD_PID)..."
    kill $PCD_PID 2>/dev/null || true
  fi
) &

cleanup() {
  echo
  echo "停止地图点云发布节点..."
  if ps -p $PCD_PID > /dev/null 2>&1; then
    kill $PCD_PID 2>/dev/null || true
    wait $PCD_PID 2>/dev/null || true
  fi
  echo "清理完成。"
}
trap cleanup EXIT

###########################
# 3. 启动 RViz，加载配置
###########################
echo "启动 RViz（关闭 RViz 后脚本会自动退出）..."
echo "使用 RViz 配置: $RVIZ_CONFIG"
rosrun rviz rviz -d "$RVIZ_CONFIG"

echo "RViz 已退出，脚本结束。"

