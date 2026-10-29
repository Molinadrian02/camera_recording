#!/usr/bin/env bash
# Arranca el nodo hk_camera con el entorno del SDK MVS (sin el, ConvertPixelType falla con MV_E_LOAD_LIBRARY)
export MVCAM_SDK_PATH=/opt/MVS MVCAM_COMMON_RUNENV=/opt/MVS/lib MVCAM_GENICAM_CLPROTOCOL=/opt/MVS/lib/CLProtocol ALLUSERSPROFILE=/opt/MVS/MVFG
# Perfil FastDDS con buffers grandes: sin el, /hk_camera/rgb (4.6 MB/msg) llega con huecos de ~0.6 s
export FASTRTPS_DEFAULT_PROFILES_FILE=${FASTRTPS_DEFAULT_PROFILES_FILE:-/colcon_ws/scripts/dds/fastdds_large_msgs.xml}
source /opt/ros/humble/setup.bash
export LD_LIBRARY_PATH=/opt/MVS/lib/64:/opt/MVS/lib/32:$LD_LIBRARY_PATH
[ -f /colcon_ws/install/setup.bash ] && source /colcon_ws/install/setup.bash
exec ros2 run hk_camera hk_camera "$@"
