#!/usr/bin/env bash
# Levanta el contenedor ROS2 de la Hikvision (si no esta ya arriba), arranca
# el nodo de la camara en segundo plano y lanza una grabacion con
# record_video_hk.py, con duracion real exacta.
#
# Uso:
#   ./record_hk.sh [duracion_segundos] [fichero_salida] [tasa_hz]
#
# Ejemplos:
#   ./record_hk.sh                       # 15s a 16 Hz, nombre con fecha/hora
#   ./record_hk.sh 30                     # 30s
#   ./record_hk.sh 30 mi_video.mp4 20     # 30s a 20Hz, nombre concreto
#
# Ejecutar desde la carpeta del proyecto (donde esta el docker-compose.yml).
# Ajusta CONTAINER/NODE_PKG/NODE_EXE/TOPIC abajo si no coinciden con tu setup.

set -euo pipefail
cd "$(dirname "$0")"

DURATION="${1:-15}"
TIMESTAMP="$(date +%Y%m%d_%H%M%S)"
OUT="${2:-recording_hk_${TIMESTAMP}.mp4}"
RATE="${3:-16}"

CONTAINER="humble"
TOPIC="/hk_camera/rgb"
CALIB_FILE="calibration_hk.yaml"
NODE_PKG="hk_camera"
NODE_EXE="hk_camera"
# Parametros ROS2 que se le pasan al nodo de la camara al arrancarlo (sobreescriben
# los defaults de hk_camera.hpp, ej. FrameRate, ExposureTime, PacketSize...).
# Formato: "-p nombre1:=valor1 -p nombre2:=valor2 ..."
NODE_ARGS="-p FrameRate:=16"

## Permite que el contenedor (cliente X) se conecte al servidor X de esta
# sesion grafica. Esto se hace aqui, en el host, porque es donde ya tenemos
# acceso de sobra a la sesion X; dentro del contenedor no serviria de nada
# (para poder correr xhost ahi ya necesitarias el acceso que te falta). Se
# pierde cada vez que se reinicia la sesion grafica (logout/reboot), por eso
# se repite en cada ejecucion; "|| true" para no abortar el script si xhost
# no esta instalado o algo raro pasa con el display.
xhost +local: >/dev/null 2>&1 || true

echo "Levantando el contenedor ${CONTAINER}..."
docker compose up -d
sleep 2

# Si ya habia un nodo de la camara corriendo de una ejecucion anterior (el
# contenedor no se para entre grabaciones), lo matamos primero. Dos nodos
# intentando abrir la misma camara GigE a la vez rompe la publicacion de
# frames (el topic existe pero deja de recibir datos nuevos).
echo "Comprobando si ya hay un nodo de la camara corriendo..."
# Se mata por nombre de proceso (-x): "ros2 run" solo es un envoltorio y el
# binario real (install/.../lib/hk_camera/hk_camera) sobrevive si se mata solo a ros2.
if docker exec "$CONTAINER" pkill -x "${NODE_EXE}" 2>/dev/null; then
    echo "Nodo anterior encontrado y detenido. Esperando a que libere la camara..."
    sleep 2
fi

echo "Arrancando el nodo de la camara (${NODE_PKG} ${NODE_EXE} ${NODE_ARGS}) en segundo plano..."
# Se arranca via run_hk.sh, que pone LD_LIBRARY_PATH con las librerias del SDK MVS;
# sin el, MV_CC_ConvertPixelType falla con 8000000c (MV_E_LOAD_LIBRARY) y no llegan frames.
docker exec -d "$CONTAINER" bash -c "
  /colcon_ws/scripts/tools/run_hk.sh --ros-args ${NODE_ARGS} > /tmp/hk_camera_node.log 2>&1
"

echo "Esperando a que el topic ${TOPIC} empiece a publicar..."
READY=0
for i in $(seq 1 15); do
    if docker exec "$CONTAINER" bash -c "source /opt/ros/humble/setup.bash && ros2 topic list" 2>/dev/null | grep -qx "$TOPIC"; then
        echo "Topic disponible."
        READY=1
        break
    fi
    sleep 1
done
if [ "$READY" -eq 0 ]; then
    echo "Aviso: el topic ${TOPIC} no aparecio en 15s. Revisa /tmp/hk_camera_node.log dentro del contenedor:"
    echo "  docker exec ${CONTAINER} cat /tmp/hk_camera_node.log"
    echo "Continuando igualmente por si tarda un poco mas en arrancar..."
fi

CALIB_ARGS=""
if docker exec "$CONTAINER" test -f "/colcon_ws/scripts/${CALIB_FILE}"; then
    CALIB_ARGS="--calib ${CALIB_FILE}"
else
    echo "Aviso: no se encontro ${CALIB_FILE} en el contenedor, se grabara sin rectificar."
fi

echo "Grabando ${DURATION}s a ${RATE}Hz con la Hikvision -> scripts/${OUT}"
docker exec -it "$CONTAINER" bash -c "
  source /opt/ros/humble/setup.bash
  [ -f /colcon_ws/install/setup.bash ] && source /colcon_ws/install/setup.bash
  cd /colcon_ws/scripts && python3 record_video_hk.py --out ${OUT} --topic ${TOPIC} --rate ${RATE} --duration ${DURATION} ${CALIB_ARGS}
"

echo "Listo. Video en: $(pwd)/scripts/${OUT}"
echo "(El nodo de la camara sigue corriendo en segundo plano en el contenedor;"
echo " para pararlo: docker exec ${CONTAINER} pkill -x ${NODE_EXE})"
