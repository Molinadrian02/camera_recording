#!/usr/bin/env bash
# Levanta el contenedor de la Basler (si no esta ya arriba) y lanza una
# grabacion con record_video.py, con duracion real exacta.
#
# Uso:
#   ./record_basler.sh [duracion_segundos] [fichero_salida] [framerate]
#
# Ejemplos:
#   ./record_basler.sh                      # 15s, nombre con fecha/hora, fps del UserSet
#   ./record_basler.sh 30                    # 30s
#   ./record_basler.sh 30 mi_video.mp4        # 30s, nombre concreto
#   ./record_basler.sh 30 mi_video.mp4 16     # 30s, fuerza Acquisition Frame Rate = 16fps
#
# El framerate (si se indica) fija la Acquisition Frame Rate REAL de la
# camara (fps de captura por hardware), igual que NODE_ARGS="-p FrameRate:=X"
# hace para la Hikvision en record_hk.sh. Si no se indica, se usa el fps que
# ya tenga guardado el UserSet cargado (ver USERSET abajo).
#
# Ejecutar desde la carpeta del proyecto (donde esta el docker-compose.yml).

set -euo pipefail
cd "$(dirname "$0")"

DURATION="${1:-15}"
TIMESTAMP="$(date +%Y%m%d_%H%M%S)"
OUT="${2:-recording_${TIMESTAMP}.mp4}"
FRAMERATE="${3:-}"

CONTAINER="camera_calib"
DEVICE="camt1"
USERSET="UserSet2"
CALIB_FILE="calibration_basler.yaml"

## Permite que el contenedor (cliente X) se conecte al servidor X de esta
# sesion grafica (necesario para la ventana de previsualizacion). Se hace
# aqui, en el host, que es donde ya tenemos acceso a la sesion X; dentro del
# contenedor no serviria de nada. Se pierde cada vez que se reinicia la
# sesion grafica, por eso se repite en cada ejecucion.
xhost +local: >/dev/null 2>&1 || true

echo "Levantando el contenedor ${CONTAINER}..."
docker compose up -d
sleep 2

CALIB_ARGS=""
if docker exec "$CONTAINER" test -f "/work/scripts/${CALIB_FILE}"; then
    CALIB_ARGS="--calib ${CALIB_FILE}"
else
    echo "Aviso: no se encontro ${CALIB_FILE} en el contenedor, se grabara sin rectificar."
fi

FPS_ARGS=""
if [ -n "$FRAMERATE" ]; then
    FPS_ARGS="--framerate ${FRAMERATE}"
    echo "Grabando ${DURATION}s con la Basler a ${FRAMERATE}fps (forzado) -> calib_images/${OUT}"
else
    echo "Grabando ${DURATION}s con la Basler (fps del UserSet ${USERSET}) -> calib_images/${OUT}"
fi

docker exec -it "$CONTAINER" bash -c "cd /work/scripts && python3 record_video.py --out ../calib_images/${OUT} --duration ${DURATION} --device ${DEVICE} --userset ${USERSET} ${CALIB_ARGS} ${FPS_ARGS}"

echo "Listo. Video en: $(pwd)/calib_images/${OUT}"
