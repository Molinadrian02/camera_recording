# Camera Recording

Herramientas para **capturar, calibrar y grabar** con dos cámaras industriales,
cada una en su propio contenedor Docker:

| Carpeta | Cámara | Cómo se accede |
|---|---|---|
| [`Pylon_Recording/`](Pylon_Recording) | Basler (GigE) | `pypylon` directamente, sin ROS |
| [`Hk_Robot_Driver/`](Hk_Robot_Driver) | Hikvision / Hikrobot (GigE / USB3) | Nodo ROS 2 Humble `hk_camera` + SDK MVS |

El flujo es el mismo en las dos:

1. **Capturar** imágenes del tablero de ajedrez (autoguardado cuando el patrón está quieto y en una pose nueva).
2. **Calibrar** con `calibrate.py` → genera un YAML (formato `camera_info` de ROS) y un PNG de cobertura.
3. **Grabar** vídeo con duración real exacta, opcionalmente rectificado con ese YAML.

Tablero usado: **9 × 6 esquinas internas, casillas de 25 mm**.

---

## Requisitos en el host

- Docker + Docker Compose.
- Sesión gráfica X11 (las ventanas de previsualización usan `cv2.imshow`). Los
  scripts `record_*.sh` ya hacen `xhost +local:`; si lanzas a mano, hazlo tú antes:

```bash
xhost +local:
```

- La cámara en la misma red que el PC (ambos contenedores usan `network_mode: host`).

---

## Basler — `Pylon_Recording/`

```
Pylon_Recording/
├── Dockerfile, docker-compose.yml, requirements.txt   # python:3.12 + pypylon + opencv
├── record_basler.sh          # levanta el contenedor y graba
├── calib_images/             # vídeos grabados (montado en /work/calib_images)
├── output/                   # montado en /work/output
└── scripts/                  # montado en /work/scripts
    ├── capture_images.py     # captura para calibración
    ├── calibrate.py          # calibración intrínseca
    ├── record_video.py       # grabación
    ├── calibration_basler.yaml
    └── calib_images/         # imágenes usadas para la calibración actual
```

### Levantar el contenedor

```bash
cd Pylon_Recording
docker compose up -d --build
docker exec -it camera_calib bash
cd /work/scripts
```

### 1. Capturar

```bash
python3 capture_images.py --out ./calib_images --device camt1 --userset UserSet2 --cols 9 --rows 6
```

- `--device`: DeviceUserID de la cámara (sin él usa la primera que encuentre).
- `--userset`: UserSet a cargar al conectar (exposición, fps, etc. guardados en la cámara).
- Teclas: `ESPACIO` guarda el frame, `q` sale. `--no-detect` desactiva la detección en vivo (más fluido).

### 2. Calibrar

```bash
python3 calibrate.py --images ./calib_images --cols 9 --rows 6 --square 25.0 \
    --out calibration_basler.yaml --camera-name camt1 --coverage coverage.png
```

Revisa `coverage.png`: muestra dónde cayeron las esquinas detectadas. Si faltan
zonas (bordes, esquinas del frame), captura más imágenes ahí y recalibra.
`--show` enseña cada imagen con las esquinas detectadas.

### 3. Grabar

Con el script (desde el host, en `Pylon_Recording/`):

```bash
./record_basler.sh                      # 15 s, nombre con fecha/hora, fps del UserSet
./record_basler.sh 30                   # 30 s
./record_basler.sh 30 mi_video.mp4      # 30 s, nombre concreto
./record_basler.sh 30 mi_video.mp4 16   # 30 s, fuerza Acquisition Frame Rate = 16 fps
```

Si existe `scripts/calibration_basler.yaml` se graba rectificado; si no, en bruto.
El vídeo queda en `Pylon_Recording/calib_images/`.

A mano, dentro del contenedor:

```bash
python3 record_video.py --out ../calib_images/recording.mp4 --duration 30 \
    --device camt1 --userset UserSet2 --calib calibration_basler.yaml
```

Opciones útiles de `record_video.py`:

| Opción | Qué hace |
|---|---|
| `--duration S` | Graba S segundos (sin ella, hasta pulsar `q`) |
| `--framerate F` | Fija los fps de captura **por hardware** en la cámara |
| `--fps F` | Fuerza los fps del fichero de salida (por defecto se calculan con frames/tiempo real para que la duración sea exacta) |
| `--mode images` | Guarda frames sueltos en una carpeta en vez de un vídeo |
| `--calib FILE` / `--alpha A` | Rectifica; `alpha=0` recorta bordes inválidos, `1` conserva todo el FOV |
| `--no-preview` | Sin ventana (graba más rápido) |

---

## Hikvision — `Hk_Robot_Driver/humble/`

```
Hk_Robot_Driver/humble/
├── Dockerfile, docker-compose.yml, .env   # ROS 2 Humble + SDK MVS
├── record_hk.sh              # levanta contenedor, arranca el nodo y graba
├── mvs-sdk/                  # aquí va el .deb del SDK MVS (NO está en el repo)
├── colcon_ws/src/
│   ├── hk_camera/            # driver ROS 2 (basado en SEUZTh/hk_camera, modificado)
│   └── hk_camera_interfaces/
└── scripts/                  # montado en /colcon_ws/scripts
    ├── capture_images_hk.py  # captura desde el topic ROS
    ├── calibrate.py
    ├── record_video_hk.py
    ├── calibration_hk.yaml
    ├── calib_images_hk/
    ├── dds/fastdds_large_msgs.xml   # perfil FastDDS con buffers grandes
    └── tools/run_hk.sh, hzstat.py
```

### Preparación (una vez)

1. **SDK MVS**: descarga el instalador `MVS-*_x86_64_*.deb` desde la web de Hikrobot
   y cópialo en `Hk_Robot_Driver/humble/mvs-sdk/` (el Dockerfile hace `COPY mvs-sdk/*.deb`).
   No se sube al repo por tamaño (~370 MB).
2. Ajusta `.env` a tu usuario (`id` te da los valores):

```bash
USER_ID=1000
USER_NAME=adrian
GROUP_ID=1000
GROUP_NAME=oompaloompas
WORKSPACE=colcon_ws
DOCKER_IMAGE_NAME=humble
```

3. Construye el contenedor y compila el workspace:

```bash
cd Hk_Robot_Driver/humble
docker compose up -d --build
docker exec -it humble bash
cd /colcon_ws && colcon build && source install/setup.bash
```

### Arrancar el nodo de la cámara

Siempre a través de `run_hk.sh`: pone `LD_LIBRARY_PATH` con las librerías del SDK
(sin él, `MV_CC_ConvertPixelType` falla con `8000000c / MV_E_LOAD_LIBRARY`) y el
perfil FastDDS de buffers grandes (sin él, `/hk_camera/rgb` llega con huecos de ~0,6 s).

```bash
/colcon_ws/scripts/tools/run_hk.sh --ros-args -p FrameRate:=16
```

Publica en `/hk_camera/rgb` (`sensor_msgs/Image`). Para comprobar la tasa:

```bash
ros2 topic hz /hk_camera/rgb
```

> Solo un nodo a la vez: dos procesos abriendo la misma cámara rompen la
> publicación. Para pararlo: `docker exec humble pkill -x hk_camera`.

### 1. Capturar

Con el nodo corriendo, en otra terminal del contenedor:

```bash
cd /colcon_ws/scripts
python3 capture_images_hk.py --out ./calib_images_hk --topic /hk_camera/rgb --cols 9 --rows 6
```

Autoguardado: guarda solo cuando el patrón se detecta, está estable
(`--stable-frames`, `--stable-thresh`) y la pose es nueva respecto a todas las
anteriores (`--min-pose-diff`), con un `--cooldown` entre capturas.
`ESPACIO` fuerza un guardado, `q` sale. `--manual` desactiva el autoguardado.
Apunta a unas 30 capturas (`--target`).

### 2. Calibrar

```bash
python3 calibrate.py --images ./calib_images_hk --cols 9 --rows 6 --square 25.0 \
    --out calibration_hk.yaml --camera-name hk_camera --coverage coverage_hk.png
```

### 3. Grabar

Con el script (desde el host, en `Hk_Robot_Driver/humble/`). Mata un nodo
anterior si lo hay, arranca el nodo, espera al topic y graba:

```bash
./record_hk.sh                       # 15 s a 16 Hz, nombre con fecha/hora
./record_hk.sh 30                    # 30 s
./record_hk.sh 30 mi_video.mp4 20    # 30 s a 20 Hz, nombre concreto
```

Los parámetros del nodo (FrameRate, ExposureTime, PacketSize…) se cambian en
`NODE_ARGS` dentro de `record_hk.sh`. El vídeo queda en `scripts/`. Si el topic
no aparece, mira el log del nodo:

```bash
docker exec humble cat /tmp/hk_camera_node.log
```

A mano, dentro del contenedor (con el nodo corriendo):

```bash
python3 record_video_hk.py --out recording.mp4 --topic /hk_camera/rgb \
    --rate 16 --duration 30 --calib calibration_hk.yaml
```

`--rate` es la tasa a la que se guardan frames; el resto de opciones
(`--fps`, `--mode images`, `--alpha`, `--no-preview`, `--keep-temp`) son las
mismas que en la Basler.

---

## Formato del YAML de calibración

Ambos `calibrate.py` generan el formato estándar `camera_info` de ROS
(`image_width`, `image_height`, `camera_matrix`, `distortion_model: plumb_bob`,
`distortion_coefficients`, `rectification_matrix`, `projection_matrix`), así que
se puede cargar directamente con `camera_info_manager`.

## Qué no está en el repo

Ver `.gitignore`: vídeos `.mp4`, el SDK MVS, `MvSdkLog/` y los `build/`,
`install/`, `log/` de colcon.
