#!/usr/bin/env python3
"""
Grabacion continua de video/secuencia con una camara Basler (pypylon).

Dos modos de salida:
    --mode video  -> un unico fichero .mp4/.avi (cv2.VideoWriter)
    --mode images -> secuencia de imagenes con timestamp (una por frame, sin
                     compresion con perdidas si usas --ext png)

Uso:
    # Grabar un video de 30s (duracion real garantizada)
    python3 record_video.py --out recording.mp4 --duration 30 --device camt1 --userset UserSet2

    # Grabar una secuencia de imagenes hasta pulsar 'q' (cada archivo lleva su timestamp real)
    python3 record_video.py --mode images --out ./sequence --device camt1 --userset UserSet2

Duracion real (--mode video):
    En vez de adivinar los fps de antemano, el script guarda cada frame como
    imagen temporal mientras grabas, y SOLO al terminar (cuando ya sabe
    exactamente cuantos frames saco y cuanto tiempo real paso) monta el video
    final con el fps exacto = frames / segundos reales. Asi un video de 10s
    de grabacion siempre dura 10s al reproducirlo, sin importar a que
    velocidad real haya podido capturar la camara. Si quieres forzar un fps
    concreto en el fichero de salida (por ejemplo para que encaje con otro
    sistema), usa --fps; en ese caso el video puede verse mas rapido o mas
    lento que la realidad si la camara no sostuvo exactamente esa tasa.

Controles durante la grabacion (con la ventana de previsualizacion en foco):
    q -> detener y guardar

Rectificacion (opcional):
    --calib calibration.yaml  -> aplica undistort con la calibracion generada
                                  por calibrate.py; lo que se guarda (video o
                                  imagenes) ya sale corregido, no en bruto.
    --alpha 0.0..1.0          -> solo con --calib. Controla cuanto recorte vs.
                                  cuanto campo de vision conservar al rectificar
                                  (0 = sin pixeles invalidos/negros en los bordes,
                                  recorta mas; 1 = conserva todo el FOV original,
                                  pero aparecen bordes negros). Por defecto 0.0.

Requisitos:
    pip install pypylon opencv-python pyyaml numpy --break-system-packages
"""

import argparse
import os
import shutil
import sys
import tempfile
import time

import cv2
import numpy as np
import yaml

try:
    from pypylon import pylon
except ImportError:
    sys.exit(
        "No se encuentra pypylon. Instalalo con:\n"
        "    pip install pypylon --break-system-packages"
    )


def load_undistort_maps(calib_path, image_size, alpha):
    """Carga camera_matrix/distortion_coefficients del YAML (formato calibrate.py /
    camera_info de ROS) y precalcula los mapas de rectificacion con
    initUndistortRectifyMap, para no recalcular nada en cada frame."""
    with open(calib_path, "r") as f:
        data = yaml.safe_load(f)

    camera_matrix = np.array(data["camera_matrix"]["data"], dtype=np.float64).reshape(3, 3)
    dist_coeffs = np.array(data["distortion_coefficients"]["data"], dtype=np.float64)

    width, height = image_size
    new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(
        camera_matrix, dist_coeffs, (width, height), alpha, (width, height)
    )
    map1, map2 = cv2.initUndistortRectifyMap(
        camera_matrix, dist_coeffs, None, new_camera_matrix, (width, height), cv2.CV_16SC2
    )
    print(f"Calibracion cargada desde '{calib_path}'. Se aplicara rectificacion (alpha={alpha}).")
    return map1, map2


def connect_camera(device_user_id, userset, framerate=None):
    tl_factory = pylon.TlFactory.GetInstance()
    devices = tl_factory.EnumerateDevices()
    if not devices:
        sys.exit("No se ha encontrado ninguna camara Pylon conectada.")

    target_device = None
    if device_user_id:
        for d in devices:
            if d.GetUserDefinedName() == device_user_id:
                target_device = d
                break
        if target_device is None:
            sys.exit(f"No se ha encontrado ninguna camara con DeviceUserID='{device_user_id}'.")
    else:
        target_device = devices[0]

    camera = pylon.InstantCamera(tl_factory.CreateDevice(target_device))
    camera.Open()

    if userset:
        try:
            camera.UserSetSelector.SetValue(userset)
            camera.UserSetLoad.Execute()
            print(f"UserSet '{userset}' cargado.")
        except Exception as e:
            print(f"Aviso: no se pudo cargar el UserSet '{userset}': {e}")

    if framerate:
        try:
            camera.AcquisitionFrameRateEnable.SetValue(True)
            camera.AcquisitionFrameRate.SetValue(framerate)
            print(f"Acquisition Frame Rate fijado a {framerate} fps.")
        except Exception as e:
            print(f"Aviso: no se pudo fijar el framerate a {framerate} fps: {e}")

    print(f"Conectado a: {camera.GetDeviceInfo().GetModelName()} "
          f"(DeviceUserID='{camera.GetDeviceInfo().GetUserDefinedName()}')")
    return camera


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--out", required=True, help="Fichero de video (modo video) o carpeta (modo images)")
    parser.add_argument("--mode", choices=["video", "images"], default="video")
    parser.add_argument("--fps", type=float, default=None, help="Fuerza unos fps concretos en el video de salida. Si no se indica (recomendado), se calcula al terminar a partir de frames/segundos reales, para que la duracion del video sea exacta.")
    parser.add_argument("--duration", type=float, default=None, help="Duracion en segundos (si no se da, graba hasta pulsar 'q')")
    parser.add_argument("--device", default=None, help="DeviceUserID de la camara (opcional)")
    parser.add_argument("--userset", default=None, help="UserSet a cargar al conectar (ej. UserSet2)")
    parser.add_argument("--framerate", type=float, default=None, help="Fuerza la Acquisition Frame Rate real de la camara (fps de captura por hardware). Si no se indica, se usa la que ya tenga cargada el UserSet.")
    parser.add_argument("--ext", default="png", choices=["png", "jpg"], help="Extension para el modo images (y para los frames temporales en modo video)")
    parser.add_argument("--no-preview", action="store_true", help="No mostrar ventana de previsualizacion (graba mas rapido)")
    parser.add_argument("--calib", default=None, help="Fichero calibration.yaml (de calibrate.py) para rectificar cada frame antes de guardarlo")
    parser.add_argument("--alpha", type=float, default=0.0, help="Solo con --calib: 0=recorta bordes invalidos, 1=conserva todo el FOV con bordes negros")
    parser.add_argument("--keep-temp", action="store_true", help="Solo en modo video: no borra la carpeta temporal de frames tras montar el video (para depurar)")
    args = parser.parse_args()

    camera = connect_camera(args.device, args.userset, args.framerate)

    converter = pylon.ImageFormatConverter()
    converter.OutputPixelFormat = pylon.PixelType_BGR8packed
    converter.OutputBitAlignment = pylon.OutputBitAlignment_MsbAligned

    camera.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)

    frame_count = 0
    undistort_maps = None  # se inicializa en el primer frame, cuando sabemos el tamano real
    frame_size = None

    temp_dir = None
    if args.mode == "images":
        os.makedirs(args.out, exist_ok=True)
    else:
        temp_dir = tempfile.mkdtemp(prefix="record_tmp_")

    start_time = time.time()
    print("Grabando... pulsa 'q' (con la ventana de previsualizacion en foco) para detener."
          if not args.duration else f"Grabando {args.duration}s...")

    try:
        while camera.IsGrabbing():
            if args.duration is not None and (time.time() - start_time) >= args.duration:
                break

            grab_result = camera.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)

            if grab_result.GrabSucceeded():
                image = converter.Convert(grab_result)
                frame = image.GetArray()

                if args.calib and undistort_maps is None:
                    h, w = frame.shape[:2]
                    undistort_maps = load_undistort_maps(args.calib, (w, h), args.alpha)

                if undistort_maps is not None:
                    map1, map2 = undistort_maps
                    frame = cv2.remap(frame, map1, map2, interpolation=cv2.INTER_LINEAR)

                if frame_size is None:
                    frame_size = (frame.shape[1], frame.shape[0])  # (w, h)

                if args.mode == "video":
                    filename = os.path.join(temp_dir, f"frame_{frame_count:06d}.{args.ext}")
                    cv2.imwrite(filename, frame)
                else:
                    ts = time.time()
                    filename = os.path.join(args.out, f"frame_{frame_count:06d}_{ts:.6f}.{args.ext}")
                    cv2.imwrite(filename, frame)

                frame_count += 1

                if not args.no_preview:
                    preview = frame.copy()
                    elapsed = time.time() - start_time
                    cv2.putText(
                        preview, f"Frames: {frame_count}  Tiempo: {elapsed:.1f}s  (q=detener)",
                        (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2
                    )
                    cv2.imshow("Grabacion", preview)
                    key = cv2.waitKey(1) & 0xFF
                    if key == ord("q"):
                        break

            grab_result.Release()
    except KeyboardInterrupt:
        print("\nInterrumpido por el usuario.")
    finally:
        camera.StopGrabbing()
        camera.Close()
        cv2.destroyAllWindows()

    elapsed = time.time() - start_time
    real_fps = frame_count / elapsed if elapsed > 0 else 0
    print(f"\nCaptura finalizada: {frame_count} frames en {elapsed:.1f}s ({real_fps:.1f} fps reales).")

    if args.mode == "images":
        print(f"Frames guardados en: {args.out}")
        return

    # Modo video: montamos el fichero final ahora que sabemos la duracion real exacta.
    if frame_count == 0 or frame_size is None:
        shutil.rmtree(temp_dir, ignore_errors=True)
        sys.exit("No se ha capturado ningun frame, no se genera video.")

    video_fps = args.fps if args.fps is not None else real_fps
    if video_fps <= 0:
        video_fps = 1.0

    # Numero de frames que, a video_fps, duran EXACTAMENTE lo mismo que la
    # grabacion real. Si coincide con frame_count, no se nota nada; si no
    # (por ejemplo porque forzaste --fps, o la camara no fue perfectamente
    # regular), se repiten o se saltan frames para encajar ahi, nunca se
    # deja que la duracion del fichero se desvie de la duracion real.
    target_frames = max(1, round(video_fps * elapsed))

    print(f"Montando video a {video_fps:.2f} fps, {target_frames} frames "
          f"(duracion objetivo: {elapsed:.2f}s)"
          + (" [fps forzado con --fps, reajustando frames]" if args.fps is not None else "") + "...")

    fourcc = cv2.VideoWriter_fourcc(*("mp4v" if args.out.endswith(".mp4") else "XVID"))
    writer = cv2.VideoWriter(args.out, fourcc, video_fps, frame_size)
    if not writer.isOpened():
        shutil.rmtree(temp_dir, ignore_errors=True)
        sys.exit(f"No se pudo abrir el fichero de salida '{args.out}' para escritura.")

    for j in range(target_frames):
        t = j / video_fps
        if frame_count > 1 and elapsed > 0:
            src_idx = int(round((t / elapsed) * (frame_count - 1)))
        else:
            src_idx = 0
        src_idx = max(0, min(frame_count - 1, src_idx))
        filename = os.path.join(temp_dir, f"frame_{src_idx:06d}.{args.ext}")
        img = cv2.imread(filename)
        if img is not None:
            writer.write(img)
    writer.release()

    if args.keep_temp:
        print(f"Frames temporales conservados en: {temp_dir}")
    else:
        shutil.rmtree(temp_dir, ignore_errors=True)

    final_duration = target_frames / video_fps
    print(f"Guardado en: {args.out}  ({target_frames} frames a {video_fps:.2f} fps = "
          f"{final_duration:.2f}s de video, grabacion real {elapsed:.2f}s)")


if __name__ == "__main__":
    main()