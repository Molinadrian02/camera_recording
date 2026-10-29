#!/usr/bin/env python3
"""
Captura de imagenes para calibracion intrinseca con una camara Basler (pypylon).

Por defecto, detecta en vivo el patron de ajedrez (mismo --cols/--rows que
luego le pasaras a calibrate.py) y lo dibuja sobre la previsualizacion, con
un indicador verde/rojo, para que sepas al momento si esa captura sirve.
Usa --no-detect para desactivar esto y tener solo la previsualizacion en
bruto (mas fluida, sin coste de deteccion por frame).

Uso:
    python3 capture_images.py --out ./calib_images --device camt1 --cols 9 --rows 6

Controles (con la ventana de previsualizacion en foco):
    ESPACIO -> guarda el frame actual
    q       -> salir

Requisitos:
    pip install pypylon opencv-python --break-system-packages
"""

import argparse
import os
import sys
import time

import cv2
import numpy as np

try:
    from pypylon import pylon
except ImportError:
    sys.exit(
        "No se encuentra pypylon. Instalalo con:\n"
        "    pip install pypylon --break-system-packages"
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", default="./calib_images", help="Carpeta de salida para las imagenes")
    parser.add_argument("--device", default=None, help="DeviceUserID de la camara (opcional; si no se da, usa la primera que encuentre)")
    parser.add_argument("--userset", default=None, help="Nombre del UserSet a cargar al conectar (ej. UserSet2). Opcional.")
    parser.add_argument("--cols", type=int, default=9, help="Esquinas internas del tablero, eje horizontal (para la deteccion en vivo)")
    parser.add_argument("--rows", type=int, default=6, help="Esquinas internas del tablero, eje vertical (para la deteccion en vivo)")
    parser.add_argument("--no-detect", action="store_true", help="Desactiva la deteccion en vivo del patron (previsualizacion en bruto, mas fluida)")
    parser.add_argument("--detect-width", type=int, default=640, help="Ancho al que se reescala el frame para detectar el patron (velocidad vs. sensibilidad)")
    args = parser.parse_args()
    pattern_size = (args.cols, args.rows)

    os.makedirs(args.out, exist_ok=True)

    tl_factory = pylon.TlFactory.GetInstance()
    devices = tl_factory.EnumerateDevices()
    if not devices:
        sys.exit("No se ha encontrado ninguna camara Pylon conectada.")

    target_device = None
    if args.device:
        for d in devices:
            if d.GetUserDefinedName() == args.device:
                target_device = d
                break
        if target_device is None:
            sys.exit(f"No se ha encontrado ninguna camara con DeviceUserID='{args.device}'.")
    else:
        target_device = devices[0]

    camera = pylon.InstantCamera(tl_factory.CreateDevice(target_device))
    camera.Open()

    if args.userset:
        try:
            camera.UserSetSelector.SetValue(args.userset)
            camera.UserSetLoad.Execute()
            print(f"UserSet '{args.userset}' cargado.")
        except Exception as e:
            print(f"Aviso: no se pudo cargar el UserSet '{args.userset}': {e}")

    print(f"Conectado a: {camera.GetDeviceInfo().GetModelName()} "
          f"(DeviceUserID='{camera.GetDeviceInfo().GetUserDefinedName()}')")

    # Asegura un formato que OpenCV pueda usar directamente para previsualizar.
    # Si la camara es color (Bayer), la convertimos a BGR con el conversor de pylon.
    converter = pylon.ImageFormatConverter()
    converter.OutputPixelFormat = pylon.PixelType_BGR8packed
    converter.OutputBitAlignment = pylon.OutputBitAlignment_MsbAligned

    camera.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)

    count = 0
    if args.no_detect:
        print("Deteccion en vivo DESACTIVADA. Pulsa ESPACIO para guardar un frame, 'q' para salir.")
    else:
        print(f"Buscando patron de {args.cols}x{args.rows} esquinas internas en vivo. "
              "Pulsa ESPACIO para guardar un frame, 'q' para salir.")

    detect_flags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE + cv2.CALIB_CB_FAST_CHECK

    try:
        while camera.IsGrabbing():
            grab_result = camera.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)

            if grab_result.GrabSucceeded():
                image = converter.Convert(grab_result)
                frame = image.GetArray()

                preview = frame.copy()
                found = False

                if not args.no_detect:
                    h, w = frame.shape[:2]
                    scale = args.detect_width / w if w > args.detect_width else 1.0
                    small = cv2.resize(frame, (int(w * scale), int(h * scale))) if scale != 1.0 else frame
                    gray_small = cv2.cvtColor(small, cv2.COLOR_BGR2GRAY)

                    found, corners_small = cv2.findChessboardCorners(gray_small, pattern_size, detect_flags)
                    if found:
                        corners_full = (corners_small / scale).astype(np.float32)
                        cv2.drawChessboardCorners(preview, pattern_size, corners_full, found)

                status_text = "PATRON DETECTADO - lista para guardar" if found else "patron no detectado"
                status_color = (0, 255, 0) if found else (0, 0, 255)
                if args.no_detect:
                    status_text = ""

                cv2.putText(
                    preview, f"Guardadas: {count}  (ESPACIO=guardar, q=salir)",
                    (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2
                )
                if status_text:
                    cv2.putText(
                        preview, status_text,
                        (20, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.9, status_color, 2
                    )
                cv2.imshow("Captura de calibracion", preview)

                key = cv2.waitKey(1) & 0xFF
                if key == ord(" "):
                    filename = os.path.join(args.out, f"calib_{count:03d}.png")
                    cv2.imwrite(filename, frame)
                    if args.no_detect:
                        print(f"Guardada: {filename}")
                    elif found:
                        print(f"Guardada: {filename}  (patron detectado)")
                    else:
                        print(f"Guardada: {filename}  (AVISO: patron NO detectado en esta captura, "
                              "es probable que calibrate.py la descarte)")
                    count += 1
                elif key == ord("q"):
                    break

            grab_result.Release()
    finally:
        camera.StopGrabbing()
        camera.Close()
        cv2.destroyAllWindows()

    print(f"\nTotal de imagenes guardadas: {count} en '{args.out}'")
    if count < 15:
        print("Aviso: para una buena calibracion se recomiendan al menos 15-20 imagenes "
              "con el patron en distintas posiciones, angulos y distancias.")


if __name__ == "__main__":
    main()