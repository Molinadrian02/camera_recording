#!/usr/bin/env python3
"""
Captura de imagenes para calibracion intrinseca desde el topic ROS2 de la
camara Hikvision (hk_camera), ya que su nodo ROS2 funciona de forma estable.

Por defecto, el programa captura AUTOMATICAMENTE: detecta en vivo el patron
de ajedrez y, cuando lo ve quieto (sin moverse) y en una pose claramente
distinta de las ya guardadas, guarda el frame solo. Asi evitas fotos
innecesarias (duplicadas o borrosas) sin tener que estar pulsando tu.
Mismo comportamiento que capture_images.py (la version para la Basler).

Requiere que el nodo este corriendo en otra terminal:
    ros2 run hk_camera hk_camera

Uso:
    python3 capture_images_hk.py --out ./calib_images_hk --topic /hk_camera/rgb --cols 9 --rows 6

Criterios para el autoguardado (los tres a la vez):
    1. Patron detectado.
    2. Estable: lleva --stable-frames fotogramas seguidos casi sin moverse.
    3. Pose nueva: se mueve al menos --min-pose-diff pixeles (de media, por
       esquina) respecto a TODAS las capturas ya guardadas.
    Ademas respeta un --cooldown minimo entre capturas automaticas.

Usa --manual para volver al modo antiguo (solo guarda al pulsar ESPACIO).
En modo automatico, ESPACIO sigue disponible como guardado forzado manual.

Controles (con la ventana de previsualizacion en foco):
    ESPACIO -> guarda el frame actual (siempre disponible, aunque sea automatico)
    q       -> salir

Requisitos (dentro del contenedor ROS2, deberian estar ya disponibles):
    - rclpy, sensor_msgs, cv_bridge (vienen con la instalacion de ROS2 Humble)
    - opencv-python, numpy (si no: pip install opencv-python numpy --break-system-packages)
"""

import argparse
import os
import sys
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

try:
    from cv_bridge import CvBridge
except ImportError:
    sys.exit(
        "No se encuentra cv_bridge. Deberia venir con ROS2 Humble "
        "(ros-humble-cv-bridge); instalalo con:\n"
        "    sudo apt-get install ros-humble-cv-bridge"
    )


def pose_descriptor(corners):
    """Aplana las esquinas detectadas a un array Nx2 de float64, en el mismo
    orden siempre (orden de la rejilla), para poder comparar pose a pose."""
    return corners.reshape(-1, 2).astype(np.float64)


def avg_corner_distance(pose_a, pose_b):
    """Distancia media (px) entre esquinas correspondientes de dos poses."""
    return float(np.mean(np.linalg.norm(pose_a - pose_b, axis=1)))


class CaptureNode(Node):
    def __init__(self, topic: str):
        super().__init__("calib_capture_hk")
        self.bridge = CvBridge()
        self.latest_frame = None
        self.new_frame = False
        self.subscription = self.create_subscription(
            Image, topic, self.image_callback, 10
        )
        self.get_logger().info(f"Suscrito a '{topic}'. Esperando frames...")

    def image_callback(self, msg: Image):
        try:
            self.latest_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            self.new_frame = True
        except Exception as e:
            self.get_logger().warn(f"No se pudo convertir el frame: {e}")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--out", default="./calib_images_hk", help="Carpeta de salida para las imagenes")
    parser.add_argument("--topic", default="/hk_camera/rgb", help="Topic de imagen a suscribirse")
    parser.add_argument("--cols", type=int, default=9, help="Esquinas internas del tablero, eje horizontal")
    parser.add_argument("--rows", type=int, default=6, help="Esquinas internas del tablero, eje vertical")
    parser.add_argument("--no-detect", action="store_true", help="Desactiva la deteccion en vivo del patron (desactiva tambien el autoguardado)")
    parser.add_argument("--detect-width", type=int, default=640, help="Ancho al que se reescala el frame para detectar el patron (velocidad vs. sensibilidad)")
    parser.add_argument("--manual", action="store_true", help="Desactiva el autoguardado: solo guarda al pulsar ESPACIO (comportamiento antiguo)")
    parser.add_argument("--stable-frames", type=int, default=6, help="Fotogramas nuevos seguidos casi sin moverse requeridos antes de autoguardar")
    parser.add_argument("--stable-thresh", type=float, default=4.0, help="Movimiento medio maximo (px/esquina) entre fotogramas consecutivos para contar como 'estable'")
    parser.add_argument("--min-pose-diff", type=float, default=60.0, help="Movimiento medio minimo (px/esquina) respecto a TODAS las capturas ya guardadas para considerar la pose 'nueva'")
    parser.add_argument("--cooldown", type=float, default=1.5, help="Segundos minimos entre autoguardados")
    parser.add_argument("--target", type=int, default=30, help="Numero orientativo de capturas objetivo (solo informativo)")
    args, ros_args = parser.parse_known_args()
    pattern_size = (args.cols, args.rows)

    if args.no_detect and not args.manual:
        print("Aviso: --no-detect desactiva tambien el autoguardado (necesita detectar el patron). "
              "Usando modo manual: pulsa ESPACIO para guardar.")
        args.manual = True

    os.makedirs(args.out, exist_ok=True)

    rclpy.init(args=ros_args)
    node = CaptureNode(args.topic)

    if args.manual:
        print("Modo MANUAL. Pulsa ESPACIO para guardar un frame, 'q' para salir.")
    else:
        print(f"Modo AUTOMATICO. El programa guardara solo cuando el patron de "
              f"{args.cols}x{args.rows} este quieto y en una pose nueva. "
              "ESPACIO fuerza un guardado manual. 'q' para salir.")

    detect_flags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE + cv2.CALIB_CB_FAST_CHECK
    subpix_criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    count = 0
    saved_poses = []
    last_frame_pose = None
    stable_counter = 0
    last_save_time = 0.0
    flash_frames = 0

    def do_save(frame_to_save, pattern_found, pose, auto):
        nonlocal count, last_save_time, stable_counter, flash_frames
        filename = os.path.join(args.out, f"calib_{count:03d}.png")
        cv2.imwrite(filename, frame_to_save)
        tag = "auto" if auto else "manual"
        if pattern_found:
            print(f"Guardada ({tag}): {filename}  (patron detectado)")
            saved_poses.append(pose)
        else:
            print(f"Guardada ({tag}): {filename}  (AVISO: patron NO detectado en esta captura, "
                  "es probable que calibrate.py la descarte)")
        count += 1
        last_save_time = time.time()
        stable_counter = 0
        flash_frames = 8

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)

            if node.latest_frame is None:
                continue
            if not node.new_frame:
                # Mismo frame que la vuelta anterior; no hay nada nuevo que procesar.
                key = cv2.waitKey(1) & 0xFF
                if key == ord("q"):
                    break
                continue

            node.new_frame = False
            frame = node.latest_frame

            preview = frame.copy()
            found = False
            corners_full = None

            if not args.no_detect:
                h, w = frame.shape[:2]
                scale = args.detect_width / w if w > args.detect_width else 1.0
                small = cv2.resize(frame, (int(w * scale), int(h * scale))) if scale != 1.0 else frame
                gray_small = cv2.cvtColor(small, cv2.COLOR_BGR2GRAY)

                found, corners_small = cv2.findChessboardCorners(gray_small, pattern_size, detect_flags)
                if found:
                    corners_full = (corners_small / scale).astype(np.float32)
                    gray_full = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                    corners_full = cv2.cornerSubPix(gray_full, corners_full, (11, 11), (-1, -1), subpix_criteria)
                    cv2.drawChessboardCorners(preview, pattern_size, corners_full, found)

            auto_status = ""
            if not args.manual:
                if found:
                    pose = pose_descriptor(corners_full)
                    movement = None
                    if last_frame_pose is not None and last_frame_pose.shape == pose.shape:
                        movement = avg_corner_distance(pose, last_frame_pose)
                        stable_counter = stable_counter + 1 if movement <= args.stable_thresh else 0
                    else:
                        stable_counter = 0
                    last_frame_pose = pose

                    mov_txt = f" (mov={movement:.1f}px)" if movement is not None else ""
                    if stable_counter < args.stable_frames:
                        auto_status = f"estabilizando {stable_counter}/{args.stable_frames}{mov_txt}"
                    elif (time.time() - last_save_time) < args.cooldown:
                        auto_status = "esperando..."
                    else:
                        min_diff = min((avg_corner_distance(pose, sp) for sp in saved_poses), default=float("inf"))
                        if min_diff >= args.min_pose_diff:
                            do_save(frame, True, pose, auto=True)
                            auto_status = "CAPTURADA"
                        else:
                            auto_status = "pose parecida a una ya guardada, mueve el tablero"
                else:
                    stable_counter = 0
                    last_frame_pose = None

            status_text = "PATRON DETECTADO" if found else "patron no detectado"
            status_color = (0, 255, 0) if found else (0, 0, 255)
            if args.no_detect:
                status_text = ""

            cv2.putText(
                preview, f"Guardadas: {count}/{args.target}  (ESPACIO=forzar guardado, q=salir)",
                (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2
            )
            if status_text:
                cv2.putText(preview, status_text, (20, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.9, status_color, 2)
            if auto_status:
                cv2.putText(preview, auto_status, (20, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 255), 2)

            if flash_frames > 0:
                cv2.rectangle(preview, (0, 0), (preview.shape[1] - 1, preview.shape[0] - 1), (0, 255, 0), 12)
                flash_frames -= 1

            cv2.imshow("Captura de calibracion - hk_camera", preview)

            key = cv2.waitKey(1) & 0xFF
            if key == ord(" "):
                pose = pose_descriptor(corners_full) if found else None
                do_save(frame, found, pose, auto=False)
            elif key == ord("q"):
                break
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

    print(f"\nTotal de imagenes guardadas: {count} en '{args.out}'")
    if count < 15:
        print("Aviso: para una buena calibracion se recomiendan al menos 15-20 imagenes "
              "con el patron en distintas posiciones, angulos y distancias.")


if __name__ == "__main__":
    main()
