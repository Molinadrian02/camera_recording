#!/usr/bin/env python3
"""
Grabacion continua de video/secuencia con la camara Hikvision via ROS2
(topic publicado por el nodo hk_camera), limitando la tasa de guardado a
una frecuencia fija (por defecto 16 Hz, para igualar el techo real de la
Basler) aunque el topic publique mas rapido: simplemente se descartan los
frames que llegan antes de que toque el siguiente segun la tasa objetivo.

Dos modos de salida:
    --mode video  -> un unico fichero .mp4/.avi (cv2.VideoWriter)
    --mode images -> secuencia de imagenes con timestamp

Uso:
    # Grabar 30s a 16 Hz, duracion real garantizada
    python3 record_video_hk.py --out recording_hk.mp4 --rate 16 --duration 30

    # Grabar una secuencia de imagenes hasta Ctrl+C o 'q' en la ventana
    python3 record_video_hk.py --mode images --out ./sequence_hk --rate 16

Duracion real (--mode video):
    Mientras grabas, cada frame guardado (ya filtrado a la tasa --rate) se
    escribe como imagen temporal. Al terminar, con el numero exacto de
    frames y el tiempo real transcurrido ya conocidos, se monta el video
    final con fps = frames / segundos reales. Asi 30s de grabacion dan
    siempre un video de 30s, aunque el topic no publicase exactamente a
    --rate Hz. Si quieres forzar los fps del fichero de salida en vez de
    usar el valor exacto medido, pasa --fps.

Rectificacion (opcional):
    --calib calibration.yaml  -> aplica undistort con la calibracion generada
                                  por calibrate.py (intrinsecos de esta
                                  camara); lo que se guarda ya sale corregido.
    --alpha 0.0..1.0          -> solo con --calib (0=recorta bordes, 1=conserva
                                  todo el FOV con bordes negros). Por defecto 0.0.

Requisitos: ejecutar dentro del contenedor ROS2 con el nodo hk_camera
publicando, y con rclpy, cv_bridge, opencv-python, pyyaml disponibles
(todo ya deberia estar en el workspace de hk_camera).
"""

import argparse
import os
import queue
import shutil
import sys
import tempfile
import threading
import time

import cv2
import numpy as np

try:
    import yaml
except ImportError:
    yaml = None

try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image
    from cv_bridge import CvBridge
except ImportError:
    sys.exit(
        "No se encuentran rclpy/cv_bridge. Ejecuta este script dentro del "
        "contenedor ROS2 donde corre el driver de la Hikvision."
    )


def load_undistort_maps(calib_path, image_size, alpha):
    """Carga camera_matrix/distortion_coefficients del YAML (formato calibrate.py /
    camera_info de ROS) y precalcula los mapas de rectificacion."""
    if yaml is None:
        sys.exit("Falta pyyaml para leer --calib. Instalalo con: pip install pyyaml --break-system-packages")
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


class RateLimitedRecorder(Node):
    def __init__(self, args, temp_dir):
        super().__init__("record_video_hk")
        self.args = args
        self.temp_dir = temp_dir
        self.bridge = CvBridge()
        self.frame_count = 0
        self.start_time = None
        self.next_due = None
        self.target_period = 1.0 / args.rate
        self.undistort_maps = None
        self.frame_size = None
        self.done = False

        # Rectificar y escribir a disco (PNG 1440x1080 ~35 ms) en el callback hace
        # perder frames del topic; se hace en un hilo aparte (cv2 suelta el GIL).
        self.write_queue = queue.Queue()
        self.writer_thread = threading.Thread(target=self._writer_loop, daemon=True)
        self.writer_thread.start()

        if args.mode == "images":
            os.makedirs(args.out, exist_ok=True)

        self.subscription = self.create_subscription(Image, args.topic, self.on_image, 10)
        self.get_logger().info(f"Suscrito a '{args.topic}', guardando a {args.rate} Hz "
                                f"(se descartan los frames de mas).")

    def on_image(self, msg):
        now = time.time()

        if self.start_time is None:
            self.start_time = now

        if self.args.duration is not None and (now - self.start_time) >= self.args.duration:
            self.done = True
            return

        # Planificacion con horario fijo: cada frame "toca" en next_due y el
        # siguiente en next_due + periodo. Comparar con el ultimo guardado
        # (now - last >= periodo) descarta frames que llegan unos ms antes por
        # jitter y la tasa real cae muy por debajo de --rate.
        if self.next_due is not None and now < self.next_due:
            return  # aun no toca el siguiente frame a la tasa objetivo; se descarta
        if self.next_due is None or now - self.next_due > self.target_period:
            self.next_due = now  # arranque, o nos hemos quedado atras: no recuperar a rafagas
        self.next_due += self.target_period

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().warn(f"No se pudo convertir el frame: {e}")
            return

        if self.args.calib and self.undistort_maps is None:
            h, w = frame.shape[:2]
            self.undistort_maps = load_undistort_maps(self.args.calib, (w, h), self.args.alpha)

        if self.frame_size is None:
            self.frame_size = (frame.shape[1], frame.shape[0])  # (w, h); remap no lo cambia

        if self.args.mode == "video":
            filename = os.path.join(self.temp_dir, f"frame_{self.frame_count:06d}.{self.args.ext}")
        else:
            filename = os.path.join(self.args.out, f"frame_{self.frame_count:06d}_{now:.6f}.{self.args.ext}")
        self.write_queue.put((filename, frame))

        self.frame_count += 1

        if not self.args.no_preview:
            preview = frame.copy()
            elapsed = now - self.start_time
            cv2.putText(
                preview, f"Frames: {self.frame_count}  Tiempo: {elapsed:.1f}s  (q=detener)",
                (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2
            )
            cv2.imshow("Grabacion HK", preview)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                self.done = True

    def _writer_loop(self):
        while True:
            item = self.write_queue.get()
            if item is None:
                return
            filename, frame = item
            if self.undistort_maps is not None:
                map1, map2 = self.undistort_maps
                frame = cv2.remap(frame, map1, map2, interpolation=cv2.INTER_LINEAR)
            cv2.imwrite(filename, frame)

    def flush_writes(self):
        """Espera a que el hilo escritor vuelque a disco todos los frames encolados."""
        pending = self.write_queue.qsize()
        if pending:
            print(f"Esperando a escribir {pending} frames pendientes...")
        self.write_queue.put(None)
        self.writer_thread.join()


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--out", required=True, help="Fichero de video (modo video) o carpeta (modo images)")
    parser.add_argument("--mode", choices=["video", "images"], default="video")
    parser.add_argument("--rate", type=float, default=16.0, help="Tasa objetivo en Hz a la que se guardan frames (filtro de entrada)")
    parser.add_argument("--fps", type=float, default=None, help="Fuerza unos fps concretos en el video de salida. Si no se indica (recomendado), se calcula al terminar a partir de frames/segundos reales, para duracion exacta.")
    parser.add_argument("--duration", type=float, default=None, help="Duracion en segundos (si no se da, graba hasta 'q' o Ctrl+C)")
    parser.add_argument("--topic", default="/hk_camera/rgb", help="Topic ROS2 de imagen (sensor_msgs/Image)")
    parser.add_argument("--ext", default="png", choices=["png", "jpg"], help="Extension para el modo images (y para los frames temporales en modo video)")
    parser.add_argument("--no-preview", action="store_true", help="No mostrar ventana de previsualizacion")
    parser.add_argument("--calib", default=None, help="Fichero calibration.yaml (de calibrate.py) para rectificar cada frame antes de guardarlo")
    parser.add_argument("--alpha", type=float, default=0.0, help="Solo con --calib: 0=recorta bordes invalidos, 1=conserva todo el FOV con bordes negros")
    parser.add_argument("--keep-temp", action="store_true", help="Solo en modo video: no borra la carpeta temporal de frames tras montar el video (para depurar)")
    args, ros_args = parser.parse_known_args()

    temp_dir = tempfile.mkdtemp(prefix="record_hk_tmp_") if args.mode == "video" else None

    rclpy.init(args=ros_args)
    node = RateLimitedRecorder(args, temp_dir)

    print(f"Grabando a {args.rate} Hz..." + (f" ({args.duration}s)" if args.duration else " pulsa 'q' o Ctrl+C para detener."))

    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        print("\nInterrumpido por el usuario.")
    finally:
        cv2.destroyAllWindows()

        elapsed = (time.time() - node.start_time) if node.start_time else 0
        node.flush_writes()
        frame_count = node.frame_count
        frame_size = node.frame_size
        real_rate = frame_count / elapsed if elapsed > 0 else 0
        print(f"\nCaptura finalizada: {frame_count} frames en {elapsed:.1f}s ({real_rate:.1f} fps reales).")

        node.destroy_node()
        rclpy.shutdown()

        if args.mode == "images":
            print(f"Frames guardados en: {args.out}")
        elif frame_count == 0 or frame_size is None:
            if temp_dir:
                shutil.rmtree(temp_dir, ignore_errors=True)
            print("No se ha capturado ningun frame, no se genera video.")
        else:
            video_fps = args.fps if args.fps is not None else real_rate
            if video_fps <= 0:
                video_fps = 1.0

            # Numero de frames que, a video_fps, duran EXACTAMENTE lo mismo que la
            # grabacion real. Se repiten o se saltan frames para encajar siempre
            # ahi, nunca se deja que la duracion del fichero se desvie.
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
