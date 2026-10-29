#!/usr/bin/env python3
"""
Calibracion intrinseca de camara (patron de tablero de ajedrez) con OpenCV.

Uso:
    python3 calibrate.py --images ./calib_images --cols 9 --rows 6 --square 25.0 \
        --out calibration.yaml --camera-name camt1

    --cols / --rows : numero de ESQUINAS INTERNAS del tablero (no de casillas).
                       Un tablero de 10x7 casillas tiene 9x6 esquinas internas.
    --square         : tamano de un lado de casilla, en milimetros (o cualquier
                        unidad consistente; solo afecta a la escala, no a la
                        calidad de la calibracion).

Salida:
    Un fichero YAML con el mismo formato que usa camera_info_manager de ROS
    (compatible con el parametro `camera_info_url` del driver, por si en el
    futuro quieres reutilizar esta calibracion desde ROS2).

Requisitos:
    pip install opencv-python numpy pyyaml --break-system-packages
"""

import argparse
import glob
import os
import sys

import cv2
import numpy as np
import yaml


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--images", required=True, help="Carpeta con las imagenes capturadas (png/jpg)")
    parser.add_argument("--cols", type=int, required=True, help="Esquinas internas del tablero, eje horizontal")
    parser.add_argument("--rows", type=int, required=True, help="Esquinas internas del tablero, eje vertical")
    parser.add_argument("--square", type=float, default=1.0, help="Tamano del lado de una casilla (unidad libre, por defecto 1.0)")
    parser.add_argument("--out", default="calibration.yaml", help="Fichero YAML de salida")
    parser.add_argument("--camera-name", default="camera", help="Nombre de la camara para el YAML de salida")
    parser.add_argument("--show", action="store_true", help="Muestra cada imagen con las esquinas detectadas")
    parser.add_argument("--coverage", default=None, help="Guarda un PNG mostrando donde cayeron todas las esquinas detectadas (para ver que zonas del frame te faltan)")
    args = parser.parse_args()

    pattern_size = (args.cols, args.rows)

    image_paths = sorted(
        glob.glob(os.path.join(args.images, "*.png"))
        + glob.glob(os.path.join(args.images, "*.jpg"))
        + glob.glob(os.path.join(args.images, "*.jpeg"))
    )
    if not image_paths:
        sys.exit(f"No se han encontrado imagenes en '{args.images}'.")

    # Puntos 3D del patron en su propio sistema de coordenadas (Z=0, plano).
    objp = np.zeros((pattern_size[0] * pattern_size[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:pattern_size[0], 0:pattern_size[1]].T.reshape(-1, 2)
    objp *= args.square

    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    obj_points = []  # puntos 3D reales
    img_points = []  # puntos 2D detectados en la imagen
    used_paths = []  # ruta de cada imagen que se uso, en el mismo orden que obj_points/img_points
    image_size = None
    used = 0

    for path in image_paths:
        img = cv2.imread(path)
        if img is None:
            print(f"Aviso: no se pudo leer '{path}', se omite.")
            continue

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        if image_size is None:
            image_size = gray.shape[::-1]  # (width, height)

        found, corners = cv2.findChessboardCorners(
            gray, pattern_size,
            cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE + cv2.CALIB_CB_FAST_CHECK
        )

        if found:
            corners_refined = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            obj_points.append(objp)
            img_points.append(corners_refined)
            used_paths.append(path)
            used += 1

            if args.show:
                vis = img.copy()
                cv2.drawChessboardCorners(vis, pattern_size, corners_refined, found)
                cv2.imshow("Esquinas detectadas", vis)
                cv2.waitKey(300)
        else:
            print(f"No se detecto el patron en: {os.path.basename(path)}")

    if args.show:
        cv2.destroyAllWindows()

    print(f"\nPatron detectado en {used}/{len(image_paths)} imagenes.")
    if used < 10:
        print("Aviso: con menos de 10 imagenes validas la calibracion suele ser poco fiable. "
              "Repite la captura con mas variedad de angulos/distancias si puedes.")
    if used < 4:
        sys.exit("Insuficientes imagenes validas para calibrar.")

    print("Calculando calibracion...")
    rms, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
        obj_points, img_points, image_size, None, None
    )

    print(f"\nError de reproyeccion (RMS): {rms:.4f} px")
    print("Camera matrix (fx, fy, cx, cy):")
    print(f"  fx = {camera_matrix[0, 0]:.3f}")
    print(f"  fy = {camera_matrix[1, 1]:.3f}")
    print(f"  cx = {camera_matrix[0, 2]:.3f}")
    print(f"  cy = {camera_matrix[1, 2]:.3f}")
    print(f"Coeficientes de distorsion (k1, k2, p1, p2, k3): {dist_coeffs.flatten()}")

    if rms > 1.0:
        print("\nAviso: un RMS > 1.0 px suele indicar una calibracion poco precisa "
              "(pocas imagenes, patron detectado con poca variedad de poses, o patron "
              "mal medido). Revisa --square y la cobertura de angulos/distancias.")

    # Error de reproyeccion POR IMAGEN: ayuda a identificar capturas concretas
    # (borrosas, el tablero se movio durante la exposicion, etc.) que estan
    # penalizando el RMS global, para poder descartarlas y recalibrar.
    per_image_errors = []
    for i in range(len(obj_points)):
        projected, _ = cv2.projectPoints(obj_points[i], rvecs[i], tvecs[i], camera_matrix, dist_coeffs)
        detected = img_points[i].reshape(-1, 2).astype(np.float64)
        projected = projected.reshape(-1, 2).astype(np.float64)
        error = float(np.sqrt(np.mean(np.sum((detected - projected) ** 2, axis=1))))
        per_image_errors.append((error, used_paths[i]))

    per_image_errors.sort(reverse=True)
    print("\nError de reproyeccion por imagen (peores primero):")
    for error, path in per_image_errors[:10]:
        flag = "  <-- revisa esta" if error > 2 * rms else ""
        print(f"  {error:.3f} px  {os.path.basename(path)}{flag}")
    if len(per_image_errors) > 10:
        print(f"  ... y {len(per_image_errors) - 10} mas.")

    worst_outliers = [p for e, p in per_image_errors if e > 2 * rms]
    if worst_outliers:
        print(f"\n{len(worst_outliers)} imagen(es) con error > 2x el RMS global — "
              "prueba a borrarlas de la carpeta y recalibrar, puede que suban la precision:")
        for p in worst_outliers:
            print(f"  {os.path.basename(p)}")

    # Mapa de cobertura: muestra donde cayeron todas las esquinas detectadas,
    # para ver a simple vista que zonas del frame (sobre todo bordes/esquinas)
    # te faltan por cubrir con mas capturas.
    if args.coverage:
        width, height = image_size
        canvas = np.full((height, width, 3), 30, dtype=np.uint8)
        for pts in img_points:
            for pt in pts.reshape(-1, 2):
                cv2.circle(canvas, (int(pt[0]), int(pt[1])), 3, (0, 200, 255), -1)
        cv2.imwrite(args.coverage, canvas)
        print(f"\nMapa de cobertura guardado en: {args.coverage}")
        print("Si ves zonas del frame vacias (sobre todo cerca de los bordes/esquinas), "
              "captura mas imagenes con el tablero ahi.")

    width, height = image_size
    data = {
        "image_width": int(width),
        "image_height": int(height),
        "camera_name": args.camera_name,
        "camera_matrix": {
            "rows": 3, "cols": 3,
            "data": camera_matrix.flatten().tolist(),
        },
        "distortion_model": "plumb_bob",
        "distortion_coefficients": {
            "rows": 1, "cols": 5,
            "data": dist_coeffs.flatten().tolist(),
        },
        "rectification_matrix": {
            "rows": 3, "cols": 3,
            "data": np.eye(3).flatten().tolist(),
        },
        "projection_matrix": {
            "rows": 3, "cols": 4,
            "data": np.hstack([camera_matrix, np.zeros((3, 1))]).flatten().tolist(),
        },
    }

    with open(args.out, "w") as f:
        yaml.dump(data, f, default_flow_style=None, sort_keys=False)

    print(f"\nGuardado en: {args.out}")
    print("Para usarlo desde el driver ROS2 (si lo necesitas mas adelante):")
    print(f"  camera_info_url: \"file://{os.path.abspath(args.out)}\"")


if __name__ == "__main__":
    main()