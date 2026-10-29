FROM python:3.12-slim

# Librerias de sistema necesarias para que OpenCV pueda abrir ventanas (cv2.imshow)
# via X11, y para que pypylon pueda acceder a la red GigE.
RUN apt-get update && apt-get install -y --no-install-recommends \
    libgl1 \
    libglib2.0-0 \
    libsm6 \
    libxext6 \
    libxrender1 \
    libgtk2.0-0 \
    iputils-ping \
    net-tools \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /work

COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

# Los scripts se montan por volumen (ver docker-compose.yml), no se copian aqui,
# para poder editarlos sin reconstruir la imagen.

CMD ["tail", "-f", "/dev/null"]
