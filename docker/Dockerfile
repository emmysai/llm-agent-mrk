# ------------------------------------------------------------------
# Basis-Image: ROS2 Humble Desktop (vollständige Variante)
# ------------------------------------------------------------------
FROM osrf/ros:humble-desktop-full

# Deaktiviert interaktive Installationsabfragen während des Builds
ENV DEBIAN_FRONTEND=noninteractive

# Standardmodell für TurtleBot3 definieren
ENV TURTLEBOT3_MODEL=burger

# ------------------------------------------------------------------
# Benutzer im Container anlegen
# ------------------------------------------------------------------
ARG USERNAME=ubuntu
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME && \
    useradd -m -s /bin/bash --uid $USER_UID --gid $USER_GID $USERNAME && \
    apt update && apt install -y sudo && \
    echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME && \
    chmod 0440 /etc/sudoers.d/$USERNAME

USER $USERNAME
WORKDIR /home/$USERNAME

# ------------------------------------------------------------------
# Installation zusätzlicher Systemabhängigkeiten
# (aus externer Paketliste)
# ------------------------------------------------------------------
COPY apt-packages.txt /tmp/apt-packages.txt

RUN sudo apt-get update && \
    grep -Ev '^\s*#' /tmp/apt-packages.txt | grep -Ev '^\s*$' | \
    xargs sudo apt-get install -y --no-install-recommends && \
    sudo rm -rf /var/lib/apt/lists/*

# ------------------------------------------------------------------
# Installation der benötigten Python-Bibliotheken
# ------------------------------------------------------------------
COPY requirements.txt /tmp/requirements.txt

RUN pip install --no-cache-dir -r /tmp/requirements.txt

# ------------------------------------------------------------------
# Automatisches Einbinden der ROS2-Umgebung beim Start
# ------------------------------------------------------------------
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

RUN mkdir -p /home/$USERNAME/ros2_ws/src && \
    echo 'if [ -f ~/ros2_ws/install/setup.bash ]; then source ~/ros2_ws/install/setup.bash; fi' >> ~/.bashrc

# Arbeitsverzeichnis auf ROS2-Workspace setzen
WORKDIR /home/$USERNAME/ros2_ws

# Standard-Startbefehl (interaktive Shell)
CMD ["/bin/bash"]