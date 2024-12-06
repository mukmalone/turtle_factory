FROM osrf/ros:noetic-desktop-full

# Update and Install dependencies
#ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y \
    git \
    ros-noetic-move-base-msgs \
    libcurl4-openssl-dev 
#    xfce4 \
#    xfce4-goodies \
#    apt-utils \
#    tightvncserver

# Create a user for VNC (if not already created by the base image)
#RUN useradd -ms /bin/bash vncuser  # or whatever username you prefer
#ENV USER vncuser

# Setup catkin workspace
WORKDIR /catkin_ws
RUN mkdir -p src && \
    cd src && \
    git clone https://github.com/mukmalone/agm_comm.git && \
    git clone https://github.com/mukmalone/agm_msgs.git && \
    git clone https://github.com/mukmalone/turtle_factory.git

# Build the ROS workspace
RUN /bin/bash -c '. /opt/ros/noetic/setup.bash; catkin_make'

# Expose ports if necessary (example using port 80 for a web UI)
#EXPOSE 80

# Install VNC and XFCE for GUI
# (Already installed in the RUN command above)

# Setup VNC password (default password is 'vncpass')
#RUN mkdir -p /home/$USER/.vnc
#RUN echo "vncpass" | vncpasswd -f > /home/$USER/.vnc/passwd
#RUN chown -R $USER:$USER /home/$USER/.vnc
#RUN chmod 700 /home/$USER/.vnc
#RUN chmod 600 /home/$USER/.vnc/passwd
#RUN echo "/usr/bin/vncserver :1 -geometry 1280x800 -depth 24" > /home/$USER/.vnc/xstartup
#RUN chmod 755 /home/$USER/.vnc/xstartup
#RUN touch /home/vncuser/.Xauthority
#RUN chown vncuser:vncuser /home/vncuser/.Xauthority
#RUN chmod 600 /home/vncuser/.Xauthority

# Create VNC startup script
#RUN echo "/usr/bin/vncserver :1 -geometry 1280x800 -depth 24" > /home/$USER/.vnc/xstartup
#RUN chmod +x /home/$USER/.vnc/xstartup

# Switch to the new user
#USER $USER

# Start VNC server when the container starts
#CMD ["vncserver", ":1", "-geometry", "1280x800", "-depth", "24"]