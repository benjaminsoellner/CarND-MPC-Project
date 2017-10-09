# Dockerfile containing software for Control C++ quizzes
FROM ubuntu:xenial

WORKDIR /project

###
# Allow remote debugging etc. via SSH
###

RUN apt-get update && apt-get install -y openssh-server
RUN mkdir /var/run/sshd

# Allow login with root user and password
RUN echo 'root:udacity' | chpasswd
RUN sed -i 's/PermitRootLogin prohibit-password/PermitRootLogin yes/' /etc/ssh/sshd_config

# SSH login fix. Otherwise user is kicked off after login
RUN sed 's@session\s*required\s*pam_loginuid.so@session optional pam_loginuid.so@g' -i /etc/pam.d/sshd

# Allows X11 forwarding also from remote host
RUN echo 'X11UseLocalhost no' >> /etc/ssh/sshd_config

# Workaround to enable X11 forwarding also from Netbeans
# See: http://w.planetnetbeans.org/topic62104.html
RUN apt-get update && apt-get install -y x11-xserver-utils
RUN echo 'export DISPLAY=10.0.2.2:0' >> /root/.profile
RUN echo 'xhost +' >> /root/.profile

EXPOSE 22
CMD ["/usr/sbin/sshd", "-D"]

###
# C++ development kit including some python stuff
###

RUN apt-get update && apt-get install -y \
    build-essential \
    gcc \
    g++ \
    gfortran \
    cmake \
    pkg-config \
    unzip \
    git \
    wget \
    cppad \
    gdb \
    python-matplotlib \ 
    python2.7-dev
    
ENV NOTVISIBLE "in users profile"
RUN echo "export VISIBLE=now" >> /etc/profile

###
# Updating
###

RUN apt-get update && apt-get upgrade

###
# IPopt optimizer
# commented out for now
###
    
ADD install_ipopt.sh .

RUN wget https://www.coin-or.org/download/source/Ipopt/Ipopt-3.12.7.zip && unzip Ipopt-3.12.7.zip && rm Ipopt-3.12.7.zip
RUN bash install_ipopt.sh Ipopt-3.12.7


###
# uWebSockets etc.
###

ADD install-ubuntu.sh .
RUN bash install-ubuntu.sh


###
# Cleanup
###

RUN rm -rf /var/lib/apt/lists/*