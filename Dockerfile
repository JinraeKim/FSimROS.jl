FROM ros:foxy

RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-turtlesim \
    ros-${ROS_DISTRO}-demo-nodes-py \
    wget \
    curl \
    xclip \
    git \
    zsh \
    trash-cli \
    tmux && \
    rm -rf /var/lib/apt/lists/*

RUN chsh -s `which zsh`  # change shell
# oh-my-zsh
RUN curl -L https://raw.github.com/robbyrussell/oh-my-zsh/master/tools/install.sh | sh

# oh-my-tmux
RUN cd && \
    git clone https://github.com/gpakosz/.tmux.git && \
    ln -s -f .tmux/.tmux.conf && \
    cp .tmux/.tmux.conf.local .

# neovim
RUN sudo apt-get update
RUN sudo apt-get install -y software-properties-common
RUN sudo add-apt-repository ppa:neovim-ppa/unstable
RUN sudo apt-get install -y neovim
RUN touch /root/.zshrc  # /root = ~
RUN echo 'alias vim="nvim"' >> /root/.zshrc
RUN sed -i '/plugins=(git)/c\plugins=(git tmux)' /root/.zshrc  # find a line and modify it

# make a workspace for ROS2
RUN mkdir -p /root/dev_ws/src
RUN cd /root/dev_ws/src && git clone https://github.com/ros/ros_tutorials.git -b foxy-devel
RUN cd /root/dev_ws && rosdep install -i --from-path src --rosdistro foxy -y
# copy for required packages
COPY /src/fsim_interfaces /root/dev_ws/src/fsim_interfaces
RUN cd /root/dev_ws && . /opt/ros/${ROS_DISTRO}/setup.sh && \
    colcon build  # source and https://answers.ros.org/question/373976/how-to-build-using-colcon-inside-a-dockerfile/
WORKDIR /root


# install Julia
RUN wget https://julialang-s3.julialang.org/bin/linux/x64/1.6/julia-1.6.3-linux-x86_64.tar.gz
RUN tar xf julia-1.6.3-linux-x86_64.tar.gz
RUN sudo ln -s ~/julia-1.6.3/bin/julia /usr/local/bin/julia

# install Julia packages
RUN mkdir -p /root/.julia/dev  # for dev
# install useful packages
RUN julia -e 'using Pkg; Pkg.add.(["PyCall", "UnPack", "Transducers", "Plots", "OnlineStats", "DataFrames", "ComponentArrays"])'
# FlightSims.jl family
RUN julia -e 'using Pkg; Pkg.develop.(["FlightSims", "FSimBase", "FSimZoo", "FSimPlots", "FSimROS"])'
# change it after FSimROS is registered
# RUN julia -e 'using Pkg; Pkg.develop.(["https://github.com/JinraeKim/FSimROS.jl"])


WORKDIR /root/dev_ws
RUN . install/setup.sh  # source; seems not work. sourcing is critical for e.g. PyCall

CMD ["zsh"]