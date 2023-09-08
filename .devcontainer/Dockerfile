FROM ros:iron

ARG USERNAME=vscode
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN apt update && apt install -y texlive texlive-latex-extra texlive-plain-generic python3-pip make ghostscript imagemagick python3-typing-extensions python3-networkx python3-matplotlib sudo doxygen
RUN pip install --upgrade sphinx~=7.0 deepdiff jsonschema sphinx-jsonschema breathe
RUN sed -i '/disable ghostscript format types/,+6d' /etc/ImageMagick-6/policy.xml

# Create the user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

USER $USERNAME
