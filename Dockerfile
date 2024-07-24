FROM archlinux
RUN pacman -Syu --noconfirm \
    base-devel \
    git \
    arm-none-eabi-gcc \
    arm-none-eabi-newlib \
    python-pyusb \
    openocd \
    clang
