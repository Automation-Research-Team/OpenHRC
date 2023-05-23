# ohrc_openxr

instraction for use Meta Quest 2 on Ubuntu 20.04
need nvidia driver version to be at least 520.56.06, and cuda

## Compile ALVR Server
Here, we use ALVR 19.1.0 on Ubuntu 20.04.

Official installation guide (https://github.com/alvr-org/ALVR/wiki/Installation-guide) is not compatible with the tested environment. The precompiled file might work on Ubuntu 22.04, but not tested.

The working directory is assumed to be `~/alvr`.

## install Nvidia driver (>520.56.06) and CUDA

The following instructions are tested with nvidia driver 530.30.02 and CUDA 12.1.
After installing cuda libraries, do not forget to add path to `~/.bashrc` or `~/.zshrc` as
```
export PATH="/usr/local/cuda/bin:$PATH"
export LD_LIBRARY_PATH="/usr/local/cuda/lib64:$LD_LIBRARY_PATH"
```


### install Rust
```
$ curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
```
### install nv-codec-headers
```
$ cd ~/alvr
$ git clone https://github.com/FFmpeg/nv-codec-headers.git
$ cd nv-codec-headers/
$ sudo make install
```

### install addition packages  
```
$ sudo apt update
$ sudo apt install build-essential pkg-config libclang-dev libssl-dev libasound2-dev libjack-dev libgtk-3-dev libvulkan-dev libunwind-dev gcc-8 g++-8 yasm nasm curl libx264-dev libx265-dev libxcb-render0-dev libxcb-shape0-dev libxcb-xfixes0-dev libspeechd-dev libxkbcommon-dev libdrm-dev libva-dev libvulkan-dev
```

### install vaulkan-headeres
```
$ cd ~/alvr
$ git clone https://github.com/KhronosGroup/Vulkan-Headers.git -b sdk-1.3.239
$ cd Vulkan-Headers
$ cmake -S . -B build/
$ sudo cmake --install build
```
Note, the latest branch does NOT work with ALVR 19.1.0.

### compile ALVR
```
$ cd ~/alvr
$ git clone https://github.com/alvr-org/ALVR.git -b v19.1.0
$ cd ALVR
$ cargo xtask prepare-deps --platform linux
$ cargo xtask build-server --release 
```

If you have some troubles on cuda libraries (e.g., failed to find cuda or libnpp), please modify packaging/deb/cuda.pc and copy it as
```
$ sudo cp  ~/alvr/packaging/deb/cuda.pc /usr/local/lib/pkgconfig/
```
This would help the compiler know where cuda is installed.

## Install ALVR client 
see "Headset side:" at https://github.com/alvr-org/ALVR/wiki/Installation-guide#basic-installation