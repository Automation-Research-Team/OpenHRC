# ohrc_openxr

instraction for use Meta Quest 2 on Ubuntu 20.04
need nvidia driver version to be at least 520.56.06, and cuda

## install ALVR Server

### install Rust
```
$ curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
```
### install nv-codec-headers
```
$ cd (ws_dir)
$ git clone https://github.com/FFmpeg/nv-codec-headers.git
$ cd nv-codec-headers/
$ sudo make install
```

### install addition packages  
```
$ sudo apt install build-essential pkg-config libclang-dev libssl-dev libasound2-dev libjack-dev libgtk-3-dev libvulkan-dev libunwind-dev gcc-8 g++-8 yasm nasm curl libx264-dev libx265-dev libxcb-render0-dev libxcb-shape0-dev libxcb-xfixes0-dev libspeechd-dev libxkbcommon-dev libdrm-dev libva-dev libvulkan-dev
```

### install vaulkan-headeres
```
$ cd (ws_dir)
$ git clone https://github.com/KhronosGroup/Vulkan-Headers.git -b sdk-1.3.239
$ cd Vulkan-Headers
$ cmake -S . -B build/
$ sudo cmake --install build
```

### install ALVR
```
$ cd (ws_dir)
$ git clone https://github.com/alvr-org/ALVR.git -b v19.1.0
$ cd ALVR
$ sudo cp packaging/deb/cuda.pc /usr/local/lib/pkgconfig/
$ cargo xtask prepare-deps --platform linux
$ cargo xtask build-server --release 
```

##