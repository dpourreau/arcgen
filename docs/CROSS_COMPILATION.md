# Cross-Compilation Guide (Host -> Raspberry Pi)

This guide explains how to cross-compile `arcgen` on a more powerful host machine (macOS, Linux, or Windows w/ WSL) and run the binaries on your Raspberry Pi. This approach is significantly faster than compiling locally on the Pi and avoids Out-of-Memory (OOM) crashes on devices with limited RAM.

## Prerequisites

1.  **Docker Desktop** (or Docker Engine on Linux) install on your host machine.
2.  **SSH Access** to your Raspberry Pi.

## Step 1: Set up the Build Environment

We use `dockcross`, a Docker image that pre-packages a complete cross-compilation toolchain for ARM64 (Linux).

1.  **Generate the runner script:**
    Run this command in your project root on your host:
    ```bash
    docker run --rm dockcross/linux-arm64 > ./dockcross-arm64
    chmod +x ./dockcross-arm64
    ```

    > **Note for Apple Silicon (M1/M2/M3) Users:** 
    > You may see a warning: `The requested image's platform (linux/amd64) does not match the detected host platform`. You can safely ignore this; Docker's emulation handles it correctly.

## Step 2: Configure

Use the `./dockcross-arm64` script to run CMake commands inside the container. The `CMakeLists.txt` is configured to automatically download dependencies (Boost 1.83.0, CGAL 5.6) so you do **not** need to install them manually.

```bash
# Clean previous builds to be safe
rm -rf build/pi

# Configure for Release
./dockcross-arm64 cmake -B build/pi -S . -GNinja \
    -DCMAKE_BUILD_TYPE=Release \
    -DAG_BUILD_TESTS=ON
```

## Step 3: Build

Compile the project. 

```bash
./dockcross-arm64 cmake --build build/pi -j 4
```

## Step 4: Deployment

Once the build finishes successfully, transfer the binary to your Raspberry Pi.

1.  **Ensure destination directory exists on Pi:**
    ```bash
    # Replace <user>, <ip>, and <remote_path> with your details
    ssh <user>@<ip> "mkdir -p <remote_path>/build/pi"
    ```

2.  **Copy the executable:**
    ```bash
    scp build/pi/ag_tests <user>@<ip>:<remote_path>/build/pi/
    ```

## Step 5: Run on Pi

SSH into your Pi and execute the binary.

```bash
ssh <user>@<ip>
cd <remote_path>/build/pi
./ag_tests
```
