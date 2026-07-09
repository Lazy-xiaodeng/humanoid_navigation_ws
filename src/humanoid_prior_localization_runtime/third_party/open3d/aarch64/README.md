# ARM64 Open3D 放置说明

本目录用于存放 Jetson Orin NX 等 ARM64/aarch64 平台的 Open3D 预编译包。

推荐在 Jetson/ARM64 机器上运行脚本自动构建并安装：

```bash
cd /path/to/humanoid_ws
bash src/humanoid_prior_localization_runtime/tools/install_open3d_aarch64.sh
```

脚本会把 ARM64 版本 Open3D 安装到：

```text
third_party/open3d/aarch64/open3d-0.18.0/
```

目录内需要至少包含：

```text
lib/cmake/Open3D/Open3DConfig.cmake
lib/libOpen3D.so
lib/open3d_tf_ops.so
include/open3d/...
```

CMake 会根据 `CMAKE_SYSTEM_PROCESSOR` 自动选择：

- `x86_64` 机器：`third_party/open3d/x86_64/open3d-0.18.0`
- `aarch64` 机器：`third_party/open3d/aarch64/open3d-0.18.0`

如果本目录没有 ARM64 Open3D，Jetson 上编译 OP 定位节点时会继续尝试外部 `Open3D_DIR` 或系统路径。

注意：Open3D 官方 ARM64 Linux 主要提供 Python wheel，本 OP 定位节点需要 C++ SDK，不能直接用 Python wheel 替代。
