sudo apt install -y qemu binfmt-support qemu-user-static

./docker_build.sh

# 中間出力先
# - scripts/open3d/out/
#
# 出力先
# - 3rdparty/open3d_jetson/install/
# - 3rdparty/open3d_jetson/3rdparty/
# - requirements/wheels/open3d*.whl
# - requirements/wheels/open3d/LICENSE
# - requirements/wheels/open3d/3rdparty/<package>/LICENSE*
