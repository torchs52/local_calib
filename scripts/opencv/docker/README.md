sudo apt install -y qemu binfmt-support qemu-user-static

./docker_build.sh

# 中間出力先
# - scripts/opencv/out/
#
# 出力先
# - 3rdparty/opencv_jetson/install/
# - 3rdparty/opencv_jetson/3rdparty/
# - requirements/wheels/opencv*.whl
# - requirements/wheels/opencv/LICENSE
# - requirements/wheels/opencv/LICENSE-3RD-PARTY.txt
# - requirements/wheels/opencv/3rdparty/<package>/LICENSE*
# - requirements/wheels/opencv/opencv-python/LICENSE*
