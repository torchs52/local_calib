sudo apt install -y qemu binfmt-support qemu-user-static

./docker_build.sh

# 中間出力先
# - scripts/onnxruntime/out/
#
# 出力先
# - requirements/wheels/onnxruntime_gpu-*.whl
# - requirements/wheels/onnxruntime/LICENSE.txt
# - requirements/wheels/onnxruntime/ThirdPartyNotices.txt
