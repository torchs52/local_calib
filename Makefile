SHELL := /bin/bash

.PHONY: install clean format format-python format-cpp format-cpp-check all install-deps gen-typings build-detect2d-trt-cache run run-with-ui prof prof-with-ui

VENV_DIR := .venv
VENV_BIN := $(VENV_DIR)/bin
VENV_PYTHON := $(VENV_BIN)/python
CPP_FORMAT_DIRS := argus_synchro_lib/cpp argus_synchro_lib/python

$(VENV_PYTHON):
	python3.12 -m venv $(VENV_DIR)

install: $(VENV_PYTHON)
	@CACHE_FILE="$$(find argus_synchro_lib/build -path '*/octotree/CMakeCache.txt' -print -quit 2>/dev/null)"; EXPECTED_HOME="CMAKE_HOME_DIRECTORY:INTERNAL=$(abspath argus_synchro_lib)"; if [[ -n "$${CACHE_FILE}" ]] && ! grep -Fqx "$${EXPECTED_HOME}" "$${CACHE_FILE}"; then echo "Removing CMake cache created in a different source directory"; rm -rf argus_synchro_lib/build; fi
	@EXTRA="$$(./scripts/select_jetson_extra.sh)"; \
	echo "Installing with extras: [$${EXTRA},dev]"; \
	source $(VENV_BIN)/activate && python -m pip install -U pip; \
	source $(VENV_BIN)/activate && pip install -v --find-links ./requirements/wheels --prefer-binary -e .[$${EXTRA},dev]
	# 起動時の必須ファイル(status.mmap)を作成.
	truncate -s 4 /dev/shm/status.mmap

clean:
	# __pycache__ を削除
	find . -type d -name '__pycache__' -exec rm -r {} +

	# ディレクトリを削除
	rm -rf $(VENV_DIR)
	rm -rf argus_synchro_lib/build
	rm -rf argus_synchro_lib/install
	rm -rf argus_synchro_lib/argus_synchro_lib.egg-info
	rm -rf 3rdparty/open3d/install
	rm -rf 3rdparty/opencv/install

	# log フォルダ内のファイルを削除
	find . -type f -path './log/*.txt' -exec rm -f {} +
	find . -type f -path './log/*.mmap' -exec rm -f {} +
	find . -type f -path './log/*.dat' -exec rm -f {} +
	find . -type f -path './log/*.log' -exec rm -f {} +

	# 校正関連の一時ファイルを削除
	find . -type f -path './calibration_mat_generator_modules/temp/*.csv' -exec rm {} +
	find . -type f -path './calibration_mat_generator_modules/temp/*.pickle' -exec rm {} +
	find . -type f -path './calibration_mat_generator_modules/temp/*.txt' -exec rm {} +

	# ファイル削除
    # find . -type f \( -name "*.profile" -o -name "*.engine" -o -name "*.timing" -o -name "*.so" \) -exec rm -f {} +

	echo "Clean completed."

format: format-python format-cpp

format-python:
	ruff check --fix || true
	ruff format .

format-cpp:
	find $(CPP_FORMAT_DIRS) -type f \( -name '*.h' -o -name '*.hpp' -o -name '*.hh' -o -name '*.c' -o -name '*.cc' -o -name '*.cxx' -o -name '*.cpp' \) -print0 | xargs -0 -r clang-format -i

format-cpp-check:
	find $(CPP_FORMAT_DIRS) -type f \( -name '*.h' -o -name '*.hpp' -o -name '*.hh' -o -name '*.c' -o -name '*.cc' -o -name '*.cxx' -o -name '*.cpp' \) -print0 | xargs -0 -r clang-format -n --Werror

install-deps:
	sh ./scripts/install_deps.sh

gen-typings:
	python scripts/gen_typings.py

build-trt-cache:
	source $(VENV_BIN)/activate && python scripts/build_detect2d_trt_cache.py

all: clean install

run:
	if [ ! -f "/dev/shm/status.mmap" ]; then truncate -s 4 /dev/shm/status.mmap; fi
	source $(VENV_BIN)/activate && python scripts/run_argus.py run --config-dir ./config --log-dir ./log --mmap-dir /dev/shm

run-with-ui:
	if [ ! -f "/dev/shm/status.mmap" ]; then truncate -s 4 /dev/shm/status.mmap; fi
	source $(VENV_BIN)/activate && python scripts/run_argus.py run --config-dir ./config --log-dir ./log --mmap-dir /dev/shm --with-ui

prof:
	if [ ! -f "/dev/shm/status.mmap" ]; then truncate -s 4 /dev/shm/status.mmap; fi
	source $(VENV_BIN)/activate && python scripts/run_argus.py prof --config-dir ./config --log-dir ./log --mmap-dir /dev/shm

prof-with-ui:
	if [ ! -f "/dev/shm/status.mmap" ]; then truncate -s 4 /dev/shm/status.mmap; fi
	source $(VENV_BIN)/activate && python scripts/run_argus.py prof --config-dir ./config --log-dir ./log --mmap-dir /dev/shm --with-ui
