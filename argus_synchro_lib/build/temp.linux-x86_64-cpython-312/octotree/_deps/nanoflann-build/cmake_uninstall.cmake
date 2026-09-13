# -----------------------------------------------
# File that provides "make uninstall" target
#  We use the file 'install_manifest.txt'
# -----------------------------------------------
IF(NOT EXISTS "/home/matsuoka/argus_pipe_filter/argus_synchro_lib/build/temp.linux-x86_64-cpython-312/octotree/_deps/nanoflann-build/install_manifest.txt")
  MESSAGE(FATAL_ERROR "Cannot find install manifest: \"/home/matsuoka/argus_pipe_filter/argus_synchro_lib/build/temp.linux-x86_64-cpython-312/octotree/_deps/nanoflann-build/install_manifest.txt\"")
ENDIF(NOT EXISTS "/home/matsuoka/argus_pipe_filter/argus_synchro_lib/build/temp.linux-x86_64-cpython-312/octotree/_deps/nanoflann-build/install_manifest.txt")

FILE(READ "/home/matsuoka/argus_pipe_filter/argus_synchro_lib/build/temp.linux-x86_64-cpython-312/octotree/_deps/nanoflann-build/install_manifest.txt" files)
STRING(REGEX REPLACE "\n" ";" files "${files}")
FOREACH(file ${files})
  MESSAGE(STATUS "Uninstalling \"$ENV{DESTDIR}${file}\"")
  IF(EXISTS "$ENV{DESTDIR}${file}")
    EXEC_PROGRAM(
      "/tmp/pip-build-env-7az6phcd/overlay/lib/python3.12/site-packages/cmake/data/bin/cmake" ARGS "-E remove \"$ENV{DESTDIR}${file}\""
      OUTPUT_VARIABLE rm_out
      RETURN_VALUE rm_retval
      )
    IF(NOT "${rm_retval}" STREQUAL 0)
      MESSAGE(FATAL_ERROR "Problem when removing \"$ENV{DESTDIR}${file}\"")
    ENDIF(NOT "${rm_retval}" STREQUAL 0)
  ELSE(EXISTS "$ENV{DESTDIR}${file}")
    MESSAGE(STATUS "File \"$ENV{DESTDIR}${file}\" does not exist.")
  ENDIF(EXISTS "$ENV{DESTDIR}${file}")
ENDFOREACH(file)


