quat_sources = ['quaternion.h',
                'quaternion.cpp']

quat_lib = shared_library('quaternion',
                            quat_sources,
                            dependencies: [vector_dep])

quat_dep = declare_dependency(link_with: quat_lib,
                                  dependencies: [vector_dep],
                                  include_directories: include_directories('.'))