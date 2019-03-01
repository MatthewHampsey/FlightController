rotation_private_sources = ['rotation_matrix.h',
                            'rotation_matrix.cpp']

rotation_private_lib = shared_library('rotation_private',
                               rotation_private_sources,
                               dependencies: [framedrag_types_dep],
)

rotation_private_dep = declare_dependency(
                                  link_with: rotation_private_lib,
                                  include_directories: include_directories('.'),
                                  dependencies: [framedrag_types_dep])