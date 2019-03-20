subdir('vector')
subdir('matrix')
subdir('quaternion')

types_sources = ['conversion.h',
                 'conversion.cpp']

types_lib = shared_library('types',
                            types_sources,
                            dependencies: [vector_dep, quat_dep])

framedrag_types_dep = declare_dependency(
                                  link_with: types_lib,
                                  dependencies: [vector_dep, matrix_dep, quat_dep],
                                  include_directories: include_directories('.'))