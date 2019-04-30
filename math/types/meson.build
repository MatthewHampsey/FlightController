subdir('vector')
subdir('matrix')
subdir('quaternion')

type_sources = ['type_conversion.h',
                'type_conversion.cpp']

type_lib = shared_library('types',
                           type_sources,
                           dependencies: [vector_dep, matrix_dep, quat_dep])

framedrag_types_dep = declare_dependency(
                                  link_with: [type_lib],
                                  include: include_directories('.'),
                                  dependencies: [vector_dep, matrix_dep, quat_dep])