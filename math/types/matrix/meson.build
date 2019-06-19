matrix_sources = ['matrix.h']
                  #'matrix.cpp']

matrix_lib = shared_library('matrix',
                            matrix_sources,
                            dependencies: [eigen_dep, vector_dep])

matrix_dep = declare_dependency(link_with: matrix_lib,
                                  include_directories: include_directories('.'),
                                  dependencies: [eigen_dep, vector_dep])
