vector_sources = ['vector.h',
                  'vector3f.h',
                  #'vector3f.cpp',
                  'vector4f.h',]
                  #'vector4f.cpp']

vector_lib = shared_library('vector',
                            vector_sources,
                            dependencies: [eigen_dep])

vector_dep = declare_dependency(link_with: vector_lib,
                                  include_directories: include_directories('.'),
				  dependencies: [eigen_dep])
