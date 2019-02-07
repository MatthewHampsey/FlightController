subdir('private')

rotation_sources = ['euler.cpp',
                    'euler.h',
                    'rotation.h',
                    'rotation.cpp']

rotation_lib = shared_library('rotation',
                               rotation_sources,
                               dependencies: [eigen_dep, vector_dep, rotation_private_dep])

rotation_external_dep = declare_dependency(link_with: [rotation_lib],
                                  include_directories: include_directories('.'),
                                  dependencies: [vector_dep])