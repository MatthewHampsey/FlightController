subdir('private')

rotation_sources = ['euler.cpp',
                    'euler.h',
                    'rotation.h',
                    'rotation.cpp',
                    'angular_velocity_conversion.cpp',
                    'angular_velocity_conversion.h']

rotation_lib = shared_library('rotation',
                               rotation_sources,
                               dependencies: [eigen_dep, framedrag_types_dep, rotation_private_dep])

rotation_external_dep = declare_dependency(link_with: [rotation_lib],
                                  include_directories: include_directories('.'),
                                  dependencies: [framedrag_types_dep])