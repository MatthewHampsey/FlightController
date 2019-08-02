controller_sources = ['dynamic_compensation_qc.cpp',
                    'dynamic_compensation_qc.h',
                    'quadcopter_controller.h',
                    'quat_control.h',
                    'quat_control.cpp']

controller_lib = shared_library('controller',
                               controller_sources,
                               dependencies: [eigen_dep, framedrag_types_dep, rotation_external_dep, rotation_private_dep])

controller_dep = declare_dependency(link_with: [controller_lib],
                                  include_directories: include_directories('.'),
                                  dependencies: [framedrag_types_dep, rotation_external_dep])
