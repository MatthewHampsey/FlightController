sensor_sources = ['grad.h',
                  'grad.cpp',
                  'madgwick_marg_filter.h',
                  'madgwick_marg_filter.cpp',
                  'madgwick_imu_filter.h',
                  'madgwick_imu_filter.cpp']

sensor_lib = shared_library('sensors',
                             sensor_sources,
                             dependencies: [framedrag_types_dep, vector_dep, matrix_dep])

sensor_dep = declare_dependency(link_with: [sensor_lib],
                                  include_directories: include_directories('.'),
                                  dependencies: [framedrag_types_dep, vector_dep])