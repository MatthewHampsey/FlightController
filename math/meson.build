math_sources = ['derivative.h']

math_lib = shared_library('math',
                           math_sources,
                           dependencies: [])

math_external_dep = declare_dependency(link_with: [math_lib],
                                  include_directories: include_directories('.'),
                                  dependencies: [])
subdir('vector')
subdir('matrix')
subdir('rotation')