subdir('sensors')

models_sources = ['model.h',
                  'quadcopter.h',
                  'quadcopter.cpp',
                  'prop_controller.h',
                  'prop_controller.cpp',
                  'prop_controller_x.h',
		  'prop_controller_x.cpp']

models_lib = shared_library('models',
                             models_sources,
                             dependencies: [framedrag_types_dep, rotation_external_dep])

models_dep = declare_dependency(link_with: [models_lib],
                                  include_directories: include_directories('.'),
                                  dependencies: [framedrag_types_dep, rotation_external_dep])
