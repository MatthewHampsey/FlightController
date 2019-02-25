
dep = dependency('boost', required: false)
if not dep.found()
  error('MESON_SKIP_TEST boost not found.')
endif

boost_test_dep = dependency('boost', 
                            modules : ['test'])

FrameDrag_test_dep = declare_dependency(dependencies: [boost_test_dep], 
                            include_directories: include_directories('.'))
subdir ('math')
subdir ('models')
subdir ('controllers')
