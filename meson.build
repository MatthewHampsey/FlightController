project('FrameDrag', 'cpp',
        default_options : ['cpp_std=c++14'])
subdir('thirdparty')
subdir('math')
subdir('test')
executable('foo', 'main.cpp', dependencies :[rotation_external_dep, framedrag_types_dep])