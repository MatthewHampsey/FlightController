project('FrameDrag', 'cpp',
        default_options : ['cpp_std=c++14'])
subdir('thirdparty')
subdir('rotation')
subdir('test')
executable('foo', 'main.cpp', dependencies :[rotation_external_dep, vector_dep])