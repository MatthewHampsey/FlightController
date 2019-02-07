subdir('vector')
subdir('matrix')
subdir('rotation')
math_test_sources = ['test_derivative.cpp']

math_test_exe = executable('test_math', sources: [math_test_sources],  dependencies: [FrameDrag_test_dep, math_external_dep, vector_dep])

test('Public rotation test', math_test_exe)