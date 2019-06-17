subdir('vector')
subdir('matrix')
subdir('quaternion')

conversion_sources = ['test_type_conversion.cpp']

conversion_test_exe = executable('test_conversion', sources: [conversion_sources],  dependencies: [FrameDrag_test_dep, framedrag_types_dep])

test('conversion test', conversion_test_exe)
