vector3f_test_sources = ['test_vector3f.cpp']
vector4f_test_sources = ['test_vector4f.cpp']

vector3f_test_exe = executable('test_vector3f', sources: [vector3f_test_sources],  dependencies: [FrameDrag_test_dep, framedrag_types_dep])
vector4f_test_exe = executable('test_vector4f', sources: [vector4f_test_sources],  dependencies: [FrameDrag_test_dep, framedrag_types_dep])

test('Vector test', vector3f_test_exe)
test('Vector test', vector4f_test_exe)