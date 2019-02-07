subdir('private')

rotation_test_sources = ['test_euler.cpp']

rotation_test_exe = executable('test_rotation', sources: [rotation_test_sources],  dependencies: [FrameDrag_test_dep, rotation_external_dep])

test('Public rotation test', rotation_test_exe)