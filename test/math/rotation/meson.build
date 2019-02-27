subdir('private')

rotation_test_sources = ['test_rotation.cpp']
angular_velocity_test_sources = ['test_angular_velocity_conversion.cpp']

rotation_test_exe = executable('test_rotation', sources: [rotation_test_sources],  dependencies: [FrameDrag_test_dep, rotation_external_dep])
angular_velocity_test_exe = executable('test_angular_velocity', sources: [angular_velocity_test_sources],  dependencies: [FrameDrag_test_dep, math_external_dep, rotation_external_dep, rotation_private_dep])

test('Public rotation test', rotation_test_exe)
test('Angular velocity conversion test', angular_velocity_test_exe)