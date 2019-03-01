quat_test_sources = ['test_quaternion.cpp']

quat_test_exe = executable('test_quat', 
                sources: [quat_test_sources],  dependencies: [FrameDrag_test_dep, rotation_private_dep])

test('Quaternion test', quat_test_exe)