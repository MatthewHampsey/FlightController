quat_test_sources = ['test_quaternion.cpp']
rot_matrix_test_sources = ['test_rotation_matrix.cpp']

quat_test_exe = executable('test_quat', 
                sources: [quat_test_sources],  dependencies: [FrameDrag_test_dep, rotation_private_dep])

rot_matrix_test_exe = executable('test_rot_matrix', 
                sources: [rot_matrix_test_sources],  dependencies: [FrameDrag_test_dep, rotation_private_dep])

test('Quaternion test', quat_test_exe)
test('Rotation matrix test', rot_matrix_test_exe)