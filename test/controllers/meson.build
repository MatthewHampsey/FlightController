controller_test_sources = ['test_dynamic_compensation_qc.cpp']
quat_controller_test_sources = ['test_quat_control.cpp']

controllers_test_exe = executable('test_controllers', sources: [controller_test_sources],  dependencies: [FrameDrag_test_dep, controller_dep])

quat_controllers_test_exe = executable('test_quat_controllers', sources: [quat_controller_test_sources],  dependencies: [FrameDrag_test_dep, controller_dep])
test('Controllers test', controllers_test_exe)
test('Quat controllers test', quat_controllers_test_exe)
