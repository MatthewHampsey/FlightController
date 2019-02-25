controller_test_sources = ['test_dynamic_compensation_qc.cpp']

controllers_test_exe = executable('test_controllers', sources: [controller_test_sources],  dependencies: [FrameDrag_test_dep, controller_dep])

test('Controllers test', controllers_test_exe)