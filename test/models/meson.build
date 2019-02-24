models_test_sources = ['test_quadcopter.cpp']

prop_controller_test_sources = ['test_prop_controller.cpp']

models_test_exe = executable('test_models', sources: [models_test_sources],  dependencies: [FrameDrag_test_dep, models_dep])
prop_controller_test_exe = executable('test_prop_controller', sources: [prop_controller_test_sources],  dependencies: [FrameDrag_test_dep, models_dep])

test('Models test', models_test_exe)
test('Prop Controller test', prop_controller_test_exe)