models_test_sources = ['test_quadcopter.cpp']

models_test_exe = executable('test_models', sources: [models_test_sources],  dependencies: [FrameDrag_test_dep, models_dep])

test('Models test', models_test_exe)