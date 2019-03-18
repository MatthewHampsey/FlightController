sensor_test_sources = ['test_madgwick_filter.cpp']

sensor_test_exe = executable('test_sensor', sources: [sensor_test_sources],  dependencies: [FrameDrag_test_dep, sensor_dep])

test('Sensor test', sensor_test_exe)