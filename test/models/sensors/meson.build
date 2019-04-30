madgwick_test_sources = ['test_madgwick_filter.cpp',
                       'common.h']

mahony_test_sources = ['test_mahony_filter.cpp',
                       'common.h']

madgwick_test_exe = executable('test_madgwick', sources: [madgwick_test_sources],  dependencies: [FrameDrag_test_dep, sensor_dep])
mahony_test_exe = executable('test_mahony', sources: [mahony_test_sources],  dependencies: [framedrag_types_dep, FrameDrag_test_dep, sensor_dep])

test('Madgwick test', madgwick_test_exe)
test('Mahony test', mahony_test_exe)