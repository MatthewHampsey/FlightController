hw_sources = ['main.cpp',
	      'mpu9250.h',
              'mpu9250.cpp']


subdir ('math')


hw_exe = executable('test_hw', sources: [hw_sources],  dependencies: [FrameDrag_test_dep, sensor_dep, framedrag_types_dep])

