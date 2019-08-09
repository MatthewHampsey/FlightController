hw_sources = ['main_quat.cpp',
			  'receiver.cpp',
			  'pru_comms.cpp',
	      	  'mpu9250.h',
              'mpu9250_spi.cpp']


hw_exe = executable('test_hw', sources: [hw_sources],  dependencies: [models_dep, controller_dep,
								      sensor_dep, framedrag_types_dep])

