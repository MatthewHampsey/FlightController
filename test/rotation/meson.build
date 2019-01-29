rotation_test_sources = ['test_euler.cpp']

testdep = dependency('boost', modules : ['test'])

rotation_test_exe = executable('test_rotation', sources: [rotation_test_sources],  dependencies: [testdep, rotation_external_dep])

test('Boost UTF test', rotation_test_exe)