
dep = dependency('boost', required: false)
if not dep.found()
  error('MESON_SKIP_TEST boost not found.')
endif

subdir ('rotation')