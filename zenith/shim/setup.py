from cx_Freeze import setup, Executable

# Dependencies are automatically detected, but it might need
# fine tuning.
build_options = {
    'packages': [
        # Dependencies required by black=24.1.1.
        "629853fdff261ed89b74__mypyc",
        "json",
        "platform",
        "click",
        "mypy_extensions",
        "pathspec",
        "_black_version",
        "platformdirs",
        "black.output",
        "blib2to3",
        "black.files",
    ],
    'excludes': [],
    'build_exe': 'build-py'
}

base = 'console'

executables = [
    Executable('shim/main.py', target_name="zenith-cli", base=base)
]

setup(name='Zenith CLI',
      version = '1.0',
      description = 'Zenith CLI',
      options = {'build_exe': build_options},
      executables = executables)
