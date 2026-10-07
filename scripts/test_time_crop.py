#!/usr/bin/env python3
"""Run time-crop regressions using an existing CMake Unix Makefiles build.

Usage: python3 scripts/test_time_crop.py [build-dir]
Reuses the application's compiled objects and link flags with a test main.
"""
import json
from pathlib import Path
import shlex
import subprocess
import sys
import tempfile

root = Path(__file__).resolve().parents[1]
build = Path(sys.argv[1] if len(sys.argv) > 1 else root / 'build').resolve()
subprocess.run(['cmake', '--build', str(build), '-j', '4'], check=True)
commands = json.loads((build / 'compile_commands.json').read_text())
entry = next(c for c in commands if Path(c['file']).name == 'main.cpp')
compile_args = shlex.split(entry['command'])
link_args = shlex.split((build / 'CMakeFiles/yawt.dir/link.txt').read_text())
with tempfile.TemporaryDirectory(prefix='yawt-time-crop-test-') as temp:
    obj = str(Path(temp) / 'regression.o')
    binary = str(Path(temp) / 'regression')
    # main.cpp also defines the application's logging categories. Retain those
    # definitions while renaming its entry point for the regression executable.
    support = str(Path(temp) / 'application_main.o')
    support_args = list(compile_args)
    support_args[support_args.index('-o') + 1] = support
    subprocess.run([*support_args, '-Dmain=yawt_application_main'],
                   cwd=entry['directory'], check=True)
    compile_args[compile_args.index('-o') + 1] = obj
    compile_args[compile_args.index('-c') + 1] = str(root / 'tests/time_crop_regression.cpp')
    # Keep assertions/checks effective even in a Release application build.
    subprocess.run(compile_args, cwd=entry['directory'], check=True)
    link_args = [obj if a.endswith('/main.cpp.o') else a for a in link_args]
    link_args.append(support)
    link_args[link_args.index('-o') + 1] = binary
    subprocess.run(link_args, cwd=build, check=True)
    import os
    environment = dict(os.environ, QT_QPA_PLATFORM="offscreen")
    subprocess.run([binary, *sys.argv[2:]], check=True, env=environment, timeout=60)
