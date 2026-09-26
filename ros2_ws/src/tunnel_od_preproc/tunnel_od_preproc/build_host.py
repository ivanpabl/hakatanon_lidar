"""Сборка libtunnel_od_canonical на машине разработчика без ROS 2 (для pytest и tools/).
В образе Docker библиотеку собирает colcon, это не нужно.

    from tunnel_od_preproc.build_host import ensure
    ensure()                       # собирает при необходимости, выставляет TUNNEL_OD_PREPROC_LIB
"""
import os
import shutil
import subprocess
import sys
from pathlib import Path

PKG = Path(__file__).resolve().parent.parent
SOURCES = [PKG / 'src' / n for n in ('canonical.cpp', 'monitor.cpp', 'c_api.cpp')]
HEADERS = sorted((PKG / 'include' / 'tunnel_od_preproc').glob('*'))
LIB = 'libtunnel_od_canonical' + {'darwin': '.dylib', 'win32': '.dll'}.get(sys.platform, '.so')


def _variants(cxx):
    base = [cxx, '-std=c++17', '-O3', '-shared', f'-I{PKG / "include"}']
    if sys.platform == 'win32':
        # MinGW-w64 (g++ из MSYS2 / WinLibs): рантайм C++ статически, чтобы ctypes не искал libstdc++-6.dll
        yield base + ['-static']
        return
    base.append('-fPIC')
    yield base
    if sys.platform == 'darwin':
        # Command Line Tools без заголовков libc++ по умолчанию -- берём их из SDK явно
        for sdk in sorted(Path('/Library/Developer/CommandLineTools/SDKs').glob('MacOSX*.sdk'), reverse=True):
            inc = sdk / 'usr' / 'include' / 'c++' / 'v1'
            if inc.is_dir():
                yield base + ['-nostdinc++', '-isystem', str(inc), '-isysroot', str(sdk)]


def ensure(out_dir=None, quiet=True):
    """Путь к собранной библиотеке (пересобирается, если исходники новее) или None."""
    if os.environ.get('TUNNEL_OD_PREPROC_LIB') and Path(os.environ['TUNNEL_OD_PREPROC_LIB']).is_file():
        return Path(os.environ['TUNNEL_OD_PREPROC_LIB'])
    out_dir = Path(out_dir or PKG / 'build_host')
    out = out_dir / LIB
    newest = max(p.stat().st_mtime for p in SOURCES + HEADERS)
    if not out.is_file() or out.stat().st_mtime < newest:
        cxx = os.environ.get('CXX') or shutil.which('c++') or shutil.which('g++') or shutil.which('clang++')
        if not cxx:
            return None
        out_dir.mkdir(parents=True, exist_ok=True)
        for cmd in _variants(cxx):
            r = subprocess.run(cmd + [str(s) for s in SOURCES] + ['-o', str(out)], capture_output=True, text=True)
            if r.returncode == 0:
                break
        else:
            if not quiet:
                print(r.stderr, file=sys.stderr)
            return None
    os.environ['TUNNEL_OD_PREPROC_LIB'] = str(out)
    return out
