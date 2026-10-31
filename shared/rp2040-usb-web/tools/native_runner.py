"""Small shared compiler entry point for the host-native regression suites."""
from pathlib import Path
import hashlib
import json
import os
import shutil
import subprocess
from tempfile import TemporaryDirectory

COMPILER_FLAGS = (
    "-std=c11", "-Wall", "-Wextra", "-Werror", "-g",
    "-fsanitize=address,undefined", "-no-pie",
)


def _cache_key(command, sources, root, directory, toolchain):
    # Preprocessing includes transitive/system headers and freshly generated assets.
    # Temporary include-directory names must not prevent reuse on the next run.
    digest = hashlib.sha256()
    options = [arg.replace(str(directory), '<test-output>') for arg in command]
    environment = {key: os.environ.get(key) for key in (
        'CPATH', 'C_INCLUDE_PATH', 'LIBRARY_PATH', 'GCC_EXEC_PREFIX',
        'COMPILER_PATH', 'SOURCE_DATE_EPOCH',
    )}
    digest.update(json.dumps([toolchain, str(root), options, environment]).encode())
    digest.update(Path(__file__).read_bytes())
    for source in sources:
        digest.update((Path(root) / source).read_bytes())
    digest.update(subprocess.check_output([*command, '-E', '-P'], cwd=root))
    return digest.hexdigest()


def _cached_program(entry):
    try:
        return hashlib.sha256((entry / 'program').read_bytes()).hexdigest() == (entry / 'sha256').read_text()
    except (FileNotFoundError, UnicodeError):
        return False


def compile_and_run(name, sources, directory, *, root, include_dirs=(), extra_flags=()):
    executable = str(Path(directory) / name)
    command = ["gcc", *COMPILER_FLAGS]
    command.extend(f"-I{include_dir}" for include_dir in include_dirs)
    command.extend(extra_flags)
    sources = list(sources)
    command.extend(str(source) for source in sources)
    cache = os.environ.get('NATIVE_TEST_CACHE')
    toolchain = os.environ.get('NATIVE_TEST_TOOLCHAIN')
    # The Docker image ID fingerprints the compiler, linker and installed libraries.
    if cache and toolchain:
        cache = Path(cache)
        cache.mkdir(parents=True, exist_ok=True)
        entry = cache / _cache_key(command, sources, root, directory, toolchain)
        # Cache access is serialized across Docker processes; execution uses a local copy.
        import fcntl
        with (cache / '.lock').open('a') as lock:
            fcntl.flock(lock, fcntl.LOCK_EX)
            if _cached_program(entry):
                shutil.copy2(entry / 'program', executable)
                print(f'Native cache hit: {name}', flush=True)
            else:
                subprocess.run([*command, '-o', executable], cwd=root, check=True)
                # Publish binary and checksum together; interrupted/competing builds
                # cannot expose a partially written executable to another test run.
                with TemporaryDirectory(dir=cache) as staging:
                    staging = Path(staging)
                    shutil.copy2(executable, staging / 'program')
                    (staging / 'sha256').write_text(hashlib.sha256(Path(executable).read_bytes()).hexdigest())
                    if entry.exists():
                        # Replace a corrupt entry without affecting any running copy.
                        shutil.rmtree(entry)
                    staging.rename(entry)
                print(f'Native cache build: {name}', flush=True)
    else:
        subprocess.run([*command, '-o', executable], cwd=root, check=True)
    # Cache successful compilation only. Assertions and sanitizers run every time.
    subprocess.run([executable], cwd=root, check=True)
