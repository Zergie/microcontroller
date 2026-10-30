"""Small shared compiler entry point for the host-native regression suites."""
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parent.parent
COMPILER_FLAGS = (
    "-std=c11", "-Wall", "-Wextra", "-Werror", "-g",
    "-fsanitize=address,undefined", "-no-pie",
)


def compile_and_run(name, sources, directory, *, include_dirs=(), extra_flags=()):
    executable = str(Path(directory) / name)
    command = ["gcc", *COMPILER_FLAGS]
    command.extend(f"-I{include_dir}" for include_dir in include_dirs)
    command.extend(extra_flags)
    command.extend(str(source) for source in sources)
    subprocess.run([*command, "-o", executable], cwd=ROOT, check=True)
    subprocess.run([executable], cwd=ROOT, check=True)
