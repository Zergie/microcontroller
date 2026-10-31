"""Exercise cache reuse/invalidation with real GCC and a disposable cache."""
import os
from pathlib import Path
import subprocess
import sys
from tempfile import TemporaryDirectory
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from native_runner import compile_and_run


class NativeCacheTests(unittest.TestCase):
    def setUp(self):
        self.temp = TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.cache = self.root / 'cache'
        self.environment = patch.dict(os.environ, {
            'NATIVE_TEST_CACHE': str(self.cache), 'NATIVE_TEST_TOOLCHAIN': 'test-image',
        })
        self.environment.start()
        self.addCleanup(self.environment.stop)
        (self.root / 'nested.h').write_text('#define VALUE 1\n')
        (self.root / 'header.h').write_text('#include "nested.h"\n')
        (self.root / 'test.c').write_text('#include "header.h"\nint main(void) { return VALUE != 1; }\n')

    def run_program(self, *, flags=(), generated='1'):
        with TemporaryDirectory(dir=self.root) as output:
            (Path(output) / 'generated.h').write_text(f'#define GENERATED {generated}\n')
            # Includes are moved on every invocation, like the real web_assets.h.
            with patch('native_runner.subprocess.run', wraps=subprocess.run) as run:
                compile_and_run('test', ['test.c'], output, root=self.root,
                                include_dirs=[self.root, output], extra_flags=flags)
            return sum('-o' in call.args[0] for call in run.call_args_list)

    def test_reuse_and_source_header_flags_toolchain_invalidation(self):
        self.assertEqual(self.run_program(), 1)
        self.assertEqual(self.run_program(), 0)
        with (self.root / 'test.c').open('a') as source:
            source.write('// Source edit\n')
        self.assertEqual(self.run_program(), 1)
        (self.root / 'nested.h').write_text('#define VALUE (1)\n')
        self.assertEqual(self.run_program(), 1)
        self.assertEqual(self.run_program(flags=['-DOTHER=1']), 1)
        self.assertEqual(self.run_program(flags=['-DOTHER=1']), 0)
        os.environ['NATIVE_TEST_TOOLCHAIN'] = 'new-image'
        self.assertEqual(self.run_program(flags=['-DOTHER=1']), 1)

    def test_generated_header_content_not_temporary_location(self):
        with (self.root / 'test.c').open('a') as source:
            source.write('#include "generated.h"\n_Static_assert(GENERATED > 0, "generated");\n')
        self.assertEqual(self.run_program(), 1)
        self.assertEqual(self.run_program(), 0)
        self.assertEqual(self.run_program(generated='2'), 1)

    def test_corrupt_or_incomplete_cache_is_rebuilt(self):
        self.run_program()
        entry = next(path for path in self.cache.iterdir() if path.is_dir())
        (entry / 'program').write_bytes(b'corrupt')
        self.assertEqual(self.run_program(), 1)
        (entry / 'sha256').unlink()
        self.assertEqual(self.run_program(), 1)
        (entry / 'sha256').write_bytes(b'\xff')
        self.assertEqual(self.run_program(), 1)

    def test_cached_program_assertions_still_run_and_compile_failure_is_not_cached(self):
        (self.root / 'test.c').write_text('int main(void) { return 1; }\n')
        for _ in range(2):
            with self.assertRaises(subprocess.CalledProcessError):
                self.run_program()
        self.assertEqual(sum(path.is_dir() for path in self.cache.iterdir()), 1)
        (self.root / 'test.c').write_text('_Static_assert(0, "expected compilation failure");\n')
        with self.assertRaises(subprocess.CalledProcessError):
            self.run_program()
        self.assertEqual(sum(path.is_dir() for path in self.cache.iterdir()), 1)

    def test_cache_can_be_disabled(self):
        os.environ.pop('NATIVE_TEST_CACHE')
        self.assertEqual(self.run_program(), 1)
        self.assertEqual(self.run_program(), 1)
        self.assertFalse(self.cache.exists())

    def test_competing_processes_publish_one_complete_program(self):
        worker = (
            'import sys; from pathlib import Path; '
            f'sys.path.insert(0, {str(Path(__file__).resolve().parents[1] / "tools")!r}); '
            'from native_runner import compile_and_run; '
            'output=Path(sys.argv[1]); output.mkdir(); '
            'compile_and_run("test", ["test.c"], output, root=Path.cwd(), include_dirs=[Path.cwd()])'
        )
        processes = [subprocess.Popen([sys.executable, '-c', worker, str(self.root / f'out{i}')],
                                      cwd=self.root, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                      text=True) for i in range(2)]
        outputs = []
        for process in processes:
            out, err = process.communicate(timeout=30)
            self.assertEqual(process.returncode, 0, err)
            outputs.append(out)
        self.assertEqual(sum('cache build:' in out for out in outputs), 1)
        self.assertEqual(sum('cache hit:' in out for out in outputs), 1)


if __name__ == '__main__':
    unittest.main()
