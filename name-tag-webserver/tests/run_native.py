"""Test production animation code with a deterministic clock under ASan/UBSan."""
from pathlib import Path
import json
import base64
import hashlib
import re
import subprocess
import sys
import posixpath
from tempfile import TemporaryDirectory

ROOT = Path(__file__).resolve().parent.parent
SHARED = ROOT.parent / 'shared/rp2040-usb-web'
sys.path.insert(0, str(SHARED / 'tools'))
from native_runner import compile_and_run
sys.path.insert(0, str(SHARED / "tests"))
from run_network import run as test_network


def run():
    test_network()
    with TemporaryDirectory() as temp:
        compile_and_run('animation-edit-tests', [
            'tests/animation_edits_test.c', 'animations/animation_engine.c',
            'animations/effects.c', 'animations/lab_effects.c',
        ], temp, root=ROOT, include_dirs=[ROOT])
        compile_and_run('animation-tests', [
            'tests/animation_test.c', 'animations/animation_engine.c',
            'animations/effects.c', 'animations/lab_effects.c', 'api/animation_api.c', 'settings/settings.c', 'pads/pads.c',
        ], temp, root=ROOT, include_dirs=[ROOT])
        compile_and_run('settings-tests', ['tests/settings_test.c', 'settings/settings.c', 'pads/pads.c',
                        'animations/animation_engine.c', 'animations/effects.c', 'animations/lab_effects.c'],
                        temp, root=ROOT, include_dirs=[ROOT])
        compile_and_run('lab-effects-tests', ['tests/lab_effects_test.c',
                        'animations/effects.c', 'animations/lab_effects.c'],
                        temp, root=ROOT, include_dirs=[ROOT])
        expected = json.loads((ROOT / 'tests/fixtures/lab-frames.json').read_text(encoding='utf-8'))
        trace = subprocess.check_output([str(Path(temp) / 'lab-effects-tests'), '--trace'], text=True).splitlines()
        assert len(trace) == len(expected)
        worst = 0
        for line, reference in zip(trace, expected):
            identity, time, *values = line.split()
            assert identity == reference['id'] and int(time) == reference['ms']
            wanted = [v for pixel in reference['pixels'] for v in pixel]
            assert len(values) == len(wanted)
            error = max(abs(int(a)-b) for a, b in zip(values, wanted))
            assert error <= 2, (identity, time, error)
            worst = max(worst, error)
        print(f'Animation Lab: 80 mockup reference frames, maximum RGB difference {worst}/255')
        # Exercise the actual linker guard at both sides of the reserved area.
        obj = Path(temp) / 'guard.o'
        subprocess.run(['gcc', '-c', '-x', 'c', '-o', obj, '-'],
                       input='void _start(void) {}', text=True, check=True)
        layout = Path(temp) / 'guard.ld'
        for end, should_link in [(0x101fe000, True), (0x101fe001, False)]:
            layout.write_text('MEMORY { FLASH (rx) : ORIGIN = 0x10000000, LENGTH = 0x200000 }\n'
                              f'__flash_binary_end = {end};\n'
                              'SECTIONS { .text : { *(.text*) } > FLASH }\n')
            result = subprocess.run(['ld', '-T', layout, '-T', ROOT / 'settings/reserve.ld',
                                     obj, '-o', Path(temp) / 'guard.elf'], capture_output=True, text=True)
            assert (result.returncode == 0) == should_link, result.stderr
            if not should_link:
                assert 'overlaps persistent settings' in result.stderr
        print('Flash linker guard: exact boundary accepted; one-byte overlap rejected')
        compile_and_run('pads-tests', ['tests/pads_test.c', 'pads/pads.c', 'api/pads_api.c'],
                        temp, root=ROOT, include_dirs=[ROOT])
        compile_and_run('pad-ws-tests', ['tests/ws_protocol_test.c', 'pads/ws_protocol.c'],
                        temp, root=ROOT, include_dirs=[ROOT])
        compile_and_run('pad-tcp-tests', ['tests/ws_server_test.c', 'pads/ws_server.c', 'pads/ws_protocol.c', 'pads/pads.c'],
                        temp, root=ROOT, include_dirs=[ROOT, ROOT / 'tests/fakes'])
        for i in range(32):
            key = base64.b64encode(hashlib.sha256(str(i).encode()).digest()[:16]).decode()
            response = subprocess.check_output([str(Path(temp) / 'pad-ws-tests'), key]).decode()
            expected = base64.b64encode(hashlib.sha1((key + '258EAFA5-E914-47DA-95CA-C5AB0DC85B11').encode()).digest()).decode()
            assert f'Sec-WebSocket-Accept: {expected}\r\n' in response
        compile_and_run('pixel-tests', ['tests/pixels_test.c', 'pixels/pixels.c'], temp,
                        root=ROOT, include_dirs=[ROOT, ROOT / 'tests/fakes'])
        data = json.loads(subprocess.check_output([str(Path(temp) / 'animation-tests'), '--json']))
        assert data['selection'] == 'random' and data['current'] in data['effects']
        assert len(data['effects']) == len(set(data['effects'])) == 21
        assert data['pin'] == 0 and data['pixel_count'] == 10
        assert data['brightness_percent'] == 20 and data['interval_seconds'] == 300
        assert data['speed_percent'] == 100 and data['color'] == '#00ff28'
        assert data['original_colors'] is True and len(data['default_colors']) == 21
        output = Path(temp) / 'web_assets.h'
        subprocess.run([sys.executable, SHARED / 'tools/embed_web.py', ROOT / 'web', output], check=True)
        generated = output.read_text()
        entries = re.findall(r'\{"/([^"\n]+)", web_asset_(\d+),', generated)
        arrays = {i: bytes(int(v) for v in values.split(',') if v.strip()) for i, values in
                  re.findall(r'web_asset_(\d+)\[\] = \{(.*?)\};', generated, re.S)}
        asset_names = {name for name, _ in entries}
        assert asset_names == {p.relative_to(ROOT / 'web').as_posix() for p in (ROOT / 'web').rglob('*') if p.is_file()}
        for name, index in entries:
            header, body = arrays[index].split(b'\r\n\r\n', 1)
            assert body == (ROOT / 'web' / name).read_bytes()
            assert f'Content-Length: {len(body)}'.encode() in header
            if name.endswith('.js'):
                imports=re.findall(r"from\s+['\"]\./([^'\"]+)['\"]",body.decode())
                assert {posixpath.normpath(posixpath.join(posixpath.dirname(name), imported)) for imported in imports} <= asset_names
        print('Name Tag: JSON schema and exact embedded frontend payloads OK')
        compile_and_run('http-tests', [
            ROOT / 'tests/http_test.c', ROOT / 'api/web_routes.c', ROOT / 'api/animation_api.c',
            ROOT / 'animations/animation_engine.c', ROOT / 'animations/effects.c', ROOT / 'animations/lab_effects.c', ROOT / 'settings/settings.c', ROOT / 'pads/pads.c', ROOT / 'api/pads_api.c',
            SHARED / 'web_fs.c', SHARED / 'captive_portal.c', SHARED / 'tests/fakes/network.c',
        ], temp, root=ROOT, include_dirs=[ROOT, ROOT / 'tests/fakes', SHARED,
                                        SHARED / 'tests/fakes', temp],
            extra_flags=['-DPICO_ON_DEVICE=1', '-Wl,--wrap=malloc'])


if __name__ == '__main__':
    run()
