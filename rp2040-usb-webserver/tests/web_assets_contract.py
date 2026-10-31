"""Verify the actual embedded payload, not just the host filesystem fake."""
from pathlib import Path
import re
import subprocess
import sys
from tempfile import TemporaryDirectory

ROOT = Path(__file__).resolve().parent.parent


def run():
    with TemporaryDirectory() as temp:
        output = Path(temp) / 'web_assets.h'
        subprocess.run([sys.executable, ROOT.parent / 'shared/rp2040-usb-web/tools/embed_web.py', ROOT / 'web', output], check=True)
        generated = output.read_text()
        arrays = {index: bytes(int(value) for value in values.split(',') if value.strip())
                  for index, values in re.findall(
                      r'web_asset_(\d+)\[\] = \{(.*?)\};', generated, re.S)}
        entries = dict(re.findall(r'\{"/([^"\n]+)", web_asset_(\d+),', generated))
        expected = {'index.html', 'style.css', *(file.name for file in (ROOT / 'web').glob('*.js'))}
        assert set(entries) == expected, 'Every frontend module must be embedded'
        cmake = (ROOT.parent / 'shared/rp2040-usb-web/CMakeLists.txt').read_text()
        for name, index in entries.items():
            header, body = arrays[index].split(b'\r\n\r\n', 1)
            assert body == (ROOT / 'web' / name).read_bytes(), name
            assert f'Content-Length: {len(body)}'.encode() in header, name
            assert 'GLOB_RECURSE assets CONFIGURE_DEPENDS' in cmake
            if name.endswith('.js'):
                imports = re.findall(r"from\s+['\"]\./([^'\"]+)['\"]", body.decode())
                assert set(imports) <= set(entries), f'Unserved module import in {name}'
        print(f'Embedded web assets: {len(entries)} exact payloads, lengths, imports and build dependencies OK')


if __name__ == '__main__':
    run()
