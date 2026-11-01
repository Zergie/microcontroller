import pathlib
import subprocess
import sys

root = pathlib.Path(__file__).resolve().parents[1]
subprocess.run([sys.executable, str(root.parent / 'shared/rp2040-usb-web/tools/embed_web.py'),
                str(root / 'web'), str(root / 'build/generated/web_assets.h')], check=True)
lwip = root / 'deps/lwip/src'
output = root / 'build/network_test'
sources = [root / 'tests/network_test.c', root / 'src/http_page.c', root / 'src/gpio_control.c', root / 'src/system_control.c']
sources += sorted((root / 'src/dhcpserver').glob('*.c'))
sources += sorted((lwip / 'core').glob('*.c'))
sources += sorted((lwip / 'core/ipv4').glob('*.c'))
sources += [lwip / 'netif/ethernet.c', lwip / 'apps/http/httpd.c', lwip / 'apps/http/fs.c']
# The native host has 64-bit pointers; ARM firmware uses 4-byte alignment.
command = ['gcc', '-std=c11', '-O1', '-g', '-Wall', '-Wextra', '-Werror', '-DMEM_ALIGNMENT=8',
           '-fsanitize=address,undefined', '-fno-sanitize-recover=all', '-fno-omit-frame-pointer', '-fno-pie', '-no-pie',
           '-I' + str(root / 'tests/fakes'), '-I' + str(root / 'src'), '-I' + str(root / 'build/generated'),
           '-I' + str(lwip / 'include'), *map(str, sources), '-o', str(output)]
subprocess.run(command, check=True)
subprocess.run([str(output)], check=True, timeout=15)
