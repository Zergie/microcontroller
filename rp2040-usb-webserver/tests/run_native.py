"""Compile production modules against SDK fakes under ASan/UBSan."""
from pathlib import Path
import subprocess
import sys
from tempfile import TemporaryDirectory
ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT.parent / "shared/rp2040-usb-web/tools"))
from native_runner import compile_and_run
SHARED = ROOT.parent / "shared/rp2040-usb-web"
from web_assets_contract import run as test_web_assets
sys.path.insert(0, str(SHARED / "tests"))
from run_network import run as test_network

# Keep each suite's source dependencies and special flags explicit.
TARGETS = [
    ('native-tests', [
        'tests/native.c', 'tests/fakes/sdk.c', 'gpio_api.c', 'gpio_control.c',
        SHARED / 'dhcpserver/dhcp_options.c',
    ], []),
    ('web-fs-tests', [
        'tests/web_fs_test.c', SHARED / 'tests/fakes/network.c', 'tests/fakes/sdk.c',
        SHARED / 'web_fs.c', 'web_routes.c', 'gpio_api.c', 'gpio_control.c', SHARED / 'captive_portal.c',
    ], ['-DPICO_ON_DEVICE=1', '-Wl,--wrap=malloc']),
]


def run():
    test_web_assets()
    test_network()
    with TemporaryDirectory() as temp:
        for name, sources, flags in TARGETS:
            compile_and_run(name, sources, temp,
                            root=ROOT, include_dirs=[ROOT, SHARED, SHARED / 'tests/fakes', ROOT / 'tests/fakes', temp],
                            extra_flags=['-fno-omit-frame-pointer', *flags])
            if name == 'native-tests':
                subprocess.run([sys.executable, ROOT / 'tests/snapshot_contract.py',
                                str(Path(temp) / name)], cwd=ROOT, check=True)


if __name__ == '__main__':
    run()
