"""Compile production modules against SDK fakes under ASan/UBSan."""
from pathlib import Path
import subprocess
import sys
from tempfile import TemporaryDirectory
ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT.parent / "shared/rp2040-usb-web/tools"))
from native_runner import compile_and_run
from run_ncm import run as run_ncm
SHARED = ROOT.parent / "shared/rp2040-usb-web"
from web_assets_contract import run as test_web_assets

# Keep each suite's source dependencies and special flags explicit.
TARGETS = [
    ('native-tests', [
        'tests/native.c', 'tests/fakes/sdk.c', 'gpio_api.c', 'gpio_control.c',
        SHARED / 'dhcpserver/dhcp_options.c',
    ], []),
    ('dhcp-server-tests', [
        'tests/dhcp_server_test.c', SHARED / 'tests/fakes/network.c',
        SHARED / 'dhcpserver/dhcpserver.c', SHARED / 'dhcpserver/dhcp_options.c',
    ], []),
    ('captive-portal-tests', [
        'tests/captive_portal_test.c', SHARED / 'captive_portal.c',
    ], []),
    ('usb-tx-tests', ['tests/usb_tx_test.c'], []),
    ('network-recovery-tests', [
        'tests/network_status_test.c', SHARED / 'network_status.c',
    ], ['-DPICO_ON_DEVICE=1']),
    ('web-fs-tests', [
        'tests/web_fs_test.c', SHARED / 'tests/fakes/network.c', 'tests/fakes/sdk.c',
        SHARED / 'web_fs.c', 'web_routes.c', 'gpio_api.c', 'gpio_control.c', SHARED / 'captive_portal.c',
    ], ['-DPICO_ON_DEVICE=1', '-Wl,--wrap=malloc']),
]


def run():
    test_web_assets()
    run_ncm()
    with TemporaryDirectory() as temp:
        # The USB transmission test also compiles the production function verbatim.
        usb_source = (SHARED / 'usb_network.c').read_text()
        start = usb_source.index('static err_t tud_output(')
        end = usb_source.index('static err_t netif_init_cb', start)
        (Path(temp) / 'usb_tx_under_test.h').write_text(usb_source[start:end])
        for name, sources, flags in TARGETS:
            compile_and_run(name, sources, temp,
                            root=ROOT, include_dirs=[ROOT, SHARED, SHARED / 'tests/fakes', ROOT / 'tests/fakes', temp],
                            extra_flags=['-fno-omit-frame-pointer', *flags])
            if name == 'native-tests':
                subprocess.run([sys.executable, ROOT / 'tests/snapshot_contract.py',
                                str(Path(temp) / name)], cwd=ROOT, check=True)


if __name__ == '__main__':
    run()
