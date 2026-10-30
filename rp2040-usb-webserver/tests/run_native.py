"""Compile production modules against SDK fakes under ASan/UBSan."""
from pathlib import Path
import subprocess
import sys
from tempfile import TemporaryDirectory
from native_runner import ROOT, compile_and_run
from run_ncm import run as run_ncm
from web_assets_contract import run as test_web_assets

# Keep each suite's source dependencies and special flags explicit.
TARGETS = [
    ('native-tests', [
        'tests/native.c', 'tests/fakes/sdk.c', 'gpio_api.c', 'gpio_control.c',
        'dhcpserver/dhcp_options.c',
    ], []),
    ('dhcp-server-tests', [
        'tests/dhcp_server_test.c', 'tests/fakes/network.c',
        'dhcpserver/dhcpserver.c', 'dhcpserver/dhcp_options.c',
    ], []),
    ('captive-portal-tests', [
        'tests/captive_portal_test.c', 'captive_portal.c',
    ], []),
    ('usb-tx-tests', ['tests/usb_tx_test.c'], []),
    ('network-recovery-tests', [
        'tests/network_status_test.c', 'network_status.c',
    ], ['-DPICO_ON_DEVICE=1']),
    ('web-fs-tests', [
        'tests/web_fs_test.c', 'tests/fakes/network.c', 'tests/fakes/sdk.c',
        'web_fs.c', 'gpio_api.c', 'gpio_control.c', 'captive_portal.c',
    ], ['-DPICO_ON_DEVICE=1', '-Wl,--wrap=malloc']),
]


def run():
    test_web_assets()
    run_ncm()
    with TemporaryDirectory() as temp:
        # The USB transmission test also compiles the production function verbatim.
        usb_source = (ROOT / 'usb_network.c').read_text()
        start = usb_source.index('static err_t tud_output(')
        end = usb_source.index('static err_t netif_init_cb', start)
        (Path(temp) / 'usb_tx_under_test.h').write_text(usb_source[start:end])
        for name, sources, flags in TARGETS:
            compile_and_run(name, sources, temp,
                            include_dirs=[ROOT, ROOT / 'tests/fakes', temp],
                            extra_flags=['-fno-omit-frame-pointer', *flags])
            if name == 'native-tests':
                subprocess.run([sys.executable, ROOT / 'tests/snapshot_contract.py',
                                str(Path(temp) / name)], cwd=ROOT, check=True)


if __name__ == '__main__':
    run()
