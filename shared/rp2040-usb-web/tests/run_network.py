"""Shared network regressions; no dependency on either application tree."""
from pathlib import Path
import sys
from tempfile import TemporaryDirectory

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / 'tools'))
from native_runner import compile_and_run
from run_ncm import run as run_ncm


def run():
    run_ncm()
    with TemporaryDirectory() as temp:
        # Compile the actual backpressure function without pulling in the whole SDK.
        source = (ROOT / 'usb_network.c').read_text(encoding='utf-8')
        start = source.index('static err_t tud_output(')
        end = source.index('static err_t netif_init_cb', start)
        (Path(temp) / 'usb_tx_under_test.h').write_text(source[start:end], encoding='utf-8')
        targets = [
            ('dhcp-server-tests', ['tests/dhcp_server_test.c', 'tests/fakes/network.c',
                                  'dhcpserver/dhcpserver.c', 'dhcpserver/dhcp_options.c'], []),
            ('captive-portal-tests', ['tests/captive_portal_test.c', 'captive_portal.c'], []),
            ('usb-tx-tests', ['tests/usb_tx_test.c'], []),
            ('network-recovery-tests', ['tests/network_status_test.c', 'network_status.c'],
             ['-DPICO_ON_DEVICE=1']),
        ]
        for name, sources, flags in targets:
            compile_and_run(name, sources, temp, root=ROOT,
                            include_dirs=[ROOT, ROOT / 'tests/fakes', temp], extra_flags=flags)


if __name__ == '__main__':
    run()
