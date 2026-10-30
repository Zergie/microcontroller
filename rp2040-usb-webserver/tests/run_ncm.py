"""Exercise the actual SDK NCM callback; fail on missing iOS negotiation support."""
from pathlib import Path
from tempfile import TemporaryDirectory
from native_runner import ROOT, compile_and_run


def run():
    source = Path('/opt/pico-sdk/lib/tinyusb/src/class/net/ncm_device.c').read_text()
    start = source.index('bool netd_control_xfer_cb(')
    end = source.index('} // netd_control_xfer_cb', start) + len('} // netd_control_xfer_cb')
    fit_start = source.index('static bool xmit_requested_datagram_fits_into_current_ntb(')
    fit_end = source.index('} // xmit_requested_datagram_fits_into_current_ntb', fit_start) + len('} // xmit_requested_datagram_fits_into_current_ntb')
    with TemporaryDirectory() as temp:
        # Compile the installed driver's real functions, not a copied test implementation.
        (Path(temp) / 'ncm_control_under_test.h').write_text(source[fit_start:fit_end] + '\n' + source[start:end])
        compile_and_run('ncm-tests', [ROOT / 'tests/ncm_control_test.c'], temp, include_dirs=[temp])


if __name__ == '__main__':
    run()
