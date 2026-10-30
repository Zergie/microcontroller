"""Parse actual native GPIO output; schema expectations are independent of C emission."""
import json
import subprocess
import sys

raw = subprocess.check_output([sys.argv[1], '--snapshot-contract'], text=True)
snapshot = json.loads(raw)
assert list(snapshot) == ['pins']
assert len(snapshot['pins']) == 30
fields = ['pin', 'available', 'reason', 'mode', 'pull', 'value', 'frequency_hz',
          'actual_frequency_hz', 'duty_percent', 'slice', 'channel',
          'drive_strength_ma', 'slew_rate', 'hysteresis']
for number, pin in enumerate(snapshot['pins']):
    extra = ['adc_raw', 'millivolts'] if number == 26 else ['output_enabled', 'output_latch'] if number in (7, 8) else []
    assert list(pin) == fields + extra
    assert pin['pin'] == number and pin['slice'] == number % 16 // 2
    assert pin['channel'] == ('B' if number % 2 else 'A')
    assert pin['available'] is (number not in (23, 24, 29))
    assert pin['reason'] == {23: 'SMPS', 24: 'VBUS', 29: 'VSYS'}.get(number, '')
    assert type(pin['hysteresis']) is bool
    assert type(pin['value']) is bool or number == 26 and pin['value'] is None
    assert type(pin['frequency_hz']) is int and type(pin['duty_percent']) is int
    assert pin['mode'] == {4: 'output', 5: 'output', 6: 'pwm', 7: 'open-drain', 8: 'open-drain', 25: 'output', 26: 'analog'}.get(number, 'input')
    assert pin['pull'] == {1: 'up', 2: 'down', 3: 'keeper'}.get(number, 'none')
    assert pin['drive_strength_ma'] == {4: 12, 5: 2, 6: 8}.get(number, 4)
    assert pin['slew_rate'] == ('fast' if number in (4, 6) else 'slow')
    assert pin['hysteresis'] is (number != 0)
    assert pin['frequency_hz'] == (2345 if number == 6 else 0)
    assert pin['duty_percent'] == (37 if number == 6 else 0)
    if number != 6:
        assert pin['actual_frequency_hz'] == 0
assert abs(snapshot['pins'][6]['actual_frequency_hz'] - 2345) <= 1
assert snapshot['pins'][4]['value'] is True and snapshot['pins'][5]['value'] is False
assert snapshot['pins'][7]['output_enabled'] is True
assert snapshot['pins'][8]['output_enabled'] is False
assert snapshot['pins'][7]['output_latch'] is snapshot['pins'][8]['output_latch'] is False
assert snapshot['pins'][26]['adc_raw'] == 2048 and snapshot['pins'][26]['millivolts'] == (2048 * 3300 + 2048) // 4096
assert raw == json.dumps(snapshot, separators=(',', ':')) + '\n'
print('GPIO JSON schema: actual bytes, order, types, reserved pins, modes, pads, PWM, OD and ADC OK')
