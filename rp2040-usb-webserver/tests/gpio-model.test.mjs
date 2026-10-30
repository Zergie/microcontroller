import { test } from 'node:test';
import assert from 'node:assert/strict';
import { editorFromPin, commandFor, pinIndicator, analogText } from '../web/gpio-model.js';

const pin = overrides => ({ pin: 26, mode: 'input', pull: 'none', value: false, frequency_hz: 1000,
  actual_frequency_hz: 997, duty_percent: 50, drive_strength_ma: 4, slew_rate: 'slow', hysteresis: true, ...overrides });
const cases = [
  [{}, 'input', 'IN', 'L', 'input/hysteresis/on'],
  [{ pull: 'up', value: true }, 'input-pullup', 'IN↑', 'H', 'input-pullup/hysteresis/on'],
  [{ pull: 'down' }, 'input-pulldown', 'IN↓', 'L', 'input-pulldown/hysteresis/on'],
  [{ pull: 'keeper', value: true }, 'input-buskeeper', 'IN↔', 'H', 'input-buskeeper/hysteresis/on'],
  [{ mode: 'output', value: true }, 'output', 'OUT', 'H', 'on/drive/4/slow'],
  [{ mode: 'output', value: false }, 'output', 'OUT', 'L', 'off/drive/4/slow'],
  [{ mode: 'pwm', duty_percent: 25 }, 'pwm', 'PWM', '25 %', 'pwm/1000/25/drive/4/slow'],
  [{ mode: 'open-drain', output_enabled: true }, 'open-drain', 'OD', 'L', 'open-drain/low/drive/4/slow'],
  [{ mode: 'open-drain', output_enabled: false, value: true }, 'open-drain', 'OD', 'Z · H', 'open-drain/release/drive/4/slow'],
  [{ mode: 'open-drain', output_enabled: false, value: false }, 'open-drain', 'OD', 'Z · L', 'open-drain/release/drive/4/slow'],
  [{ mode: 'analog', value: null, millivolts: 1650, adc_raw: 2048 }, 'analog', 'ADC', '1.65V', 'analog']
];
for (const [overrides, mode, code, value, command] of cases) {
  test(`Applied ${mode}/${value} converts consistently to editor, indicator and command`, () => {
    const state = pin(overrides), values = editorFromPin(state), indicator = pinIndicator(state);
    assert.equal(values.mode, mode);
    assert.equal(indicator.code, code);
    assert.equal(indicator.value, value);
    assert.ok(indicator.legend.startsWith(`${code} = `));
    assert.equal(commandFor(26, values), `/api/gpio/26/${command}`);
  });
}

test('Defaults, pad suffixes and analog pin boundaries are explicit', () => {
  assert.deepEqual(editorFromPin(null), { mode: '', level: '0', frequency: '1000', duty: '50', drive: '4', slew: 'slow', hysteresis: 'on' });
  assert.equal(commandFor(0, { ...editorFromPin(pin()), hysteresis: 'off' }), '/api/gpio/0/input/hysteresis/off');
  assert.equal(commandFor(25, { ...editorFromPin(pin({mode:'output'})), drive:'12', slew:'fast' }), '/api/gpio/25/off/drive/12/fast');
  assert.throws(() => commandFor(25, { mode: 'analog' }), /not available/);
  assert.throws(() => commandFor(26, { mode: 'missing' }), /not available/);
  assert.equal(analogText(pin({mode:'analog',millivolts:1650,adc_raw:2048})), '1.650 V · 2048 / 4095');
  assert.equal(analogText(pin()), '— V');
  assert.deepEqual(pinIndicator(pin({mode:'missing'})), {code:'?',value:'—',legend:'Unknown mode'});
});


test('One applied presentation supplies accessibility, LED summary and highlight state', () => {
  const presentations = [
    [{}, 'Input · LOW', 'Input', false, false, false],
    [{pull:'up',value:true}, 'Input – Pull-up · HIGH', 'Input', false, false, false],
    [{pull:'down'}, 'Input – Pull-down · LOW', 'Input', false, false, false],
    [{pull:'keeper',value:true}, 'Input – Bus Keeper · HIGH', 'Input', false, false, false],
    [{mode:'output',value:true}, 'ON · HIGH', 'ON', true, false, true],
    [{mode:'output'}, 'OFF · LOW', 'OFF', false, false, false],
    [{mode:'pwm',duty_percent:25}, '997 Hz · 25 %', '25 %', false, true, true],
    [{mode:'pwm',duty_percent:0,actual_frequency_hz:2000}, '2 kHz · 0 %', '0 %', false, true, false],
    [{mode:'open-drain',output_enabled:true}, 'Open Drain · LOW', 'LOW', false, false, false],
    [{mode:'open-drain',output_enabled:false,value:true}, 'Open Drain · high impedance', 'Hi-Z', false, false, false],
    [{mode:'analog',value:null,millivolts:1650,adc_raw:2048}, '1.65 V · ADC 2048', 'OFF', false, false, false]
  ];
  for (const [overrides,status,ledStatus,high,pwm,lit] of presentations) {
    const result=pinIndicator(pin(overrides));
    assert.deepEqual({status:result.status,ledStatus:result.ledStatus,high:result.high,pwm:result.pwm,lit:result.lit}, {status,ledStatus,high,pwm,lit});
  }
});
