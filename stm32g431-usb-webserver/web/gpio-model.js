import { boardPins } from './stm32-pins.js';

// Canonical frontend contract for modes, editable fields and API commands.
const descriptions = {
    input: 'Reads the digital level without driving the pin. Internal pull resistors are off; without an external signal, the level is undefined.',
    'input-pullup': 'Reads the digital level. An internal pull-up weakly pulls an unconnected input HIGH; a connected switch can pull it LOW.',
    'input-pulldown': 'Reads the digital level. An internal pull-down weakly pulls an unconnected input LOW; an external signal can drive it HIGH.',
    'input-buskeeper': 'Reads the digital level and weakly holds the last HIGH or LOW state using internal pulls. An external signal can change this state.',
    analog: 'Measures the ADC input voltage. Available on GP26–GP28; the digital input buffer and internal pull resistors are disabled in this mode.',
    output: 'Actively drives the pin LOW or HIGH, approximately 0 V or 3.3 V on the STM32.',
    'open-drain': 'Drives the pin LOW or releases it to high impedance. An external pull-up produces HIGH; the pin never actively drives HIGH in this mode.',
    pwm: 'Generates a periodic digital signal in hardware. Frequency and duty cycle set the repetition rate and HIGH fraction; this is not an analog output voltage.'
  };
function hz(value) {
  return value >= 1000 ? `${Number((value / 1000).toFixed(2))} kHz` : `${value} Hz`;
}

const inputFields = [];
const outputFields = ['level', 'slew'];
export const modes = {
  input: { label: 'Input', badge: 'IN', fields: inputFields, pull: 'none', command: () => 'input' },
  'input-pullup': { label: 'Input – Pull-up', badge: 'IN↑', fields: inputFields, pull: 'up', command: () => 'input-pullup' },
  'input-pulldown': { label: 'Input – Pull-down', badge: 'IN↓', fields: inputFields, pull: 'down', command: () => 'input-pulldown' },
  'input-buskeeper': { label: 'Input – Bus Keeper', badge: 'IN↔', fields: inputFields, pull: 'keeper', command: () => 'input-buskeeper' },
  analog: { label: 'Input – Analog', badge: 'ADC',
    status: pin => `${(pin.millivolts / 1000).toFixed(2)} V · ADC ${pin.adc_raw}`, readout: pin => `${(pin.millivolts / 1000).toFixed(2)}V`,
    fields: ['analog'], supports: pin => boardPins.gpio(pin)?.adc != null, command: () => 'analog' },
  output: { label: 'Output', badge: 'OUT', status: pin => pin.value ? 'ON · HIGH' : 'OFF · LOW', fields: outputFields, command: values => values.level === '1' ? 'on' : 'off' },
  'open-drain': { label: 'Output – Open Drain', badge: 'OD',
    status: pin => pin.output_enabled ? 'Open Drain · LOW' : 'Open Drain · high impedance',
    ledStatus: pin => pin.output_enabled ? 'LOW' : 'Hi-Z',
    readout: pin => pin.output_enabled ? 'L' : `Z · ${pin.value ? 'H' : 'L'}`, fields: outputFields,
    levels: ['LOW · active', 'Release (Hi-Z)'], command: values => values.level === '0' ? 'open-drain/low' : 'open-drain/release' },
  pwm: { label: 'Output – PWM', badge: 'PWM',
    status: pin => `${hz(pin.actual_frequency_hz)} · ${pin.duty_percent} %`,
    ledStatus: pin => `${pin.duty_percent} %`, readout: pin => `${pin.duty_percent} %`, fields: ['frequency', 'duty', 'slew'],
    command: values => `pwm/${Number(values.frequency)}/${Number(values.duty)}` }
};
delete modes['input-buskeeper'];
 delete modes.analog;
 modes.pwm.supports = pin => boardPins.gpio(pin)?.pwm != null;
 for (const [name, mode] of Object.entries(modes)) {
  mode.description = descriptions[name];
  mode.status ||= pin => `${mode.label} · ${pin.value ? 'HIGH' : 'LOW'}`;
  mode.ledStatus ||= pin => mode.pull !== undefined ? 'Input' : pin.value ? 'ON' : 'OFF';
  mode.supports ||= () => true;
  mode.levels ||= ['LOW · Off', 'HIGH · On'];
}

function modeKey(pin) {
  return pin ? (pin.mode === 'input' ? Object.keys(modes).find(key => modes[key].pull === pin.pull) : pin.mode) : '';
}

export function pinIndicator(pin) {
  const mode = modes[modeKey(pin)];
  if (!mode) return { code: '?', value: '—', legend: 'Unknown mode' };
  const value = mode.readout ? mode.readout(pin) : pin.value ? 'H' : 'L';
  const meanings = { H: 'HIGH', L: 'LOW', 'Z · H': 'released to high impedance, level HIGH', 'Z · L': 'released to high impedance, level LOW' };
  const reading = meanings[value] ? `${value} = ${meanings[value]}` : value;
  const high = mode === modes.output && Boolean(pin.value);
  const pwm = mode === modes.pwm;
  return { code: mode.badge, value, legend: `${mode.badge} = ${mode.label} · ${reading}`,
    status: mode.status(pin), ledStatus: mode.ledStatus(pin), high, pwm,
    lit: pwm ? pin.duty_percent > 0 : high };
}

export function editorFromPin(pin) {
  const mode = modeKey(pin);
  return { mode: mode || '', level: pin && (pin.mode === 'open-drain' ? !pin.output_enabled : pin.mode === 'output' && pin.value) ? '1' : '0',
    frequency: String(pin && pin.mode === 'pwm' ? pin.frequency_hz : 1000),
    duty: String(pin && pin.mode === 'pwm' ? pin.duty_percent : 50),
    drive: String(pin ? pin.drive_strength_ma : 4), slew: pin ? pin.slew_rate : 'slow',
    hysteresis: pin && pin.hysteresis === false ? 'off' : 'on' };
}

export function commandFor(pin, values) {
  const mode = modes[values.mode];
  if (!mode || !mode.supports(pin)) throw new Error('Mode is not available for this pin.');
  let command = mode.command(values);
  if (mode.fields.includes('drive')) command += `/drive/${values.drive}/${values.slew}`;
  if (mode.fields.includes('slew')) command += '/slew/' + values.slew;
  if (mode.fields.includes('hysteresis')) command += `/hysteresis/${values.hysteresis}`;
  return `/api/gpio/${pin}/${command}`;
}

export function analogText(pin) {
  return pin && pin.mode === 'analog' ? `${(pin.millivolts / 1000).toFixed(3)} V · ${pin.adc_raw} / 4095` : '— V';
}

export const fields = {
  level: { label: 'Level', value: (values, mode) => mode.levels[Number(values.level)],
    explain: mode => mode === modes['open-drain'] ?
      'LOW enables the output driver. Release (Hi-Z) disables it; the actual level depends on the external circuit.' :
      'LOW drives the output to approximately 0 V; HIGH drives it to approximately 3.3 V.' },
  frequency: { label: 'Frequency', value: values => `${values.frequency || '—'} Hz`,
    explain: () => 'Number of complete PWM periods per second. Range: 10–100,000 Hz. Channels of a timer share this frequency; conflicting assignments are rejected.' },
  duty: { label: 'Duty Cycle', value: values => `${values.duty || '—'} %`,
    explain: () => 'HIGH fraction of each period. 0% means always LOW, 50% means equal HIGH and LOW times, and 100% means always HIGH.' },
  drive: { label: 'Drive strength', value: values => `${values.drive} mA`,
    explain: () => 'Nominal output drive strength: 2, 4, 8 or 12 mA. This affects load-driving capability; it is not a regulated current source or current limit.' },
  slew: { label: 'Slew rate', value: values => values.slew === 'fast' ? 'Fast' : 'Slow',
    explain: mode => 'Slow limits the signal edge rate; Fast switches more sharply. This does not change the PWM frequency.' +
      (mode === modes['open-drain'] ? ' In Open Drain mode, this affects the actively driven falling edge; the rising edge depends on the external pull-up and load.' : '') },
  hysteresis: { label: 'Hysteresis (Schmitt trigger)', value: values => values.hysteresis === 'on' ? 'On' : 'Off',
    explain: () => 'On uses different thresholds for rising and falling levels, improving detection of slow or noisy signals. Off disables this hysteresis. This does not debounce switches.' },
  analog: { label: 'Reading', value: (_values, _mode, pin) => analogText(pin),
    explain: () => '12-bit ADC: raw value 0–4095. The displayed voltage assumes a 3.3 V reference and is not calibrated. Nominal input range: 0–3.3 V. New values are read on status requests or Refresh.' }
};
