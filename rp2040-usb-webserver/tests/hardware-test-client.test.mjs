import { test } from 'node:test';
import assert from 'node:assert/strict';
import http from 'node:http';
import { spawn } from 'node:child_process';
import { fileURLToPath } from 'node:url';

const root = fileURLToPath(new URL('../', import.meta.url));

function configuration(pin) {
  const fields = ['mode', 'pull', 'frequency_hz', 'duty_percent',
    'drive_strength_ma', 'slew_rate', 'hysteresis'];
  if (pin.mode === 'output') fields.push('value');
  if (pin.mode === 'open-drain') fields.push('output_enabled', 'output_latch');
  return Object.fromEntries(fields.map(field => [field, pin[field]]));
}

// Independent test protocol model; no firmware/production command builder is reused.
function applyCommand(pins, path, body) {
  const legacy = /^\/api\/led\/(on|off)$/.exec(path);
  const match = /^\/api\/gpio\/([^/]+)\/(.+)$/.exec(path);
  const number = legacy ? 25 : Number(match?.[1]);
  const action = legacy ? legacy[1] : match?.[2];
  if (body || !Number.isInteger(number) || number < 0 || number > 29) return 400;
  const pin = pins[number];
  if (!pin.available) return 403;

  const parts = action.split('/');
  const pad = parts.indexOf('drive');
  const hysteresis = parts.indexOf('hysteresis');
  const base = parts.slice(0, pad >= 0 ? pad : hysteresis >= 0 ? hysteresis : parts.length).join('/');
  const input = /^input(?:-(pullup|pulldown|buskeeper))?$/.exec(base);
  const pwm = /^pwm\/(\d+)\/(\d+)$/.exec(base);
  const output = ['on', 'off', 'open-drain/low', 'open-drain/release'].includes(base) || pwm;
  if (pad >= 0 && (!output || ![2, 4, 8, 12].includes(Number(parts[pad + 1])) ||
      !['slow', 'fast'].includes(parts[pad + 2]) || parts.length !== pad + 3)) return 400;
  if (hysteresis >= 0 && (!input || !['on', 'off'].includes(parts[hysteresis + 1]) ||
      parts.length !== hysteresis + 2)) return 400;
  if (pwm && (+pwm[1] < 10 || +pwm[1] > 100000 || +pwm[2] > 100)) return 400;
  if (base.startsWith('pwm/') && !pwm) return 400;
  if (base === 'analog' && (number < 26 || number > 28)) return 400;
  if (pwm && pins.some(other => other.pin !== number && other.mode === 'pwm' && other.slice === pin.slice &&
      (other.pin % 2 === number % 2 || other.frequency_hz !== +pwm[1]))) return 409;
  if (!input && !output && base !== 'analog') return 404;

  Object.assign(pin, { pull: 'none', frequency_hz: 0, actual_frequency_hz: 0, duty_percent: 0 });
  if (input) {
    const pull = { pullup: 'up', pulldown: 'down', buskeeper: 'keeper' }[input[1]] || 'none';
    Object.assign(pin, { mode: 'input', pull, value: input[1] === 'pullup' });
  } else if (pwm) {
    Object.assign(pin, { mode: 'pwm', frequency_hz: +pwm[1], actual_frequency_hz: +pwm[1],
      duty_percent: +pwm[2], value: false });
  } else if (base === 'analog') {
    Object.assign(pin, { mode: 'analog', value: null, adc_raw: 2048, millivolts: 1650 });
  } else if (base.startsWith('open-drain/')) {
    Object.assign(pin, { mode: 'open-drain', output_enabled: base.endsWith('/low'),
      output_latch: false, value: false });
  } else {
    Object.assign(pin, { mode: 'output', value: base === 'on' });
  }
  if (pad >= 0) Object.assign(pin, { drive_strength_ma: +parts[pad + 1], slew_rate: parts[pad + 2] });
  if (hysteresis >= 0) pin.hysteresis = parts[hysteresis + 1] === 'on';
  return 200;
}

function runPowerShell(args) {
  return new Promise((resolve, reject) => {
    const child = spawn('pwsh', args, { cwd: root, env: { ...process.env,
      HTTP_PROXY: 'http://127.0.0.1:1', HTTPS_PROXY: 'http://127.0.0.1:1', NO_PROXY: '' } });
    let output = '';
    let timedOut = false;
    const timeout = setTimeout(() => { timedOut = true; child.kill('SIGKILL'); }, 20000);
    child.stdout.on('data', chunk => { output += chunk; });
    child.stderr.on('data', chunk => { output += chunk; });
    child.on('error', error => { clearTimeout(timeout); reject(error); });
    child.on('close', code => {
      clearTimeout(timeout);
      if (timedOut) reject(new Error(`PowerShell exceeded 20 seconds:\n${output}`));
      else resolve({ code, output });
    });
  });
}

// Stateful loopback fixture exercises real PowerShell scripts, never an attached board.
async function exercise(script, original = {}, failures = [], external = {}) {
  const pins = Array.from({ length: 30 }, (_, pin) => ({
    pin, available: ![23, 24, 29].includes(pin), mode: 'input', pull: 'none', value: false,
    frequency_hz: 0, actual_frequency_hz: 0, duty_percent: 0, slice: Math.floor(pin % 16 / 2),
    drive_strength_ma: 4, slew_rate: 'slow', hysteresis: true,
    ...(pin === 25 ? original : external[pin] || {})
  }));
  const before = structuredClone(pins);
  const requests = [];
  const server = http.createServer(async (req, res) => {
    let body = '';
    for await (const chunk of req) body += chunk;
    const path = req.url;
    requests.push({ path, method: req.method, body });
    let status = 200;
    let result = { pins };
    const failure = failures.find(candidate => !candidate.used && candidate.path === path);
    if (failure) {
      failure.used = true;
      status = 500;
      result = { error: failure.message };
    } else if (path === '/') {
      res.setHeader('Content-Type', 'text/html; charset=utf-8');
      res.end('RP2040 USB Webserver');
      return;
    } else if (req.method === 'GET' && path === '/api/gpio.json') {
      result = { pins };
    } else if (req.method === 'GET' && path === '/api/led.json') {
      result = { on: pins[25].value };
    } else if (req.method === 'POST') {
      status = applyCommand(pins, path, body);
      if (path.startsWith('/api/led/')) result = { on: pins[25].value };
    } else {
      status = 404;
    }
    res.writeHead(status, { 'Content-Type': 'application/json' });
    res.end(JSON.stringify(result));
  });
  await new Promise(resolve => server.listen(0, '127.0.0.1', resolve));
  try {
    const args = ['-NoProfile', '-File', script, '-BaseUri', `http://127.0.0.1:${server.address().port}`];
    if (script === 'Test-Device.ps1') args.push('-TestLed', '-WaitSeconds', '1');
    return { ...await runPowerShell(args), pins, before, requests };
  } finally {
    server.closeAllConnections();
    await new Promise(resolve => server.close(resolve));
  }
}

test('Device LED assertions restore the complete original GP25 configuration', async () => {
  const run = await exercise('Test-Device.ps1', { mode: 'input', pull: 'keeper',
    hysteresis: false, drive_strength_ma: 12, slew_rate: 'fast' });
  assert.equal(run.code, 0, run.output);
  assert.deepEqual(configuration(run.pins[25]), configuration(run.before[25]));
  assert.ok(run.requests.some(request => request.path === '/api/led/on'));
  assert.ok(run.requests.some(request => request.path === '/api/led/off'));
});

const originals = [
  { mode: 'input', pull: 'none' }, { mode: 'input', pull: 'up' },
  { mode: 'input', pull: 'down' }, { mode: 'input', pull: 'keeper' },
  { mode: 'output', value: true },
  { mode: 'pwm', frequency_hz: 2345, actual_frequency_hz: 2345, duty_percent: 37 },
  { mode: 'open-drain', output_enabled: true, output_latch: false },
  { mode: 'open-drain', output_enabled: false, output_latch: false }
];
for (const script of ['Test-Device.ps1', 'Test-Gpio.ps1', 'Test-InputModes.ps1', 'Test-PadSettings.ps1']) {
  test(`${script} preserves modes and both visible/hidden pad categories without a proxy`, async () => {
    for (const original of originals) {
      const run = await exercise(script, { ...original, drive_strength_ma: 12, slew_rate: 'fast', hysteresis: false });
      assert.equal(run.code, 0, `${JSON.stringify(original)}\n${run.output}`);
      for (const number of [25, 26, 27, 28]) {
        assert.deepEqual(configuration(run.pins[number]), configuration(run.before[number]));
      }
      if (script === 'Test-InputModes.ps1') {
        for (const request of run.requests.filter(request => /^\/api\/gpio\/2[678]\//.test(request.path))) {
          assert.match(request.path, /\/(input(?:\/hysteresis\/(?:on|off))?|analog)$/,
            'external ADC pins must not be test outputs');
        }
      }
    }
  });
}

test('A primary command failure survives restoration failures and remaining pins are restored', async () => {
  const run = await exercise('Test-InputModes.ps1', { hysteresis: false }, [
    { path: '/api/gpio/25/on', message: 'PRIMARY COMMAND' },
    { path: '/api/gpio/25/input/hysteresis/off', message: 'CLEANUP FAILURE' }
  ]);
  assert.notEqual(run.code, 0);
  assert.match(run.output, /PRIMARY COMMAND/);
  assert.match(run.output, /CLEANUP FAILURE/);
  for (const number of [26, 27, 28]) {
    assert.ok(run.requests.some(request => request.path === `/api/gpio/${number}/input/hysteresis/on`));
  }
});

test('Input mode tests restore original analog and driven external configurations', async () => {
  const run = await exercise('Test-InputModes.ps1', {}, [], {
    26: { mode: 'analog', value: null, adc_raw: 2048, millivolts: 1650 },
    27: { mode: 'output', value: true, drive_strength_ma: 8, slew_rate: 'fast' }
  });
  assert.equal(run.code, 0, run.output);
  for (const number of [26, 27, 28]) {
    assert.deepEqual(configuration(run.pins[number]), configuration(run.before[number]));
  }
  const externalWrites = run.requests.filter(request => /^\/api\/gpio\/2[678]\//.test(request.path));
  assert.deepEqual(externalWrites.filter(request => /^\/api\/gpio\/\d+\/(on|off|pwm)\b/.test(request.path)),
    [{ path: '/api/gpio/27/on', method: 'POST', body: '' }]);
});

test('A cleanup-only failure fails the test and still attempts every remaining pin', async () => {
  const run = await exercise('Test-InputModes.ps1', {}, [
    { path: '/api/gpio/25/input/hysteresis/on', message: 'CLEANUP ONLY' }
  ]);
  assert.notEqual(run.code, 0);
  assert.match(run.output, /CLEANUP ONLY/);
  for (const number of [26, 27, 28]) {
    assert.ok(run.requests.some(request => request.path === `/api/gpio/${number}/input/hysteresis/on`));
  }
});
