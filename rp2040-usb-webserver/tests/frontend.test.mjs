import { test, before, after } from 'node:test';
import assert from 'node:assert/strict';
import { fileURLToPath } from 'node:url';
import { chromium } from 'playwright';
import { createDevServer } from '../../shared/rp2040-usb-web/tools/dev-server.mjs';

let browser, app, base;
before(async () => {
  app = await createDevServer({ target: 'http://127.0.0.1:9', webRoot: fileURLToPath(new URL('../web/', import.meta.url)) });
  await new Promise(resolve => app.server.listen(0, '127.0.0.1', resolve));
  base = `http://127.0.0.1:${app.server.address().port}`;
  browser = await chromium.launch({ channel: process.env.PLAYWRIGHT_CHANNEL || (process.platform === 'win32' ? 'msedge' : undefined) });
});
after(async () => { await browser?.close(); await app?.close(); });

async function fixture(t) {
  const context = await browser.newContext({ viewport: { width: 375, height: 667 } });
  t.after(() => context.close());
  const page = await context.newPage();
  const state = {
    pins: Array.from({ length: 30 }, (_, pin) => ({ pin, available: ![23, 24, 29].includes(pin), reason: '',
      mode: pin === 25 ? 'output' : 'input', pull: 'none', value: pin === 25, frequency_hz: 0,
      actual_frequency_hz: 0, duty_percent: 0, drive_strength_ma: 4, slew_rate: 'slow', hysteresis: true })),
    posts: [], delay: 0, status: 200, onPost() {}
  };
  const errors = [];
  page.on('pageerror', error => errors.push(error.message));
  t.after(() => assert.deepEqual(errors, []));
  await page.route('**/api/**', async route => {
    if (route.request().method() === 'POST') {
      state.posts.push(new URL(route.request().url()).pathname);
      if (state.delay) await new Promise(resolve => setTimeout(resolve, state.delay));
      if (state.status !== 200) {
        await route.fulfill({ status: state.status, json: { error: 'Rejected' } }); return;
      }
      state.onPost();
    }
    await route.fulfill({ json: { pins: state.pins } });
  });
  await page.goto(base);
  await idle(page);
  return { page, state };
}

async function idle(page) {
  await page.waitForFunction(() => document.querySelector('.editor').getAttribute('aria-busy') === 'false' &&
    document.querySelector('#message').textContent === 'Connected');
}
async function refresh(page) { await page.locator('#refresh').click(); await idle(page); }
async function apply(page) { await page.locator('#apply-pin').click(); await idle(page); }

test('Pin selection, editor inputs, Apply and Refresh are wired to the session', async t => {
  const { page, state } = await fixture(t);
  state.pins[25].value = false;
  await refresh(page);
  assert.equal(await page.locator('#led-status').textContent(), 'OFF');
  assert.equal(await page.locator('#output-level').inputValue(), '0');
  await page.locator('[data-pin="0"]').click();
  await page.locator('#led-select').click();
  assert.equal(await page.locator('#output-level').inputValue(), '0');
  await apply(page);
  assert.deepEqual(state.posts, ['/api/gpio/25/off/drive/4/slow']);
});

test('Switching pins during a request acknowledges only the submitted pin', async t => {
  const { page, state } = await fixture(t);
  await page.locator('[data-pin="0"]').click();
  await page.locator('#pin-mode').selectOption('input-pullup');
  await page.locator('#led-select').click();
  state.delay = 250;
  await page.locator('#apply-pin').click();
  await page.locator('[data-pin="0"]').click();
  await idle(page);
  assert.equal(await page.locator('#pin-mode').inputValue(), 'input-pullup');
  assert.equal(state.posts.length, 1);
  assert.match(state.posts[0], /gpio\/25\//);
});

test('Apply keeps opacity, controls and layout stable while preventing duplicate submissions', async t => {
  const { page, state } = await fixture(t);
  state.delay = 350;
  const sample = () => page.evaluate(() => ['apply-pin', 'pin-mode', 'output-level', 'drive-strength', 'slew-rate', 'refresh'].map(id => {
    const el = document.getElementById(id), r = el.getBoundingClientRect();
    return { id, opacity: getComputedStyle(el).opacity, x: r.x, y: r.y, width: r.width, height: r.height };
  }));
  const before = await sample();
  await page.locator('#apply-pin').click();
  assert.equal(await page.locator('#apply-pin').isDisabled(), true);
  assert.deepEqual(await sample(), before);
  await page.locator('#pin-form').evaluate(form => form.dispatchEvent(new Event('submit', { cancelable: true })));
  await idle(page);
  assert.deepEqual(await sample(), before);
  assert.equal(state.posts.length, 1);
});

test('All modes share field visibility, explanations and command policy', async t => {
  const { page, state } = await fixture(t);
  assert.equal(await page.locator('html').getAttribute('lang'), 'en');
  assert.equal(await page.locator('#refresh').getAttribute('aria-label'), 'Refresh status');
  await page.locator('[data-pin="26"]').click();
  const cases = [
    ['input', 'input/hysteresis/on', ['Hysteresis']],
    ['input-pullup', 'input-pullup/hysteresis/on', ['Hysteresis']],
    ['input-pulldown', 'input-pulldown/hysteresis/on', ['Hysteresis']],
    ['input-buskeeper', 'input-buskeeper/hysteresis/on', ['Hysteresis']],
    ['analog', 'analog', ['Reading']],
    ['output', 'off/drive/4/slow', ['Level', 'Drive strength', 'Slew rate']],
    ['open-drain', 'open-drain/low/drive/4/slow', ['Level', 'Drive strength', 'Slew rate']],
    ['pwm', 'pwm/1000/50/drive/4/slow', ['Frequency', 'Duty Cycle', 'Drive strength', 'Slew rate']]
  ];
  for (const [mode, command, labels] of cases) {
    await page.locator('#pin-mode').selectOption(mode);
    const visible = await page.locator('#pin-form select:visible, #pin-form input:visible').count();
    assert.equal(visible, 1 + labels.length - (mode === 'analog' ? 1 : 0));
    await page.locator('#pin-info').click();
    assert.deepEqual(await page.locator('.legend-support').allTextContents(), Array(4).fill('not supported'));
    const explanations = await page.locator('#setting-explanations dt').allTextContents();
    assert.equal(explanations.length, labels.length);
    labels.forEach((label, i) => assert.ok(explanations[i].startsWith(label)));
    await page.locator('#close-info').click();
    await apply(page);
    assert.equal(state.posts.at(-1), `/api/gpio/26/${command}`);
    assert.equal(await page.locator('#apply-pin').textContent(), 'Apply');
  }
  await page.locator('[data-pin="0"]').click();
  assert.equal(await page.locator('#pin-mode option[value="analog"]').evaluate(el => el.disabled), true);
});

test('Board markings distinguish every applied mode and fit both edges at iPhone 8 width', async t => {
  const { page, state } = await fixture(t);
  const cases = [
    [0, { mode: 'input', value: true }, 'IN', 'H', 'Input'],
    [1, { mode: 'input', pull: 'up' }, 'IN↑', 'L', 'Pull-up'],
    [2, { mode: 'input', pull: 'down' }, 'IN↓', 'L', 'Pull-down'],
    [3, { mode: 'input', pull: 'keeper', value: true }, 'IN↔', 'H', 'Bus Keeper'],
    [4, { mode: 'output', value: true }, 'OUT', 'H', 'Output'],
    [5, { mode: 'output', value: false }, 'OUT', 'L', 'Output'],
    [6, { mode: 'pwm', duty_percent: 100, actual_frequency_hz: 1000 }, 'PWM', '100 %', 'PWM'],
    [7, { mode: 'open-drain', output_enabled: true }, 'OD', 'L', 'Open Drain'],
    [8, { mode: 'open-drain', output_enabled: false, value: true }, 'OD', 'Z · H', 'high impedance'],
    [26, { mode: 'analog', millivolts: 3300, adc_raw: 4095 }, 'ADC', '3.30V', 'Analog'],
    [28, { mode: 'input', pull: 'keeper', value: true }, 'IN↔', 'H', 'Bus Keeper']
  ];
  for (const [pin, values] of cases) Object.assign(state.pins[pin], values);
  await refresh(page);
  for (const [pin, _values, code, value, meaning] of cases) {
    const button = page.locator(`[data-pin="${pin}"]`);
    assert.equal(await button.locator('.pin-mode').textContent(), code);
    assert.equal(await button.locator('.pin-level').textContent(), value);
    assert.ok((await button.getAttribute('aria-label')).includes(meaning));
    assert.ok(await button.evaluate(el => {
      const label=el.querySelector('.pin-label').getBoundingClientRect();
      const state=el.querySelector('.pin-state').getBoundingClientRect();
      const bounds=el.getBoundingClientRect();
      return (label.right <= state.left || state.right <= label.left) && state.left >= bounds.left && state.right <= bounds.right;
    }), `Overlapping marking on GP${pin}`);
  }
  await page.locator('[data-pin="8"]').click();
  await page.locator('#pin-info').click();
  assert.match(await page.locator('#pin-marking').textContent(), /OD = Output – Open Drain.*high impedance.*HIGH/);
  await page.locator('#close-info').click();
  // Changing an editor draft must not pretend the hardware mode has changed.
  await page.locator('#pin-mode').selectOption('input');
  assert.equal(await page.locator('[data-pin="8"] .pin-mode').textContent(), 'OD');
  assert.equal(state.posts.length, 0);
});
