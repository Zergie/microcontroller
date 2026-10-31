import { describe, test, before, after } from 'node:test';
import assert from 'node:assert/strict';
import { fileURLToPath } from 'node:url';
import { chromium } from 'playwright';
import { createDevServer } from '../../shared/rp2040-usb-web/tools/dev-server.mjs';

describe('Animation controller', { concurrency: 4 }, () => {
  let browser, app, base;
  before(async () => {
    app = await createDevServer({ target: 'http://127.0.0.1:9', webRoot: fileURLToPath(new URL('../web/', import.meta.url)) });
    await new Promise(resolve => app.server.listen(0, '127.0.0.1', resolve));
    base = `http://127.0.0.1:${app.server.address().port}`;
    browser = await chromium.launch({ channel: process.platform === 'win32' ? 'msedge' : undefined });
  });
  after(async () => { await browser?.close(); await app?.close(); });

  async function fixture(t, { offline = false, originalColors = false, legacy = false } = {}) {
    const context = await browser.newContext({ viewport: { width: 375, height: 667 } });
    t.after(() => context.close());
    const page = await context.newPage();
    // Exercise picker dispatch without opening a platform-native modal in tests.
    await page.addInitScript(() => {
      window.colorPickerOpens = 0;
      HTMLInputElement.prototype.showPicker = function () {
        if (this.type === 'color') window.colorPickerOpens++;
      };
    });
    const state = { boot_id: '0000000000000001', selection: 'random', current: 'rainbow', brightness_percent: 20,
      speed_percent: 100, interval_seconds: 300, color: originalColors ? '#00ff28' : '#00ff64', pixel_count: 10, pin: 0,
      original_colors: originalColors, default_colors: ['#00ff28','#ff0014','#b400ff','#ffffff','#ff6400',
        '#b400ff','#00ff28','#000000','#00ff28','#000000','#000000','#000000','#000000'],
      effects: ['blink', 'color-cycle', 'comet', 'chase', 'pulse', 'sparkle', 'solid', 'rainbow',
        'sparkle-pulse', 'rainbow-comet', 'rainbow-chase', 'rainbow-sparkle', 'custom-color-chase'] };
    if (!legacy) state.color_mode = originalColors ? 'original' : 'custom';
    const control = { status: 200, delay: 0, posts: [], state, offline, disconnect: false };
    const errors = [];
    page.on('pageerror', error => errors.push(error.message));
    t.after(() => assert.deepEqual(errors, []));
    await page.route('**/api/**', async route => {
      if (control.disconnect) return route.abort('connectionfailed');
      if (control.offline) return route.fulfill({ status: 502, json: { error: 'Unreachable' } });
      if (route.request().method() === 'POST') {
        const path = new URL(route.request().url()).pathname;
        control.posts.push(path);
        if (control.holdPost) await control.holdPost;
        if (control.delay) await new Promise(resolve => setTimeout(resolve, control.delay));
        if (control.status !== 200) return route.fulfill({ status: control.status, json: { error: 'Rejected' } });
        const [, , , , selection, brightness, speed, interval, color] = path.split('/');
        Object.assign(state, { selection, brightness_percent: Number(brightness), speed_percent: Number(speed),
          interval_seconds: Number(interval), color: `#${color}`,
          color_mode: path.endsWith('/original') ? 'original' : path.endsWith('/random') ? 'random' : 'custom',
          original_colors: path.endsWith('/original'), current: selection === 'random' ? 'comet' : selection });
      }
      await route.fulfill({ json: state });
    });
    await page.goto(base);
    if (!offline) await page.waitForFunction(() => !document.getElementById('animation').disabled);
    return { page, ...control, control };
  }

  test('Mobile page offers all effects and random loop without horizontal overflow', async t => {
    const { page } = await fixture(t);
    assert.equal(await page.locator('header').count(), 0);
    assert.equal(await page.locator('main > :first-child').getAttribute('class'), 'now-playing');
    assert.equal(await page.locator('#animation option').count(), 14);
    assert.equal(await page.locator('#animation').inputValue(), 'random');
    assert.equal(await page.locator('#current').textContent(), 'Rainbow');
    assert.equal(await page.evaluate(() => document.documentElement.scrollWidth <= innerWidth), true);
  });

  test('A changed board boot reloads the page once and keeps fetching fresh assets', async t => {
    const { page, control } = await fixture(t);
    const navigation = page.waitForEvent('load');
    control.state.boot_id = '0000000000000002';
    // Returning from iOS background must check immediately, without a missed-outage dependency.
    await page.evaluate(() => document.dispatchEvent(new Event('visibilitychange')));
    await navigation;
    await page.waitForFunction(() => document.getElementById('settings').dataset.online === 'true');
    let reloads = 0;
    page.on('load', () => reloads++);
    const reply = page.waitForResponse(r => r.url().endsWith('/api/animation.json'));
    await page.evaluate(() => window.dispatchEvent(new PageTransitionEvent('pageshow', { persisted: true })));
    await reply;
    assert.equal(reloads, 0);
    assert.equal(control.posts.length, 0);
    const response = await page.request.get(`${base}/app.js`);
    assert.match(response.headers()['cache-control'], /no-store/);
  });

  test('Reconnection after unplug/flash reloads; transient failures with the same boot do not', async t => {
    const { page, control } = await fixture(t);
    let reloads = 0;
    page.on('load', () => reloads++);
    control.disconnect = true;
    await page.evaluate(() => document.dispatchEvent(new Event('visibilitychange')));
    await page.waitForFunction(() => document.getElementById('animation').disabled);
    control.disconnect = false;
    await page.evaluate(() => document.dispatchEvent(new Event('visibilitychange')));
    await page.waitForFunction(() => !document.getElementById('animation').disabled);
    assert.equal(reloads, 0);
    control.disconnect = true;
    await page.evaluate(() => document.dispatchEvent(new Event('visibilitychange')));
    await page.waitForFunction(() => document.getElementById('animation').disabled);
    control.state.boot_id = '0000000000000003';
    control.disconnect = false;
    const navigation = page.waitForEvent('load');
    await page.evaluate(() => document.dispatchEvent(new Event('visibilitychange')));
    await navigation;
    await page.waitForFunction(() => !document.getElementById('animation').disabled);
    assert.equal(reloads, 1);
  });

  test('Selections apply immediately without a button or decorative pixels', async t => {
    const { page, control } = await fixture(t);
    assert.equal(await page.locator('#settings button, .pixels').count(), 0);
    await page.selectOption('#animation', 'solid');
    await page.waitForFunction(() => document.getElementById('current').textContent === 'Solid');
    assert.deepEqual(control.posts, ['/api/animation/set/solid/20/100/300/00ff64']);
    assert.equal(await page.locator('#interval-field').isVisible(), false);
  });

  test('Continuous edits are coalesced and send the latest complete settings', async t => {
    const { page, control } = await fixture(t);
    await page.evaluate(() => {
      for (const [id, values] of [['brightness', ['30', '35', '40']], ['speed', ['125', '150']], ['color', ['#ff6400']]]) {
        for (const value of values) {
          const input = document.getElementById(id);
          input.value = value; input.dispatchEvent(new Event('input', { bubbles: true }));
        }
      }
    });
    await page.waitForFunction(() => document.getElementById('brightness-value').value === '40%');
    await page.waitForResponse(response => response.request().method() === 'POST');
    assert.deepEqual(control.posts, ['/api/animation/set/random/40/150/300/ff6400']);
  });

  test('Random loop interval is applied automatically', async t => {
    const { page, control } = await fixture(t);
    const applied = page.waitForResponse(response => response.request().method() === 'POST');
    await page.fill('#interval', '15');
    await applied;
    assert.equal(await page.locator('#animation').inputValue(), 'random');
    assert.deepEqual(control.posts, ['/api/animation/set/random/20/100/15/00ff64']);
  });

  test('New edits during a pending request survive its response and are sent next', async t => {
    const { page, control } = await fixture(t);
    let release;
    control.holdPost = new Promise(resolve => { release = resolve; });
    const pending = page.waitForRequest(request => request.method() === 'POST');
    await page.selectOption('#animation', 'solid');
    await pending;
    try {
      assert.equal(await page.locator('#animation').isEnabled(), true);
      await page.selectOption('#animation', 'pulse');
      await page.locator('#brightness').fill('35');
    } finally {
      control.holdPost = undefined; release();
    }
    await page.waitForFunction(() => document.getElementById('current').textContent === 'Pulse');
    assert.equal(await page.locator('#brightness').inputValue(), '35');
    assert.deepEqual(control.posts, [
      '/api/animation/set/solid/20/100/300/00ff64',
      '/api/animation/set/pulse/35/100/300/00ff64'
    ]);
  });

  test('Invalid input is not transmitted or overwritten by background status', async t => {
    const { page, control } = await fixture(t);
    await page.fill('#interval', '');
    control.state.current = 'chase';
    await page.waitForFunction(() => document.getElementById('current').textContent === 'Chase');
    assert.equal(await page.locator('#interval').inputValue(), '');
    assert.deepEqual(control.posts, []);
    const applied = page.waitForResponse(response => response.request().method() === 'POST');
    await page.fill('#interval', '10');
    await applied;
    assert.equal(control.posts.length, 1);
  });

  test('Rejected changes stay editable without an automatic retry loop', async t => {
    const { page, control } = await fixture(t);
    control.status = 400;
    await page.selectOption('#animation', 'rainbow-comet');
    await page.locator('#error').waitFor({ state: 'visible' });
    assert.equal(await page.locator('#animation').inputValue(), 'rainbow-comet');
    assert.equal(await page.locator('#animation').isEnabled(), true);
    assert.equal(control.posts.length, 1);
    control.status = 200;
    await page.selectOption('#animation', 'solid');
    await page.waitForFunction(() => document.getElementById('current').textContent === 'Solid');
    assert.equal(control.posts.length, 2);
  });

  test('Offline startup and disconnect lock all controls until automatic recovery', async t => {
    const { page, control } = await fixture(t, { offline: true });
    const allDisabled = () => page.waitForFunction(() =>
      [...document.querySelectorAll('#settings input, #settings select')].every(input => input.disabled));
    await allDisabled();
    await page.waitForFunction(() => document.getElementById('settings').dataset.online === 'false');
    control.offline = false;
    await page.waitForFunction(() => !document.getElementById('animation').disabled);
    control.disconnect = true;
    await page.waitForFunction(() => document.getElementById('settings').dataset.online === 'false');
    await allDisabled();
    control.disconnect = false;
    await page.waitForFunction(() => !document.getElementById('animation').disabled);
    assert.deepEqual(control.posts, []);
  });

  test('A failed automatic update locks controls and retries the pending settings after recovery', async t => {
    const { page, control } = await fixture(t);
    control.offline = true;
    await page.selectOption('#animation', 'pulse');
    await page.waitForFunction(() => document.getElementById('settings').dataset.online === 'false');
    assert.equal(await page.locator('#settings input:enabled, #settings select:enabled').count(), 0);
    control.offline = false;
    await page.waitForFunction(() => document.getElementById('current').textContent === 'Pulse');
    assert.equal(await page.locator('#animation').inputValue(), 'pulse');
    assert.deepEqual(control.posts, ['/api/animation/set/pulse/20/100/300/00ff64']);
  });

  test('One color control switches between original colors and a custom color', async t => {
    const { page, control } = await fixture(t, { originalColors: true });
    assert.equal(await page.locator('#original-colors').count(), 0);
    assert.equal(await page.locator('#color-mode').inputValue(), 'original');
    assert.equal(await page.locator('#color').isVisible(), false);
    await page.selectOption('#animation', 'comet');
    await page.waitForFunction(() => document.getElementById('current').textContent === 'Comet');
    assert.equal(control.posts.at(-1), '/api/animation/set/comet/20/100/300/00ff28/original');
    await page.selectOption('#color-mode', 'custom');
    assert.equal(await page.locator('#color').isVisible(), true);
    const sent = page.waitForResponse(response => response.request().url().endsWith('/123456'));
    await page.fill('#color', '#123456');
    await sent;
    assert.equal(control.posts.at(-1), '/api/animation/set/comet/20/100/300/123456');
  });

  test('Random Color submits the mode once and retains the custom color', async t => {
    const { page, control } = await fixture(t);
    const sent = page.waitForResponse(response => response.request().url().endsWith('/random'));
    await page.selectOption('#color-mode', 'random');
    await sent;
    assert.equal(control.posts.at(-1), '/api/animation/set/random/20/100/300/00ff64/random');
    assert.equal(await page.locator('#color').isVisible(), false);
    control.state.current = 'chase';
    await page.waitForFunction(() => document.getElementById('current').textContent === 'Chase');
    assert.equal(await page.locator('#color-mode').inputValue(), 'random');
    assert.equal(control.posts.length, 1);
    await page.selectOption('#color-mode', 'custom');
    assert.equal(await page.locator('#color').inputValue(), '#00ff64');
  });

  test('Existing firmware retains original/custom color control until Random Color support is installed', async t => {
    const { page } = await fixture(t, { originalColors: true, legacy: true });
    assert.equal(await page.locator('#color-mode').inputValue(), 'original');
    assert.equal(await page.locator('#color-mode option[value=random]').isDisabled(), true);
    await page.selectOption('#color-mode', 'custom');
    assert.equal(await page.locator('#color').isVisible(), true);
  });

  test('Saving stays hidden while pending, appears only during writing and reports failures', async t => {
    const { page, control } = await fixture(t);
    control.state.settings_saved = false;
    control.state.settings_error = false;
    control.state.settings_saving = false;
    await page.reload();
    await page.waitForFunction(() => document.getElementById('settings').dataset.online === 'true');
    assert.equal(await page.locator('#storage-status').isVisible(), false);
    control.state.settings_saving = true;
    await page.locator('#storage-status').waitFor({ state: 'visible' });
    assert.equal(await page.locator('#storage-status').textContent(), 'Saving\u2026');
    control.state.settings_saving = false;
    control.state.settings_saved = true;
    await page.locator('#storage-status').waitFor({ state: 'hidden' });
    control.state.settings_saved = false;
    control.state.settings_error = true;
    await page.locator('#storage-status').waitFor({ state: 'visible' });
    assert.equal(await page.locator('#storage-status').textContent(), 'Settings could not be saved.');
    assert.equal(await page.locator('#animation').isEnabled(), true);
  });

  test('Speed supports and automatically transmits both 5% and 800%', async t => {
    const { page, control } = await fixture(t);
    assert.equal(await page.locator('#speed').getAttribute('min'), '5');
    assert.equal(await page.locator('#speed').getAttribute('max'), '800');
    for (const speed of ['5', '800']) {
      const sent = page.waitForResponse(response => response.request().method() === 'POST' &&
        new URL(response.url()).pathname.includes(`/20/${speed}/300/`));
      await page.locator('#speed').fill(speed);
      await sent;
      assert.equal(await page.locator('#speed-value').textContent(), `${speed}%`);
      assert.equal(control.state.speed_percent, Number(speed));
    }
  });

  test('Choosing Color opens the picker after revealing and enabling it', async t => {
    const { page } = await fixture(t, { originalColors: true });
    assert.equal(await page.evaluate(() => window.colorPickerOpens), 0);
    await page.selectOption('#color-mode', 'custom');
    assert.equal(await page.evaluate(() => window.colorPickerOpens), 1);
    assert.equal(await page.locator('#color').isVisible(), true);
    assert.equal(await page.locator('#color').isEnabled(), true);
    await page.selectOption('#color-mode', 'original');
    assert.equal(await page.evaluate(() => window.colorPickerOpens), 1);
  });

  test('Picker fallback works when showPicker is unavailable or refuses activation', async t => {
    const { page } = await fixture(t, { originalColors: true });
    await page.evaluate(() => {
      const original = HTMLInputElement.prototype.click;
      window.fallbackOpens = 0;
      HTMLInputElement.prototype.click = function () {
        if (this.type === 'color') window.fallbackOpens++;
        else original.call(this);
      };
      HTMLInputElement.prototype.showPicker = undefined;
    });
    await page.selectOption('#color-mode', 'custom');
    assert.equal(await page.evaluate(() => window.fallbackOpens), 1);
    await page.selectOption('#color-mode', 'original');
    await page.evaluate(() => {
      HTMLInputElement.prototype.showPicker = () => { throw new DOMException('No activation', 'NotAllowedError'); };
    });
    await page.selectOption('#color-mode', 'custom');
    assert.equal(await page.evaluate(() => window.fallbackOpens), 2);
  });

  test('Color menu uses Graphite colors and a subtle group focus without an inner stripe', async t => {
    const { page } = await fixture(t);
    await page.locator('#color-mode').focus();
    const style = await page.evaluate(() => {
      const select = document.getElementById('color-mode');
      const option = select.querySelector('option[value=custom]');
      const group = select.closest('.color-control');
      return { optionBackground: getComputedStyle(option).backgroundColor,
        optionText: getComputedStyle(option).color, selectOutline: getComputedStyle(select).outlineStyle,
        groupBorder: getComputedStyle(group).borderColor, groupShadow: getComputedStyle(group).boxShadow };
    });
    assert.equal(style.optionBackground, 'rgb(68, 69, 79)');
    assert.equal(style.optionText, 'rgb(226, 227, 230)');
    assert.equal(style.selectOutline, 'none');
    assert.equal(style.groupBorder, 'rgb(119, 121, 134)');
    assert.match(style.groupShadow, /0px 0px 0px 1px/);
  });

  test('Brightness supports and automatically transmits 1% and 100%', async t => {
    const { page, control } = await fixture(t);
    assert.equal(await page.locator('#brightness').getAttribute('min'), '1');
    assert.equal(await page.locator('#brightness').getAttribute('max'), '100');
    for (const brightness of ['1', '100']) {
      const sent = page.waitForResponse(response => response.request().method() === 'POST' &&
        new URL(response.url()).pathname.includes(`/set/random/${brightness}/100/300/`));
      await page.locator('#brightness').fill(brightness);
      await sent;
      assert.equal(await page.locator('#brightness-value').textContent(), `${brightness}%`);
      assert.equal(control.state.brightness_percent, Number(brightness));
    }
  });
});
