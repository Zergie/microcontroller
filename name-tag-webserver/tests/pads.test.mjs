import { describe, test, before, after } from 'node:test';
import assert from 'node:assert/strict';
import { fileURLToPath } from 'node:url';
import { chromium } from 'playwright';
import { installFakeAudio } from './fakes/pad-audio.mjs';
import { createDevServer } from '../../shared/rp2040-usb-web/tools/dev-server.mjs';

describe('LED pad controller', { concurrency: 4 }, () => {
  let browser, app, base;
  before(async () => {
    app = await createDevServer({ target: 'http://127.0.0.1:9', webRoot: fileURLToPath(new URL('../web/', import.meta.url)) });
    await new Promise(resolve => app.server.listen(0, '127.0.0.1', resolve));
    base = `http://127.0.0.1:${app.server.address().port}`;
    browser = await chromium.launch({ channel: process.platform === 'win32' ? 'msedge' : undefined });
  });
  after(async () => { await browser?.close(); await app?.close(); });
  async function until(condition) {
    const end = Date.now() + 6000;
    while (!condition()) { assert.ok(Date.now() < end, 'Expected device request was not observed'); await new Promise(r => setTimeout(r, 15)); }
  }
  async function fixture(t, { width = 375, height = 667, supported = true, audio, fast = false } = {}) {
    const context = await browser.newContext({ viewport: { width, height }, isMobile: true, hasTouch: true });
    const page = await context.newPage();
    if (audio) await page.addInitScript(installFakeAudio, { mode: audio });
    const state = { active: false, pressed_mask: 0, fade_ms: 1000,
      colors: ['#00ff28', '#00ffff', '#0000ff', '#b400ff', '#ff0014', '#ff2800', '#ff9600', '#ffffff'],
      brightness: 20, posts: [], delay: 0, offline: false, inFlight: 0, maxInFlight: 0, sockets: [], messages: [], pulses: [], ack: true };
    const errors = [];
    page.on('pageerror', error => errors.push(error.message));
    t.after(async () => { state.delay = 0; await context.close(); assert.deepEqual(errors, []); });
    await page.addInitScript(() => {
      window.touches = [];
      document.addEventListener('pointerdown', e => {
        if (e.target.matches('.led-pad')) window.touches.push({ id: e.pointerId, label: e.target.textContent });
      }, true);
    });
    if (fast) await page.routeWebSocket('**/api/pads/ws', ws => {
      state.sockets.push(ws);
      ws.onMessage(message => {
        const bytes = [...message]; state.messages.push(bytes);
        if (bytes[1]) state.pulses.push({ mask: bytes[1], fade: state.fade_ms });
        assert.equal(bytes.length, 2); assert.equal(bytes[0], 1);
        if (state.active) state.pressed_mask = bytes[1];
        if (state.ack) ws.send(Buffer.from(bytes));
      });
      ws.onClose(() => { state.pressed_mask = 0; });
    });
    await page.route('**/api/**', async route => {
      if (state.offline) return route.abort('connectionfailed');
      const path = new URL(route.request().url()).pathname;
      if (route.request().method() === 'POST') {
        state.posts.push(path); state.inFlight++; state.maxInFlight = Math.max(state.maxInFlight, state.inFlight);
        try {
          if (state.delay) await new Promise(r => setTimeout(r, state.delay));
          if (path.startsWith('/api/animation/set/')) state.brightness = Number(path.split('/')[5]);
          else if (path === '/api/view/animations') { state.active = false; state.pressed_mask = 0; }
          else if (path.startsWith('/api/pads/set/')) {
            const parts = path.split('/');
            state.active = true; state.pressed_mask = Number(parts[4]); state.fade_ms = Number(parts[5]);
            if (state.pressed_mask) state.pulses.push({ mask: state.pressed_mask, fade: state.fade_ms });
            state.colors = parts[6].match(/.{6}/g).map(c => `#${c}`);
          } else if (path.startsWith('/api/pads/config/')) {
            const parts = path.split('/');
            state.active = true; state.fade_ms = Number(parts[4]);
            state.colors = parts[5].match(/.{6}/g).map(c => `#${c}`);
          } else if (path === '/api/pads/release') state.pressed_mask = 0;
        } finally { state.inFlight--; }
      }
      const data = path.includes('/animation') && path !== '/api/view/animations'
        ? { selection: 'random', current: 'rainbow', brightness_percent: state.brightness, speed_percent: 100,
          interval_seconds: 10, color: '#00ff28', color_mode: 'original', effects: ['rainbow'],
          pads_supported: supported, ...(fast ? { pads_transport: 'websocket-v1' } : {}), control_view: state.active ? 'pads' : 'animations' }
        : { active: state.active, pressed_mask: state.pressed_mask, fade_ms: state.fade_ms, colors: state.colors };
      if (!page.isClosed()) await route.fulfill({ json: data });
    });
    await page.goto(base);
    await page.waitForFunction(() => !document.getElementById('animation').disabled);
    async function enter() {
      await page.click('#menu-toggle'); await page.click('#view-pads');
      if (supported) await page.waitForFunction(() => !document.querySelector('.led-pad').disabled);
      if (supported) await until(() => state.active);
      if (supported && fast) await until(() => state.messages.length > 0);
    }
    return { page, context, state, enter };
  }
  async function startTouches(context, page, indices) {
    const cdp = await context.newCDPSession(page);
    const points = [];
    for (let i = 0; i < indices.length; i++) {
      const box = await page.getByRole('button', { name: `LED ${indices[i] + 1}`, exact: true }).boundingBox();
      points.push({ id: i + 1, x: box.x + box.width / 2 + i * 2, y: box.y + box.height / 2, radiusX: 4, radiusY: 4 });
    }
    await cdp.send('Input.dispatchTouchEvent', { type: 'touchStart', touchPoints: points });
    return cdp;
  }

  test('WebSocket sends masks immediately while HTTP settings or acknowledgements are pending', async t => {
    const { page, context, state, enter } = await fixture(t, { fast: true, audio: 'normal' }); await enter();
    const posts = state.posts.length;
    state.ack = false; // Sending the next state must not wait for a reply.
    const cdp = await startTouches(context, page, [0, 7]);
    await until(() => state.pressed_mask === 129);
    assert.equal(state.posts.length, posts);
    state.delay = 350;
    await page.locator('input[aria-label="LED 2 color"]').fill('#123456');
    await until(() => state.inFlight === 1);
    await cdp.send('Input.dispatchTouchEvent', { type: 'touchEnd', touchPoints: [] });
    await until(() => state.pressed_mask === 0);
    assert.equal(state.inFlight, 1); // Release passed the pending HTTP configuration.
    state.ack = true;
    await until(() => state.colors[1] === '#123456');
    assert.ok(state.posts.every(path => !path.includes('/pads/set/')));
    assert.ok(state.messages.every(bytes => bytes.length === 2 && bytes[0] === 1));
  });

  test('WebSocket failure silences audio, releases fingers and reconnects without held notes', async t => {
    const { page, context, state, enter } = await fixture(t, { fast: true, audio: 'normal' }); await enter();
    await startTouches(context, page, [0, 7]); await until(() => state.pressed_mask === 129);
    await state.sockets.at(-1).close({ code: 1001 });
    await page.waitForFunction(() => document.querySelector('.led-pad').disabled);
    assert.equal(await audioMask(page), 0);
    await page.waitForFunction(() => !document.querySelector('.led-pad').disabled);
    const pad = page.getByRole('button', { name: 'LED 4', exact: true });
    await pad.focus(); await page.keyboard.down('Space');
    await until(() => state.sockets.length >= 2 && state.pressed_mask === 8);
    await page.keyboard.up('Space'); await until(() => state.pressed_mask === 0);
    assert.equal(await audioMask(page), 0);
  });

  test('Leaving the fast view during a settings request cannot reactivate held pads', async t => {
    const { page, context, state, enter } = await fixture(t, { fast: true, audio: 'normal' }); await enter();
    state.delay = 300;
    await page.selectOption('#pads-palette', 'ocean'); await until(() => state.inFlight === 1);
    const cdp = await startTouches(context, page, [0]); await until(() => state.pressed_mask === 1);
    await page.click('#menu-toggle'); await page.click('#view-animations');
    await cdp.send('Input.dispatchTouchEvent', { type: 'touchEnd', touchPoints: [] });
    await until(() => state.active === false && state.inFlight === 0);
    assert.equal(state.pressed_mask, 0); assert.equal(await audioMask(page), 0);
    assert.equal(state.posts.at(-1), '/api/view/animations');
  });

  test('An unresponsive WebSocket is detected even while new masks keep being sent', async t => {
    const { page, context, state, enter } = await fixture(t, { fast: true, audio: 'normal' }); await enter();
    state.ack = false;
    await startTouches(context, page, [0]);
    await page.waitForFunction(() => document.querySelector('.led-pad').disabled);
    assert.equal(await audioMask(page), 0);
  });

  async function audioMask(page) {
    return page.evaluate(() => audioTest.contexts[0].gains.slice(1).reduce((mask, node, index) =>
      node.gain.events.at(-1)?.value > 0 ? mask | (1 << index) : mask, 0));
  }

  async function enterSongs(page) {
    await page.click('#menu-toggle'); await page.click('#view-songs');
    await page.waitForFunction(() => !document.getElementById('song-play').disabled);
  }

  for (const [width, height] of [[375, 667], [667, 375]]) {
    test(`Songs offers shared colors, eight pickers, brightness and the song catalogue at ${width}x${height}`, async t => {
      const { page, state } = await fixture(t, { width, height, fast: true, audio: 'normal' });
      await enterSongs(page);
      assert.equal(await page.locator('#current').textContent(), 'Songs');
      assert.equal(await page.locator('.led-pad:visible').count(), 0);
      assert.equal(await page.locator('.pad-slot input:visible').count(), 8);
      assert.equal(await page.locator('#pads-palette').isVisible(), true);
      assert.equal(await page.locator('#pads-fade').isVisible(), false);
      assert.equal(await page.locator('#song-select option').count(), 5);
      assert.equal(await page.locator('#song-play').textContent(), 'Play');
      assert.equal(await page.evaluate(() => audioTest.contexts.length), 0);
      assert.equal(await page.evaluate(() => document.documentElement.scrollWidth <= innerWidth), true);
      await page.selectOption('#pads-palette', 'ocean');
      await until(() => state.colors[7] === '#00ffcc');
      await page.locator('.pad-slot input').first().fill('#123abc');
      await until(() => state.colors[7] === '#123abc');
      await page.locator('#pads-brightness').fill('45');
      await until(() => state.brightness === 45);
      const play = await page.locator('#song-play').boundingBox();
      assert.ok(play.width >= 200 && play.height >= 76);
      await page.selectOption('#song-select', 'mahna');
      assert.match(await page.locator('#song-select option:checked').textContent(), /Mahna Mahna.*MIDI/);
      await page.click('#menu-toggle'); await page.click('#view-pads');
      assert.equal(await page.locator('.led-pad:visible').count(), 8);
      assert.equal(await page.locator('.pad-slot input').first().inputValue(), '#123abc');
      await until(() => state.fade_ms === 1000);
    });
  }

  for (const fast of [true, false]) {
    test(`Songs Play/Stop drives successive notes and silence (WebSocket: ${fast})`, async t => {
      const { page, state } = await fixture(t, { fast, audio: 'normal' }); await enterSongs(page);
      await page.click('#song-play');
      await until(() => state.pulses.some(pulse => pulse.mask === 128));
      assert.equal(await page.locator('#song-play').textContent(), 'Stop');
      assert.equal(await page.evaluate(() => audioTest.calls.every(call => call.gesture)), true);
      const received = () => fast ? state.messages.map(bytes => bytes[1]) :
        state.posts.filter(path => path.startsWith('/api/pads/set/')).map(path => Number(path.split('/')[4]));
      await until(() => received().includes(64));
      assert.ok(received().indexOf(128) < received().indexOf(64));
      assert.equal(await page.evaluate(() => audioTest.contexts[0].gains[7].gain.events.some(e => e.type === 'ramp' && e.value === 0.125)), true);
      await until(() => received().lastIndexOf(0) > received().indexOf(128));
      await page.click('#song-play'); await until(() => state.pressed_mask === 0);
      assert.equal(await audioMask(page), 0);
      assert.equal(await page.locator('#song-play').textContent(), 'Play');
      if (fast) assert.equal(state.posts.filter(path => path.includes('/pads/set/')).length, 0);
    });
  }

  for (const reason of ['view', 'offline', 'hidden', 'pagehide', 'blur', 'song-change']) {
    test(`Song playback stops on ${reason} without restarting automatically`, async t => {
      const { page, state } = await fixture(t, { fast: true, audio: 'normal' }); await enterSongs(page);
      await page.click('#song-play'); await until(() => state.pulses.some(pulse => pulse.mask === 128));
      if (reason === 'view') { await page.click('#menu-toggle'); await page.click('#view-animations'); }
      else if (reason === 'offline') state.sockets[0].close();
      else if (reason === 'song-change') await page.selectOption('#song-select', 'jakob');
      else await page.evaluate(reason => {
        if (reason === 'hidden') {
          Object.defineProperty(document, 'hidden', { configurable: true, value: true });
          document.dispatchEvent(new Event('visibilitychange'));
        } else window.dispatchEvent(new Event(reason));
      }, reason);
      await until(() => state.pressed_mask === 0);
      if (reason === 'view') await until(() => !state.active && state.fade_ms === 1000);
      assert.equal(await audioMask(page), 0);
      assert.equal(await page.locator('#song-play').textContent(), 'Play');
      if (reason === 'hidden' || reason === 'blur') {
        await page.evaluate(() => {
          Object.defineProperty(document, 'hidden', { configurable: true, value: false });
          document.dispatchEvent(new Event('visibilitychange')); window.dispatchEvent(new Event('focus'));
        });
        await page.waitForFunction(() => !document.getElementById('song-play').disabled);
        assert.equal(await audioMask(page), 0);
      }
    });
  }

  test('Song LED pulses capture the full quarter/half-note fade and immediately release', async t => {
    const { page, state } = await fixture(t, { fast: true, audio: 'normal' }); await enterSongs(page);
    await page.click('#song-play');
    await until(() => state.pulses.some(pulse => pulse.mask === 8)); // First half note, G4.
    await until(() => state.messages.at(-1)?.[1] === 0);
    const first = state.pulses.find(pulse => pulse.mask === 128);
    const half = state.pulses.find(pulse => pulse.mask === 8);
    assert.equal(first.fade, Math.round(60000 / 112));
    assert.equal(half.fade, Math.round(2 * 60000 / 112));
    const masks = state.messages.map(bytes => bytes[1]);
    for (let i = 0; i < masks.length; i++) if (masks[i]) assert.equal(masks[i + 1], 0);
    await page.click('#song-play');
  });

  test('Uploaded Mahna MIDI starts on C5, uses its note-length fade and finishes naturally', async t => {
    const { page, state } = await fixture(t, { fast: true, audio: 'normal' }); await enterSongs(page);
    await page.selectOption('#song-select', 'mahna');
    await page.click('#song-play');
    await until(() => state.pulses.some(pulse => pulse.mask === 1));
    assert.equal(state.pulses.find(pulse => pulse.mask === 1).fade, 167);
    await page.waitForFunction(() => document.getElementById('song-play').textContent === 'Play');
    assert.equal(await audioMask(page), 0);
    assert.equal(state.pressed_mask, 0);
    assert.ok(state.pulses.every(pulse => [167, 333].includes(pulse.fade)));
  });

  test('Leaving Songs during an HTTP pulse restores the manual fade before resuming animation', async t => {
    const { page, state } = await fixture(t); await enterSongs(page);
    state.delay = 250;
    await page.click('#song-play');
    await until(() => state.posts.some(path => path.startsWith('/api/pads/set/128/')));
    await page.click('#menu-toggle'); await page.click('#view-animations');
    await until(() => !state.active && state.fade_ms === 1000 && state.inFlight === 0);
    assert.equal(state.pressed_mask, 0);
    assert.equal(state.posts.at(-1), '/api/view/animations');
  });

  test('Real Web Audio rendering has a soft attack, bounded polyphony and silence after release', async t => {
    const { page } = await fixture(t);
    const samples = await page.evaluate(async () => {
      const { createPadAudio } = await import('/pad-audio.js');
      async function render(mask, fadeMs) {
        const offline = new OfflineAudioContext(1, 24000, 48000);
        let now = 0;
        const adapter = { state: 'running', get currentTime() { return now; },
          destination: offline.destination, createGain: () => offline.createGain(),
          createOscillator: () => offline.createOscillator(), addEventListener() {} };
        const Original = window.AudioContext;
        window.AudioContext = class { constructor() { return adapter; } };
        const audio = createPadAudio(); audio.unlock(); audio.setMask(mask, fadeMs);
        now = 0.1; audio.setMask(0, fadeMs);
        window.AudioContext = Original;
        const data = (await offline.startRendering()).getChannelData(0);
        const peak = (start, end) => data.slice(start * 48000, end * 48000).reduce((m, x) => Math.max(m, Math.abs(x)), 0);
        return { first: data[0], attack: peak(0, 0.001), held: peak(0.02, 0.09),
          middle: peak(0.2, 0.21), end: peak(0.32, 0.49), peak: peak(0, 0.5), zeroReleaseEnd: peak(0.111, 0.49) };
      }
      return { single: await render(128, 200), chord: await render(255, 200), zero: await render(128, 0) };
    });
    assert.equal(samples.single.first, 0);
    assert.ok(samples.single.attack < samples.single.held * 0.11);
    assert.ok(samples.single.middle > 0 && samples.single.middle < samples.single.held * 0.6);
    assert.equal(samples.single.end, 0);
    assert.ok(samples.chord.peak > 0 && samples.chord.peak <= 0.25);
    assert.equal(samples.chord.end, 0);
    assert.equal(samples.zero.zeroReleaseEnd, 0);
  });

  test('Landscape pads stay fixed with reduced Safari space and offline messages', async t => {
    const { page, state, enter } = await fixture(t, { width: 667, height: 260 });
    await enter();
    state.offline = true;
    await page.fill('#pads-fade', '2');
    await page.waitForFunction(() => document.querySelector('.led-pad').disabled);
    await page.mouse.move(5, 180);
    await page.mouse.wheel(0, 300);
    await page.evaluate(() => new Promise(resolve => requestAnimationFrame(() => requestAnimationFrame(resolve))));
    assert.equal(await page.evaluate(() => scrollY), 0);
    for (const id of ['menu-toggle', 'pads-fade', 'pads-brightness']) {
      const box = await page.locator(`#${id}`).boundingBox();
      assert.ok(box.y >= 0 && box.y + box.height <= 260, `${id} stays in the viewport`);
    }
  });

  test('Landscape touch swipes cannot pan the page; portrait and animation scrolling recover', async t => {
    const { page, context, enter } = await fixture(t); await enter();
    await page.mouse.move(5, 300); await page.mouse.wheel(0, 300);
    await page.waitForFunction(() => scrollY > 0);
    await page.setViewportSize({ width: 667, height: 300 });
    await page.waitForFunction(() => scrollY === 0);
    const cdp = await context.newCDPSession(page);
    await cdp.send('Input.dispatchTouchEvent', { type: 'touchStart', touchPoints: [{ id: 1, x: 5, y: 250 }] });
    for (const y of [220, 180, 140, 100, 60]) {
      await cdp.send('Input.dispatchTouchEvent', { type: 'touchMove', touchPoints: [{ id: 1, x: 5, y }] });
    }
    await cdp.send('Input.dispatchTouchEvent', { type: 'touchEnd', touchPoints: [] });
    await page.mouse.wheel(0, 300);
    await page.evaluate(() => new Promise(resolve => requestAnimationFrame(() => requestAnimationFrame(resolve))));
    assert.equal(await page.evaluate(() => scrollY), 0);
    await page.click('#menu-toggle'); await page.click('#view-animations');
    await page.mouse.move(5, 180); await page.mouse.wheel(0, 300);
    await page.waitForFunction(() => scrollY > 0);
    await page.click('#menu-toggle'); await page.click('#view-pads');
    await page.setViewportSize({ width: 375, height: 667 });
    await page.mouse.move(5, 300); await page.mouse.wheel(0, 300);
    await page.waitForFunction(() => scrollY > 0);
  });

  for (const [width, height] of [[375, 667], [667, 375], [390, 844], [844, 390], [568, 320]]) {
    test(`Both views and eight large pads fit iPhone ${width}x${height}`, async t => {
      const { page, enter } = await fixture(t, { width, height });
      assert.equal(await page.evaluate(() => document.documentElement.scrollWidth <= innerWidth), true);
      await enter();
      if (width > height) {
        assert.equal(await page.locator('#current').isVisible(), false);
        assert.equal(await page.locator('#menu-toggle').isVisible(), true);
        assert.equal(await page.evaluate(() => document.documentElement.scrollHeight <= innerHeight), true);
        const pad = await page.locator('.led-pad').first().boundingBox();
        assert.ok(pad.height >= height - 160);
        assert.equal(await page.locator('#pads-brightness').isVisible(), true);
      }
      assert.equal(await page.locator('.led-pad').count(), 8);
      assert.equal(await page.locator('#pads-palette').isVisible(), width <= height);
      assert.deepEqual(await page.locator('.led-pad').allTextContents(), ['8', '7', '6', '5', '4', '3', '2', '1']);
      assert.equal(await page.locator('.now-playing #menu-toggle').count(), 1);
      assert.equal(await page.locator('#loop-status').count(), 0);
      for (const pad of await page.locator('.led-pad').all()) {
        const box = await pad.boundingBox(); assert.ok(box.width >= 44 && box.height >= 76);
      }
      assert.equal(await page.evaluate(() => document.documentElement.scrollWidth <= innerWidth), true);
      assert.equal(await page.locator('#pads-all-color').count(), 0);
      await page.locator('#pads-fade').scrollIntoViewIfNeeded();
      assert.equal(await page.locator('#pads-fade').isVisible(), true);
      await page.click('#menu-toggle'); await page.click('#view-animations');
      assert.equal(await page.locator('#settings').isVisible(), true);
    });
  }

  test('Pad brightness applies immediately, shares animation brightness and keeps held pads', async t => {
    const { page, context, state, enter } = await fixture(t, { audio: 'normal' }); await enter();
    const cdp = await startTouches(context, page, [0]);
    await until(() => state.pressed_mask === 1);
    const envelope = await page.evaluate(() => audioTest.contexts[0].gains[1].gain.events);
    await page.locator('#pads-brightness').fill('1');
    await until(() => state.brightness === 1);
    assert.equal(await page.locator('#pads-brightness-value').textContent(), '1%');
    assert.equal(state.active, true); assert.equal(state.pressed_mask, 1);
    await page.locator('#pads-brightness').fill('100');
    await until(() => state.brightness === 100);
    assert.deepEqual(await page.evaluate(() => audioTest.contexts[0].gains[1].gain.events), envelope);
    await cdp.send('Input.dispatchTouchEvent', { type: 'touchEnd', touchPoints: [] });
    await page.click('#menu-toggle'); await page.click('#view-animations');
    assert.equal(await page.locator('#brightness').inputValue(), '100');
  });

  test('Keyboard pad activation unlocks audio synchronously and resumes after an interruption', async t => {
    const { page, enter } = await fixture(t, { audio: 'normal' }); await enter();
    const pad = page.getByRole('button', { name: 'LED 8', exact: true });
    await pad.focus(); await page.keyboard.down('Space');
    assert.equal(await audioMask(page), 128);
    await page.keyboard.up('Space'); assert.equal(await audioMask(page), 0);
    await page.evaluate(() => {
      const context = audioTest.contexts[0]; context.state = 'interrupted'; context.listeners.statechange();
    });
    await page.keyboard.down('Space');
    assert.equal(await audioMask(page), 128);
    assert.equal(await page.evaluate(() => audioTest.calls.every(call => call.gesture)), true);
    assert.equal(await page.evaluate(() => audioTest.calls.filter(c => c.type === 'resume').length), 2);
    await page.keyboard.up('Space');
  });

  test('Long presses suppress native copy/selection without releasing pads or blocking colors', async t => {
    const { page, context, state, enter } = await fixture(t, { audio: 'normal' }); await enter();
    const cdp = await startTouches(context, page, [7]);
    await until(() => state.pressed_mask === 128);
    await page.waitForTimeout(800);
    const cancelled = await page.getByRole('button', { name: 'LED 8', exact: true }).evaluate(button =>
      ['touchstart', 'contextmenu', 'selectstart', 'dragstart'].map(type => {
        const event = new Event(type, { bubbles: true, cancelable: true });
        button.dispatchEvent(event);
        return event.defaultPrevented;
      }));
    assert.deepEqual(cancelled, [true, true, true, true]);
    assert.equal(state.pressed_mask, 128);
    assert.equal(await audioMask(page), 128);
    await cdp.send('Input.dispatchTouchEvent', { type: 'touchEnd', touchPoints: [] });
    await until(() => state.pressed_mask === 0);
    assert.equal(await audioMask(page), 0);
    const color = page.locator('input[aria-label="LED 8 color"]');
    assert.equal(await color.evaluate(input => input.dispatchEvent(new Event('contextmenu', { bubbles: true, cancelable: true }))), true);
    await color.fill('#123456');
    await until(() => state.colors[7] === '#123456');
  });

  test('Real simultaneous touches produce a combined mask and release independently', async t => {
    const { page, context, state, enter } = await fixture(t);
    await enter();
    const cdp = await startTouches(context, page, [0, 7]);
    await until(() => state.pressed_mask === 129);
    assert.equal(await page.locator('.led-pad[aria-pressed=true]').count(), 2);
    const firstId = await page.evaluate(() => window.touches.find(t => t.label === '1').id);
    await page.getByRole('button', { name: 'LED 1', exact: true }).dispatchEvent('pointerup', { pointerId: firstId });
    await until(() => state.pressed_mask === 128);
    await cdp.send('Input.dispatchTouchEvent', { type: 'touchEnd', touchPoints: [] });
    await until(() => state.pressed_mask === 0);
  });

  for (const [width, height, fast] of [[375, 667, false], [667, 375, false], [667, 375, true]]) {
    test(`Piano swipes track each finger across pads and gaps at ${width}x${height} (WebSocket: ${fast})`, async t => {
      const { page, context, state, enter } = await fixture(t, { width, height, fast, audio: 'normal' }); await enter();
      assert.equal(await page.evaluate(() => audioTest.contexts.length), 0);
      const point = async (led, id) => {
        const box = await page.getByRole('button', { name: `LED ${led}`, exact: true }).boundingBox();
        return { id, x: box.x + box.width / 2, y: box.y + box.height / 2, radiusX: 4, radiusY: 4 };
      };
      const cdp = await context.newCDPSession(page);
      const stationary = await point(1, 2);
      await cdp.send('Input.dispatchTouchEvent', { type: 'touchStart', touchPoints: [await point(8, 1), stationary] });
      await until(() => state.pressed_mask === 129);
      assert.equal(await audioMask(page), 129);
      assert.equal(await page.evaluate(() => audioTest.calls.every(call => call.gesture)), true);
      for (const led of [7, 6, 5]) {
        await cdp.send('Input.dispatchTouchEvent', { type: 'touchMove', touchPoints: [await point(led, 1), stationary] });
        await until(() => state.pressed_mask === ((1 << (led - 1)) | 1));
        assert.equal(await audioMask(page), state.pressed_mask);
      }
      const left = await page.getByRole('button', { name: 'LED 8', exact: true }).boundingBox();
      const right = await page.getByRole('button', { name: 'LED 7', exact: true }).boundingBox();
      const gap = { id: 1, x: (left.x + left.width + right.x) / 2, y: left.y + left.height / 2 };
      await cdp.send('Input.dispatchTouchEvent', { type: 'touchMove', touchPoints: [gap, stationary] });
      await until(() => state.pressed_mask === 1);
      assert.equal(await audioMask(page), 1);
      await cdp.send('Input.dispatchTouchEvent', { type: 'touchMove', touchPoints: [await point(8, 1), stationary] });
      await until(() => state.pressed_mask === 129);
      // Both fingers can converge on one LED, then separate without releasing it.
      await cdp.send('Input.dispatchTouchEvent', { type: 'touchMove', touchPoints: [await point(1, 1), stationary] });
      await until(() => state.pressed_mask === 1);
      await cdp.send('Input.dispatchTouchEvent', { type: 'touchMove', touchPoints: [await point(2, 1), stationary] });
      await until(() => state.pressed_mask === 3);
      assert.equal(await audioMask(page), 3);
      await cdp.send('Input.dispatchTouchEvent', { type: 'touchEnd', touchPoints: [] });
      await until(() => state.pressed_mask === 0);
      assert.equal(await audioMask(page), 0);
    });
  }

  test('Mouse dragging also plays adjacent pads and releases outside them', async t => {
    const { page, state, enter } = await fixture(t); await enter();
    const first = await page.getByRole('button', { name: 'LED 8', exact: true }).boundingBox();
    const second = await page.getByRole('button', { name: 'LED 7', exact: true }).boundingBox();
    await page.mouse.move(first.x + first.width / 2, first.y + first.height / 2);
    await page.mouse.down(); await until(() => state.pressed_mask === 128);
    await page.mouse.move(second.x + second.width / 2, second.y + second.height / 2);
    await until(() => state.pressed_mask === 64);
    await page.mouse.move(1, 1); await until(() => state.pressed_mask === 0);
    await page.mouse.up();
  });

  test('Two fingers on one pad keep it held until both are released', async t => {
    const { page, context, state, enter } = await fixture(t, { audio: 'normal' });
    await enter(); const cdp = await startTouches(context, page, [2, 2]);
    await until(() => state.pressed_mask === 4);
    const ids = await page.evaluate(() => window.touches.filter(t => t.label === '3').map(t => t.id));
    assert.equal(ids.length, 2);
    await page.getByRole('button', { name: 'LED 3', exact: true }).dispatchEvent('pointercancel', { pointerId: ids[0] });
    assert.equal(await page.getByRole('button', { name: 'LED 3', exact: true }).getAttribute('aria-pressed'), 'true');
    assert.equal(await audioMask(page), 4);
    await cdp.send('Input.dispatchTouchEvent', { type: 'touchEnd', touchPoints: [] });
    await until(() => state.pressed_mask === 0);
    assert.equal(await audioMask(page), 0);
  });

  for (const reason of ['view', 'offline', 'hidden', 'pagehide', 'blur']) {
    test(`Audio releases every voice and fading tail on ${reason}`, async t => {
      const { page, context, state, enter } = await fixture(t, { audio: 'normal' }); await enter();
      await page.fill('#pads-fade', '2');
      await startTouches(context, page, [0, 7]);
      assert.equal(await audioMask(page), 129);
      // Also leave a long release tail, which must be cut short by lifecycle cleanup.
      const pointer = await page.evaluate(() => window.touches.find(t => t.label === '1').id);
      await page.getByRole('button', { name: 'LED 1', exact: true }).dispatchEvent('pointerup', { pointerId: pointer });
      if (reason === 'view') { await page.click('#menu-toggle'); await page.click('#view-animations'); }
      else if (reason === 'offline') {
        state.offline = true;
        await page.waitForFunction(() => document.querySelector('.led-pad').disabled);
      } else await page.evaluate(reason => {
        if (reason === 'hidden') {
          Object.defineProperty(document, 'hidden', { configurable: true, value: true });
          document.dispatchEvent(new Event('visibilitychange'));
        } else window.dispatchEvent(new Event(reason));
      }, reason);
      assert.equal(await audioMask(page), 0);
      assert.equal(await page.evaluate(() => audioTest.contexts[0].gains.slice(1).every(g =>
        g.gain.events.at(-1).time <= 0.011)), true);
    });
  }

  for (const audio of ['missing', 'throws', 'rejects']) {
    test(`LED multitouch still works when audio ${audio}`, async t => {
      const { page, context, state, enter } = await fixture(t, { audio }); await enter();
      const cdp = await startTouches(context, page, [0, 7]);
      await until(() => state.pressed_mask === 129);
      await cdp.send('Input.dispatchTouchEvent', { type: 'touchEnd', touchPoints: [] });
      await until(() => state.pressed_mask === 0);
    });
  }

  test('Individual colors and global fade are applied', async t => {
    const { page, state, enter } = await fixture(t); await enter();
    await page.locator('input[aria-label="LED 2 color"]').fill('#123456');
    await until(() => state.colors[1] === '#123456');
    assert.notEqual(state.colors[0], '#123456');
    assert.equal(await page.locator('#pads-fade').getAttribute('max'), '2');
    await page.locator('#pads-fade').fill('2');
    await until(() => state.fade_ms === 2000);
    assert.equal(await page.locator('#pads-fade-value').textContent(), '2.0 s');
  });

  test('Portrait palettes apply all LED colors in visual order and detect custom colors', async t => {
    const { page, state, enter } = await fixture(t); await enter();
    const select = page.locator('#pads-palette');
    assert.equal(await select.inputValue(), 'original');
    const picker = await select.boundingBox();
    const pad = await page.locator('.led-pad').first().boundingBox();
    assert.ok(picker.y + picker.height < pad.y);
    for (const palette of ['rainbow', 'sunset', 'ocean', 'pastel', 'original']) {
      const previous = [...state.colors];
      await select.selectOption(palette);
      await until(() => state.colors.some((color, index) => color !== previous[index]));
      assert.equal(await select.inputValue(), palette);
      const displayed = await page.locator('.pad-slot input').evaluateAll(inputs => inputs.map(input => input.value));
      assert.deepEqual(displayed, [...state.colors].reverse());
      if (palette === 'rainbow') assert.deepEqual(displayed, ['#ff0000', '#ff8000', '#ffff00', '#80ff00', '#00ff80', '#00bfff', '#4000ff', '#bf00ff']);
    }
    await page.locator('input[aria-label="LED 8 color"]').fill('#123456');
    await until(() => state.colors[7] === '#123456');
    assert.equal(await select.inputValue(), 'custom');
    state.offline = true;
    await page.fill('#pads-fade', '2');
    await page.waitForFunction(() => document.getElementById('pads-palette').disabled);
    state.offline = false;
    await page.waitForFunction(() => !document.getElementById('pads-palette').disabled);
    await select.selectOption('ocean');
    await until(() => state.colors[7] === '#00ffcc');
    await page.reload();
    await page.waitForFunction(() => !document.getElementById('pads-palette').disabled);
    assert.equal(await select.inputValue(), 'ocean');
  });

  test('Release during a slow press request is queued, never dropped or reordered', async t => {
    const { page, context, state, enter } = await fixture(t); await enter();
    state.delay = 250;
    const before = state.posts.length;
    const cdp = await startTouches(context, page, [0]);
    await until(() => state.posts.slice(before).some(path => path.includes('/set/1/')));
    await cdp.send('Input.dispatchTouchEvent', { type: 'touchEnd', touchPoints: [] });
    await until(() => state.posts.slice(before).some(path => path.includes('/set/0/')) && state.inFlight === 0);
    assert.equal(state.pressed_mask, 0); assert.equal(state.maxInFlight, 1);
  });

  test('Leaving pads while a request is pending ends in animation mode with no held pad', async t => {
    const { page, context, state, enter } = await fixture(t); await enter();
    state.delay = 200;
    const cdp = await startTouches(context, page, [1]);
    await page.click('#menu-toggle'); await page.click('#view-animations');
    await cdp.send('Input.dispatchTouchEvent', { type: 'touchEnd', touchPoints: [] });
    await until(() => state.active === false && state.inFlight === 0);
    assert.equal(state.pressed_mask, 0); assert.equal(state.posts.at(-1), '/api/view/animations');
  });

  test('Offline locks all pad controls; older firmware still permits view navigation', async t => {
    const { page, state, enter } = await fixture(t); await enter();
    state.offline = true;
    await page.locator('#pads-fade').fill('2');
    await page.waitForFunction(() => document.querySelector('.led-pad').disabled);
    assert.equal(await page.locator('#pads-view input:enabled, #pads-view button:enabled').count(), 0);
    assert.equal(await page.locator('#menu-toggle').isEnabled(), true);
    state.offline = false;
    await page.waitForFunction(() => !document.querySelector('.led-pad').disabled);
  });

  test('Reload restores the saved pad view, colors and fade setting', async t => {
    const { page, state } = await fixture(t);
    state.active = true; state.fade_ms = 1700; state.colors[3] = '#123abc';
    await page.reload();
    await page.waitForFunction(() => !document.querySelector('.led-pad').disabled);
    assert.equal(await page.locator('#pads-view').isVisible(), true);
    assert.equal(await page.locator('#settings').isVisible(), false);
    assert.equal(await page.locator('#pads-fade').inputValue(), '1.7');
    assert.equal(await page.locator('input[aria-label="LED 4 color"]').inputValue(), '#123abc');
    assert.equal(await page.locator('.led-pad[aria-pressed=true]').count(), 0);
  });

  test('The pad view previews its layout safely with older firmware', async t => {
    const { page, state, enter } = await fixture(t, { supported: false });
    await enter();
    assert.equal(await page.locator('.led-pad').count(), 8);
    assert.equal(await page.locator('#pads-view input:enabled, #pads-view button:enabled').count(), 0);
    assert.match(await page.locator('#pads-notice').textContent(), /Update firmware/);
    assert.deepEqual(state.posts, []);
  });
});
