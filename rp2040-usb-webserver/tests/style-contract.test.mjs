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

for (const [width, height] of [[375, 667], [375, 449], [699, 800], [700, 800]]) {
  test(`Graphite, PCB and control style contract at ${width}x${height}`, async t => {
    const context = await browser.newContext({ viewport: { width, height }, reducedMotion: 'reduce' });
    t.after(() => context.close());
    const page = await context.newPage();
    const pins = Array.from({ length: 30 }, (_, pin) => ({ pin, available: ![23, 24, 29].includes(pin), reason: '',
      mode: pin === 25 || pin === 2 ? 'output' : pin === 3 ? 'pwm' : 'input', pull: 'none', value: pin === 25 || pin === 2,
      frequency_hz: 1000, actual_frequency_hz: 1000, duty_percent: 50, drive_strength_ma: 4, slew_rate: 'slow', hysteresis: true }));
    await page.route('**/api/**', async route => {
      if (route.request().method() === 'POST') await new Promise(resolve => setTimeout(resolve, 200));
      await route.fulfill({ json: { pins } });
    });
    await page.goto(base);
    await page.waitForFunction(() => document.querySelector('#message').textContent === 'Connected');
    const style = (selector, properties, pseudo = null) => page.locator(selector).evaluate((el, { properties, pseudo }) => {
      const computed = getComputedStyle(el, pseudo);
      return Object.fromEntries(properties.map(property => [property, computed.getPropertyValue(property)]));
    }, { properties, pseudo });
    assert.deepEqual(await style('#pin-mode', ['height', 'border-radius', 'padding-left', 'padding-right', 'font-size', 'font-weight', 'background-color', 'color']), {
      height: '42px', 'border-radius': '12px', 'padding-left': '4px', 'padding-right': '12px',
      'font-size': '16px', 'font-weight': '400', 'background-color': 'rgb(223, 226, 230)', color: 'rgb(48, 54, 64)'
    });
    assert.deepEqual(await style('#apply-pin', ['height', 'border-radius', 'font-size', 'font-weight', 'background-color']), {
      height: '44px', 'border-radius': '12px', 'font-size': '16px', 'font-weight': '400', 'background-color': 'rgb(223, 226, 230)'
    });
    assert.equal((await style('.app', ['max-width']))['max-width'], width >= 700 ? '540px' : '560px');
    assert.equal((await style('.pin-section', ['padding-top']))['padding-top'], '12px');
    assert.equal((await style('.topbar', ['min-height']))['min-height'], width >= 700 ? '64px' : '56px');
    assert.equal((await style('.editor', ['position'])).position, height <= 450 ? 'relative' : 'static');
    assert.equal((await style('.page-scroll', ['overflow-y']))['overflow-y'], 'auto');
    assert.equal((await style('.topbar', ['position'])).position, 'static');
    assert.equal((await style('.pinout', ['--line']))['--line'], '#29404b');
    assert.match((await style('.pinout', ['background-image'], '::before'))['background-image'], /rgb\(57, 125, 90\)/);
    assert.equal((await style('[data-selection="pin-3"] .pin-state', ['display'])).display, 'none');
    assert.equal((await style('[data-pin="2"] .pin-state', ['font-weight']))['font-weight'], '700');
    assert.equal((await style('[data-pin="3"] .pin-state', ['font-weight']))['font-weight'], '700');
    await page.locator('[data-pin="2"]').click();
    assert.match((await style('[data-pin="2"]', ['background-color']))['background-color'], /83, 134, 100/);
    await page.keyboard.press('Tab');
    await page.locator('#pin-mode').focus();
    assert.deepEqual(await style('#pin-mode', ['outline-color', 'outline-offset']), { 'outline-color': 'rgb(225, 228, 233)', 'outline-offset': '3px' });
    const applyStyle = await style('#apply-pin', ['opacity', 'height', 'border-radius']);
    await page.locator('#apply-pin').click();
    assert.deepEqual(await style('#apply-pin', ['opacity', 'height', 'border-radius']), applyStyle);
    assert.equal(await page.locator('#pin-mode').isDisabled(), true);
    assert.equal((await style('#pin-mode', ['opacity'])).opacity, '1');
    await page.waitForFunction(() => document.querySelector('.editor').getAttribute('aria-busy') === 'false');
    await page.locator('#pin-info').click();
    assert.deepEqual(await style('#pin-info-dialog', ['background-color', 'border-top-left-radius']), { 'background-color': 'rgb(43, 47, 54)', 'border-top-left-radius': '20px' });
    assert.equal((await style('.info-heading', ['position'])).position, 'sticky');
    await page.locator('#close-info').click();
    await page.locator('[data-selection="pin-3"]').click();
    assert.equal(await page.locator('#apply-pin').isDisabled(), true);
    assert.equal((await style('#apply-pin', ['opacity'])).opacity, '0.45');
    assert.equal((await style('#pin-mode', ['opacity'])).opacity, '0.5');
    if (height > 450) {
      const editorTop = (await page.locator('.editor').boundingBox()).y;
      const headerTop = (await page.locator('.topbar').boundingBox()).y;
      await page.locator('.page-scroll').evaluate(el => { el.scrollTop = 100; });
      assert.equal((await page.locator('.editor').boundingBox()).y, editorTop);
      assert.ok((await page.locator('.topbar').boundingBox()).y < headerTop);
    }
  });
}
