import { test } from 'node:test';
import assert from 'node:assert/strict';
import { picoPins } from '../web/pico-pins.js';
import { modes } from '../web/gpio-model.js';

// Physical Pico layout is independent of the descriptions under test.
const left = [0, 1, 'GND', 2, 3, 4, 5, 'GND', 6, 7, 8, 9, 'GND', 10, 11, 12, 13, 'GND', 14, 15];
const right = ['VBUS', 'VSYS', 'GND', '3V3_EN', '3V3', 'ADC_REF', 28, 'AGND', 27, 26,
  'RUN', 22, 'GND', 21, 20, 19, 18, 'GND', 17, 16];

test('All 40 physical positions have the correct identity, order and GPIO mapping', () => {
  for (const [descriptions, expected, physical] of [
    [picoPins.left, left, index => index + 1],
    [picoPins.right, right, index => 40 - index]
  ]) {
    assert.equal(descriptions.length, 20);
    descriptions.forEach((pin, index) => {
      const gpio = typeof expected[index] === 'number' ? expected[index] : null;
      assert.equal(pin.physical, physical(index));
      assert.equal(pin.gpio, gpio);
      assert.equal(pin.label, gpio === null ? expected[index] : `GP${gpio}`);
      assert.equal(pin.key, gpio === null ? `pin-${pin.physical}` : `gpio-${gpio}`);
      assert.ok(pin.legend.length > 0);
      if (gpio !== null) assert.equal(picoPins.gpio(gpio), pin);
      else assert.equal(pin.legend.length, 1);
    });
  }
  assert.equal(new Set([...picoPins.left, ...picoPins.right].map(pin => pin.physical)).size, 40);
});

test('ADC, internal LED and reserved GPIO facts are shared with mode eligibility', () => {
  const supported = Array.from({ length: 30 }, (_, pin) => pin).filter(pin => picoPins.gpio(pin));
  assert.deepEqual(supported, [...Array.from({ length: 23 }, (_, pin) => pin), 25, 26, 27, 28]);
  assert.equal(picoPins.led, picoPins.gpio(25));
  assert.equal(picoPins.led.physical, null);
  assert.match(picoPins.led.note, /internally.*not.*pin header/);
  for (let number = -1; number <= 30; number++) {
    const pin = picoPins.gpio(number);
    const analog = number >= 26 && number <= 28;
    assert.equal(modes.analog.supports(number), analog);
    if (!pin) continue;
    assert.equal(pin.adc, analog ? number - 26 : null);
    assert.equal(pin.legend.some(entry => entry.category === 'adc'), analog);
    assert.equal(pin.legend.some(entry => entry.category === 'led'), number === 25);
    assert.equal(Boolean(pin.note), number === 25);
  }
  for (const invalid of [null, undefined, '26', NaN]) assert.equal(modes.analog.supports(invalid), false);
});

test('Every GPIO exposes the correct peripheral signals without claiming firmware support', () => {
  const uartAssignments = ['UART0 TX', 'UART0 RX', 'UART0 CTS', 'UART0 RTS',
    'UART1 TX', 'UART1 RX', 'UART1 CTS', 'UART1 RTS',
    'UART1 TX', 'UART1 RX', 'UART1 CTS', 'UART1 RTS',
    'UART0 TX', 'UART0 RX', 'UART0 CTS', 'UART0 RTS',
    'UART0 TX', 'UART0 RX', 'UART0 CTS', 'UART0 RTS',
    'UART1 TX', 'UART1 RX', 'UART1 CTS', 'UART1 RTS',
    'UART1 TX', 'UART1 RX', 'UART1 CTS', 'UART1 RTS', 'UART0 TX'];
  for (const pin of [...picoPins.left, ...picoPins.right, picoPins.led].filter(pin => pin.gpio !== null)) {
    const entries = Object.fromEntries(pin.legend.map(entry => [entry.category, entry]));
    assert.equal(entries.uart.detail, uartAssignments[pin.gpio]);
    assert.equal(entries.pio.detail, 'PIO0 / PIO1');
    assert.equal(entries.spi.detail, `SPI${Math.floor(pin.gpio / 8) % 2} ${['RX', 'CSn', 'SCK', 'TX'][pin.gpio % 4]}`);
    assert.equal(entries.i2c.detail, `I2C${Math.floor(pin.gpio / 2) % 2} ${pin.gpio % 2 ? 'SCL' : 'SDA'}`);
    for (const category of ['pio', 'uart', 'spi', 'i2c']) assert.equal(entries[category].supported, false);
    assert.equal(entries.gpio.supported, true);
  }
  const passive = [...picoPins.left, ...picoPins.right].filter(pin => pin.gpio === null);
  assert.equal(passive.filter(pin => pin.kind === 'ground').length, 8);
  assert.match(passive.find(pin => pin.label === 'AGND').legend[0].detail, /Analog ground/);
  assert.match(passive.find(pin => pin.label === 'RUN').legend[0].detail, /Reset input/);
  assert.match(passive.find(pin => pin.label === 'ADC_REF').legend[0].detail, /Reference voltage/);
  assert.ok(Object.isFrozen(picoPins.left) && Object.isFrozen(picoPins.led.legend[0]));
});
