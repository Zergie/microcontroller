// Standard Pico only, viewed from above with USB at the top.
const left = [
  [1, 0], [2, 1], [3, 'GND', 'ground'], [4, 2], [5, 3], [6, 4], [7, 5],
  [8, 'GND', 'ground'], [9, 6], [10, 7], [11, 8], [12, 9],
  [13, 'GND', 'ground'], [14, 10], [15, 11], [16, 12], [17, 13],
  [18, 'GND', 'ground'], [19, 14], [20, 15]
];
const right = [
  [40, 'VBUS', 'power'], [39, 'VSYS', 'power'], [38, 'GND', 'ground'],
  [37, '3V3_EN', 'control'], [36, '3V3', 'power'], [35, 'ADC_REF', 'power'],
  [34, 28], [33, 'AGND', 'ground'], [32, 27], [31, 26], [30, 'RUN', 'control'],
  [29, 22], [28, 'GND', 'ground'], [27, 21], [26, 20], [25, 19], [24, 18],
  [23, 'GND', 'ground'], [22, 17], [21, 16]
];

function describe(physical, pin, kind) {
  const gpio = typeof pin === 'number' ? pin : null;
  const label = gpio === null ? pin : `GP${gpio}`;
  const adc = gpio >= 26 && gpio <= 28 ? gpio - 26 : null;
  const internalLed = gpio === 25;
  let legend;
  if (gpio !== null) {
    // RP2040 function-select table; capability is distinct from firmware support.
    const uart = [0, 1, 1, 0, 0, 1, 1, 0][Math.floor(gpio / 4)];
    legend = [
      ['gpio', 'GPIO · PWM', `${label} · PWM ${Math.floor(gpio % 16 / 2)}${gpio % 2 ? 'B' : 'A'}`],
      ['pio', 'PIO', 'PIO0 / PIO1', false],
      ['uart', 'UART', `UART${uart} ${['TX', 'RX', 'CTS', 'RTS'][gpio % 4]}`, false],
      ['spi', 'SPI', `SPI${Math.floor(gpio / 8) % 2} ${['RX', 'CSn', 'SCK', 'TX'][gpio % 4]}`, false],
      ['i2c', 'I²C', `I2C${Math.floor(gpio / 2) % 2} ${gpio % 2 ? 'SCL' : 'SDA'}`, false]
    ];
    if (adc !== null) legend.push(['adc', 'ADC', `ADC${adc} · Analog input`]);
    if (internalLed) legend.push(['led', 'Onboard LED', 'Connected internally to GP25']);
  } else if (kind === 'ground') {
    legend = [['ground', 'Ground', label === 'AGND' ? 'AGND · Analog ground' : 'GND · Reference potential']];
  } else {
    const functions = {
      VBUS: ['power', 'Power', 'VBUS · USB power'],
      VSYS: ['power', 'Power', 'VSYS · System power'],
      '3V3': ['power', 'Power', '3V3(OUT) · 3.3 V output'],
      '3V3_EN': ['system', 'System Control', '3V3_EN · Voltage regulator enable'],
      RUN: ['system', 'System Control', 'RUN · Reset input'],
      ADC_REF: ['adc', 'ADC', 'ADC_VREF · Reference voltage']
    };
    legend = [functions[label]];
  }
  return Object.freeze({
    key: gpio === null ? `pin-${physical}` : `gpio-${gpio}`,
    physical, gpio, label, kind: gpio === null ? kind : 'gpio', adc,
    note: internalLed ? 'GP25 is connected internally to the LED, not to the pin header.' : '',
    legend: Object.freeze(legend.map(([category, title, detail, supported = true]) =>
      Object.freeze({ category, title, detail, supported })))
  });
}

const leftPins = Object.freeze(left.map(pin => describe(...pin)));
const rightPins = Object.freeze(right.map(pin => describe(...pin)));
const led = describe(null, 25);
const gpioPins = new Map([...leftPins, ...rightPins, led]
  .filter(pin => pin.gpio !== null).map(pin => [pin.gpio, pin]));

export const picoPins = Object.freeze({
  left: leftPins, right: rightPins, led,
  gpio: number => gpioPins.get(number)
});
