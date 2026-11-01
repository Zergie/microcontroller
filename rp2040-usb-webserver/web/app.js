import { modes, fields, analogText, pinIndicator } from './gpio-model.js';
import { picoPins } from './pico-pins.js';
import { PinSession } from './pin-session.js';
import { createHttpTransport } from './gpio-transport.js';

const message = document.getElementById('message');
const actionMessage = document.getElementById('action-message');
const frequencyInput = document.getElementById('frequency');
const dutyInput = document.getElementById('duty');
const modeSelect = document.getElementById('pin-mode');
const levelSelect = document.getElementById('output-level');
const infoDialog = document.getElementById('pin-info-dialog');
const driveSelect = document.getElementById('drive-strength');
const slewSelect = document.getElementById('slew-rate');
const hysteresisSelect = document.getElementById('input-hysteresis');
const pinButtons = new Map();
const session = new PinSession(createHttpTransport(), renderSession);
let renderedPins = null;
let selectedDefinition = picoPins.led;

const controls = { level: levelSelect, frequency: frequencyInput, duty: dutyInput,
  drive: driveSelect, slew: slewSelect, hysteresis: hysteresisSelect };
for (const [value, mode] of Object.entries(modes)) {
  const option = document.createElement('option');
  option.value = value;
  option.textContent = mode.label;
  modeSelect.append(option);
}

function selectedPin() { return selectedDefinition.gpio; }

function buildEdge(container, layout) {
  for (const definition of layout) {
    const { physical, gpio, kind, label } = definition;
    const selectable = gpio !== null;
    const element = document.createElement('button');
    element.className = `pin${selectable ? '' : ` passive ${kind}`}`;
    element.dataset.physicalPin = String(physical);
    // Decorative plated through-hole and castellated solder pad; CSS mirrors
    // the right-hand strip so both PCB edges face the outside of the screen.
    const padShape = kind === 'ground' || physical === 1
      ? '<rect x="0" y="10" width="28" height="24" rx="2"/>'
      : '<path d="M0 11H17a11 11 0 0 1 0 22H0Z"/>';
    const pad = `<svg class="pin-pad" viewBox="0 0 34 44" aria-hidden="true" focusable="false">
      <path class="pad-trace" d="M24 22H34"/>
      <g class="pad-gold">${padShape}</g>
      <path class="pad-bevel" d="M1 12H17"/>
      <circle class="pad-rim" cx="14" cy="22" r="6.5"/>
      <circle class="pad-hole" cx="14" cy="22" r="4.8"/>
      <circle class="pad-castellation" cx="0" cy="22" r="4"/>
    </svg>`;
    element.innerHTML = `${pad}<span class="pin-label">${label}</span><span class="pin-state"><span class="pin-mode"></span><span class="pin-level"></span></span>`;
    element.type = 'button';
    element.dataset.selection = definition.key;
    element.setAttribute('aria-pressed', 'false');
    element.addEventListener('click', () => choosePin(definition));
    if (selectable) {
      element.type = 'button';
      element.dataset.pin = String(gpio);
      element.setAttribute('aria-label', `${label}, Pin ${physical}`);
      element.setAttribute('aria-pressed', 'false');
      pinButtons.set(gpio, element);
    } else {
      element.setAttribute('aria-label', `Pin ${physical}, ${label}, Info`);
    }
    container.append(element);
  }
}

function currentPin() {
  return session.pin(selectedPin());
}

function updateModeExplanation() {
  const section = document.getElementById('mode-explanation');
  const mode = modes[modeSelect.value];
  section.hidden = !currentPin() || !mode;
  if (section.hidden) return;
  document.getElementById('mode-explanation-title').textContent = `Mode: ${mode.label}`;
  document.getElementById('mode-explanation-description').textContent = mode.description;
  const values = session.values(selectedPin());
  const list = document.getElementById('setting-explanations');
  list.replaceChildren();
  for (const name of mode.fields) {
    const field = fields[name];
    const title = document.createElement('dt');
    title.textContent = `${field.label}: ${field.value(values, mode, currentPin())}`;
    const text = document.createElement('dd');
    text.textContent = field.explain(mode);
    list.append(title, text);
  }
}

function updateLegend() {
  const suffix = selectedDefinition.physical ? ` · Pin ${selectedDefinition.physical}` : ' · internal';
  document.getElementById('pin-info-title').textContent = selectedDefinition.label + suffix;
  const list = document.getElementById('pin-legend');
  list.replaceChildren();
  for (const { category, title, detail, supported } of selectedDefinition.legend) {
    const item = document.createElement('li');
    item.className = `legend-item legend-${category}`;
    item.dataset.category = category;
    const swatch = document.createElement('span');
    swatch.className = 'legend-color';
    swatch.setAttribute('aria-hidden', 'true');
    const text = document.createElement('div');
    text.className = 'legend-copy';
    const heading = document.createElement('strong');
    heading.textContent = title;
    const description = document.createElement('small');
    description.textContent = detail;
    text.append(heading, description);
    item.append(swatch, text);
    if (!supported) {
      const status = document.createElement('span');
      status.className = 'legend-support';
      status.textContent = 'not supported';
      item.append(status);
    }
    list.append(item);
  }
  document.getElementById('pin-info-note').textContent = selectedDefinition.note;
  const marking = document.getElementById('pin-marking');
  marking.hidden = !currentPin();
  marking.textContent = currentPin() ? `Applied state: ${pinIndicator(currentPin()).legend}` : '';
  updateModeExplanation();
}

function renderEditor(values, pin) {
  modeSelect.value = values.mode;
  for (const [field, control] of Object.entries(controls)) control.value = values[field];
  const mode = modes[values.mode];
  const available = pin && pin.available;
  const visible = name => available && mode && mode.fields.includes(name);
  for (const option of modeSelect.options) {
    if (!modes[option.value]) continue;
    const allowed = selectedPin() !== null && modes[option.value].supports(selectedPin());
    option.hidden = !allowed;
    option.disabled = !allowed;
  }
  for (const [id, field] of Object.entries({ 'level-field': 'level', 'pwm-settings': 'frequency',
    'analog-value': 'analog', 'output-advanced': 'drive', 'input-advanced': 'hysteresis' })) {
    document.getElementById(id).hidden = !visible(field);
  }
  document.getElementById('advanced-settings').hidden = !visible('drive') && !visible('hysteresis');
  (mode ? mode.levels : modes.output.levels).forEach((label, index) => { levelSelect.options[index].textContent = label; });
  document.getElementById('analog-value').textContent = analogText(pin);
}

function renderSelection() {
  document.querySelector('.editor').setAttribute('aria-label', `Configure ${selectedDefinition.label}`);
  document.querySelectorAll('[data-selection]').forEach(button => {
    button.setAttribute('aria-pressed', String(button.dataset.selection === selectedDefinition.key));
  });
  document.getElementById('led-select').setAttribute('aria-pressed', String(selectedPin() === picoPins.led.gpio));
}

function renderUI() {
  renderSelection();
  renderEditor(session.values(selectedPin()), currentPin());
  renderBusy();
  if (infoDialog.open) updateLegend();
}

function choosePin(definition) {
  selectedDefinition = definition;
  session.clearError();
  renderUI();
}

function renderBusy() {
  const { busy } = session.state;
  const available = currentPin() && currentPin().available;
  const mode = modes[modeSelect.value];
  modeSelect.disabled = busy || !available;
  for (const [field, control] of Object.entries(controls)) {
    control.disabled = busy || !available || !mode || !mode.fields.includes(field);
  }
  document.getElementById('apply-pin').disabled = busy || !available || !mode;
  document.getElementById('refresh').disabled = busy;
  document.getElementById('refresh').setAttribute('aria-busy', String(busy));
  document.querySelector('.editor').setAttribute('aria-busy', String(busy));
}

function renderBoard(pins) {
  for (const [number, button] of pinButtons) {
    const pin = pins[number];
    const indicator = pinIndicator(pin);
    button.querySelector('.pin-mode').textContent = indicator.code;
    button.querySelector('.pin-level').textContent = indicator.value;
    button.title = indicator.legend;
    button.classList.toggle('high', indicator.high);
    button.classList.toggle('pwm', indicator.pwm);
    button.setAttribute('aria-label', `GP${number}, Pin ${button.dataset.physicalPin}, ${indicator.legend}, ${indicator.status}`);
  }
  const led = pinIndicator(pins[picoPins.led.gpio]);
  document.getElementById('led-status').textContent = led.ledStatus;
  document.getElementById('led-select').classList.toggle('lit', led.lit);
}

function renderSession() {
  const state = session.state;
  // Request-start/end notifications must not rewrite editor values or repaint
  // the board. Only a newly accepted snapshot updates those views.
  if (state.pins && state.pins !== renderedPins) {
    renderedPins = state.pins;
    renderBoard(state.pins);
    if (!state.levelsOnly) renderUI();
  }
  renderBusy();
  message.textContent = { connecting: 'Connecting …', online: 'Connected', offline: 'Offline' }[state.connection];
  message.className = state.connection === 'connecting' ? 'connection' :
    `connection ${state.connection === 'online' ? 'online' : 'error'}`;
  actionMessage.textContent = state.error;
}

buildEdge(document.getElementById('left-edge'), picoPins.left);
buildEdge(document.getElementById('right-edge'), picoPins.right);
document.getElementById('led-select').addEventListener('click', () => choosePin(picoPins.led));
for (const [field, control] of Object.entries(controls)) {
  control.addEventListener('input', () => session.edit(selectedPin(), field, control.value));
}
modeSelect.addEventListener('change', () => {
  session.edit(selectedPin(), 'mode', modeSelect.value);
  renderEditor(session.values(selectedPin()), currentPin());
  renderBusy();
});

document.getElementById('pin-form').addEventListener('submit', event => {
  event.preventDefault();
  const pin = selectedPin();
  const mode = modes[modeSelect.value];
  if (session.state.busy || !currentPin() || !currentPin().available || !mode) return;
  for (const name of mode.fields) {
    const control = controls[name];
    if (control && !control.reportValidity()) return;
  }
  session.apply(pin);
});

document.getElementById('pin-info').addEventListener('click', () => { updateLegend(); infoDialog.showModal(); });
document.getElementById('close-info').addEventListener('click', () => infoDialog.close());
infoDialog.addEventListener('click', event => { if (event.target === infoDialog) infoDialog.close(); });
document.getElementById('refresh').addEventListener('click', () => session.refresh());
renderUI();
session.refresh();

async function pollInputs() {
  if (!document.hidden) await session.poll();
  setTimeout(pollInputs, 250);
}
setTimeout(pollInputs, 250);
