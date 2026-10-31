import { createPadsController } from './pads.js';
import { createStatisticsController } from './statistics.js';

const $ = id => document.getElementById(id);
const form = $('settings');
const fields = ['animation', 'brightness', 'speed', 'interval', 'color', 'color-mode'];
const title = id => id.split('-').map(word => word[0].toUpperCase() + word.slice(1)).join(' ');
let busy = false;
let loaded = false;
let dirty = false;
let online = false;
let pending = false;
let sendTimer;
let appliedPath;
let supportsRandomColor = false;
let supportsPads = false;
let currentView = 'animations';
let latestStatus;
let bootId;
let reloading = false;
const pads = createPadsController({ onOffline() {
  online = false; form.dataset.online = 'false'; updateControls();
} });
const statistics = createStatisticsController();
function closeMenu() {
  $('view-menu').hidden = true;
  $('menu-toggle').setAttribute('aria-expanded', 'false');
}
function showView(view, notify = true) {
  currentView = view;
  const manual = view === 'pads' || view === 'songs';
  const stats = view === 'statistics';
  const songView = view === 'songs';
  form.hidden = manual || stats;
  $('statistics-view').hidden = !stats;
  statistics.setVisible(stats);
  $('pads-view').hidden = !manual;
  document.querySelector('main').classList.toggle('pads-active', view === 'pads');
  document.documentElement.classList.toggle('pads-active', view === 'pads');
  document.querySelector('main').classList.toggle('songs-active', songView);
  $('songs-controls').hidden = !songView;
  for (const [id, selected] of [['view-statistics', stats], ['view-animations', view === 'animations'], ['view-pads', view === 'pads'], ['view-songs', songView]]) {
    if (selected) $(id).setAttribute('aria-current', 'page');
    else $(id).removeAttribute('aria-current');
  }
  if (manual) {
    $('current').textContent = songView ? 'Songs' : 'LED Pads';
  } else if (stats) $('current').textContent = 'Statistics';
  else if (latestStatus) showPlaying(latestStatus);
  pads.setView(stats ? 'animations' : view, notify);
  closeMenu();
}
function showPlaying(data) {
  if (currentView !== 'animations') return;
  $('current').textContent = title(data.current);
}
$('menu-toggle').addEventListener('click', () => {
  const expanded = $('view-menu').hidden;
  $('view-menu').hidden = !expanded;
  $('menu-toggle').setAttribute('aria-expanded', String(expanded));
});
$('view-animations').addEventListener('click', () => showView('animations'));
$('view-pads').addEventListener('click', () => showView('pads'));
$('view-songs').addEventListener('click', () => showView('songs'));
$('view-statistics').addEventListener('click', () => showView('statistics'));
document.addEventListener('click', event => {
  if (!event.target.closest('#view-menu, #menu-toggle')) closeMenu();
});
document.addEventListener('keydown', event => {
  if (event.key === 'Escape' && !$('view-menu').hidden) { closeMenu(); $('menu-toggle').focus(); }
});


function pathFor(selection, brightness, speed, interval, color, mode = 'custom') {
  return ['/api/animation/set', selection, brightness, speed,
    selection === 'random' ? interval : '300', color.slice(1)].join('/') + (mode === 'custom' ? '' : `/${mode}`);
}

function settingsPath() {
  return pathFor($('animation').value, $('brightness').value, $('speed').value,
    $('interval').value, $('color').value, $('color-mode').value);
}

function flushSettings() {
  if (busy || !online || !loaded || !pending) return;
  pending = false;
  if (!form.checkValidity()) return;
  const path = settingsPath();
  if (path === appliedPath) { dirty = false; return; }
  void request(path, 'POST');
}

function scheduleSettings(delay) {
  dirty = true;
  pending = true;
  showFields();
  clearTimeout(sendTimer);
  sendTimer = setTimeout(() => { sendTimer = undefined; flushSettings(); }, delay);
}

function updateControls() {
  for (const id of fields) {
    $(id).disabled = !online || (id === 'interval' && $('interval-field').hidden) ||
      (id === 'color' && $('color-mode').value !== 'custom');
  }
  $('color-mode').querySelector('[value="random"]').disabled = !supportsRandomColor;
  pads.setConnection(online, supportsPads, latestStatus?.pads_transport);
  $('pads-brightness').disabled = !online || !supportsPads;
  statistics.setConnection(online, latestStatus);
}

function showFields() {
  $('interval-field').hidden = $('animation').value !== 'random';
  $('color').hidden = $('color-mode').value !== 'custom';
  $('color-hint').hidden = $('color-mode').value !== 'random';
  // A hidden interval must not prevent submitting a single effect.
  updateControls();
  $('brightness-value').value = `${$('brightness').value}%`;
  $('pads-brightness').value = $('brightness').value;
  $('pads-brightness-value').value = `${$('brightness').value}%`;
  $('speed-value').value = `${$('speed').value}%`;
}

function showStatus(data, acknowledge = false) {
  latestStatus = data;
  supportsPads = data.pads_supported === true;
  supportsRandomColor = typeof data.color_mode === 'string';
  if (!loaded) {
    for (const effect of data.effects) $('animation').add(new Option(title(effect), effect));
  }
  if (!loaded || acknowledge || !dirty) {
    $('animation').value = data.selection;
    $('brightness').value = data.brightness_percent;
    $('speed').value = data.speed_percent;
    $('interval').value = data.interval_seconds;
    $('color').value = data.color;
    $('color-mode').value = data.color_mode || (data.original_colors ? 'original' : 'custom');
    dirty = false;
    showFields();
  }
  if (!loaded) showView(data.control_view === 'pads' ? 'pads' : 'animations', false);
  loaded = true;
  showPlaying(data);
  $('storage-status').textContent = data.settings_error ? 'Settings could not be saved.' : 'Saving…';
  $('storage-status').hidden = !data.settings_error && data.settings_saving !== true;
}

async function request(path, method) {
  if (busy || reloading) return;
  busy = true;
  updateControls();
  const controller = new AbortController();
  const timeout = setTimeout(() => controller.abort(), 6000);
  try {
    const response = await fetch(path, { method, cache: 'no-store', signal: controller.signal,
      ...(method === 'POST' ? { body: '' } : {}) });
    if (!response.ok) {
      const error = new Error(response.status === 400 ? 'Invalid settings.' : 'Board is unreachable. Check USB.');
      error.status = response.status;
      throw error;
    }
    const data = await response.json();
    if (typeof data.boot_id === 'string' && data.boot_id) {
      if (bootId && bootId !== data.boot_id) {
        // Stop audio, held pads and queued edits before replacing the old page.
        reloading = true; online = false; pending = false;
        clearTimeout(sendTimer); sendTimer = undefined;
        form.dataset.online = 'false';
        updateControls();
        location.reload();
        return;
      }
      bootId = data.boot_id;
    }
    appliedPath = pathFor(data.selection, data.brightness_percent, data.speed_percent,
      data.interval_seconds, data.color, data.color_mode || (data.original_colors ? 'original' : 'custom'));
    // An older response must not overwrite a newer edit made during transfer.
    showStatus(data, method === 'POST' && appliedPath === settingsPath());
    online = true;
    form.dataset.online = 'true';
    $('error').hidden = true;
  } catch (error) {
    // A rejected command still proves the board is reachable. Transport and
    // server failures lock the editor until a successful automatic refresh.
    online = loaded && error.status >= 400 && error.status < 500;
    if (method === 'POST' && !online) pending = true;
    form.dataset.online = String(online);
    $('error').textContent = error.name === 'AbortError' ? 'No response. Check USB.' : error.message;
    $('error').hidden = false;
  } finally {
    clearTimeout(timeout);
    busy = false;
    updateControls();
    if (online && pending && !sendTimer) queueMicrotask(flushSettings);
  }
}

// Changes commit automatically. Continuous input is coalesced; a committed
// change (selection, slider release, color confirmation) flushes immediately.
form.addEventListener('input', () => scheduleSettings(200));
form.addEventListener('change', event => {
  scheduleSettings(0);
  if (event.target === $('color-mode') && $('color-mode').value === 'custom' && online) {
    // Keep this synchronous with the user's choice: showPicker requires
    // transient user activation. Older browsers retain the swatch click path.
    const picker = $('color');
    try {
      if (typeof picker.showPicker === 'function') picker.showPicker();
      else picker.click();
    } catch {
      picker.click();
    }
  }
});
form.addEventListener('submit', event => event.preventDefault());

// Both views edit the same persisted brightness through the existing queue.
for (const type of ['input', 'change']) {
  $('pads-brightness').addEventListener(type, () => {
    $('brightness').value = $('pads-brightness').value;
    scheduleSettings(type === 'input' ? 200 : 0);
  });
}

showFields();
await request('/api/animation.json', 'GET');
setInterval(() => { if (!document.hidden) void request('/api/animation.json', 'GET'); }, 2000);
document.addEventListener('visibilitychange', () => {
  if (!document.hidden) void request('/api/animation.json', 'GET');
});
window.addEventListener('pageshow', event => {
  if (event.persisted) void request('/api/animation.json', 'GET');
});
