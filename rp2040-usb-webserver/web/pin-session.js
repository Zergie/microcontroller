import { commandFor } from './gpio-model.js';
import { PinStore } from './pin-store.js';

// Owns one device's snapshots, drafts and request lifecycle, independently of DOM.
// request(path, method) returns parsed JSON or rejects with Error (+ HTTP status).
// changed() observes request/error state changes; pin()/values() are read-only views.
export class PinSession {
  #store = new PinStore();
  #request;
  #changed;
  #busy = false;
  #connection = 'connecting';
  #error = '';

  constructor(request, changed = () => {}) {
    this.#request = request;
    this.#changed = changed;
  }

  get state() {
    return { pins: this.#store.pins, busy: this.#busy,
      connection: this.#connection, error: this.#error };
  }

  pin(number) { return this.#store.pin(number); }
  values(number) { return this.#store.values(number); }
  edit(number, field, value) { this.#store.edit(number, field, value); }

  clearError() {
    this.#error = '';
    this.#changed();
  }

  refresh() { return this.#perform(null); }

  apply(number) {
    if (!this.pin(number)?.available) return Promise.resolve(false);
    return this.#perform(number);
  }

  async #perform(appliedPin) {
    if (this.#busy) return false;
    this.#busy = true;
    this.#error = '';
    this.#changed();
    try {
      const path = appliedPin === null ? '/api/gpio.json' : commandFor(appliedPin, this.values(appliedPin));
      const data = await this.#request(path, appliedPin === null ? 'GET' : 'POST');
      // Legacy command responses do not contain a snapshot. Acknowledge drafts
      // only once a complete, valid GPIO snapshot has also been received.
      this.#store.receive(Array.isArray(data.pins) ? data : await this.#request('/api/gpio.json', 'GET'), appliedPin);
      this.#connection = 'online';
      return true;
    } catch (error) {
      this.#connection = error.status && error.status < 500 ? 'online' : 'offline';
      this.#error = error.name === 'AbortError' ? 'No response. Check USB.' : error.message;
      return false;
    } finally {
      this.#busy = false;
      this.#changed();
    }
  }
}
