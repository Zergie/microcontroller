import { editorFromPin } from './gpio-model.js';

export class PinStore {
  pins = null;
  drafts = new Map();

  pin(number) { return number !== null && this.pins ? this.pins[number] : null; }

  values(number) {
    return { ...editorFromPin(this.pin(number)), ...this.drafts.get(number) };
  }

  edit(number, field, value) {
    if (!this.pin(number)) return;
    const draft = { ...this.drafts.get(number), [field]: value };
    if (editorFromPin(this.pin(number))[field] === value) delete draft[field];
    if (Object.keys(draft).length) this.drafts.set(number, draft);
    else this.drafts.delete(number);
  }

  receive(data, appliedPin = null) {
    if (!Array.isArray(data.pins) || data.pins.length !== 30 ||
        data.pins.some((pin, index) => pin.pin !== index)) throw new Error('Invalid GPIO status.');
    this.pins = data.pins;
    // Only the submitted pin is acknowledged; drafts for other pins survive.
    if (appliedPin !== null) this.drafts.delete(appliedPin);
  }
}
