import board
import neopixel
import usb_midi
import adafruit_midi

from adafruit_led_animation.color import *
from adafruit_midi.note_on import NoteOn
from adafruit_midi.note_off import NoteOff

BLACK = (0,0,0)

pixels = neopixel.NeoPixel(board.GP24, 10)
pixels.brightness = 0.5

midi_in = adafruit_midi.MIDI(midi_in=usb_midi.ports[0])
pixels.fill(BLACK)
pixels.show()

colors = len(RAINBOW)

while True:
    msg = midi_in.receive()
    if msg:
        if isinstance(msg, NoteOff):
            pixels[msg.note % 8] = BLACK
        elif isinstance(msg, NoteOn):
            pixels[msg.note % 8] = RAINBOW[msg.note % colors]
        pixels.show()
