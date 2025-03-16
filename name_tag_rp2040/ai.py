import board
import neopixel
import ulab.numpy as np
import random
import time

# Anzahl der NeoPixel
NUM_PIXELS = 10
# Helligkeit
BRIGHTNESS = 0.2
# Intervall für zufällige Neugenerierung (in Sekunden)
RESET_INTERVAL = 300

# Initialisiere NeoPixel
pixels = neopixel.NeoPixel(board.GP24, NUM_PIXELS, brightness=BRIGHTNESS, auto_write=False)

# Funktion zur Initialisierung der Gewichte
def init_weights():
    W1 = np.array([[random.uniform(-1, 1) for _ in range(16)] for _ in range(4)])  # 4 Eingaben -> 16 Neuronen
    W2 = np.array([[random.uniform(-1, 1) for _ in range(NUM_PIXELS * 3)] for _ in range(16)])  # 16 -> 8*3 (RGB)
    return W1, W2

# Initialisierung der ersten Gewichte
W1, W2 = init_weights()

# Aktivierungsfunktion (ReLU)
def relu(x):
    return np.maximum(x, 0)

# Funktion zum Generieren von Farben
def generate_colors(step, W1, W2):
    # Erstelle Eingabe: Takt, Sinus-Takt, Zufallszahl, Modulo-Wert
    input_vector = np.array([
        step % 100 / 100,        # Normierter Taktwert (0-1)
        np.sin(step / 20),       # Sinus für weiche Übergänge
        random.uniform(-1, 1),   # Zufallswert für Variabilität
        (step % 50) / 50         # Zweiter normierter Wert
    ])

    # Forme input_vector um (1x4 Matrix)
    input_vector = np.array([[input_vector[0], input_vector[1], input_vector[2], input_vector[3]]])  # Explizite 2D-Form

    # Matrix-Multiplikation & ReLU
    hidden = relu(np.dot(input_vector, W1))  

    # Ausgabe mit tanh
    output = np.tanh(np.dot(hidden, W2))  

    # Konvertiere in RGB-Werte (0-255)
    rgb_values = np.array(((output + 1) / 2 * 255), dtype=np.uint8)  # Explizite Typumwandlung
    return [[rgb_values[i*3], rgb_values[i*3+1], rgb_values[i*3+2]] for i in range(len(rgb_values) // 3)]


# Hauptloop
step = 0
last_reset_time = time.monotonic()

while True:
    colors = generate_colors(step, W1, W2)
    print(len(W1))
    print(len(W2))
    print(colors)
    for i in range(NUM_PIXELS):
        pixels[i] = tuple(colors[i])
    
    pixels.show()
    step += 1
    time.sleep(0.1)  # 10 FPS

    # Prüfe, ob 5 Minuten vergangen sind
    if time.monotonic() - last_reset_time > RESET_INTERVAL:
        print("Resetting neural network weights...")
        W1, W2 = init_weights()  # Neue Zufallsgewichte
        last_reset_time = time.monotonic()  # Reset Timer
