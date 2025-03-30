#include <Adafruit_GFX.h>
#include <Adafruit_NeoMatrix.h>
#include <Adafruit_NeoPixel.h>

#define PIN 6 // Pin de datos para la matriz

// Definir la matriz 8x8
Adafruit_NeoMatrix matrix = Adafruit_NeoMatrix(8, 8, PIN, 
  NEO_MATRIX_TOP + NEO_MATRIX_LEFT + NEO_MATRIX_ROWS + NEO_MATRIX_ZIGZAG,
  NEO_GRB + NEO_KHZ800);

// Definir las matrices para las figuras
const uint8_t caritaFeliz[8][8] = {
  {0, 0, 1, 1, 1, 1, 0, 0},
  {0, 1, 0, 0, 0, 0, 1, 0},
  {1, 0, 1, 0, 0, 1, 0, 1},
  {1, 0, 0, 0, 0, 0, 0, 1},
  {1, 0, 1, 0, 0, 1, 0, 1},
  {1, 0, 0, 1, 1, 0, 0, 1},
  {0, 1, 0, 0, 0, 0, 1, 0},
  {0, 0, 1, 1, 1, 1, 0, 0}
};

void setup() {
  matrix.begin();
  matrix.setBrightness(10);
  matrix.fillScreen(0);
  matrix.show();
}

void loop() {
  dibujarFigura(caritaFeliz, matrix.Color(255, 165, 86)); // Amarillo
  delay(2000);
}

void dibujarFigura(const uint8_t figura[8][8], uint16_t color) {
  matrix.fillScreen(0);
  for (int y = 0; y < 8; y++) {
    for (int x = 0; x < 8; x++) {
      if (figura[y][x]) {
        matrix.drawPixel(x, y, color);
      }
    }
  }
  matrix.show();
}
