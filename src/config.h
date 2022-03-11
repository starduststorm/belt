#define PANEL_WIDTH 32
#define PANEL_HEIGHT 8
#define TOTAL_WIDTH (PANEL_COUNT * PANEL_WIDTH)
#define TOTAL_HEIGHT PANEL_HEIGHT

#define PANEL_LEDS (PANEL_WIDTH * PANEL_HEIGHT)
#define NUM_LEDS (PANEL_LEDS * PANEL_COUNT)

typedef struct {
  // Panel and belt dimensions in cm, specific to individual sizing
  const int frontGap = 4;
  const int panelLength = 32; // yay about 1cm per pixel
  const int backGap = 26;
  const int circumference = frontGap + 2*panelLength + backGap;
  const int panelStartX[2] = {frontGap/2, frontGap/2 + panelLength + backGap}; // measuring from front center
} PanelGeometry;
static constexpr PanelGeometry kPanelGeometry;
