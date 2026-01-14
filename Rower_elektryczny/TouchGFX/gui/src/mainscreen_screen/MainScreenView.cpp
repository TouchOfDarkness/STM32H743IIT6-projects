#include <gui/mainscreen_screen/MainScreenView.hpp>
#include <touchgfx/Utils.hpp>

MainScreenView::MainScreenView()
{

}

void MainScreenView::setupScreen()
{
    MainScreenViewBase::setupScreen();
}

void MainScreenView::tearDownScreen()
{
    MainScreenViewBase::tearDownScreen();
}

void MainScreenView::updateScreenValues(int toggleVal, float rampVal)
{
    // 1. Aktualizacja Toggle (0 lub 1)
    // textToggleBuffer to nazwa wygenerowana automatycznie przez Designera
    Unicode::snprintf(textToggleBuffer, TEXTTOGGLE_SIZE, "%d", toggleVal);
    // Odśwież widget
    textToggle.invalidate();

    // 2. Aktualizacja Ramp (0.00 - 1.00)
    // Uwaga: snprintf dla float (%f) często jest wyłączony w embedded.
    // Bezpieczniej jest zamienić to na liczby całkowite (np. x100)
    int rampInt = (int)(rampVal * 100);

    // Wyświetlimy jako "0.XX"
    Unicode::snprintf(textRampBuffer, TEXTRAMP_SIZE, "0.%02d", rampInt);

    // Odśwież widget
    textRamp.invalidate();
}
