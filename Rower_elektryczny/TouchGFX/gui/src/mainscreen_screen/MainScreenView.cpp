#include <gui/mainscreen_screen/MainScreenView.hpp>
#include <touchgfx/Utils.hpp>
#include "main.h"

static const int NUMBER_OF_PARAMS = 13;

MainScreenView::MainScreenView()
{

}

void MainScreenView::setupScreen()
{
    MainScreenViewBase::setupScreen();
    scrollListL.setNumberOfItems(NUMBER_OF_PARAMS);
    scrollListM.setNumberOfItems(NUMBER_OF_PARAMS);
    scrollListR.setNumberOfItems(NUMBER_OF_PARAMS);
}

void MainScreenView::tearDownScreen()
{
    MainScreenViewBase::tearDownScreen();
}
static void updateItemCommon(CustomContainer1& item, int16_t itemIndex, VescData_t data)
{
    switch(itemIndex)
    {
        case 0: item.updateElementInt("RPM", data.rpm); break;
        case 1: item.updateElementFloat("Prad Silnika", data.current_motor, 1); break;
        case 2: item.updateElementFloat("Wypelnienie", data.duty_cycle, 1); break;
        case 3: item.updateElementFloat("Ah", data.amp_hours, 3); break;
        case 4: item.updateElementFloat("Ah Lad.", data.amp_hours_chg, 3); break;
        case 5: item.updateElementFloat("Wh", data.watt_hours, 1); break;
        case 6: item.updateElementFloat("Wh Lad.", data.watt_hours_chg, 1); break;
        case 7: item.updateElementFloat("Temp FET", data.temp_fet, 0); break;
        case 8: item.updateElementFloat("Temp Motor", data.temp_motor, 0); break;
        case 9: item.updateElementFloat("Prad Wej.", data.current_in, 1); break;
        case 10: item.updateElementFloat("PID Pos", data.pid_pos, 2); break;
        case 11: item.updateElementInt("Tacho", data.tacho_value); break;
        case 12: item.updateElementFloat("Napiecie Wej.", data.v_in, 1); break;
        default: item.updateElementInt("???", 0); break;
    }
}

// --- Callback dla Listy L (Lewej - index 0) ---
void MainScreenView::scrollListLUpdateItem(CustomContainer1& item, int16_t itemIndex)
{
    // Pobierz dane dla silnika 0
    VescData_t data = presenter->getVescData(0);
    updateItemCommon(item, itemIndex, data);
}

// --- Callback dla Listy M (Środkowej - index 1) ---
void MainScreenView::scrollListMUpdateItem(CustomContainer1& item, int16_t itemIndex)
{
    // Pobierz dane dla silnika 1
    VescData_t data = presenter->getVescData(1);
    updateItemCommon(item, itemIndex, data);
}

// --- Callback dla Listy R (Prawej - index 2) ---
void MainScreenView::scrollListRUpdateItem(CustomContainer1& item, int16_t itemIndex)
{
    // Pobierz dane dla silnika 2
    VescData_t data = presenter->getVescData(2);
    updateItemCommon(item, itemIndex, data);
}

void MainScreenView::handleTickEvent()
{
    // Wymuś przerysowanie elementów listy (nie jest to super wydajne dla dużych list, ale dla testu OK)
    // Lepszą metodą jest sprawdzanie czy wartość się zmieniła w Modelu
	for (int i = 0; i < NUMBER_OF_PARAMS; i++)
	    {
	        scrollListL.itemChanged(i);
	        scrollListM.itemChanged(i);
	        scrollListR.itemChanged(i);
	    }
	// 1. Obsługa listy (to co miałeś wcześniej)
	    static int tickCounter = 0;
	    tickCounter++;
	    if (tickCounter >= 10)
	    {
	        // ... odświeżanie listy ...
	        tickCounter = 0;
	    }

	    // 2. Obsługa Zegara (Gauge)
	    // Pobieramy prędkość (float)
	    float speed = presenter->getSpeed();
	    Unicode::snprintf(VelocityTextBuffer, VELOCITYTEXT_SIZE, "%d", (int)speed);
	    VelocityText.invalidate();

}
