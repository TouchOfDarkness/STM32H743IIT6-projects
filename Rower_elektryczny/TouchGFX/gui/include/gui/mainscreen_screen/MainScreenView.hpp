#ifndef MAINSCREENVIEW_HPP
#define MAINSCREENVIEW_HPP

#include <gui_generated/mainscreen_screen/MainScreenViewBase.hpp>
#include <gui/mainscreen_screen/MainScreenPresenter.hpp>
#include <gui/containers/CustomContainer1.hpp>

class MainScreenView : public MainScreenViewBase
{
public:
    MainScreenView();
    virtual ~MainScreenView() {}
    virtual void setupScreen();
    virtual void tearDownScreen();
    virtual void scrollListLUpdateItem(CustomContainer1& item, int16_t itemIndex);
    virtual void scrollListMUpdateItem(CustomContainer1& item, int16_t itemIndex);
    virtual void scrollListRUpdateItem(CustomContainer1& item, int16_t itemIndex);
    virtual void handleTickEvent();


protected:

private:
    int currentMotorIndex = 0;
};

#endif // MAINSCREENVIEW_HPP
