#ifndef LOGOSCREENVIEW_HPP
#define LOGOSCREENVIEW_HPP

#include <gui_generated/logoscreen_screen/LogoScreenViewBase.hpp>
#include <gui/logoscreen_screen/LogoScreenPresenter.hpp>

class LogoScreenView : public LogoScreenViewBase
{
public:
    LogoScreenView();
    virtual ~LogoScreenView() {}
    virtual void setupScreen();
    virtual void tearDownScreen();
protected:
};

#endif // LOGOSCREENVIEW_HPP
