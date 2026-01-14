#ifndef PARAMETERSSCREENVIEW_HPP
#define PARAMETERSSCREENVIEW_HPP

#include <gui_generated/parametersscreen_screen/ParametersScreenViewBase.hpp>
#include <gui/parametersscreen_screen/ParametersScreenPresenter.hpp>

class ParametersScreenView : public ParametersScreenViewBase
{
public:
    ParametersScreenView();
    virtual ~ParametersScreenView() {}
    virtual void setupScreen();
    virtual void tearDownScreen();
protected:
};

#endif // PARAMETERSSCREENVIEW_HPP
