#ifndef PARAMETERSSCREENPRESENTER_HPP
#define PARAMETERSSCREENPRESENTER_HPP

#include <gui/model/ModelListener.hpp>
#include <mvp/Presenter.hpp>

using namespace touchgfx;

class ParametersScreenView;

class ParametersScreenPresenter : public touchgfx::Presenter, public ModelListener
{
public:
    ParametersScreenPresenter(ParametersScreenView& v);

    /**
     * The activate function is called automatically when this screen is "switched in"
     * (ie. made active). Initialization logic can be placed here.
     */
    virtual void activate();

    /**
     * The deactivate function is called automatically when this screen is "switched out"
     * (ie. made inactive). Teardown functionality can be placed here.
     */
    virtual void deactivate();

    virtual ~ParametersScreenPresenter() {}

private:
    ParametersScreenPresenter();

    ParametersScreenView& view;
};

#endif // PARAMETERSSCREENPRESENTER_HPP
