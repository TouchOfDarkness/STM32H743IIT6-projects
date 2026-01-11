#ifndef LOGOSCREENPRESENTER_HPP
#define LOGOSCREENPRESENTER_HPP

#include <gui/model/ModelListener.hpp>
#include <mvp/Presenter.hpp>

using namespace touchgfx;

class LogoScreenView;

class LogoScreenPresenter : public touchgfx::Presenter, public ModelListener
{
public:
    LogoScreenPresenter(LogoScreenView& v);

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

    virtual ~LogoScreenPresenter() {}

private:
    LogoScreenPresenter();

    LogoScreenView& view;
};

#endif // LOGOSCREENPRESENTER_HPP
