#include <gui/model/Model.hpp>
#include <gui/model/ModelListener.hpp>

extern volatile int test_toggle_var;
extern volatile float test_ramp_var;

Model::Model() : modelListener(0)
{

}

void Model::tick()
{
	if (modelListener != 0)
	    {
	        modelListener->updateTestValues(test_toggle_var, test_ramp_var);
	    }


}
