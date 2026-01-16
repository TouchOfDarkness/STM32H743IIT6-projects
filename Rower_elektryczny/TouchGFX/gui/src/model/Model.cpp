#include <gui/model/Model.hpp>
#include <gui/model/ModelListener.hpp>



Model::Model() : modelListener(0)
{

}

void Model::tick()
{



}

VescData_t Model::getVescData(int controllerIndex)
{
    switch(controllerIndex) {
        case 0: return vescL;
        case 1: return vescM;
        case 2: return vescR;
        default: return vescL; // Zabezpieczenie
    }
}
float Model::getVehicleSpeed()
{
    return vehicleSpeedKmh;
}
