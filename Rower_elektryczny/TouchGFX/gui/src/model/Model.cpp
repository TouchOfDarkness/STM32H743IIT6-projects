#include <gui/model/Model.hpp>
#include <gui/model/ModelListener.hpp>

extern "C" {
    #include "main.h" // Tutaj mamy dostęp do vescL
}

Model::Model() : modelListener(0)
{

}

void Model::tick()
{



}

int Model::getVescLRpm()
{
    return vescL.rpm;
}
