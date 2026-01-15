#ifndef MODEL_HPP
#define MODEL_HPP

extern "C" {
    #include "main.h" // Tutaj mamy dostęp do vescL
}
class ModelListener;

class Model
{
public:
    Model();

    void bind(ModelListener* listener)
    {
        modelListener = listener;
    }

    void tick();
    VescData_t getVescData(int controllerIndex);
protected:
    ModelListener* modelListener;
};

#endif // MODEL_HPP
