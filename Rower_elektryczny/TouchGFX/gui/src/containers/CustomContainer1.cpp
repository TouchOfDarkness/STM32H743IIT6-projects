#include <gui/containers/CustomContainer1.hpp>
#include <touchgfx/Utils.hpp>

CustomContainer1::CustomContainer1()
{

}

void CustomContainer1::initialize()
{
    CustomContainer1Base::initialize();
}
void CustomContainer1::updateElement(const char* name, int value)
{
    // 1. Ustaw nazwę parametru (ParameterText)
    // Zakładamy, że ParameterText ma wildcard, jeśli nie - użyj Unicode::strncpy
    Unicode::strncpy(ParameterTextBuffer, name, PARAMETERTEXT_SIZE);
    ParameterText.invalidate();

    // 2. Ustaw wartość (ValueText)
    Unicode::snprintf(ValueTextBuffer, VALUETEXT_SIZE, "%d", value);
    ValueText.invalidate();
}
