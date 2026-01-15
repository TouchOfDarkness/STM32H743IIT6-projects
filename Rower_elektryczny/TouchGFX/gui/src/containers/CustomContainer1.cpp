#include <gui/containers/CustomContainer1.hpp>
#include <touchgfx/Utils.hpp>
#include <math.h>

CustomContainer1::CustomContainer1()
{

}

void CustomContainer1::initialize()
{
    CustomContainer1Base::initialize();
}
void CustomContainer1::updateElementInt(const char* name, int value)
{
    // 1. Ustaw nazwę parametru (ParameterText)
    // Zakładamy, że ParameterText ma wildcard, jeśli nie - użyj Unicode::strncpy
    Unicode::strncpy(ParameterTextBuffer, name, PARAMETERTEXT_SIZE);
    ParameterText.invalidate();

    // 2. Ustaw wartość (ValueText)
    Unicode::snprintf(ValueTextBuffer, VALUETEXT_SIZE, "%d", value);
    ValueText.invalidate();
}

void CustomContainer1::updateElementFloat(const char* name, float value, int precision)
{
    // 1. Ustaw nazwę
    Unicode::strncpy(ParameterTextBuffer, name, PARAMETERTEXT_SIZE);
    ParameterText.invalidate();

    // 2. Konwersja Float na String (metoda "Ręczna")
    // Dzięki temu nie musisz włączać obsługi %f w linkerze, co oszczędza pamięć.

    int intPart = (int)value;

    // Oblicz część ułamkową
    // Np. dla 12.34: (12.34 - 12) * 100 = 0.34 * 100 = 34
    float remainder = value - intPart;
    if (remainder < 0) remainder = -remainder; // Pozbądź się minusa z ułamka

    int multiplier = 1;
    for(int i=0; i<precision; i++) multiplier *= 10;

    int fracPart = (int)((remainder * multiplier) + 0.5f); // +0.5f dla zaokrąglenia

    // Obsługa przypadku specjalnego: ujemne zero (np. -0.5)
    if (value < 0 && intPart == 0)
    {
        // Musimy ręcznie dodać minus, bo intPart wynosi 0
        Unicode::snprintf(ValueTextBuffer, VALUETEXT_SIZE, "-0.%0*d", precision, fracPart);
    }
    else
    {
        // Standardowy format: Całkowita.Ułamek
        // %0*d oznacza: wypełnij zerami do szerokości określonej przez 'precision'
        Unicode::snprintf(ValueTextBuffer, VALUETEXT_SIZE, "%d.%0*d", intPart, precision, fracPart);
    }

    ValueText.invalidate();
}
