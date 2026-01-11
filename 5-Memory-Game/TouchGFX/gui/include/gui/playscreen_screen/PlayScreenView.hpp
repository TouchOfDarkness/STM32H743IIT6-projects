#ifndef PLAYSCREENVIEW_HPP
#define PLAYSCREENVIEW_HPP

#include <gui_generated/playscreen_screen/PlayScreenViewBase.hpp>
#include <gui/playscreen_screen/PlayScreenPresenter.hpp>

typedef enum {
	GAME_NONE,
	GAME_COUNT_DOWN,
	GAME_INITIALIZE,
    GAME_SHOW_CHALLEGE,
	GAME_ANSWER,
    GAME_WAIT_TO_NEXT_ROUND,
	GAME_OVER
} GAME_STATE;

typedef struct {
    uint8_t aubColorStore[128];
    uint8_t aubIndexStore[128];
    uint8_t ubColorIndex;
    uint8_t ubDisplayColorCount;
    uint8_t ubColorAnswerIndex;
    bool bButtonIsPress;
    uint8_t ubRemainColor;
    uint8_t ubScore;
} GAME_CHALLENGE_STRUCTURE;

class PlayScreenView : public PlayScreenViewBase
{
public:
    PlayScreenView();
    virtual ~PlayScreenView() {}
    virtual void setupScreen();
    virtual void tearDownScreen();

    virtual void handleTickEvent();
    virtual void ColorButton1CallBack();
    virtual void ColorButton2CallBack();
    virtual void ColorButton3CallBack();
    virtual void ColorButton4CallBack();

    uint32_t UniqueRand_Get(uint32_t min, uint32_t max);
    void SetTimer(uint16_t uwRemainTime);
    void EnterColor(uint8_t ubColor);

    uint16_t uwMainStick = 0;
    uint16_t uwSubStick = 0;
    uint16_t uwRandom = 0;
    uint8_t ubCountDownIndex = 0;
    uint16_t uwGameScore = 0;
    uint16_t uwRemainTime = 0;
    touchgfx::Unicode::UnicodeChar TextBuffer[15];
    touchgfx::Unicode::UnicodeChar TextTimeBuffer[15];
    touchgfx::Unicode::UnicodeChar TextRemainingColorBuffer[5];
    touchgfx::Unicode::UnicodeChar TextScoreBuffer[5];
    GAME_STATE tGameCurrentState = GAME_NONE;
    GAME_CHALLENGE_STRUCTURE tGameStructure;
    
protected:
};

#endif // PLAYSCREENVIEW_HPP
