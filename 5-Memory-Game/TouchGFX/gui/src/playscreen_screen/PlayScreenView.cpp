#include <gui/playscreen_screen/PlayScreenView.hpp>
#include <images/BitmapDatabase.hpp>
#include <touchgfx/Color.hpp>
#include "stm32h7xx_hal.h"

#define SECONDS_TO_STICK(seconds)	(seconds*30)
#define STICK_TO_SECONDS(stick)     (stick)
#define COLOR_WHITE_CODE    (touchgfx::Color::getColorFromRGB(255, 255, 255))
#define COLOR_RED_CODE    (touchgfx::Color::getColorFromRGB(255, 0, 0))
#define COLOR_GREEN_CODE    (touchgfx::Color::getColorFromRGB(0, 255, 0))
#define COLOR_BLUE_CODE    (touchgfx::Color::getColorFromRGB(0, 0, 255))
#define COLOR_YELLOW_CODE    (touchgfx::Color::getColorFromRGB(255, 255, 0))

#define COLOR_RED_INDEX     (0)
#define COLOR_GREEN_INDEX   (1)
#define COLOR_BLUE_INDEX    (2)
#define COLOR_YELLOW_INDEX  (3)

#define DEFAULT_REMAIN_TIMER    (10) // seconds

static uint8_t s_aubCountDownNumberIndex[4] = {0xFF, BITMAP_NUMBER1_ID, BITMAP_NUMBER2_ID, BITMAP_NUMBER3_ID};

PlayScreenView::PlayScreenView()
{

}

void PlayScreenView::setupScreen()
{
    PlayScreenViewBase::setupScreen();
    // Init variable
    ubCountDownIndex = 3;
    tGameCurrentState = GAME_COUNT_DOWN;
    uwGameScore = 0;
    for(uint8_t i = 0 ; i < 128 ; i++)
    {
        tGameStructure.aubColorStore[i] = 0;
        tGameStructure.aubIndexStore[i] = 0;
    }
    tGameStructure.ubColorIndex = 0;
    tGameStructure.ubDisplayColorCount = 0;
    tGameStructure.ubColorAnswerIndex = 0;
    tGameStructure.ubRemainColor = 0;
    tGameStructure.ubScore = 0;
    // Init First State
    CountDownImage.setBitmap(Bitmap(s_aubCountDownNumberIndex[ubCountDownIndex]));
    CountDownImage.invalidate();
    
    ColorBox1.setAlpha(0);
    ColorBox1.setColor(COLOR_WHITE_CODE);
    ColorBox1.invalidate();
    ColorBox2.setAlpha(0);
    ColorBox2.setColor(COLOR_WHITE_CODE);
    ColorBox2.invalidate();
    ColorBox3.setAlpha(0);
    ColorBox3.setColor(COLOR_WHITE_CODE);
    ColorBox3.invalidate();
    ColorBox4.setAlpha(0);
    ColorBox4.setColor(COLOR_WHITE_CODE);
    ColorBox4.invalidate();

    BorderColor.setAlpha(0);
    BorderColor.invalidate();

    RemainLableText.setAlpha(0);
    RemainLableText.invalidate();
    RemainingColorsText.setAlpha(0);
    RemainingColorsText.invalidate();
    TimerText.setAlpha(0);
    TimerText.invalidate();

    ScoreText.setAlpha(0);
    ScoreText.invalidate();


    ScoreImageBackGround.setAlpha(0);
    ScoreImageBackGround.invalidate();

    Unicode::snprintf(TextBuffer, 15, "%d", uwGameScore);
    ScoreText.setWildcard(TextBuffer);
    ScoreText.invalidate();
}

void PlayScreenView::tearDownScreen()
{
    PlayScreenViewBase::tearDownScreen();
}

void PlayScreenView::handleTickEvent()
{
    uwRandom++;
    //
    uwMainStick++;
	switch(tGameCurrentState)
	{
		case GAME_NONE:
		{
			break;
		}
        case GAME_COUNT_DOWN:
        {
            if(uwMainStick >= SECONDS_TO_STICK(1))
            {
                uwMainStick = 0;
                if(ubCountDownIndex > 1)
                {
                    ubCountDownIndex--;
                    CountDownImage.setBitmap(Bitmap(s_aubCountDownNumberIndex[ubCountDownIndex]));
                    CountDownImage.invalidate();
                }
                else
                {
                    //ubCountDownIndex = 3;
                	CountDownImage.setAlpha(0);
                	CountDownImage.invalidate();
                	BorderColor.setAlpha(255);
                	BorderColor.invalidate();
                	ColorBox1.setAlpha(255);
                	ColorBox1.invalidate();
                	ColorBox2.setAlpha(255);
                	ColorBox2.invalidate();
                	ColorBox3.setAlpha(255);
                	ColorBox3.invalidate();
                	ColorBox4.setAlpha(255);
                	ColorBox4.invalidate();
                    ScoreImageBackGround.setAlpha(255);
                    ScoreImageBackGround.invalidate();
                    ScoreText.setAlpha(255);
                    ScoreText.invalidate();
                    RemainLableText.setAlpha(255);
                    RemainLableText.invalidate();
                    RemainingColorsText.setAlpha(255);
                    RemainingColorsText.invalidate();
                    TimerText.setAlpha(255);
                    TimerText.invalidate();
                    tGameCurrentState = GAME_INITIALIZE;
                }
            }
            break;
        }
        case GAME_INITIALIZE:
        {
            tGameStructure.aubColorStore[tGameStructure.ubColorIndex] = tGameStructure.aubIndexStore[tGameStructure.ubColorIndex] = UniqueRand_Get(0, 3);
            tGameStructure.ubColorIndex++;
            tGameStructure.ubDisplayColorCount = 0;
            tGameStructure.ubColorAnswerIndex = 0;
            tGameCurrentState = GAME_SHOW_CHALLEGE;
            break;
        }
        case GAME_SHOW_CHALLEGE:
        {
            if(uwMainStick >= STICK_TO_SECONDS(10))
            {
                uwMainStick = 0;

                ColorBox1.setColor(COLOR_WHITE_CODE);
                ColorBox1.invalidate();
                ColorBox2.setColor(COLOR_WHITE_CODE);
                ColorBox2.invalidate();
                ColorBox3.setColor(COLOR_WHITE_CODE);
                ColorBox3.invalidate();
                ColorBox4.setColor(COLOR_WHITE_CODE);
                ColorBox4.invalidate();

                if(tGameStructure.ubDisplayColorCount >= tGameStructure.ubColorIndex)
                {
                    tGameCurrentState = GAME_ANSWER;
                    uwRemainTime = DEFAULT_REMAIN_TIMER + tGameStructure.ubColorIndex*1.5f;
                    tGameStructure.ubRemainColor = tGameStructure.ubColorIndex;
                    Unicode::snprintf(TextRemainingColorBuffer, 5, "%d", tGameStructure.ubRemainColor);
                    RemainingColorsText.setWildcard(TextRemainingColorBuffer);
                    RemainingColorsText.invalidate();
                    break;
                }

                switch (tGameStructure.aubIndexStore[tGameStructure.ubDisplayColorCount])
                {
                    case 0:
                    {
                        ColorBox1.setColor(COLOR_RED_CODE);
                        ColorBox1.invalidate();
                        break;
                    }
                    case 1:
                    {
                        ColorBox2.setColor(COLOR_GREEN_CODE);
                        ColorBox2.invalidate();
                        break;
                    }
                    case 2:
                    {
                        ColorBox3.setColor(COLOR_BLUE_CODE);
                        ColorBox3.invalidate();
                        break;
                    }
                    case 3:
                    {
                        ColorBox4.setColor(COLOR_YELLOW_CODE);
                        ColorBox4.invalidate();
                        break;
                    }
                    default:
                        break;
                }
                tGameStructure.ubDisplayColorCount++;
            }
            break;
        }
        case GAME_ANSWER:
        {
            if(uwMainStick >= SECONDS_TO_STICK(1))
            {
                uwMainStick = 0;
                if(uwRemainTime == 0)
                {
                    tGameCurrentState = GAME_OVER;
                    break;
                }
                uwRemainTime--;
                SetTimer(uwRemainTime);
            }

            if(tGameStructure.bButtonIsPress == true)
            {
                uwSubStick++;
                if(uwSubStick >= STICK_TO_SECONDS(10))
                {
                    ColorBox1.setColor(COLOR_WHITE_CODE);
                    ColorBox1.invalidate();
                    ColorBox2.setColor(COLOR_WHITE_CODE);
                    ColorBox2.invalidate();
                    ColorBox3.setColor(COLOR_WHITE_CODE);
                    ColorBox3.invalidate();
                    ColorBox4.setColor(COLOR_WHITE_CODE);
                    ColorBox4.invalidate();
                    tGameStructure.bButtonIsPress = false;
                    uwSubStick = 0;
                } 
            }
            break;
        }
        case GAME_WAIT_TO_NEXT_ROUND:
        {
            if(tGameStructure.bButtonIsPress == true)
            {
                uwSubStick++;
                if(uwSubStick >= STICK_TO_SECONDS(10))
                {
                    ColorBox1.setColor(COLOR_WHITE_CODE);
                    ColorBox1.invalidate();
                    ColorBox2.setColor(COLOR_WHITE_CODE);
                    ColorBox2.invalidate();
                    ColorBox3.setColor(COLOR_WHITE_CODE);
                    ColorBox3.invalidate();
                    ColorBox4.setColor(COLOR_WHITE_CODE);
                    ColorBox4.invalidate();
                    tGameStructure.bButtonIsPress = false;
                    uwMainStick = uwSubStick = 0;
                    tGameCurrentState = GAME_INITIALIZE;
                    tGameStructure.ubScore++;
                    Unicode::snprintf(TextScoreBuffer, 5, "%d", tGameStructure.ubScore);
                    ScoreText.setWildcard(TextScoreBuffer);
                    ScoreText.invalidate();
                } 
            }
            break;
        }
        case GAME_OVER:
        {
            handleKeyEvent(0);
            tGameCurrentState = GAME_NONE;
            break;
        }
        default:
        {
            break;
        }
	}
}

uint32_t PlayScreenView::UniqueRand_Get(uint32_t min, uint32_t max)
{
    static uint32_t prev = 0xFFFFFFFFu;
    static uint32_t counter = 0;
    counter++;

    //
    // if(counter >= 4)
    // {
    //     counter = 0;
    // }
    // return counter;
    //

    if (min > max) {
        uint32_t t = min; min = max; max = t;
    }

    uint32_t range = max - min + 1;
    if (range == 1) {
        prev = min;
        return min;
    }

    uint32_t candidate;
    do {
        uint32_t tick = HAL_GetTick() ^ (counter * 2654435761UL);
        // uint32_t tick = uwRandom ^ (counter * 2654435761UL);
        tick ^= (tick << 13);
        tick ^= (tick >> 7);
        tick ^= (tick << 17);

        candidate = (tick % range) + min;
    } while (candidate == prev);

    prev = candidate;
    return candidate;
}

void PlayScreenView::SetTimer(uint16_t uwRemainTime)
{
    uint8_t ubMinuters = uwRemainTime/60;
    uint8_t ubSeconds = uwRemainTime%60;
    Unicode::snprintf(TextTimeBuffer, 15, "%02d:%02d", ubMinuters, ubSeconds);
    TimerText.setWildcard(TextTimeBuffer);
    TimerText.invalidate();
}

void PlayScreenView::ColorButton1CallBack()
{
    EnterColor(COLOR_RED_INDEX);
}

void PlayScreenView::ColorButton2CallBack()
{
    EnterColor(COLOR_GREEN_INDEX);
}

void PlayScreenView::ColorButton3CallBack()
{
    EnterColor(COLOR_BLUE_INDEX);
}

void PlayScreenView::ColorButton4CallBack()
{
    EnterColor(COLOR_YELLOW_INDEX);
}

void PlayScreenView::EnterColor(uint8_t ubColor)
{
    if(tGameCurrentState == GAME_ANSWER)
    {
        if(tGameStructure.ubColorIndex > tGameStructure.ubColorAnswerIndex)
        switch (ubColor)
        {
            case COLOR_RED_INDEX:
            {
                ColorBox1.setColor(COLOR_RED_CODE);
                ColorBox1.invalidate();
                break;
            }
            case COLOR_GREEN_INDEX:
            {
                ColorBox2.setColor(COLOR_GREEN_CODE);
                ColorBox2.invalidate();
                break;
            }
            case COLOR_BLUE_INDEX:
            {
                ColorBox3.setColor(COLOR_BLUE_CODE);
                ColorBox3.invalidate();
                break;
            }
            case COLOR_YELLOW_INDEX:
            {
                ColorBox4.setColor(COLOR_YELLOW_CODE);
                ColorBox4.invalidate();
                break;
            }
            default:
                break;
        }
        tGameStructure.bButtonIsPress = true;
        
        if(tGameStructure.aubColorStore[tGameStructure.ubColorAnswerIndex] == ubColor)
        {
            tGameStructure.ubColorAnswerIndex++;
            tGameStructure.ubRemainColor--;
            Unicode::snprintf(TextRemainingColorBuffer, 5, "%d", tGameStructure.ubRemainColor);
            RemainingColorsText.setWildcard(TextRemainingColorBuffer);
            RemainingColorsText.invalidate();
            if(tGameStructure.ubColorAnswerIndex == tGameStructure.ubColorIndex)
            {
                uwSubStick = 0;
                tGameCurrentState = GAME_WAIT_TO_NEXT_ROUND;
            }
        }
        else
        {
            tGameCurrentState = GAME_OVER;
        }
    }
}
