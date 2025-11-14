#ifndef TONE_PLAYER_H_
#define TONE_PLAYER_H_

#include <stdint.h>
#include "tone_map.h"

class TonePlayer
{
    typedef void(*BeepSet_t)(uint32_t, uint16_t);
public:
    typedef struct
    {
        uint16_t freq;
        uint16_t time;
        uint16_t volume;
    } MusicNode_t;

    enum Speed
    {
        SPEED_HALF = 512,
        SPEED_NORMAL = 256,
        SPEED_DOUBLE = 128
    };

    TonePlayer();
    ~TonePlayer() {}
    bool setMusic(const MusicNode_t *music, uint16_t length, bool loop = false);
    void setCallback(BeepSet_t func)
    {
        beep_set_ = func;
    }
    void play();
    void play(const MusicNode_t *music, uint16_t length, bool loop = false);
    void stop();
    void setSpeed(uint16_t speed);
    bool update(uint32_t tick);
private:
    uint16_t length_;
    uint16_t current_pos_;
    uint32_t next_time_;
    uint16_t speed_;
    bool loop_play_;
    const MusicNode_t *current_music_;
    BeepSet_t beep_set_;
};

#define MUSIC_DEF(name) static const TonePlayer::MusicNode_t Music_##name[] =

/*列表数据类型定义*/
typedef struct
{
    const TonePlayer::MusicNode_t *mc;
    uint16_t length;
    const char *name;
} MusicList_t;

#define ADD_MUSIC(mc) {Music_##mc,(sizeof(Music_##mc) / sizeof(Music_##mc[0])), #mc}
#define GET_MUSIC(mc) Music_##mc
#define GET_MUSIC_LEN(mc) (sizeof(Music_##mc) / sizeof(Music_##mc[0]))

#endif
