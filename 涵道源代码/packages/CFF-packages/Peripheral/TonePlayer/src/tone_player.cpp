#include "tone_player.hpp"

TonePlayer::TonePlayer() :
    length_(0),
    current_pos_(0),
    next_time_(0),
    speed_(SPEED_NORMAL),
    current_music_(nullptr),
    beep_set_(nullptr)
{}

bool TonePlayer::setMusic(const MusicNode_t *music, uint16_t length, bool loop)
{
    if (music == current_music_)
    {
        return false;
    }

    current_music_ = music;
    length_ = length;
    current_pos_ = length;
    loop_play_ = loop;
    return true;
}


void TonePlayer::setSpeed(uint16_t speed)
{
    speed_ = speed;
}

void TonePlayer::play()
{
    current_pos_ = 0;
}

void TonePlayer::play(const MusicNode_t *music, uint16_t length, bool loop)
{
    if (setMusic(music, length, loop) || !loop)
    {
        play();
    }
}

void TonePlayer::stop()
{
    loop_play_ = false;
    current_music_ = nullptr;
    current_pos_ = length_;
}

bool TonePlayer::update(uint32_t tick)
{
    if (beep_set_ == nullptr)
    {
        return false;
    }

    if(current_pos_ < length_)
    {
        if(tick > next_time_)
        {
            beep_set_(current_music_[current_pos_].freq, current_music_[current_pos_].volume);

            next_time_ = tick + current_music_[current_pos_].time * speed_ / SPEED_NORMAL;
            current_pos_++;
        }
        return true;
    }
    else if(current_pos_ == length_ && tick > next_time_)
    {
        if (loop_play_)
        {
            play();
        }
        else
        {
            beep_set_(0, 0);
            current_pos_++;
        }

    }

    return false;
}
