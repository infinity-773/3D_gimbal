#include "Debouncer.h"

#include <algorithm>

Debouncer::Debouncer(unsigned int requiredSamples, DebounceEdge edge)
    : requiredSamples_(std::max(1U, requiredSamples)),
      edge_(edge),
      candidateSamples_(0),
      stableAimed_(false),
      candidateAimed_(false)
{
}

bool Debouncer::update(bool aimed)
{
    if (aimed == stableAimed_)
    {
        candidateAimed_ = aimed;
        candidateSamples_ = 0;
        return stableAimed_;
    }

    const bool rising = !stableAimed_ && aimed;
    if ((edge_ == DebounceEdge::Rising && !rising) ||
        (edge_ == DebounceEdge::Falling && rising))
    {
        stableAimed_ = aimed;
        candidateAimed_ = aimed;
        candidateSamples_ = 0;
        return stableAimed_;
    }

    if (aimed != candidateAimed_)
    {
        candidateAimed_ = aimed;
        candidateSamples_ = 1;
    }
    else
    {
        ++candidateSamples_;
    }

    if (candidateSamples_ >= requiredSamples_)
    {
        stableAimed_ = candidateAimed_;
        candidateSamples_ = 0;
    }

    return stableAimed_;
}

void Debouncer::reset(bool aimed)
{
    stableAimed_ = aimed;
    candidateAimed_ = aimed;
    candidateSamples_ = 0;
}
