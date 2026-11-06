#ifndef DEBOUNCER_H_
#define DEBOUNCER_H_

enum class DebounceEdge
{
    Rising,
    Falling,
    Both
};

class Debouncer
{
public:
    explicit Debouncer(unsigned int requiredSamples = 8,
                       DebounceEdge edge = DebounceEdge::Both);

    bool update(bool aimed);
    void reset(bool aimed = false);

private:
    unsigned int requiredSamples_;
    DebounceEdge edge_;
    unsigned int candidateSamples_;
    bool stableAimed_;
    bool candidateAimed_;
};

#endif // AIMED_DEBOUNCER_H_
