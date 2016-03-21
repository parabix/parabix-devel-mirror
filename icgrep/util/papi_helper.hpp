#ifndef PAPICOUNTER_HPP
#define PAPICOUNTER_HPP

#include <papi.h>
#include <string>
#include <iostream>
#include <stdexcept>

namespace papi {

class IPapiCounter {
    public:
        virtual void start() = 0;
        virtual void stop() = 0;
        virtual void write(std::ostream & out) const = 0;
};

template <unsigned N>
class PapiCounter : public IPapiCounter
{
    typedef long_long papi_counter_t;

public:

    inline PapiCounter(std::initializer_list<int> events);
    inline ~PapiCounter();

    virtual void start();
    virtual void stop();
    virtual void write(std::ostream & out) const;

private:

    int                    fEventSet;									// PAPI event set
    papi_counter_t         fStartedAt[N];
    papi_counter_t         fIntervals[N];
    int                    fEvent[N];
};

template <unsigned N>
PapiCounter<N>::PapiCounter(std::initializer_list<int> events) {

    // PAPI init
    int rval = PAPI_library_init(PAPI_VER_CURRENT);
    if (rval != PAPI_VER_CURRENT) {
        throw std::runtime_error("PAPI Library Init Error: " + std::string(PAPI_strerror(rval)));
    }
    std::copy(events.begin(), events.end(), fEvent);
    memset(fStartedAt, 0, sizeof(papi_counter_t) * N);
    memset(fIntervals, 0, sizeof(papi_counter_t) * N);

    // PAPI create event set
    fEventSet = PAPI_NULL;
    if ((rval = PAPI_create_eventset(&fEventSet)) != PAPI_OK) {
        throw std::runtime_error("PAPI Create Event Set Error: " + std::string(PAPI_strerror(rval)));
    }

    // PAPI fill event set
    if ((rval = PAPI_add_events(fEventSet, fEvent, N)) != PAPI_OK) {
        throw std::runtime_error("PAPI Add Events Error: " + std::string(PAPI_strerror(rval < PAPI_OK ? rval : PAPI_EINVAL)));
    }

    // Call PAPI start on construction, to force PAPI initialization
    rval = PAPI_start(fEventSet);
    if (rval != PAPI_OK) {
        throw std::runtime_error("PAPI Start Error: " + std::string(PAPI_strerror(rval)));
    }
}

template <unsigned N>
PapiCounter<N>::~PapiCounter() {
    // Call PAPI stop on destruction
    int rval = PAPI_stop(fEventSet, nullptr);
    if (rval != PAPI_OK) {
        throw std::runtime_error(" PAPI code: " + std::string(PAPI_strerror(rval)));
    }
    if ((rval =  PAPI_cleanup_eventset(fEventSet) != PAPI_OK)) {
        throw std::runtime_error("PAPI code: " + std::string(PAPI_strerror(rval)));
    }
    // PAPI free all events
    if ((rval = PAPI_destroy_eventset(&fEventSet) != PAPI_OK)) {
        throw std::runtime_error("PAPI code: " + std::string(PAPI_strerror(rval)));
    }
}

template <unsigned N>
void PapiCounter<N>::start() {
    int rval = PAPI_read(fEventSet, fStartedAt);
    if (rval != PAPI_OK) {
        throw std::runtime_error("PAPI code: " + std::string(PAPI_strerror(rval)));
    }
}

// PAPI Low Level API Wrapper: Records the difference Events of the current start interval array values set into the values.
template <unsigned N>
void PapiCounter<N>::stop() {
    papi_counter_t endedAt[N];
    int rval = PAPI_read(fEventSet, endedAt);
    if (rval != PAPI_OK) {
        throw std::runtime_error("PAPI code: " + std::string(PAPI_strerror(rval)));
    }
    for (unsigned i = 0; i != N; i++) {
        fIntervals[i] = (endedAt[i] - fStartedAt[i]);
    }
}

template <unsigned N>
void PapiCounter<N>::write(std::ostream & out) const {
    // Convert PAPI codes to names
    char eventName[PAPI_MAX_STR_LEN + 1];
    for (unsigned i = 0; i != N; i++) {
        int rval = PAPI_event_code_to_name(fEvent[i], eventName);
        if (rval != PAPI_OK) {
            memset(eventName + 1, '?', 16);
            eventName[16 + 1] = 0;
        }
        out << ';' << eventName << '|' << fIntervals[i];
    }
}

inline static std::ostream & operator << (std::ostream & out, const IPapiCounter & papiCounter) {
    papiCounter.write(out);
    return out;
}

}

#endif // PAPICOUNTER_HPP
