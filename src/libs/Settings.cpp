#include "Settings.h"

Settings::Sink *Settings::sinks = nullptr;

void Settings::add(Sink &slot, ReportFn fn, void *module)
{
    Sink **end = &sinks;
    while(*end != nullptr) {
        if(*end == &slot) return;
        end = &(*end)->next;
    }

    slot = Sink{fn, module, nullptr};
    *end = &slot;
}

void Settings::report_all(StreamOutput *stream)
{
    for (Sink *s = sinks; s != nullptr; s = s->next) s->report(s->module, stream);
}
