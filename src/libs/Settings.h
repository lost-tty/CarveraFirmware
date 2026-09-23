#pragma once

class StreamOutput;

// M503 asks every module for the gcode that would restore its settings.
class Settings {
public:
    using ReportFn = void (*)(void *module, StreamOutput *stream);

    struct Sink {
        ReportFn report;
        void *module;
        Sink *next;
    };

    static void add(Sink &slot, ReportFn fn, void *module);
    static void report_all(StreamOutput *stream);

private:
    static Sink *sinks;
};
