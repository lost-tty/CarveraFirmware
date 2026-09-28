#pragma once

// M503 asks every module for the gcode that would restore its settings.
class Settings {
public:
    using ReportFn = void (*)(void *module);

    struct Sink {
        ReportFn report;
        void *module;
        Sink *next;
    };

    static void add(Sink &slot, ReportFn fn, void *module);
    static void report_all();

private:
    static Sink *sinks;
};
