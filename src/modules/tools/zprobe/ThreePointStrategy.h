#ifndef _THREEPOINTSTRATEGY
#define _THREEPOINTSTRATEGY

#include "LevelingStrategy.h"

#include <string.h>
#include <tuple>

class StreamOutput;
class Plane3D;

#include "libs/McodeRegistry.h"

class ThreePointStrategy : public LevelingStrategy
{
public:
    ThreePointStrategy(ZProbe *zprobe);
    ~ThreePointStrategy();
    bool handleGcode(Gcode* gcode);
    void report_settings() override;
    void register_mcodes() override;

    void set_probe_points(Gcode *);
    void set_plane(Gcode *);
    void set_probe_offsets(Gcode *);
    bool handleConfig();
    float getZOffset(float x, float y);

private:
    McodeRegistry::Mcode m557, m561, m565;

    void homeXY();
    bool doProbing();
    std::tuple<float, float> parseXY(const char *str);
    std::tuple<float, float, float> parseXYZ(const char *str);
    void setAdjustFunction(bool);
    bool test_probe_points(Gcode *gcode);

    std::tuple<float, float, float> probe_offsets;
    std::tuple<float, float> probe_points[3];
    Plane3D *plane;
    float tolerance;
    bool home_first;
    bool save_plane;
};

#endif
