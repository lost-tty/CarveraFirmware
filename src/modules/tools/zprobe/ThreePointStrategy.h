#ifndef _THREEPOINTSTRATEGY
#define _THREEPOINTSTRATEGY

#include "LevelingStrategy.h"

#include <string.h>
#include <tuple>

#define three_point_leveling_strategy_checksum CHECKSUM("three-point-leveling")

class StreamOutput;
class Plane3D;

#include "GcodeDispatch.h"

class ThreePointStrategy : public LevelingStrategy
{
public:
    ThreePointStrategy(ZProbe *zprobe);
    ~ThreePointStrategy();
    bool handleGcode(Gcode* gcode);
    void report_settings(StreamOutput *stream) override;
    void register_mcodes() override;

    void set_probe_points(Gcode *);
    void set_plane(Gcode *);
    void set_probe_offsets(Gcode *);
    bool handleConfig();
    float getZOffset(float x, float y);

private:
    GcodeDispatch::Mcode m557, m561, m565;

    void homeXY();
    bool doProbing(StreamOutput *stream);
    std::tuple<float, float> parseXY(const char *str);
    std::tuple<float, float, float> parseXYZ(const char *str);
    void setAdjustFunction(bool);
    bool test_probe_points(Gcode *gcode);

    std::tuple<float, float, float> probe_offsets;
    std::tuple<float, float> probe_points[3];
    Plane3D *plane;
    struct {
        bool home:1;
        bool save:1;
    };
    float tolerance;
};

#endif
