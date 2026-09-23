/*
 * A strategy called from ZProbe to handle a strategy for leveling
 * examples are delta calibration, three point bed leveling, z height map
 */

#ifndef _LEVELINGSTRATEGY
#define _LEVELINGSTRATEGY

class ZProbe;
class Gcode;
class StreamOutput;

class LevelingStrategy
{
public:
    LevelingStrategy(ZProbe* zprobe) : zprobe(zprobe){};
    virtual ~LevelingStrategy(){};
    virtual bool handleGcode(Gcode* gcode)= 0;
    virtual void report_settings(StreamOutput *stream) {}
    virtual void register_mcodes() {}
    virtual bool handleConfig()= 0;

protected:
    ZProbe *zprobe;

};
#endif
