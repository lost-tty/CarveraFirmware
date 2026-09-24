/*
    Author: Jim Morris (wolfmanjm@gmail.com)
    License: GPL3 or better see <http://www.gnu.org/licenses/>

    Summary
    -------
    Probes three user specified points on the bed and determines the plane of the bed relative to the probe.
    as the head moves in X and Y it will adjust Z to keep the head tram with the bed.

    Configuration
    -------------
    The strategy must be enabled in the cofnig as well as zprobe.

    leveling-strategy.three-point-leveling.enable         true

    Three probe points must be defined, these are best if they are the three points of an equilateral triangle, as far apart as possible.
    They can be defined in the config file as:-

    leveling-strategy.three-point-leveling.point1         100.0,0.0   # the first probe point (x,y)
    leveling-strategy.three-point-leveling.point2         200.0,200.0 # the second probe point (x,y)
    leveling-strategy.three-point-leveling.point3         0.0,200.0   # the third probe point (x,y)

    or they may be defined (and saved with M500) using M557 P0 X30 Y40.5  where P is 0,1,2

    probe offsets from the nozzle or tool head can be defined with

    leveling-strategy.three-point-leveling.probe_offsets  0,0,0  # probe offsetrs x,y,z

    they may also be set with M565 X0 Y0 Z0

    To force homing in X and Y before G32 does the probe the following can be set in config, this is the default

    leveling-strategy.three-point-leveling.home_first    true   # disable by setting to false

    The probe tolerance can be set using the config line

    leveling-strategy.three-point-leveling.tolerance   0.03    # the probe tolerance in mm, default is 0.03mm


    Usage
    -----
    G29 probes the three probe points and reports the Z at each point, if a plane is active it will be used to level the probe.
    G32 probes the three probe points and defines the bed plane, this will remain in effect until reset or M561
    G31 reports the status

    M557 defines the probe points
    M561 clears the plane and the bed leveling is disabled until G32 is run again
    M565 defines the probe offsets from the nozzle or tool head

    M500 saves the probe points and the probe offsets
    M503 displays the current settings
*/

#include "ThreePointStrategy.h"
#include "Endstops.h"
#include "Kernel.h"
#include "Config.h"
#include "Robot.h"
#include "Logging.h"
#include "Gcode.h"
#include "GcodeDispatch.h"
#include "checksumm.h"
#include "ConfigValue.h"
#include "Conveyor.h"
#include "ZProbe.h"
#include "Plane3D.h"
#include "nuts_bolts.h"
#include "StreamOutput.h"

#include <string>
#include <algorithm>
#include <cstdlib>
#include <cmath>

#define probe_point_1_checksum       CHECKSUM("point1")
#define probe_point_2_checksum       CHECKSUM("point2")
#define probe_point_3_checksum       CHECKSUM("point3")
#define probe_offsets_checksum       CHECKSUM("probe_offsets")
#define home_checksum                CHECKSUM("home_first")
#define tolerance_checksum           CHECKSUM("tolerance")
#define save_plane_checksum          CHECKSUM("save_plane")

ThreePointStrategy::ThreePointStrategy(ZProbe *zprobe) : LevelingStrategy(zprobe)
{
    for (int i = 0; i < 3; ++i) {
        probe_points[i] = std::make_tuple(NAN, NAN);
    }
    plane = nullptr;
}

ThreePointStrategy::~ThreePointStrategy()
{
    delete plane;
}

bool ThreePointStrategy::handleConfig()
{
    // format is xxx,yyy for the probe points
    std::string p1 = THEKERNEL->config->value(leveling_strategy_checksum, three_point_leveling_strategy_checksum, probe_point_1_checksum)->by_default("")->as_string();
    std::string p2 = THEKERNEL->config->value(leveling_strategy_checksum, three_point_leveling_strategy_checksum, probe_point_2_checksum)->by_default("")->as_string();
    std::string p3 = THEKERNEL->config->value(leveling_strategy_checksum, three_point_leveling_strategy_checksum, probe_point_3_checksum)->by_default("")->as_string();
    if(!p1.empty()) probe_points[0] = parseXY(p1.c_str());
    if(!p2.empty()) probe_points[1] = parseXY(p2.c_str());
    if(!p3.empty()) probe_points[2] = parseXY(p3.c_str());

    // Probe offsets xxx,yyy,zzz
    std::string po = THEKERNEL->config->value(leveling_strategy_checksum, three_point_leveling_strategy_checksum, probe_offsets_checksum)->by_default("0,0,0")->as_string();
    this->probe_offsets= parseXYZ(po.c_str());

    this->home= THEKERNEL->config->value(leveling_strategy_checksum, three_point_leveling_strategy_checksum, home_checksum)->by_default(true)->as_bool();
    this->tolerance= THEKERNEL->config->value(leveling_strategy_checksum, three_point_leveling_strategy_checksum, tolerance_checksum)->by_default(0.03F)->as_number();
    this->save= THEKERNEL->config->value(leveling_strategy_checksum, three_point_leveling_strategy_checksum, save_plane_checksum)->by_default(false)->as_bool();
    return true;
}

void ThreePointStrategy::report_settings(StreamOutput *stream)
{
    float x, y, z;
    stream->printf(";Probe points:\n");
    for (int i = 0; i < 3; ++i) {
        std::tie(x, y) = probe_points[i];
        stream->printf("M557 P%d X%1.5f Y%1.5f\n", i, x, y);
    }
    stream->printf(";Probe offsets:\n");
    std::tie(x, y, z) = probe_offsets;
    stream->printf("M565 X%1.5f Y%1.5f Z%1.5f\n", x, y, z);

    if(this->save && this->plane != nullptr) {
        uint32_t a, b, c, d;
        this->plane->encode(a, b, c, d);
        stream->printf(";Saved bed plane:\nM561 A%lu B%lu C%lu D%lu \n", a, b, c, d);
    }
}

bool ThreePointStrategy::handleGcode(Gcode *gcode)
{
    if(gcode->has_g) {
        // G code processing
        if(gcode->g == 29) { // test probe points for level
            if(!test_probe_points(gcode)) {
                gcode->stream->printf("Probe failed to complete, probe not triggered or other error\n");
            }
            return true;

        } else if( gcode->g == 31 ) { // report status
            if(this->plane == nullptr) {
                 gcode->stream->printf("Bed leveling plane is not set\n");
            }else{
                 gcode->stream->printf("Bed leveling plane normal= %f, %f, %f\n", plane->getNormal()[0], plane->getNormal()[1], plane->getNormal()[2]);
            }
            gcode->stream->printf("Probe is %s\n", zprobe->getProbeStatus() ? "Triggered" : "Not triggered");
            return true;

        } else if( gcode->g == 32 ) { // three point probe
            // first wait for an empty queue i.e. no moves left
            THECONVEYOR.wait_for_idle();

             // clear any existing plane and compensation
            delete this->plane;
            this->plane= nullptr;
            setAdjustFunction(false);

            if(!doProbing(gcode->stream)) {
                gcode->stream->printf("Probe failed to complete, probe not triggered or other error\n");
            } else {
                gcode->stream->printf("Probe completed, bed plane defined\n");
            }
            return true;
        }

    }

    return false;
}

void ThreePointStrategy::register_mcodes()
{
    ADD_MCODE(m557, 557, IMMEDIATE, ThreePointStrategy::set_probe_points);
    ADD_MCODE(m561, 561, BARRIER, ThreePointStrategy::set_plane);
    ADD_MCODE(m565, 565, IMMEDIATE, ThreePointStrategy::set_probe_offsets);
}

// M557 P0 X30 Y40.5, where P is 0, 1 or 2
void ThreePointStrategy::set_probe_points(Gcode *gcode)
{
    int idx = 0;
    float x = NAN, y = NAN;
    if(gcode->has_letter('P')) idx = gcode->get_value('P');
    if(gcode->has_letter('X')) x = gcode->get_value('X');
    if(gcode->has_letter('Y')) y = gcode->get_value('Y');
    if(idx < 0 || idx > 2) {
        gcode->stream->printf("only 3 probe points allowed P0-P2\n");
        return;
    }
    probe_points[idx] = std::make_tuple(x, y);
}

// M561: identity transform with no parameters, or the saved plane from A B C D
void ThreePointStrategy::set_plane(Gcode *gcode)
{
    delete this->plane;
    if(gcode->get_num_args() == 0) {
        this->plane= nullptr;
        setAdjustFunction(false);
        gcode->stream->printf("saved plane cleared\n");
        return;
    }

    uint32_t a,b,c,d;
    a=b=c=d= 0;
    if(gcode->has_letter('A')) a = gcode->get_uint('A');
    if(gcode->has_letter('B')) b = gcode->get_uint('B');
    if(gcode->has_letter('C')) c = gcode->get_uint('C');
    if(gcode->has_letter('D')) d = gcode->get_uint('D');
    this->plane= new Plane3D(a, b, c, d);
    setAdjustFunction(true);
}

void ThreePointStrategy::set_probe_offsets(Gcode *gcode)
{
    float x= 0, y= 0, z= 0;
    if(gcode->has_letter('X')) x = gcode->get_value('X');
    if(gcode->has_letter('Y')) y = gcode->get_value('Y');
    if(gcode->has_letter('Z')) z = gcode->get_value('Z');
    probe_offsets = std::make_tuple(x, y, z);
}

void ThreePointStrategy::homeXY()
{
    Endstops::axis_bitmap_t xy;
    xy.reset();
    xy.set(X_AXIS);
    xy.set(Y_AXIS);
    endstops.home_axes(xy);
}

bool ThreePointStrategy::doProbing(StreamOutput *stream)
{
    float x, y;
    // check the probe points have been defined
    for (int i = 0; i < 3; ++i) {
        std::tie(x, y) = probe_points[i];
        if(isnan(x) || isnan(y)) {
            stream->printf("Probe point P%d has not been defined, use M557 P%d Xnnn Ynnn to define it\n", i, i);
            return false;
        }
    }

    // optionally home XY axis first, but allow for manual homing
    if(this->home)
        homeXY();

    // move to the first probe point
    std::tie(x, y) = probe_points[0];
    // offset by the probe XY offset
    x -= std::get<X_AXIS>(this->probe_offsets);
    y -= std::get<Y_AXIS>(this->probe_offsets);
    zprobe->coordinated_move(x, y, NAN, zprobe->getFastFeedrate());

    // for now we use probe to find bed and not the Z min endstop
    // the first probe point becomes Z == 0 effectively so if we home Z or manually set z after this, it needs to be at the first probe point

    // TODO this needs to be configurable to use min z or probe

    // find bed via probe
    float mm;
    if(!zprobe->run_probe(mm, zprobe->getSlowFeedrate())) return false;

    // TODO if using probe then we probably need to set Z to 0 at first probe point, but take into account probe offset from head
    THEROBOT.reset_axis_position(std::get<Z_AXIS>(this->probe_offsets), Z_AXIS);

    // move up to specified probe start position
    zprobe->coordinated_move(NAN, NAN, zprobe->getProbeHeight(), zprobe->getSlowFeedrate()); // move to probe start position

    // probe the three points
    Vector3 v[3];
    for (int i = 0; i < 3; ++i) {
        float z;
        std::tie(x, y) = probe_points[i];
        // offset moves by the probe XY offset
        if(!zprobe->doProbeAt(z, x-std::get<X_AXIS>(this->probe_offsets), y-std::get<Y_AXIS>(this->probe_offsets))) return false;

        z= zprobe->getProbeHeight() - z; // relative distance between the probe points, lower is negative z
        stream->printf("DEBUG: P%d:%1.4f\n", i, z);
        v[i] = Vector3(x, y, z);
    }

    // if first point is not within tolerance report it, it should ideally be 0
    if(fabsf(v[0][2]) > this->tolerance) {
        stream->printf("WARNING: probe is not within tolerance: %f > %f\n", fabsf(v[0][2]), this->tolerance);
    }

    // define the plane
    delete this->plane;
    // check tolerance level here default 0.03mm
    auto mmx = std::minmax({v[0][2], v[1][2], v[2][2]});
    if((mmx.second - mmx.first) <= this->tolerance) {
        this->plane= nullptr; // plane is flat no need to do anything
        stream->printf("DEBUG: flat plane\n");
        setAdjustFunction(false);

    }else{
        this->plane = new Plane3D(v[0], v[1], v[2]);
        stream->printf("DEBUG: plane normal= %f, %f, %f\n", plane->getNormal()[0], plane->getNormal()[1], plane->getNormal()[2]);
        setAdjustFunction(true);
    }

    return true;
}

// Probes the 3 points and reports heights
bool ThreePointStrategy::test_probe_points(Gcode *gcode)
{
    // check the probe points have been defined
    float max_delta= 0;
    float last_z= NAN;
    for (int i = 0; i < 3; ++i) {
        float x, y;
        std::tie(x, y) = probe_points[i];
        if(isnan(x) || isnan(y)) {
            gcode->stream->printf("Probe point P%d has not been defined, use M557 P%d Xnnn Ynnn to define it\n", i, i);
            return false;
        }

        float z;
        if(!zprobe->doProbeAt(z, x-std::get<X_AXIS>(this->probe_offsets), y-std::get<Y_AXIS>(this->probe_offsets))) return false;

        gcode->stream->printf("X:%1.4f Y:%1.4f Z:%1.4f\n", x, y, z);

        if(isnan(last_z)) {
            last_z= z;
        }else{
            max_delta= std::max(max_delta, fabsf(z-last_z));
        }
    }

    gcode->stream->printf("max delta: %f\n", max_delta);

    return true;
}

void ThreePointStrategy::setAdjustFunction(bool on)
{
    if(on) {
        THEROBOT.set_compensation([this](float *target, bool inverse, bool debug) { if(inverse) target[2] -= this->plane->getz(target[0], target[1]); else target[2] += this->plane->getz(target[0], target[1]); });
    }else{
        // clear it
        THEROBOT.clear_compensation();
    }
}

// find the Z offset for the point on the plane at x, y
float ThreePointStrategy::getZOffset(float x, float y)
{
    if(this->plane == nullptr) return NAN;
    return this->plane->getz(x, y);
}

// parse a "X,Y" string return x,y
std::tuple<float, float> ThreePointStrategy::parseXY(const char *str)
{
    float x = NAN, y = NAN;
    char *p;
    x = strtof(str, &p);
    if(p + 1 < str + strlen(str)) {
        y = strtof(p + 1, nullptr);
    }
    return std::make_tuple(x, y);
}

// parse a "X,Y,Z" string return x,y,z tuple
std::tuple<float, float, float> ThreePointStrategy::parseXYZ(const char *str)
{
    float x = 0, y = 0, z= 0;
    char *p;
    x = strtof(str, &p);
    if(p + 1 < str + strlen(str)) {
        y = strtof(p + 1, &p);
        if(p + 1 < str + strlen(str)) {
            z = strtof(p + 1, nullptr);
        }
    }
    return std::make_tuple(x, y, z);
}
