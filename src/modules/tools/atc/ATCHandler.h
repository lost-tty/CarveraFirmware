#ifndef _ATCHANDLER_H
#define _ATCHANDLER_H

using namespace std;
#include "Module.h"
#include "libs/Killable.h"
class Gcode;
#include "Pin.h"

#include "SoftTimer.h"
#include "SimpleShell.h"
#include "libs/McodeRegistry.h"
#include "libs/Watch.h"
#include "Parameters.h"

class ATCHandler : public Module, public Killable
{

public:
    // M497.<n> sets it, the macros pick the numbers, the status line reports it as |A:<n>
    uint8_t state() const { return atc_state; }
    void set_state(uint8_t s) { atc_state = s; }

    bool get_tool_status(struct tool_status *t) const;
    void get_pin_status(char *data) const;
    void set_ref_tool_mz();

    ATCHandler()
    : probe_laser_timer("ProbeLaserCountdown", 1000, true, this, &ATCHandler::countdown_probe_laser)
    {}

    void on_module_loaded();
    void clamp_gcode(Gcode *);
    void detect_gcode(Gcode *);
    void tool_gcode(Gcode *);
    void probe_laser_gcode(Gcode *);
    void state_gcode(Gcode *);

    McodeRegistry::Mcode m490, m492, m493, m494, m497;
    void kill() override {}
    void cleanup() override;
    void on_config_reload(void *argument);

private:
    static float param_clamp_state(void *c);
    static float param_tool_detected(void *c);
    static float param_active_tool(void *);
    void register_params();
    static void shell(void *self, const char *name, std::string args, StreamOutput *stream);
    static const SimpleShell::Sub<ATCHandler> SUBS[];
    void sub_state(std::string args, StreamOutput *stream);
    void sub_rack(std::string args, StreamOutput *stream);
    SimpleShell::Registered shell_slot;
    Parameters::Table params_slot;
    volatile uint8_t atc_state{0};


    typedef enum {
    	UNHOMED,	// need to home first
		CLAMPED,	// status after home or clamp
		LOOSED,		// status after loose
    } CLAMP_STATUS;


    void countdown_probe_laser();

    void switch_probe_laser(bool state);

    // clamp actions
    void clamp_tool();
    void loose_tool();
    void home_clamp();

    // laser detect
    bool laser_detect();

    // probe check
    bool probe_detect();


    // set tool offset afteer calibrating
    void set_tool_offset();

    //

    //







    Watch atc_watch;   // the isr holds a pointer to this, so it outlives the move
    bool tool_detected; // result of the last M492 laser check


    uint16_t probe_laser_countdown;
    SoftTimer probe_laser_timer;

    using atc_homing_info_t = struct {
        Pin pin;

        struct {
            bool triggered:1;
            CLAMP_STATUS clamp_status;
        };
    };
    atc_homing_info_t atc_home_info;

    using detector_info_t = struct {
        Pin detect_pin;
    };
    detector_info_t detector_info;

    static float probe_mx();
    static float probe_my();
    static float probe_mz();
};

#endif /* _ATCHANDLER_H */

extern ATCHandler atc_handler;
