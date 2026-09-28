#ifndef _VERSION__H
#define _VERSION__H

#define VERSION "1.0.7"

class Version {
    public:
        const char *get_build(void) const;
        const char *get_build_date(void) const;
};

#endif
