#ifndef PLAYERPUBLICACCESS_H
#define PLAYERPUBLICACCESS_H

struct pad_progress {
    unsigned int percent_complete;
    unsigned long played_lines;
    unsigned long elapsed_secs;
    std::string filename;
};
#endif
