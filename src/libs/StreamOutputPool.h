/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef STREAMOUTPUTPOOL_H
#define STREAMOUTPUTPOOL_H

#include <set>

#include "libs/StreamOutput.h"

class StreamOutputPool : public StreamOutput {

public:
    int puts(const char* s, int size)
    {
        int r = 0;
        StreamOutput::lock_broadcast();
        for(std::set<StreamOutput*>::iterator i = this->streams.begin(); i != this->streams.end(); i++)
        {
            if (!(*i)->accept_event()) continue; // text would corrupt the transfer
            int k = (*i)->puts(s, size);
            if (k > r)
                r = k;
        }
        StreamOutput::unlock_broadcast();
        return r;
    }

    void send(uint8_t type, const void *payload, size_t len)
    {
        StreamOutput::lock_broadcast();
        for(std::set<StreamOutput*>::iterator i = this->streams.begin(); i != this->streams.end(); i++)
        {
            if (!(*i)->accept_event()) continue; // text would corrupt the transfer
            (*i)->send(type, payload, len);
        }
        StreamOutput::unlock_broadcast();
    }

    // under the broadcast lock: other tasks may be broadcasting over the set
    void append_stream(StreamOutput* stream)
    {
        StreamOutput::lock_broadcast();
        this->streams.insert(stream);
        StreamOutput::unlock_broadcast();
    }

    void remove_stream(StreamOutput* stream)
    {
        StreamOutput::lock_broadcast();
        this->streams.erase(stream);
        StreamOutput::unlock_broadcast();
    }

private:
    std::set<StreamOutput*> streams;
};

#endif
