/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef STREAMOUTPUTPOOL_H
#define STREAMOUTPUTPOOL_H

using namespace std;
#include <set>
#include <string>
#include <cstdio>
#include <cstdarg>

#include "libs/StreamOutput.h"

class StreamOutputPool : public StreamOutput {

public:
    int puts(const char* s, int size)
    {
        int r = 0;
        for(set<StreamOutput*>::iterator i = this->streams.begin(); i != this->streams.end(); i++)
        {
            if ((*i)->is_transferring()) continue; // text would corrupt the transfer
            int k = (*i)->puts(s, size);
            if (k > r)
                r = k;
        }
        return r;
    }

    void send(uint8_t type, const void *payload, size_t len)
    {
        for(set<StreamOutput*>::iterator i = this->streams.begin(); i != this->streams.end(); i++)
        {
            (*i)->send(type, payload, len);
        }
    }

    void append_stream(StreamOutput* stream)
    {
        this->streams.insert(stream);
    }

    void remove_stream(StreamOutput* stream)
    {
        this->streams.erase(stream);
    }

private:
    set<StreamOutput*> streams;
};

#endif
