#include "AppendFileStream.h"
#include <stdio.h>

int AppendFileStream::puts(const char *str, int size)
{
    FILE *fd= fopen(this->fn, "a");
    if(fd == NULL) return 0;

    int n= fwrite(str, 1, size == 0 ? strlen(str) : size, fd);
    fclose(fd);
    return n;
}
