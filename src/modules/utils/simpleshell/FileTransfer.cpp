#include "FileTransfer.h"
#include "Transfer.h"
#include "libs/MainWake.h"

#include <new>
#include <unistd.h>

void FileTransfer::begin(const std::string& filename, StreamOutput* stream, bool upload)
{
    Transfer* t = start(filename, stream, upload);
    if (t == nullptr)
        return;

    if (upload)
        t->start_upload();
    else
        t->start_download();

    if (t->done())
        end(t);
}

// A transfer for this client, or the reason it cannot have one now. Downloads of one file may
// run side by side; an upload replaces the file, so it shares it with nothing.
Transfer* FileTransfer::start(const std::string& filename, StreamOutput* stream, bool upload)
{
    if (Transfer* t = of(stream)) {
        stream->printf("error:another transfer is in progress: %s %s\r\n",
                       t->is_download ? "download of" : "upload to", t->filename.c_str());
        return nullptr;
    }
    // a.nc.lz unpacks to a.nc: the same file
    std::string base = without_lz(filename);
    int slot = -1;
    for (int i = 0; i < MAX_TRANSFERS; i++) {
        Transfer* t = transfers[i];
        if (t == nullptr) {
            if (slot < 0)
                slot = i;

            continue;
        }
        if (without_lz(t->filename) == base && (upload || !t->is_download)) {
            stream->printf("error:another transfer of %s is in progress\r\n", filename.c_str());
            return nullptr;
        }
    }
    if (slot < 0) {
        stream->printf("error:too many transfers at once\r\n");
        return nullptr;
    }
    Transfer* t = new (std::nothrow) Transfer(*this, stream, filename);
    if (t == nullptr) {
        stream->printf("error:no memory for another transfer\r\n");
        return nullptr;
    }
    transfers[slot] = t;
    count_changed();
    return t;
}

void FileTransfer::service()
{
    for (Transfer* t : transfers) {
        if (t == nullptr)
            continue;

        if (!t->done())
            t->service();

        if (t->done())
            end(t);
        else if (t->working())
            wake_main();
    }
}

// Called from Session::bind/release on the main loop, so no step is mid-flight: end it here,
// before the slot's next client can inherit the failure report.
void FileTransfer::cancel_if(StreamOutput* s)
{
    Transfer* t = of(s);
    if (t == nullptr)
        return;

    t->finish(false, false, true);
    end(t);
}

bool FileTransfer::take_frame(StreamOutput* s, uint8_t type, const uint8_t* p, uint16_t len)
{
    Transfer* t = of(s);
    if (t == nullptr || !t->is_download)
        return false;

    t->take_frame(type, p, len);
    if (t->done())
        end(t);

    return true;
}

Transfer* FileTransfer::of(const StreamOutput* s) const
{
    for (Transfer* t : transfers) {
        if (t != nullptr && !t->done() && t->stream == s)
            return t;
    }
    return nullptr;
}

// a download still sending stays until service() finds it done
void FileTransfer::end(Transfer* t)
{
    shared.give(t);
    if (t->sending())
        return;

    for (Transfer*& slot : transfers) {
        if (slot == t)
            slot = nullptr;
    }
    delete t;
    count_changed();
}

int FileTransfer::hash_piece(FILE* f, MD5& md5, const void* who)
{
    // below stdio, so a whole xbuff goes to the card as one multi-block read
    if (shared.take(who)) {
        int n = ::read(fileno(f), xbuff, sizeof(xbuff));
        if (n > 0)
            md5.update(xbuff, n);

        shared.give(who);
        return n;
    }
    uint8_t own[512];
    int n = ::read(fileno(f), own, sizeof(own));
    if (n > 0)
        md5.update(own, n);

    return n;
}

void FileTransfer::tick()
{
    wake_main();
}

void FileTransfer::count_changed()
{
    for (Transfer* t : transfers) {
        if (t != nullptr) {
            ticker.start();
            return;
        }
    }
    ticker.stop();
}
