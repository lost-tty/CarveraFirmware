#ifndef SESSION_H
#define SESSION_H

#include "FrameConsole.h"
#include "modules/utils/wifi/Endpoint.h"

#include <string>

class WifiProvider;

class Session : public FrameConsole {
public:
    Session() {}

    void bind(WifiProvider* provider, uint8_t link, const uint8_t ip[4], uint16_t port);
    void release();

    Endpoint who;
    uint8_t link = 0;
    bool live() const { return who.port != 0; }
    bool is(const uint8_t ip[4], uint16_t port) const;

    bool fresh = false;
    static const uint8_t STALL_TICKS = 5;
    uint8_t stall = 0;

    int puts(const char* s, int size = 0) override;
    int gets(char** buf, int size = 0) override;
    bool ready() override;
    void set_transferring(bool f) override;

    const std::string& cwd() const override { return path; }
    void set_cwd(const std::string& p) override { path = p; }

    void pump();

private:
    WifiProvider* owner = nullptr;
    std::string path{"/"};
};

#endif
