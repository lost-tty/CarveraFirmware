#include "ConfigTable.h"
#include "PinSpec.h"
#include <cstdio>
#include <cstring>
#include <string>
#include <cmath>

// A copy of remove_non_number() from utils.cpp, so ConfigTable builds alone in host tests.
static std::string strip_non_number(const char *text)
{
    static const char *mask = "0123456789-.abcdefpxABCDEFPX";
    std::string s(text);
    size_t found = s.find_first_not_of(mask);
    while (found != std::string::npos) {
        s.erase(found, 1);
        found = s.find_first_not_of(mask);
    }
    return s;
}

static char *arena;       // The built structs, or nullptr.
static int arena_users;   // Depth of nested build() calls.
static const char *source = "/sd/config.txt";   // The path build() was last given.

static size_t aligned(size_t n) { return (n + 3) & ~(size_t)3; }

#define FOR_EACH_GROUP(g) \
    for (const ConfigTable::Group *const *m_ = config_groups; *m_ != nullptr; m_++) \
        for (const ConfigTable::Group *g = *m_; g->prefix != nullptr; g++)

static bool joins_flat(const char *prefix)
{
    size_t n = strlen(prefix);
    return n > 0 && prefix[n - 1] == '_';
}

// A root group (empty prefix) matches the whole name.
bool ConfigTable::find(const char *name, const Group **out_group, const Key **out_key)
{
    const Group *best_group = nullptr;
    const Key *best_key = nullptr;
    size_t best_prefix_len = 0;

    FOR_EACH_GROUP(g) {
        size_t plen = strlen(g->prefix);
        const char *rest;
        if (plen == 0) {
            rest = name;
        } else if (joins_flat(g->prefix)) {
            if (strncmp(name, g->prefix, plen) != 0) continue;
            rest = name + plen;
        } else {
            if (strncmp(name, g->prefix, plen) != 0) continue;
            if (name[plen] != '.') continue;
            rest = name + plen + 1;
        }
        for (uint8_t i = 0; i < g->n; i++) {
            if (strcmp(g->keys[i].name, rest) == 0) {
                // The longest matching prefix wins, so a switch key is never taken for a root key.
                if (plen >= best_prefix_len) {
                    best_group = g;
                    best_key = &g->keys[i];
                    best_prefix_len = plen;
                }
            }
        }
    }

    if (best_key == nullptr) return false;
    *out_group = best_group;
    *out_key = best_key;
    return true;
}

// Parses "M801", "G28" or "". A subcode such as G28.1 is ignored.
static uint16_t parse_gcode(const char *text)
{
    text += strspn(text, " \t");
    char letter = *text;
    if (letter != 'M' && letter != 'G' && letter != 'm' && letter != 'g') {
        return ConfigTable::GCODE_NONE;
    }
    char *end;
    long n = strtol(text + 1, &end, 10);
    if (end == text + 1 || n < 0 || n >= ConfigTable::GCODE_G) return ConfigTable::GCODE_NONE;
    return (letter == 'G' || letter == 'g') ? (n | ConfigTable::GCODE_G) : n;
}

static void parse_field(char *field, const ConfigTable::Key *key, const char *text)
{
    switch (key->type) {
        case ConfigTable::FLOAT: {
            std::string s = strip_non_number(text);
            char *endptr = nullptr;
            float v = strtof(s.c_str(), &endptr);
            if (endptr > s.c_str()) memcpy(field, &v, sizeof(v));
            break;
        }
        case ConfigTable::INT: {
            std::string s = strip_non_number(text);
            char *endptr = nullptr;
            long v = strtol(s.c_str(), &endptr, 10);
            if (endptr > s.c_str()) { int32_t iv = (int32_t)v; memcpy(field, &iv, sizeof(iv)); }
            break;
        }
        case ConfigTable::BOOL: {
            bool v = strpbrk(text, "ty1") != nullptr;
            memcpy(field, &v, sizeof(v));
            break;
        }
        case ConfigTable::STR:
            strncpy(field, text, key->size - 1);
            field[key->size - 1] = '\0';
            break;
        case ConfigTable::PIN: {
            uint16_t s = PinSpec::parse(text);
            memcpy(field, &s, sizeof(s));
            break;
        }
        case ConfigTable::ENUM: {
            uint8_t v = ConfigTable::ENUM_INVALID;
            for (uint8_t i = 0; key->names[i] != nullptr; i++) {
                if (strcmp(key->names[i], text) == 0) { v = i; break; }
            }
            *(uint8_t *)field = v;
            break;
        }
        case ConfigTable::GCODE: {
            uint16_t v = parse_gcode(text);
            memcpy(field, &v, sizeof(v));
            break;
        }
        case ConfigTable::FLOATS: {
            // Missing values of "x,y,z" are NAN, so "" leaves the whole list unset.
            const char *p = text;
            for (uint8_t i = 0; i < key->size; i++) {
                float v = NAN;
                char *end;
                if (*p) {
                    v = strtof(p, &end);
                    if (end == p) v = NAN;
                    p = end + strspn(end, ", \t");
                }
                memcpy(field + i * sizeof(float), &v, sizeof(v));
            }
            break;
        }
    }
}

static void format_field(const char *field, const ConfigTable::Key *key, char *buf, size_t bufsize)
{
    switch (key->type) {
        case ConfigTable::FLOAT: {
            float v; memcpy(&v, field, sizeof(v));
            snprintf(buf, bufsize, "%g", v);
            break;
        }
        case ConfigTable::INT: {
            int32_t v; memcpy(&v, field, sizeof(v));
            snprintf(buf, bufsize, "%ld", (long)v);
            break;
        }
        case ConfigTable::BOOL: {
            bool v; memcpy(&v, field, sizeof(v));
            snprintf(buf, bufsize, "%s", v ? "true" : "false");
            break;
        }
        case ConfigTable::STR:
            snprintf(buf, bufsize, "%s", field);
            break;
        case ConfigTable::PIN: {
            uint16_t s; memcpy(&s, field, sizeof(s));
            PinSpec::format(s, buf, bufsize);
            break;
        }
        case ConfigTable::ENUM: {
            uint8_t v = *(const uint8_t *)field;
            snprintf(buf, bufsize, "%s", v == ConfigTable::ENUM_INVALID ? "?" : key->names[v]);
            break;
        }
        case ConfigTable::GCODE: {
            uint16_t v; memcpy(&v, field, sizeof(v));
            if (v == ConfigTable::GCODE_NONE) {
                snprintf(buf, bufsize, "%s", "");
            } else {
                snprintf(buf, bufsize, "%c%u", (v & ConfigTable::GCODE_G) ? 'G' : 'M',
                         (unsigned)(v & ~ConfigTable::GCODE_G));
            }
            break;
        }
        case ConfigTable::FLOATS: {
            size_t n = 0;
            buf[0] = '\0';
            for (uint8_t i = 0; i < key->size; i++) {
                float v; memcpy(&v, field + i * sizeof(float), sizeof(v));
                if (std::isnan(v)) break;
                n += snprintf(buf + n, n < bufsize ? bufsize - n : 0, "%s%g", i ? "," : "", v);
            }
            break;
        }
    }
}

void ConfigTable::set_field(void *base, const Key *key, const char *text)
{
    parse_field((char *)base + key->off, key, text);
}

static void store_value(char *field, const ConfigTable::Key *key, ConfigTable::Value v)
{
    switch (key->type) {
        case ConfigTable::FLOAT: memcpy(field, &v.f, sizeof(v.f)); break;
        case ConfigTable::INT:   memcpy(field, &v.i, sizeof(v.i)); break;
        case ConfigTable::BOOL: {
            bool b = v.i != 0;
            memcpy(field, &b, sizeof(b));
            break;
        }
        default: parse_field(field, key, v.s); break;
    }
}

void ConfigTable::set_value(void *base, const Key *key, Value v)
{
    store_value((char *)base + key->off, key, v);
}




// A group with size 0 shares the struct of the last sized group before it.
char *ConfigTable::base(const Group *group)
{
    if (arena == nullptr) return nullptr;
    size_t off = 0, cur = 0;
    FOR_EACH_GROUP(g) {
        if (g->size != 0) { cur = off; off += aligned(g->size); }
        if (g == group) return arena + cur;
    }
    return nullptr;
}

const void *ConfigTable::block(const Group *group)
{
    return base(group);
}

// An instance override takes precedence over the key table's default.
static ConfigTable::Value default_value(const ConfigTable::Group *group,
    const ConfigTable::Key *key)
{
    ConfigTable::Value v = key->dflt;
    for (uint8_t i = 0; i < group->n_ov; i++) {
        if (group->ov[i].off == key->off) v = group->ov[i].v;
    }
    return v;
}

bool ConfigTable::build(const char *path)
{
    source = path;
    if (arena_users++ > 0) return true;
    size_t total = 0;
    FOR_EACH_GROUP(g) total += aligned(g->size);
    arena = (char *)calloc(total ? total : 1, 1);
    if (arena == nullptr) { arena_users = 0; return false; }
    FOR_EACH_GROUP(g) {
        char *b = base(g);
        for (uint8_t i = 0; i < g->n; i++) set_value(b, &g->keys[i], default_value(g, &g->keys[i]));
    }
    load(path);
    return true;
}

void ConfigTable::release()
{
    if (arena_users == 0 || --arena_users > 0) return;
    free(arena);
    arena = nullptr;
}

void ConfigTable::notify(const char *name)
{
    const Group *g; const Key *k;
    if (!find(name, &g, &k) || g->changed == nullptr) return;
    g->changed(g, base(g));
}

bool ConfigTable::set(const char *path, const char *name, const char *value)
{
    if (!rewrite(path, name, value)) return false;
    if (build(path)) { notify(name); release(); }
    return true;
}

bool ConfigTable::unset(const char *path, const char *name)
{
    if (!remove_key(path, name)) return false;
    if (build(path)) { notify(name); release(); }
    return true;
}

void ConfigTable::format_default(const Group *group, const Key *key, char *buf, size_t bufsize)
{
    Value v = default_value(group, key);
    alignas(float) char field[MAX_FIELD];
    store_value(field, key, v);
    format_field(field, key, buf, bufsize);
}

bool ConfigTable::get(const char *name, char *buf, size_t bufsize)
{
    const Group *g; const Key *k;
    if (!find(name, &g, &k) || !build(source)) return false;
    format(g, k, buf, bufsize);
    release();
    return true;
}

void ConfigTable::format(const Group *group, const Key *key, char *buf, size_t bufsize)
{
    format_field(base(group) + key->off, key, buf, bufsize);
}

// A value ends at whitespace, '#' or the end of the line.
void ConfigTable::parse_line(const char *line)
{
    if (line[0] == '#') return;
    size_t len = strlen(line);
    if (len < 3) return;

    size_t begin_key = strspn(line, " \t");
    if (line[begin_key] == '\0' || line[begin_key] == '#') return;

    size_t end_key = begin_key + strcspn(line + begin_key, " \t");
    if (line[end_key] == '\0') return;

    size_t begin_value = end_key + strspn(line + end_key, " \t");
    if (line[begin_value] == '\0' || line[begin_value] == '#') return;

    size_t end_value = begin_value + strcspn(line + begin_value, "\r\n# \t");

    char key[132];
    size_t klen = end_key - begin_key;
    if (klen >= sizeof(key)) klen = sizeof(key) - 1;
    memcpy(key, line + begin_key, klen);
    key[klen] = '\0';

    char value[132];
    size_t vlen = end_value - begin_value;
    if (vlen >= sizeof(value)) vlen = sizeof(value) - 1;
    memcpy(value, line + begin_value, vlen);
    value[vlen] = '\0';

    const Group *g; const Key *k;
    // Unregistered keys are ignored; config.txt also holds MakeraStudio's settings.
    if (find(key, &g, &k)) set_field(base(g), k, value);
}

// Copies the rest of an over-long line, including its newline.
static void copy_rest(FILE *in, FILE *out)
{
    int c;
    while ((c = fgetc(in)) != EOF) {
        fputc(c, out);
        if (c == '\n') return;
    }
}

static void skip_rest(FILE *in)
{
    int c;
    while ((c = fgetc(in)) != '\n' && c != EOF) { }
}

// Takes back <path>.bak left by an edit that lost power between its two renames.
static void recover(const char *path)
{
    char bak_path[160];
    snprintf(bak_path, sizeof(bak_path), "%s.bak", path);
    FILE *fp = fopen(path, "r");
    if (fp != nullptr) { fclose(fp); return; }
    rename(bak_path, path);
}

void ConfigTable::load(const char *path)
{
    recover(path);
    FILE *fp = fopen(path, "r");
    if (fp == nullptr) return;

    char line[132];
    while (fgets(line, sizeof(line), fp) != nullptr) {
        // An over-long line keeps its key and value in the buffer; the rest is skipped.
        size_t n = strlen(line);
        if (n > 0 && line[n - 1] != '\n' && !feof(fp)) {
            int c;
            while ((c = fgetc(fp)) != '\n' && c != EOF) { /* discard */ }
        }
        parse_line(line);
    }
    fclose(fp);
}

void ConfigTable::for_each(EachFn fn, void *user)
{
    char name[132];
    FOR_EACH_GROUP(g) {
        for (uint8_t i = 0; i < g->n; i++) {
            if (g->prefix[0] == '\0' || joins_flat(g->prefix)) {
                snprintf(name, sizeof(name), "%s%s", g->prefix, g->keys[i].name);
            } else {
                snprintf(name, sizeof(name), "%s.%s", g->prefix, g->keys[i].name);
            }
            fn(name, g, &g->keys[i], user);
        }
    }
}

static bool edit_file(const char *path, const char *setting, const char *value)
{
    char tmp_path[160], bak_path[160];
    snprintf(tmp_path, sizeof(tmp_path), "%s.tmp", path);
    snprintf(bak_path, sizeof(bak_path), "%s.bak", path);
    recover(path);

    FILE *in = fopen(path, "r");
    FILE *out = fopen(tmp_path, "w");
    if (out == nullptr) { if (in) fclose(in); return false; }

    bool found = false;
    char line[132];
    if (in != nullptr) {
        while (fgets(line, sizeof(line), in) != nullptr) {
            // An over-long line continues in the stream; its key is always in this part.
            size_t n = strlen(line);
            bool partial = n > 0 && line[n - 1] != '\n';

            size_t begin_key = strspn(line, " \t");
            size_t end_key = begin_key + strcspn(line + begin_key, " \t");
            bool is_comment_or_blank = (line[begin_key] == '#' || line[begin_key] == '\0'
                || line[begin_key] == '\n' || line[begin_key] == '\r');
            bool match = !is_comment_or_blank && line[end_key] != '\0'
                && strncmp(line + begin_key, setting, end_key - begin_key) == 0
                && strlen(setting) == end_key - begin_key;

            if (!match) {
                fputs(line, out);
                if (partial) copy_rest(in, out);
                continue;
            }
            if (value == nullptr || found) {
                found = true;
                if (partial) skip_rest(in);
                continue;
            }
            found = true;
            size_t begin_value = end_key + strspn(line + end_key, " \t");
            size_t comment_at = begin_value + strcspn(line + begin_value, "#\r\n");
            int c = line[comment_at];
            if (c == '#') {
                fprintf(out, "%s\t%s\t%s", setting, value, line + comment_at);
                if (partial) copy_rest(in, out);
            } else if (c == '\0' && partial) {
                // The old value runs past the buffer, and its comment may follow.
                while ((c = fgetc(in)) != EOF && c != '#' && c != '\n') { }
                if (c == '#') {
                    fprintf(out, "%s\t%s\t#", setting, value);
                    copy_rest(in, out);
                } else {
                    fprintf(out, "%s\t%s\n", setting, value);
                }
            } else {
                fprintf(out, "%s\t%s\n", setting, value);
                if (partial) skip_rest(in);
            }
        }
        fclose(in);
    }

    if (!found && value != nullptr) {
        fprintf(out, "%s\t%s\t# added\n", setting, value);
    }
    bool written = !ferror(out);
    if (fclose(out) != 0) written = false;
    if (!written) {
        remove(tmp_path);
        return false;
    }

    // FAT cannot rename over an existing file, so the old one is moved aside first.
    remove(bak_path);
    bool moved = rename(path, bak_path) == 0;
    if (rename(tmp_path, path) != 0) {
        if (moved) rename(bak_path, path);
        remove(tmp_path);
        return false;
    }
    remove(bak_path);
    return true;
}

bool ConfigTable::rewrite(const char *path, const char *setting, const char *value)
{
    return edit_file(path, setting, value);
}

bool ConfigTable::remove_key(const char *path, const char *setting)
{
    return edit_file(path, setting, nullptr);
}
