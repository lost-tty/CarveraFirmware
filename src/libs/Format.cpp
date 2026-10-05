#include "Format.h"

#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstring>

namespace fmt {

namespace {

struct Out {
    char *buf;
    size_t size;
    size_t n;

    void put(char c)
    {
        if (n + 1 < size)
            buf[n] = c;

        n++;
    }
    void put(const char *s, size_t len)
    {
        for (size_t i = 0; i < len; i++) {
            put(s[i]);
        }
    }
    void fill(char c, int count)
    {
        for (int i = 0; i < count; i++) {
            put(c);
        }
    }
};

enum Length : uint8_t { NONE, HH, H, L, LL, Z, J, T, LD };

struct Spec {
    bool left, plus, space, zero, alt;
    int width;
    int prec;   // -1: none
    Length length;
    char conv;
};

// the sign a value of len characters starts with, 0 for none
char sign(const Spec &s, bool negative, bool signs)
{
    if (negative)
        return '-';

    if (signs && s.plus)
        return '+';

    return signs && s.space ? ' ' : 0;
}

// spaces or zeros up to the width around len characters that begin with the prefix
void pad_left(Out &out, const Spec &s, int len, const char *prefix, size_t plen)
{
    int fill = s.width > len ? s.width - len : 0;
    if (!s.left && !s.zero)
        out.fill(' ', fill);

    out.put(prefix, plen);
    if (!s.left && s.zero)
        out.fill('0', fill);
}

void pad_right(Out &out, const Spec &s, int len)
{
    if (s.left && s.width > len)
        out.fill(' ', s.width - len);
}

void pad_body(Out &out, const Spec &s, const char *prefix, size_t plen, const char *body, size_t blen,
              int zeros)
{
    int len = (int)(plen + blen) + zeros;
    pad_left(out, s, len, prefix, plen);
    out.fill('0', zeros);
    out.put(body, blen);
    pad_right(out, s, len);
}

__attribute__((noinline)) void put_integer(Out &out, Spec s, uint64_t value, bool negative)
{
    const char *digits = s.conv == 'X' ? "0123456789ABCDEF" : "0123456789abcdef";
    unsigned base = s.conv == 'x' || s.conv == 'X' || s.conv == 'p' ? 16 : 10;
    bool zero = value == 0;
    char body[24];
    char *end = body + sizeof(body), *p = end;
    // a 64-bit division is a library call on the M3: only the part above 32 bits takes it
    while (value > UINT32_MAX) {
        *--p = digits[value % base];
        value /= base;
    }
    for (uint32_t v = (uint32_t)value; v != 0; v /= base) {
        *--p = digits[v % base];
    }
    if (zero && s.prec != 0)
        *--p = '0';

    size_t blen = (size_t)(end - p);
    char prefix[3];
    size_t plen = 0;
    if (char c = sign(s, negative, s.conv == 'd' || s.conv == 'i'))
        prefix[plen++] = c;

    if (s.conv == 'p' || (s.alt && base == 16 && !zero)) {
        prefix[plen++] = '0';
        prefix[plen++] = s.conv == 'X' ? 'X' : 'x';
    }
    int zeros = s.prec > (int)blen ? s.prec - (int)blen : 0;
    if (s.prec >= 0)
        s.zero = false;

    pad_body(out, s, prefix, plen, p, blen, zeros);
}

// The decimal digits of a double, exactly: the integer part, then the fraction without end.
// One array holds the fraction's bits, or the integer part's base-1e9 chunks at its top end.
class Exact {
public:
    explicit Exact(double v)
    {
        uint64_t bits;
        memcpy(&bits, &v, sizeof(bits));
        int e = (int)((bits >> 52) & 0x7ff);
        uint64_t m = bits & ((1ULL << 52) - 1);
        int exp2 = e == 0 ? -1074 : e - 1075;
        if (e != 0)
            m |= 1ULL << 52;

        memset(w, 0, sizeof(w));
        nwords = shift = nchunks = first_len = 0;
        top = false;
        at = 0;
        int_digits = 0;
        if (m == 0)
            return;

        if (exp2 >= 0) {
            integer_from_big(m, exp2);
        } else {
            shift = -exp2;
            uint64_t whole = shift >= 64 ? 0 : m >> shift;
            uint64_t frac = shift >= 64 ? m : m & ((1ULL << shift) - 1);
            nwords = shift / 32 + 2;
            w[0] = (uint32_t)frac;
            w[1] = (uint32_t)(frac >> 32);
            if (whole >= 1000000000ULL)
                small[nchunks++] = (uint32_t)(whole / 1000000000ULL);

            if (whole != 0)
                small[nchunks++] = (uint32_t)(whole % 1000000000ULL);
        }
        if (nchunks > 0) {
            first_len = count_digits(chunk(0));
            int_digits = first_len + 9 * (nchunks - 1);
        }
    }

    int int_digits;   // 0: the integer part is zero

    int next()
    {
        if (at < int_digits)
            return int_digit(at++);

        at++;
        return frac_digit();
    }

    bool rest_zero() const
    {
        for (int i = at; i < int_digits; i++) {
            if (int_digit(i) != 0)
                return false;
        }
        for (int i = 0; i < nwords; i++) {
            if (w[i] != 0)
                return false;
        }
        return true;
    }

private:
    static const int k_words = 36;   // 1074 fraction bits and the digit above them; 2^1024 in base 1e9
    uint32_t w[k_words];
    uint32_t small[2];
    int nwords, shift, nchunks, first_len, at;
    bool top;   // the chunks are at the top of w

    uint32_t chunk(int i) const
    {
        return top ? w[k_words - nchunks + i] : small[i];
    }

    static int count_digits(uint32_t v)
    {
        int n = 1;
        while (v >= 10) {
            v /= 10;
            n++;
        }
        return n;
    }

    int int_digit(int i) const
    {
        int c = 0, pos = i, len = first_len;
        if (i >= first_len) {
            c = 1 + (i - first_len) / 9;
            pos = (i - first_len) % 9;
            len = 9;
        }
        uint32_t v = chunk(c);
        for (int k = len - 1 - pos; k > 0; k--) {
            v /= 10;
        }
        return (int)(v % 10);
    }

    // m << exp2 in w, divided down by 1e9; each remainder goes to the top, where the quotient has left
    void integer_from_big(uint64_t m, int exp2)
    {
        int ww = exp2 / 32, b = exp2 % 32;
        w[ww] = (uint32_t)(m << b);
        w[ww + 1] = (uint32_t)(b == 0 ? m >> 32 : m >> (32 - b));
        if (b != 0)
            w[ww + 2] = (uint32_t)(m >> (64 - b));

        int n = ww + 3;
        while (n > 0 && w[n - 1] == 0) {
            n--;
        }
        int k = 0;
        while (n > 0) {
            uint64_t rem = 0;
            for (int i = n - 1; i >= 0; i--) {
                uint64_t cur = (rem << 32) | w[i];
                w[i] = (uint32_t)(cur / 1000000000ULL);
                rem = cur % 1000000000ULL;
            }
            while (n > 0 && w[n - 1] == 0) {
                n--;
            }
            w[k_words - 1 - k] = (uint32_t)rem;
            k++;
        }
        // the last remainder, the most significant, landed lowest
        nchunks = k;
        top = true;
    }

    int frac_digit()
    {
        if (nwords == 0)
            return 0;

        uint64_t carry = 0;
        for (int i = 0; i < nwords; i++) {
            uint64_t t = (uint64_t)w[i] * 10 + carry;
            w[i] = (uint32_t)t;
            carry = t >> 32;
        }
        int ww = shift / 32, b = shift % 32;
        uint32_t d;
        if (b == 0) {
            d = w[ww];
            w[ww] = 0;
        } else {
            d = (w[ww] >> b) | (w[ww + 1] << (32 - b));
            w[ww] &= (1u << b) - 1;
            w[ww + 1] = 0;
        }
        return (int)d;
    }
};

// rounding to a digit count: up or not, from the last digit that is not a 9
struct Rounding {
    bool up;
    int carry_at;   // -1: all nines, a new leading 1
    int last_nonzero;
};

Rounding round_digits(int r, bool sticky, int last_digit, int last_non9, int last_nonzero)
{
    Rounding out;
    out.up = r > 5 || (r == 5 && (sticky || (last_digit & 1)));
    out.carry_at = last_non9;
    out.last_nonzero = out.up ? (last_non9 < 0 ? 0 : last_non9) : last_nonzero;
    return out;
}

int apply(const Rounding &rd, int i, int d)
{
    if (rd.up && i == rd.carry_at)
        return d + 1;

    if (rd.up && i > rd.carry_at)
        return 0;

    return d;
}

// the significant digits: the decimal exponent of the first, and how they round
struct Sig {
    bool zero;
    int exp10;
    Rounding rd;
};

int skip_to_first(Exact &x, int &first)
{
    if (x.int_digits > 0) {
        first = x.next();
        return x.int_digits - 1;
    }
    int lead = 0;
    do {
        first = x.next();
        lead++;
    } while (first == 0);
    return -lead;
}

// each expansion in a frame of its own, so a writer's never stands on top of another
__attribute__((noinline)) Sig significant(double v, int nsig)
{
    Sig s{v == 0, 0, Rounding{false, 0, 0}};
    if (s.zero)
        return s;

    Exact x(v);
    int first;
    s.exp10 = skip_to_first(x, first);
    int last_non9 = -1, last_nonzero = -1, last = 0;
    for (int i = 0; i < nsig; i++) {
        int d = i == 0 ? first : x.next();
        if (d != 9)
            last_non9 = i;

        if (d != 0)
            last_nonzero = i;

        last = d;
    }
    int r = x.next();
    s.rd = round_digits(r, !x.rest_zero(), last, last_non9, last_nonzero < 0 ? 0 : last_nonzero);
    if (s.rd.up && s.rd.carry_at < 0)
        s.exp10++;

    return s;
}

// the rounded significant digits again, from a fresh expansion
class SigReader {
public:
    SigReader(double v, const Sig &s) : x(v), s(s), i(0), first(0)
    {
        if (!s.zero)
            skip_to_first(x, first);
    }
    int next()
    {
        int idx = i++;
        if (s.rd.up && s.rd.carry_at < 0)
            return idx == 0 ? 1 : 0;

        int d = s.zero ? 0 : (idx == 0 ? first : x.next());
        return apply(s.rd, idx, d);
    }

private:
    Exact x;
    const Sig &s;
    int i;
    int first;
};

void put_special(Out &out, Spec s, double v, bool negative)
{
    bool upper = s.conv == 'F' || s.conv == 'E' || s.conv == 'G';
    const char *word = std::isnan(v) ? (upper ? "NAN" : "nan") : (upper ? "INF" : "inf");
    // newlib-nano: a nan takes the + or space of the spec, never a minus
    char c = sign(s, negative && !std::isnan(v), true);
    s.zero = false;
    pad_body(out, s, &c, c ? 1 : 0, word, 3, 0);
}

struct Fixed {
    int nint;
    Rounding rd;
};

__attribute__((noinline)) Fixed fixed_rounding(double v, int prec)
{
    Exact x(v);
    Fixed f;
    f.nint = x.int_digits > 0 ? x.int_digits : 1;
    int total = f.nint + prec;
    int last_non9 = -1, last = 0;
    for (int i = 0; i < total; i++) {
        int d = (i < f.nint && x.int_digits == 0) ? 0 : x.next();
        if (d != 9)
            last_non9 = i;

        last = d;
    }
    int r = x.next();
    f.rd = round_digits(r, !x.rest_zero(), last, last_non9, 0);
    return f;
}

__attribute__((noinline)) void put_fixed(Out &out, const Spec &s, double v, bool negative, int prec,
                                         const Fixed &f)
{
    int total = f.nint + prec;
    bool point = prec > 0 || s.alt;
    bool extra = f.rd.up && f.rd.carry_at < 0;
    char c = sign(s, negative, true);
    int len = (c ? 1 : 0) + total + (extra ? 1 : 0) + (point ? 1 : 0);

    pad_left(out, s, len, &c, c ? 1 : 0);
    Exact x(v);
    if (extra)
        out.put('1');

    for (int i = 0; i < total; i++) {
        int d = (i < f.nint && x.int_digits == 0) ? 0 : x.next();
        out.put((char)('0' + apply(f.rd, i, d)));
        if (i == f.nint - 1 && point)
            out.put('.');
    }
    pad_right(out, s, len);
}

void put_exponent_digits(char *buf, int &n, int exp10, bool upper)
{
    buf[n++] = upper ? 'E' : 'e';
    buf[n++] = exp10 < 0 ? '-' : '+';
    int e = exp10 < 0 ? -exp10 : exp10;
    char tmp[4];
    int t = 0;
    do {
        tmp[t++] = (char)('0' + e % 10);
        e /= 10;
    } while (e != 0);
    if (t < 2)
        tmp[t++] = '0';

    while (t > 0) {
        buf[n++] = tmp[--t];
    }
}

// d.ddde+xx with prec digits after the point
__attribute__((noinline)) void put_exp_style(Out &out, const Spec &s, double v, bool negative, int prec,
                                             bool strip, const Sig &sig)
{
    int keep = strip && !s.alt ? sig.rd.last_nonzero : prec;
    bool point = keep > 0 || s.alt;
    char ex[8];
    int nex = 0;
    put_exponent_digits(ex, nex, sig.exp10, s.conv == 'E' || s.conv == 'G');
    char c = sign(s, negative, true);
    int len = (c ? 1 : 0) + 1 + keep + (point ? 1 : 0) + nex;

    pad_left(out, s, len, &c, c ? 1 : 0);
    SigReader rd(v, sig);
    for (int i = 0; i <= keep; i++) {
        out.put((char)('0' + rd.next()));
        if (i == 0 && point)
            out.put('.');
    }
    out.put(ex, nex);
    pad_right(out, s, len);
}

// %g in fixed form: the same p significant digits, trailing zeros dropped unless #
__attribute__((noinline)) void put_general_fixed(Out &out, const Spec &s, double v, bool negative, int p,
                                                 const Sig &sig)
{
    int x = sig.exp10;
    int last = s.alt ? p - 1 : sig.rd.last_nonzero;   // the last significant digit written
    int int_len = x >= 0 ? x + 1 : 1;
    int frac_len = last > x ? last - x : 0;          // after the point, leading zeros included
    bool point = frac_len > 0 || s.alt;
    char c = sign(s, negative, true);
    int len = (c ? 1 : 0) + int_len + (point ? 1 : 0) + frac_len;

    pad_left(out, s, len, &c, c ? 1 : 0);
    SigReader rd(v, sig);
    for (int i = 0; i < int_len; i++) {
        out.put(x >= 0 ? (char)('0' + rd.next()) : '0');
    }
    if (point)
        out.put('.');

    for (int i = 0; i < frac_len; i++) {
        out.put(x + 1 + i < 0 ? '0' : (char)('0' + rd.next()));
    }
    pad_right(out, s, len);
}

__attribute__((noinline)) void put_float(Out &out, Spec s, double v)
{
    bool negative = std::signbit(v);
    if (negative)
        v = -v;

    if (!std::isfinite(v)) {
        put_special(out, s, v, negative);
        return;
    }
    if (s.conv == 'f' || s.conv == 'F') {
        int prec = s.prec < 0 ? 6 : s.prec;
        Fixed f = fixed_rounding(v, prec);
        put_fixed(out, s, v, negative, prec, f);
        return;
    }
    if (s.conv == 'e' || s.conv == 'E') {
        int prec = s.prec < 0 ? 6 : s.prec;
        Sig sig = significant(v, prec + 1);
        put_exp_style(out, s, v, negative, prec, false, sig);
        return;
    }
    int p = s.prec < 0 ? 6 : (s.prec == 0 ? 1 : s.prec);
    Sig sig = significant(v, p);
    if (p > sig.exp10 && sig.exp10 >= -4) {
        put_general_fixed(out, s, v, negative, p, sig);
    } else {
        put_exp_style(out, s, v, negative, p - 1, true, sig);
    }
}

const char *read_spec(const char *f, Spec &s, va_list &args)
{
    s = Spec{false, false, false, false, false, 0, -1, NONE, 0};
    for (;; f++) {
        if (*f == '-') {
            s.left = true;
        } else if (*f == '+') {
            s.plus = true;
        } else if (*f == ' ') {
            s.space = true;
        } else if (*f == '0') {
            s.zero = true;
        } else if (*f == '#') {
            s.alt = true;
        } else {
            break;
        }
    }
    if (*f == '*') {
        s.width = va_arg(args, int);
        if (s.width < 0) {
            s.left = true;
            s.width = -s.width;
        }
        f++;
    } else {
        while (isdigit((unsigned char)*f)) {
            s.width = s.width * 10 + (*f++ - '0');
        }
    }
    if (*f == '.') {
        f++;
        s.prec = 0;
        if (*f == '*') {
            s.prec = va_arg(args, int);
            if (s.prec < 0)
                s.prec = -1;

            f++;
        } else {
            while (isdigit((unsigned char)*f)) {
                s.prec = s.prec * 10 + (*f++ - '0');
            }
        }
    }
    if (*f == 'h' || *f == 'l') {
        bool twice = f[1] == *f;
        s.length = *f == 'h' ? (twice ? HH : H) : (twice ? LL : L);
        f += twice ? 2 : 1;
    } else if (*f == 'z' || *f == 'j' || *f == 't' || *f == 'L') {
        s.length = *f == 'z' ? Z : *f == 'j' ? J : *f == 't' ? T : LD;
        f++;
    }
    s.conv = *f;
    return *f ? f + 1 : f;
}

// the integer argument as its spec has it, made unsigned
void put_int_arg(Out &out, const Spec &s, va_list &args, bool is_signed)
{
    long long v;
    switch (s.length) {
        case LL: v = va_arg(args, long long); break;
        case L: v = is_signed ? va_arg(args, long) : (long long)va_arg(args, unsigned long); break;
        case Z: v = (long long)va_arg(args, size_t); break;
        case J: v = va_arg(args, intmax_t); break;
        case T: v = va_arg(args, ptrdiff_t); break;
        default: v = is_signed ? va_arg(args, int) : (long long)va_arg(args, unsigned); break;
    }
    if (s.length == H)
        v = is_signed ? (long long)(short)v : (long long)(unsigned short)v;

    if (s.length == HH)
        v = is_signed ? (long long)(signed char)v : (long long)(unsigned char)v;

    bool negative = is_signed && v < 0;
    put_integer(out, s, negative ? 0 - (uint64_t)v : (uint64_t)v, negative);
}

bool starts_nocase(const char *p, const char *word)
{
    for (; *word; p++, word++) {
        if (tolower((unsigned char)*p) != *word)
            return false;
    }
    return true;
}

}   // namespace

int vformat(char *buf, size_t size, const char *format, va_list args)
{
    Out out{buf, size, 0};
    va_list ap;
    va_copy(ap, args);
    for (const char *f = format; *f;) {
        if (*f != '%') {
            out.put(*f++);
            continue;
        }
        const char *start = f;
        Spec s;
        f = read_spec(f + 1, s, ap);
        switch (s.conv) {
            case '%':
                out.put('%');
                break;
            case 'd':
            case 'i':
                put_int_arg(out, s, ap, true);
                break;
            case 'u':
            case 'x':
            case 'X':
                put_int_arg(out, s, ap, false);
                break;
            case 'p':
                put_integer(out, s, (uintptr_t)va_arg(ap, void *), false);
                break;
            case 'c': {
                char c = (char)va_arg(ap, int);
                s.zero = false;
                pad_body(out, s, "", 0, &c, 1, 0);
                break;
            }
            case 's': {
                const char *str = va_arg(ap, const char *);
                if (str == nullptr)
                    str = "(null)";

                size_t len = 0;
                while (str[len] && (s.prec < 0 || (int)len < s.prec)) {
                    len++;
                }
                s.zero = false;
                pad_body(out, s, "", 0, str, len, 0);
                break;
            }
            case 'f':
            case 'F':
            case 'e':
            case 'E':
            case 'g':
            case 'G':
                put_float(out, s, s.length == LD ? (double)va_arg(ap, long double) : va_arg(ap, double));
                break;
            default:
                // seen at once rather than swallowing its argument
                out.put("<?", 2);
                out.put(start, (size_t)(f - start));
                out.put('>');
                break;
        }
    }
    va_end(ap);
    if (size > 0)
        buf[out.n < size ? out.n : size - 1] = 0;

    return (int)out.n;
}

int format(char *buf, size_t size, const char *format, ...)
{
    va_list args;
    va_start(args, format);
    int n = vformat(buf, size, format, args);
    va_end(args);
    return n;
}

double parse(const char *s, char **end)
{
    const char *p = s;
    while (isspace((unsigned char)*p)) {
        p++;
    }
    bool negative = false;
    if (*p == '+' || *p == '-') {
        negative = *p == '-';
        p++;
    }
    if (starts_nocase(p, "inf") || starts_nocase(p, "nan")) {
        bool nan = tolower((unsigned char)*p) == 'n';
        p += 3;
        if (!nan && starts_nocase(p, "inity"))
            p += 5;

        if (end)
            *end = (char *)p;

        double v = nan ? NAN : INFINITY;
        return negative ? -v : v;
    }

    uint64_t mant = 0;
    int digits = 0, exp10 = 0;
    bool any = false;
    for (; isdigit((unsigned char)*p); p++) {
        any = true;
        if (digits < 19) {
            mant = mant * 10 + (uint64_t)(*p - '0');
            if (mant != 0)
                digits++;
        } else {
            exp10++;
        }
    }
    if (*p == '.') {
        const char *q = p + 1;
        for (; isdigit((unsigned char)*q); q++) {
            any = true;
            if (digits < 19) {
                mant = mant * 10 + (uint64_t)(*q - '0');
                if (mant != 0)
                    digits++;

                exp10--;
            }
        }
        if (any)
            p = q;
    }
    if (!any) {
        if (end)
            *end = (char *)s;

        return 0;
    }
    if (*p == 'e' || *p == 'E') {
        const char *q = p + 1;
        bool eneg = false;
        if (*q == '+' || *q == '-') {
            eneg = *q == '-';
            q++;
        }
        if (isdigit((unsigned char)*q)) {
            int e = 0;
            for (; isdigit((unsigned char)*q); q++) {
                if (e < 10000)
                    e = e * 10 + (*q - '0');
            }
            exp10 += eneg ? -e : e;
            p = q;
        }
    }
    if (end)
        *end = (char *)p;

    // exact powers of ten: one rounding when both factors are exact
    static const double k_pow10[] = {1e0,  1e1,  1e2,  1e3,  1e4,  1e5,  1e6,  1e7,
                                     1e8,  1e9,  1e10, 1e11, 1e12, 1e13, 1e14, 1e15,
                                     1e16, 1e17, 1e18, 1e19, 1e20, 1e21, 1e22};
    double v;
    if (mant == 0) {
        v = 0;
    } else if (mant < (1ULL << 53) && exp10 >= -22 && exp10 <= 22) {
        v = exp10 >= 0 ? (double)mant * k_pow10[exp10] : (double)mant / k_pow10[-exp10];
    } else {
        v = (double)mant;
        int e = exp10;
        while (e >= 22) {
            v *= 1e22;
            e -= 22;
        }
        while (e <= -22) {
            v /= 1e22;
            e += 22;
        }
        v = e >= 0 ? v * k_pow10[e] : v / k_pow10[-e];
    }
    return negative ? -v : v;
}

}   // namespace fmt

#ifndef FORMAT_HOST_TEST
// newlib's versions share one state across tasks: these take their place at link time
extern "C" {

int vsnprintf(char *buf, size_t size, const char *format, va_list args)
{
    return fmt::vformat(buf, size, format, args);
}

int snprintf(char *buf, size_t size, const char *format, ...)
{
    va_list args;
    va_start(args, format);
    int n = fmt::vformat(buf, size, format, args);
    va_end(args);
    return n;
}

int vsprintf(char *buf, const char *format, va_list args)
{
    return fmt::vformat(buf, SIZE_MAX, format, args);
}

int sprintf(char *buf, const char *format, ...)
{
    va_list args;
    va_start(args, format);
    int n = fmt::vformat(buf, SIZE_MAX, format, args);
    va_end(args);
    return n;
}

// newlib's integer-only variants, which strftime uses
int vsniprintf(char *, size_t, const char *, va_list) __attribute__((alias("vsnprintf")));
int sniprintf(char *, size_t, const char *, ...) __attribute__((alias("snprintf")));
int vsiprintf(char *, const char *, va_list) __attribute__((alias("vsprintf")));
int siprintf(char *, const char *, ...) __attribute__((alias("sprintf")));

double strtod(const char *s, char **end)
{
    return fmt::parse(s, end);
}

float strtof(const char *s, char **end)
{
    return (float)fmt::parse(s, end);
}

double atof(const char *s)
{
    return fmt::parse(s, nullptr);
}

}
#endif
