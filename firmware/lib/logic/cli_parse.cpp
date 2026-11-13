#include "cli_parse.h"

#include <cctype>
#include <cmath>
#include <cstdlib>

namespace cubli {

int cli_tokenize(char* line, char** argv, int max) {
    int argc = 0;
    char* p = line;
    while (*p && argc < max) {
        while (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n') *p++ = '\0';
        if (!*p) break;
        argv[argc++] = p;
        while (*p && *p != ' ' && *p != '\t' && *p != '\r' && *p != '\n') ++p;
    }
    while (*p) {  // terminate trailing whitespace of the last token
        if (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n') *p = '\0';
        ++p;
    }
    if (argc > 0) {
        for (char* c = argv[0]; *c; ++c) *c = static_cast<char>(std::tolower(static_cast<unsigned char>(*c)));
    }
    return argc;
}

bool cli_parse_int(const char* s, long lo, long hi, int* out) {
    if (!s || !*s) return false;
    char* end = nullptr;
    const long v = std::strtol(s, &end, 10);
    if (*end != '\0' || v < lo || v > hi) return false;
    if (out) *out = static_cast<int>(v);
    return true;
}

bool cli_parse_float(const char* s, float lo, float hi, float* out) {
    if (!s || !*s) return false;
    char* end = nullptr;
    const float v = std::strtof(s, &end);
    if (*end != '\0' || !std::isfinite(v) || v < lo || v > hi) return false;
    if (out) *out = v;
    return true;
}

}  // namespace cubli
