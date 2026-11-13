#include "params.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "crc.h"

namespace cubli {
namespace {

const ParamInfo kTable[] = {
#define CUBLI_PARAM_INFO(name, kind, def, lo, hi, help) \
    {#name, ParamKind::kind, static_cast<float>(def), static_cast<float>(lo), static_cast<float>(hi), help, offsetof(Params, name)},
    CUBLI_PARAMS(CUBLI_PARAM_INFO)
#undef CUBLI_PARAM_INFO
};
constexpr size_t kCount = sizeof(kTable) / sizeof(kTable[0]);

uint32_t fnv1a(const char* s) {
    uint32_t h = 2166136261u;
    while (*s) {
        h ^= static_cast<uint8_t>(*s++);
        h *= 16777619u;
    }
    return h;
}

float* field(Params& p, size_t i) {
    return reinterpret_cast<float*>(reinterpret_cast<uint8_t*>(&p) + kTable[i].offset);
}
const float* field(const Params& p, size_t i) {
    return reinterpret_cast<const float*>(reinterpret_cast<const uint8_t*>(&p) + kTable[i].offset);
}

struct BlobHeader {
    uint32_t magic;
    uint16_t count;
    uint16_t crc;
};
struct BlobEntry {
    uint32_t hash;
    float value;
};

}  // namespace

size_t param_count() { return kCount; }
const ParamInfo& param_info(size_t i) { return kTable[i < kCount ? i : 0]; }

int param_find(const char* name) {
    for (size_t i = 0; i < kCount; ++i)
        if (std::strcmp(kTable[i].name, name) == 0) return static_cast<int>(i);
    return -1;
}

void params_set_defaults(Params& p) {
    for (size_t i = 0; i < kCount; ++i) *field(p, i) = kTable[i].def;
}

float param_get(const Params& p, size_t i) { return i < kCount ? *field(p, i) : 0.0f; }

const char* param_set(Params& p, size_t i, float value) {
    if (i >= kCount) return "unknown parameter";
    const ParamInfo& info = kTable[i];
    if (!std::isfinite(value)) return "value is not a number";
    if (info.kind != ParamKind::kFloat) value = std::nearbyint(value);
    if (value < info.min || value > info.max) return "value out of range";
    *field(p, i) = value;
    return nullptr;
}

const char* param_set_text(Params& p, const char* name, const char* text) {
    const int i = param_find(name);
    if (i < 0) return "unknown parameter";
    if (!text || !*text) return "missing value";
    float v;
    if (!std::strcmp(text, "on") || !std::strcmp(text, "true") || !std::strcmp(text, "yes")) {
        v = 1.0f;
    } else if (!std::strcmp(text, "off") || !std::strcmp(text, "false") || !std::strcmp(text, "no")) {
        v = 0.0f;
    } else {
        char* end = nullptr;
        v = std::strtof(text, &end);
        if (end == text || *end != '\0') return "not a number";
    }
    return param_set(p, static_cast<size_t>(i), v);
}

void param_format(const Params& p, size_t i, char* buf, size_t len) {
    if (i >= kCount || len == 0) return;
    const float v = *field(p, i);
    switch (kTable[i].kind) {
        case ParamKind::kBool: std::snprintf(buf, len, "%s", v > 0.5f ? "1" : "0"); break;
        case ParamKind::kInt: std::snprintf(buf, len, "%ld", static_cast<long>(std::lround(v))); break;
        default: std::snprintf(buf, len, "%.6g", static_cast<double>(v)); break;
    }
}

size_t params_blob_size() { return sizeof(BlobHeader) + kCount * sizeof(BlobEntry); }

size_t params_serialize(const Params& p, uint8_t* out, size_t cap) {
    const size_t need = params_blob_size();
    if (cap < need) return 0;
    BlobHeader h{kParamBlobMagic, static_cast<uint16_t>(kCount), 0};
    uint8_t* body = out + sizeof(BlobHeader);
    for (size_t i = 0; i < kCount; ++i) {
        const BlobEntry e{fnv1a(kTable[i].name), *field(p, i)};
        std::memcpy(body + i * sizeof(BlobEntry), &e, sizeof(e));
    }
    h.crc = crc16_ccitt(body, kCount * sizeof(BlobEntry));
    std::memcpy(out, &h, sizeof(h));
    return need;
}

int params_deserialize(Params& p, const uint8_t* in, size_t len) {
    params_set_defaults(p);
    if (!in || len < sizeof(BlobHeader)) return -1;
    BlobHeader h;
    std::memcpy(&h, in, sizeof(h));
    const size_t body_len = static_cast<size_t>(h.count) * sizeof(BlobEntry);
    if (h.magic != kParamBlobMagic || len < sizeof(BlobHeader) + body_len) return -1;
    const uint8_t* body = in + sizeof(BlobHeader);
    if (crc16_ccitt(body, body_len) != h.crc) return -1;
    int applied = 0;
    for (size_t n = 0; n < h.count; ++n) {
        BlobEntry e;
        std::memcpy(&e, body + n * sizeof(BlobEntry), sizeof(e));
        for (size_t i = 0; i < kCount; ++i) {
            if (fnv1a(kTable[i].name) == e.hash) {
                if (param_set(p, i, e.value) == nullptr) ++applied;
                break;
            }
        }
    }
    return applied;
}

}  // namespace cubli
