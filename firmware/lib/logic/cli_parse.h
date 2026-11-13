// Command-line tokenising helpers (shared by the USB and BLE consoles).
#pragma once

namespace cubli {

/// Splits `line` in place on spaces/tabs/CR/LF. The first token (command word)
/// is lower-cased. Returns the number of tokens (<= max).
int cli_tokenize(char* line, char** argv, int max);

/// Strict integer / float parsing with range check. `out` may be null.
bool cli_parse_int(const char* s, long lo, long hi, int* out);
bool cli_parse_float(const char* s, float lo, float hi, float* out);

}  // namespace cubli
