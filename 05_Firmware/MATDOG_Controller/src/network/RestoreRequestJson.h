#ifndef MATDOG_NETWORK_RESTORE_REQUEST_JSON_H
#define MATDOG_NETWORK_RESTORE_REQUEST_JSON_H
// Issue #48 Dashboard body: {"generation":<1..2^32-1>,"confirm":"CONFIRM_CURRENT_INSTALLATION"}
// Strict and bounded: exactly these two keys, no others, no duplicates, no
// whitespace tricks beyond JSON insignificant spaces. A missing or different
// confirm string yields confirmed=false (the policy then refuses with
// OPERATOR_CONFIRMATION_REQUIRED); malformed JSON is rejected outright.
#include <stddef.h>
#include <stdint.h>
#include <string.h>

namespace matdog {
namespace network {

constexpr const char* kRestoreConfirmToken = "CONFIRM_CURRENT_INSTALLATION";

inline bool parseRestoreRequestJson(const char* s, size_t n, uint32_t* generation, bool* confirmed,
                                    const char** reason) {
  auto fail = [&](const char* why) { if (reason) *reason = why; return false; };
  if (!s || !generation || !confirmed || n == 0 || n > 256) return fail("BODY_SIZE");
  *generation = 0; *confirmed = false;
  size_t i = 0;
  auto ws = [&]() { while (i < n && (s[i] == ' ' || s[i] == '\t' || s[i] == '\r' || s[i] == '\n')) ++i; };
  auto lit = [&](char c) { ws(); if (i < n && s[i] == c) { ++i; return true; } return false; };
  auto str = [&](char* out, size_t cap) {
    ws(); if (i >= n || s[i] != '"') return false; ++i;
    size_t k = 0;
    while (i < n && s[i] != '"') {
      const char c = s[i++];
      if (c == '\\' || static_cast<unsigned char>(c) < 0x20 || k + 1 >= cap) return false;
      out[k++] = c;
    }
    if (i >= n) return false;
    ++i; out[k] = 0; return true;
  };
  bool seen_generation = false, seen_confirm = false;
  if (!lit('{')) return fail("JSON_OBJECT_REQUIRED");
  for (int member = 0; member < 2; ++member) {
    char key[16];
    if (!str(key, sizeof(key)) || !lit(':')) return fail("JSON_MEMBER");
    if (!strcmp(key, "generation") && !seen_generation) {
      ws(); uint64_t v = 0; size_t digits = 0;
      while (i < n && s[i] >= '0' && s[i] <= '9' && digits < 11) { v = v * 10 + static_cast<unsigned>(s[i] - '0'); ++i; ++digits; }
      if (!digits || digits > 10 || v == 0 || v > 0xFFFFFFFFull || (digits > 1 && s[i - digits] == '0')) return fail("GENERATION");
      *generation = static_cast<uint32_t>(v); seen_generation = true;
    } else if (!strcmp(key, "confirm") && !seen_confirm) {
      char value[40];
      if (!str(value, sizeof(value))) return fail("CONFIRM");
      *confirmed = !strcmp(value, kRestoreConfirmToken); seen_confirm = true;
    } else {
      return fail("UNKNOWN_OR_DUPLICATE_KEY");
    }
    if (member == 0 && !lit(',')) return fail("TWO_MEMBERS_REQUIRED");
  }
  if (!lit('}')) return fail("JSON_OBJECT_END");
  ws();
  if (i != n) return fail("TRAILING_BYTES");
  if (!seen_generation || !seen_confirm) return fail("TWO_MEMBERS_REQUIRED");
  if (reason) *reason = "OK";
  return true;
}

}  // namespace network
}  // namespace matdog
#endif
