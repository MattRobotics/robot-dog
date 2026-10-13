#ifndef MATDOG_MOTION_CONTRACT_H
#define MATDOG_MOTION_CONTRACT_H
#include <stdint.h>
#include <stddef.h>
namespace matdog { namespace motion {
constexpr uint16_t kMotionContractVersion = 1;
constexpr size_t kMotionActionCount = 48;
enum class Intent : uint8_t { STAND, LIE_DOWN, GAIT_SELECT, BASE_VELOCITY, BODY_POSE, ACTION_EXECUTE, STOP, STATUS, CAPABILITIES, UNKNOWN };
enum class ClientOwner : uint8_t { DASHBOARD, JETSON, LOCAL, UNKNOWN };
// Timestamp uses Controller uptime, exposed by STATUS; no wall clock inference.
// Clients estimate uptime from the most recent response and must stop on stale data.
struct MotionCommand {
  uint16_t version = kMotionContractVersion;
  char id[49]{};
  uint32_t sequence = 0, issued_at_ms = 0, ttl_ms = 0;
  ClientOwner owner = ClientOwner::UNKNOWN;
  Intent intent = Intent::UNKNOWN;
  double target_height_m = .150, vx = 0, vy = 0, yaw_rate = 0;
  double x = 0, y = 0, z = 0, roll = 0, pitch = 0, yaw = 0;
  char gait[17]{}, action_id[49]{};
};
enum class CommandOutcome : uint8_t { REJECTED, ACCEPTED, COMPLETED, ABORTED };
struct MotionAck {
  uint16_t version = kMotionContractVersion;
  char id[49]{};
  uint32_t sequence = 0;
  CommandOutcome outcome = CommandOutcome::REJECTED;
  const char* reason = "NOT_INITIALIZED";
  const char* executor_state = "OFF";
  ClientOwner owner = ClientOwner::UNKNOWN;
};
struct MotionAction {
  const char *id, *category, *label, *parameters, *prerequisites, *capability;
  bool implemented;
  Intent intent;
};
const MotionAction* actionCatalog();
const MotionAction* findAction(const char* id);
const char* toString(Intent);
const char* toString(ClientOwner);
const char* toString(CommandOutcome);
Intent parseIntent(const char*);
ClientOwner parseOwner(const char*);
bool validCommand(const MotionCommand&, uint32_t now_ms, const char** reason);
} }
#endif
