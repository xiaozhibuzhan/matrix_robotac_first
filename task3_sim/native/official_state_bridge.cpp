// Read-only observation of the unmodified official robot_mujoco process.
// RobotState and eCAL are supplied by the official Ubuntu .deb packages.
// This program never publishes a command, integrates a gait, or steps MuJoCo.
#include <ecal/ecal.h>
#include <ecal/msg/protobuf/subscriber.h>
#include <robot_sdk.pb.h>

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <locale>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>

namespace {
volatile std::sig_atomic_t stopped = 0;
void stop_signal(int) { stopped = 1; }

using Values = google::protobuf::RepeatedField<float>;

void check_values(const Values& values, int count, const char* name) {
  if (values.size() != count) {
    throw std::runtime_error(std::string(name) + " must contain " +
                             std::to_string(count) + " values");
  }
  for (float value : values) {
    if (!std::isfinite(value)) {
      throw std::runtime_error(std::string(name) + " contains a nonfinite value");
    }
  }
}

void write_values(std::ostream& out, const Values& values) {
  out << '[';
  for (int index = 0; index < values.size(); ++index) {
    if (index) out << ',';
    out << values.Get(index);
  }
  out << ']';
}

// Official SDK order is FR, FL, RR, RL. Each leg's MuJoCo joint order is
// ABAD, HIP, KNEE; source protobuf arrays are grouped by joint family.
void write_joints(std::ostream& out, const Values& abad, const Values& hip,
                  const Values& knee) {
  out << '[';
  for (int leg = 0; leg < 4; ++leg) {
    if (leg) out << ',';
    out << abad.Get(leg) << ',' << hip.Get(leg) << ',' << knee.Get(leg);
  }
  out << ']';
}

int parse_port(int argc, char** argv) {
  if (argc != 3 || std::string(argv[1]) != "--port") {
    throw std::runtime_error("usage: task3_official_state_bridge --port PORT");
  }
  std::size_t consumed = 0;
  int port = std::stoi(argv[2], &consumed);
  if (consumed != std::strlen(argv[2]) || port < 1024 || port > 65535) {
    throw std::runtime_error("PORT must be an integer from 1024 to 65535");
  }
  return port;
}

class StateForwarder {
 public:
  explicit StateForwarder(int port) {
    socket_ = ::socket(AF_INET, SOCK_DGRAM | SOCK_CLOEXEC | SOCK_NONBLOCK, 0);
    if (socket_ < 0) throw std::runtime_error("cannot create loopback UDP socket");
    destination_.sin_family = AF_INET;
    destination_.sin_port = htons(static_cast<std::uint16_t>(port));
    destination_.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  }

  ~StateForwarder() { if (socket_ >= 0) ::close(socket_); }

  void receive(const char*, const robot_sdk::pb::RobotState& state,
               long long send_time_us, long long send_clock, long long send_id) {
    // Serialize callbacks even if a second publisher appears. The outer loop
    // also rejects multiple publishers; it must never silently blend robots.
    std::lock_guard<std::mutex> guard(mutex_);
    if (failed_.load()) return;
    try {
      const auto received_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::steady_clock::now().time_since_epoch()).count();
      const auto received_ecal_us = eCAL::Time::GetMicroSeconds();
      const auto received_system_us = std::chrono::duration_cast<std::chrono::microseconds>(
          std::chrono::system_clock::now().time_since_epoch()).count();
      if (send_time_us <= 0 || send_clock <= 0) {
        throw std::runtime_error("official eCAL source time/clock is invalid");
      }
      if (sequence_ && (send_clock <= last_clock_ || send_time_us < last_time_us_)) {
        throw std::runtime_error("official source clock repeated or regressed");
      }
      check_values(state.q_abad(), 4, "q_abad");
      check_values(state.q_hip(), 4, "q_hip");
      check_values(state.q_knee(), 4, "q_knee");
      check_values(state.qd_abad(), 4, "qd_abad");
      check_values(state.qd_hip(), 4, "qd_hip");
      check_values(state.qd_knee(), 4, "qd_knee");
      check_values(state.tau_abad_fb(), 4, "tau_abad_fb");
      check_values(state.tau_hip_fb(), 4, "tau_hip_fb");
      check_values(state.tau_knee_fb(), 4, "tau_knee_fb");
      check_values(state.quat(), 4, "quat");
      check_values(state.gyro(), 3, "gyro");
      check_values(state.acc(), 3, "acc");
      check_values(state.position(), 3, "position");
      check_values(state.v_world(), 3, "v_world");
      double norm_squared = 0.;
      for (float value : state.quat()) norm_squared += value * value;
      if (std::abs(norm_squared - 1.) > .02) {
        throw std::runtime_error("official quaternion is not a unit quaternion");
      }
      last_clock_ = send_clock;
      last_time_us_ = send_time_us;
      // At most 100 Hz, always from this callback's current source sample.
      // There is no timer that can retransmit a previously acquired pose.
      if (sequence_ && received_ns - last_forwarded_ns_ < 10000000) return;

      std::ostringstream out;
      out.imbue(std::locale::classic());
      out << std::setprecision(9)
          << "{\"schema\":\"task3.official_state.v1\",\"bridge_seq\":" << ++sequence_
          << ",\"source_ecal_time_us\":" << send_time_us
          << ",\"source_ecal_clock\":" << send_clock
          // SetID is an application-controlled signed topic filter ID;
          // zero means no filter, and it is NOT a publisher identity.
          << ",\"source_ecal_id\":" << send_id
          // The official protobuf declares uint64 time_stamp without a unit.
          // Keep its exact raw value; zero is not fabricated into simulation time.
          << ",\"source_time_stamp_raw\":" << state.time_stamp()
          << ",\"received_monotonic_ns\":" << received_ns
          // eCAL time may use a loaded synchronization plugin. Forward its
          // receive-side clock too; do not assume a UNIX epoch for send time.
          << ",\"received_ecal_time_us\":" << received_ecal_us
          << ",\"received_system_time_us\":" << received_system_us
          << ",\"ecal_time_synchronized\":"
          << (eCAL::Time::IsSynchronized() ? "true" : "false") << ",\"q\":";
      write_joints(out, state.q_abad(), state.q_hip(), state.q_knee());
      out << ",\"qd\":";
      write_joints(out, state.qd_abad(), state.qd_hip(), state.qd_knee());
      out << ",\"tau\":";
      write_joints(out, state.tau_abad_fb(), state.tau_hip_fb(), state.tau_knee_fb());
      out << ",\"quat_wxyz\":"; write_values(out, state.quat());
      out << ",\"gyro_body\":"; write_values(out, state.gyro());
      out << ",\"acc_body\":"; write_values(out, state.acc());
      out << ",\"imu_position_world\":"; write_values(out, state.position());
      out << ",\"velocity_world\":"; write_values(out, state.v_world());
      out << '}';
      const std::string packet = out.str();
      if (packet.size() > 8192) throw std::runtime_error("state packet is too large");
      const auto sent = ::sendto(socket_, packet.data(), packet.size(), 0,
          reinterpret_cast<const sockaddr*>(&destination_), sizeof(destination_));
      // Dropping a full local UDP buffer is safe: the Python receiver enforces
      // freshness and never integrates a missing sample as new motion.
      if (sent < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == ENOBUFS)) {
        ++dropped_;
      } else if (sent != static_cast<ssize_t>(packet.size())) {
        throw std::runtime_error("cannot send state over loopback UDP");
      }
      last_forwarded_ns_ = received_ns;
    } catch (const std::exception& error) {
      fail(error.what());
    }
  }

  void fail(const std::string& error) {
    if (!failed_.exchange(true)) {
      std::cerr << "TASK3_OFFICIAL_STATE_ERROR " << error << std::endl;
    }
  }
  bool failed() const { return failed_.load(); }

 private:
  int socket_ = -1;
  sockaddr_in destination_{};
  std::mutex mutex_;
  std::atomic<bool> failed_{false};
  std::uint64_t sequence_ = 0;
  std::uint64_t dropped_ = 0;
  long long last_clock_ = 0;
  long long last_time_us_ = 0;
  std::int64_t last_forwarded_ns_ = 0;
};
}  // namespace

int main(int argc, char** argv) {
  bool initialized = false;
  try {
    const int port = parse_port(argc, argv);
    std::signal(SIGINT, stop_signal);
    std::signal(SIGTERM, stop_signal);
    StateForwarder forwarder(port);
    if (eCAL::Initialize(0, nullptr, "task3_official_state_bridge") < 0) {
      throw std::runtime_error("official eCAL initialization failed");
    }
    initialized = true;
    {
      eCAL::protobuf::CSubscriber<robot_sdk::pb::RobotState> subscriber("mujoco_state");
      // Registration-only observer: no receive callback and no command
      // forwarding. Detect another controller appearing during this mission.
      eCAL::protobuf::CSubscriber<robot_sdk::pb::RobotCmd> command_observer("mujoco_cmd");
      if (!subscriber.AddReceiveCallback(
              [&forwarder](const char* topic, const robot_sdk::pb::RobotState& state,
                           long long time_us, long long clock, long long id) {
                forwarder.receive(topic, state, time_us, clock, id);
              })) {
        throw std::runtime_error("cannot subscribe to official mujoco_state");
      }
      std::cerr << "TASK3_OFFICIAL_STATE_READY port=" << port << std::endl;
      while (!stopped && eCAL::Ok() && !forwarder.failed()) {
        if (subscriber.GetPublisherCount() > 1) {
          forwarder.fail("multiple official mujoco_state publishers detected");
        }
        if (command_observer.GetPublisherCount() > 1) {
          forwarder.fail("multiple official mujoco_cmd publishers detected");
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
      command_observer.Destroy();
      subscriber.RemReceiveCallback();
      subscriber.Destroy();
    }
    eCAL::Finalize();
    initialized = false;
    return forwarder.failed() ? 2 : 0;
  } catch (const std::exception& error) {
    std::cerr << "TASK3_OFFICIAL_STATE_ERROR " << error.what() << std::endl;
    if (initialized) eCAL::Finalize();
    return 2;
  }
}
