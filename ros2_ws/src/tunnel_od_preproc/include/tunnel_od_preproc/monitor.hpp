#pragma once

#include <cstdint>
#include <map>
#include <string>
#include <utility>
#include <vector>

#include "tunnel_od_preproc/canonical.hpp"

namespace tunnel_od
{

enum class Level : uint8_t { Ok = 0, Warn = 1, Error = 2, Unknown = 3 };

struct Value
{
  std::string key;
  std::string text;
  bool number = false;
};

struct Check
{
  std::string id;
  std::string title;
  Level level = Level::Unknown;
  std::string message;
  std::vector<Value> values;
};

struct FrameInfo
{
  int64_t stamp_ns = 0;
  std::string frame_id;
  CloudDesc cloud;
  std::string layout_signature;
  ParseStats stats;
  double recv_s = 0.0;
  std::string error;
};

struct MonitorConfig
{
  Axes configured_axes = Axes::Auto;
  Format configured_format = Format::Auto;
  double period_s = 0.1;
  double period_tol_s = 0.001;
  double gap_factor = 1.5;
  int min_year = 2020;
  double min_half_sector_deg = 50.0;
  double sector_tol_deg = 1.0;
  double rate_tol_hz = 0.5;
  double axes_warn_deg = 45.0;
  uint32_t grace_frames = 20;
  bool speed_expected = false;
  bool description_expected = true;
};

class InputMonitor
{
public:
  explicit InputMonitor(MonitorConfig cfg = {});

  void on_frame(const FrameInfo & f);
  void on_tf(const std::string & parent, const std::string & child, const double xyz[3],
    const double rpy_deg[3]);
  void on_speed(double stamp_s, double speed_mps);
  void on_description(const std::string & text);

  std::vector<Check> checks() const;
  std::vector<std::string> take_new_warnings();
  std::string summary_json() const;
  uint64_t frames() const {return frames_;}

private:
  void warn(const std::string & key, const std::string & text, uint64_t add = 1);
  void evaluate_late();

  MonitorConfig cfg_;
  uint64_t frames_ = 0, rejected_ = 0;
  std::string last_error_;
  std::map<std::string, uint64_t> formats_;
  std::string frame_id_, signature_;
  uint32_t width_ = 0, height_ = 0;
  uint64_t frame_id_changes_ = 0, layout_changes_ = 0, width_changes_ = 0;
  uint64_t n_in_ = 0, n_zero_ = 0, n_nonfinite_ = 0, unordered_frames_ = 0, ring_bad_frames_ = 0;
  uint64_t dense_lies_ = 0, ring_frames_ = 0;
  uint64_t pairs_valid_ = 0, pairs_coincident_ = 0, dedupe_frames_ = 0;
  uint64_t n_second_ = 0, n_second_valid_ = 0, n_second_coincident_ = 0, rid_frames_ = 0;
  bool have_prev_ = false;
  int64_t prev_stamp_ = 0, first_stamp_ = 0;
  uint64_t nonmonotonic_ = 0, off_step_ = 0, gaps_ = 0;
  double gap_total_s_ = 0.0, gap_max_s_ = 0.0;
  std::vector<double> dts_, sweep_, t_first_;
  uint64_t no_time_frames_ = 0;
  std::vector<double> az_min_, az_max_, az_center_;
  double first_recv_ = 0.0, last_recv_ = 0.0;
  bool tf_found_ = false;
  std::string tf_parent_;
  double tf_xyz_[3] = {0, 0, 0}, tf_rpy_[3] = {0, 0, 0};
  std::vector<std::string> tf_children_;
  bool description_ = false;
  std::string description_text_;
  uint64_t speed_msgs_ = 0;
  double speed_first_ = 0.0, speed_last_t_ = 0.0, speed_last_ = 0.0;
  std::map<std::string, uint64_t> warn_counts_;
  std::map<std::string, std::string> warn_text_;
  std::vector<std::string> new_warnings_;
};

std::string json_escape(const std::string & s);
const char * to_string(Level l);

}

namespace tunnel_od
{
std::string to_json(const ParseStats & s);
}
