// Проверка входного потока на соответствие контракту v1 (docs/INPUT_FORMAT.md).
// На детекцию не влияет: только считает, пишет предупреждения и сводку. Без ROS 2 --
// тот же код работает в узле и офлайн по записям (python -m evaluation input).
#pragma once

#include <cstdint>
#include <map>
#include <string>
#include <utility>
#include <vector>

#include "tunnel_od_preproc/canonical.hpp"

namespace tunnel_od
{

enum class Level : uint8_t { Ok = 0, Warn = 1, Error = 2, Unknown = 3 };   // как DiagnosticStatus

struct Value
{
  std::string key;
  std::string text;
  bool number = false;       // в JSON без кавычек
};

struct Check
{
  std::string id;            // "M1", "M4", "S1", ...
  std::string title;
  Level level = Level::Unknown;
  std::string message;
  std::vector<Value> values;
};

struct FrameInfo
{
  int64_t stamp_ns = 0;          // header.stamp
  std::string frame_id;
  CloudDesc cloud;
  std::string layout_signature;
  ParseStats stats;
  double recv_s = 0.0;           // время приёма (любые монотонные часы), для частоты
  std::string error;             // непусто -- кадр не разобран
};

struct MonitorConfig
{
  Axes configured_axes = Axes::Auto;
  Format configured_format = Format::Auto;
  double period_s = 0.1;         // M4: шаг штампов 100 +- 1 мс
  double period_tol_s = 0.001;
  double gap_factor = 1.5;       // разрыв -- шаг больше 1.5 периода
  int min_year = 2020;           // штамп раньше -- часы не синхронизированы
  double min_half_sector_deg = 50.0;
  double sector_tol_deg = 1.0;
  double rate_tol_hz = 0.5;
  double axes_warn_deg = 45.0;   // середина сектора дальше от "вперёд" -- оси, видимо, не те
  uint32_t grace_frames = 20;    // сколько кадров ждать TF / описание / скорость до предупреждения
  bool speed_expected = false;   // задан speed_topic
  bool description_expected = true;
};

class InputMonitor
{
public:
  explicit InputMonitor(MonitorConfig cfg = {});

  void on_frame(const FrameInfo & f);
  // TF base_link -> frame_id из /tf_static: перенос, м, и крен/тангаж/рыскание, град
  void on_tf(const std::string & parent, const std::string & child, const double xyz[3],
    const double rpy_deg[3]);
  void on_speed(double stamp_s, double speed_mps);
  void on_description(const std::string & text);

  std::vector<Check> checks() const;
  // Предупреждения, появившиеся впервые с прошлого вызова (каждое -- один раз за работу).
  std::vector<std::string> take_new_warnings();
  std::string summary_json() const;
  uint64_t frames() const {return frames_;}

private:
  void warn(const std::string & key, const std::string & text, uint64_t add = 1);
  void evaluate_late();

  MonitorConfig cfg_;
  uint64_t frames_ = 0, rejected_ = 0;
  std::string last_error_;
  // формат
  std::map<std::string, uint64_t> formats_;   // "legacy_hesai/legacy" -> кадров
  std::string frame_id_, signature_;
  uint32_t width_ = 0, height_ = 0;
  uint64_t frame_id_changes_ = 0, layout_changes_ = 0, width_changes_ = 0;
  // M1
  uint64_t n_in_ = 0, n_zero_ = 0, n_nonfinite_ = 0, unordered_frames_ = 0, ring_bad_frames_ = 0;
  uint64_t dense_lies_ = 0, ring_frames_ = 0;
  // M2
  uint64_t pairs_valid_ = 0, pairs_coincident_ = 0, dedupe_frames_ = 0;
  uint64_t n_second_ = 0, n_second_valid_ = 0, n_second_coincident_ = 0, rid_frames_ = 0;
  // M4
  bool have_prev_ = false;
  int64_t prev_stamp_ = 0, first_stamp_ = 0;
  uint64_t nonmonotonic_ = 0, off_step_ = 0, gaps_ = 0;
  double gap_total_s_ = 0.0, gap_max_s_ = 0.0;
  std::vector<double> dts_, sweep_, t_first_;
  uint64_t no_time_frames_ = 0;
  // M5
  std::vector<double> az_min_, az_max_, az_center_;
  double first_recv_ = 0.0, last_recv_ = 0.0;
  // M3
  bool tf_found_ = false;
  std::string tf_parent_;
  double tf_xyz_[3] = {0, 0, 0}, tf_rpy_[3] = {0, 0, 0};
  std::vector<std::string> tf_children_;
  // M6, S1
  bool description_ = false;
  std::string description_text_;
  uint64_t speed_msgs_ = 0;
  double speed_first_ = 0.0, speed_last_t_ = 0.0, speed_last_ = 0.0;
  // предупреждения
  std::map<std::string, uint64_t> warn_counts_;
  std::map<std::string, std::string> warn_text_;
  std::vector<std::string> new_warnings_;
};

std::string json_escape(const std::string & s);
const char * to_string(Level l);

}  // namespace tunnel_od

namespace tunnel_od
{
// Статистика разбора одного кадра в JSON (для input_meta и ctypes).
std::string to_json(const ParseStats & s);
}  // namespace tunnel_od
