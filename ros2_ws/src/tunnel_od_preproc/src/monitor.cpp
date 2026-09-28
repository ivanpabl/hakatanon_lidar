#include "tunnel_od_preproc/monitor.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <ctime>
#include <sstream>

namespace tunnel_od
{

namespace
{

double median(std::vector<double> v)
{
  if (v.empty()) {return std::nan("");}
  const size_t m = v.size() / 2;
  std::nth_element(v.begin(), v.begin() + m, v.end());
  double hi = v[m];
  if (v.size() % 2) {return hi;}
  return 0.5 * (hi + *std::max_element(v.begin(), v.begin() + m));
}

std::string num(double v, int prec = 3)
{
  if (!std::isfinite(v)) {return "null";}
  char buf[64];
  std::snprintf(buf, sizeof(buf), "%.*f", prec, v);
  return buf;
}

std::string num(uint64_t v) {return std::to_string(v);}

Value nv(const std::string & k, const std::string & v) {return Value{k, v, true};}
Value sv(const std::string & k, const std::string & v) {return Value{k, v, false};}

int year_of(int64_t stamp_ns)
{
  std::time_t t = static_cast<std::time_t>(stamp_ns / 1000000000LL);
  std::tm tm{};
#ifdef _WIN32
  gmtime_s(&tm, &t);
#else
  gmtime_r(&t, &tm);
#endif
  return tm.tm_year + 1900;
}

Level worst(Level a, Level b)
{
  if (a == Level::Unknown) {return b;}
  if (b == Level::Unknown) {return a;}
  return static_cast<uint8_t>(a) > static_cast<uint8_t>(b) ? a : b;
}

}

const char * to_string(Level l)
{
  switch (l) {
    case Level::Ok: return "ok";
    case Level::Warn: return "warn";
    case Level::Error: return "error";
    case Level::Unknown: return "unknown";
  }
  return "?";
}

std::string json_escape(const std::string & s)
{
  std::string o;
  o.reserve(s.size() + 2);
  for (unsigned char c : s) {
    switch (c) {
      case '"': o += "\\\""; break;
      case '\\': o += "\\\\"; break;
      case '\n': o += "\\n"; break;
      case '\t': o += "\\t"; break;
      default:
        if (c < 0x20) {
          char buf[8];
          std::snprintf(buf, sizeof(buf), "\\u%04x", c);
          o += buf;
        } else {
          o += static_cast<char>(c);
        }
    }
  }
  return o;
}

InputMonitor::InputMonitor(MonitorConfig cfg)
: cfg_(cfg) {}

void InputMonitor::warn(const std::string & key, const std::string & text, uint64_t add)
{
  auto it = warn_counts_.find(key);
  if (it == warn_counts_.end()) {
    warn_counts_[key] = add;
    warn_text_[key] = text;
    new_warnings_.push_back(text);
  } else {
    it->second += add;
  }
}

std::vector<std::string> InputMonitor::take_new_warnings()
{
  std::vector<std::string> out;
  out.swap(new_warnings_);
  return out;
}

void InputMonitor::on_frame(const FrameInfo & f)
{
  ++frames_;
  if (frames_ == 1) {first_recv_ = f.recv_s;}
  last_recv_ = f.recv_s;

  if (frames_ > 1 && f.frame_id != frame_id_) {
    ++frame_id_changes_;
    warn("frame_id_change", "frame_id сменился посреди потока: '" + frame_id_ + "' -> '" + f.frame_id + "'");
  }
  if (frames_ > 1 && f.layout_signature != signature_) {
    ++layout_changes_;
    warn("layout_change", "раскладка точки (fields) сменилась посреди потока");
  }
  if (frames_ > 1 && (f.cloud.width != width_ || f.cloud.height != height_)) {
    ++width_changes_;
    warn("width_change", "размер облака сменился посреди потока: " + std::to_string(height_) + "x" +
      std::to_string(width_) + " -> " + std::to_string(f.cloud.height) + "x" + std::to_string(f.cloud.width));
  }
  frame_id_ = f.frame_id;
  signature_ = f.layout_signature;
  width_ = f.cloud.width;
  height_ = f.cloud.height;
  if (f.frame_id.empty()) {warn("frame_id_empty", "M3: пустой header.frame_id");}

  if (frames_ == 1) {
    first_stamp_ = f.stamp_ns;
    const int y = year_of(f.stamp_ns);
    if (y < cfg_.min_year) {
      warn("clock", "M4: header.stamp в " + std::to_string(y) + " году -- часы датчика не синхронизированы");
    }
  }
  if (have_prev_) {
    const double dt = (f.stamp_ns - prev_stamp_) * 1e-9;
    dts_.push_back(dt);
    if (dt <= 0) {
      ++nonmonotonic_;
      warn("nonmonotonic", "M4: штампы немонотонны");
    } else if (dt > cfg_.gap_factor * cfg_.period_s) {
      ++gaps_;
      gap_total_s_ += dt - cfg_.period_s;
      gap_max_s_ = std::max(gap_max_s_, dt);
      warn("gap", "M4: разрыв в штампах " + num(dt, 2) + " с (потерянные кадры)");
    } else if (std::fabs(dt - cfg_.period_s) > cfg_.period_tol_s) {
      ++off_step_;
      warn("step", "M4: шаг штампов " + num(dt * 1e3, 1) + " мс вне 100 +- 1 мс");
    }
  }
  have_prev_ = true;
  prev_stamp_ = f.stamp_ns;

  if (!f.error.empty()) {
    ++rejected_;
    last_error_ = f.error;
    warn("rejected", "облако не разобрано: " + f.error);
    return;
  }
  const ParseStats & s = f.stats;
  formats_[std::string(to_string(s.format)) + "/" + to_string(s.axes)]++;
  if (s.format == Format::Generic) {
    warn("generic", "формат не распознан (generic): только x, y, z, без удаления дублей; "
      "оси -- " + std::string(to_string(s.axes)) + ", задайте input_format / input_axes");
  }

  n_in_ += s.n_in;
  n_zero_ += s.n_zero;
  n_nonfinite_ += s.n_nonfinite;
  if (f.cloud.height != kColumnHeight) {
    ++unordered_frames_;
    warn("unordered", "M1: облако неупорядоченное (height = " + std::to_string(f.cloud.height) +
      ", нужно 128 строк по ring)");
  }
  if (s.n_zero) {warn("zeros", "M1: пустые лучи -- (0, 0, 0) вместо NaN");}
  if (f.cloud.is_dense && (s.n_zero + s.n_nonfinite)) {
    ++dense_lies_;
    warn("dense", "M1: is_dense = true, но в облаке есть пустые лучи");
  }
  if (s.ring_checked) {
    ++ring_frames_;
    if (s.ring_mismatch) {
      ++ring_bad_frames_;
      warn("ring", "M1: ring не совпадает с номером строки/канала в столбце");
    }
  }

  if (s.dedupe_applied) {
    ++dedupe_frames_;
    pairs_valid_ += s.pairs_both_valid;
    pairs_coincident_ += s.pairs_coincident;
    warn("dual_by_distance", "M2: нет return_id -- дубли dual return удаляются по совпадению "
      "соседних столбцов (< 1 см)");
  }
  if (s.n_second) {
    ++rid_frames_;
    n_second_ += s.n_second;
    n_second_valid_ += s.n_second_valid;
    n_second_coincident_ += s.n_second_coincident;
    if (s.n_second_coincident) {
      warn("second_coincident", "M2: второе отражение совпадает с первым, а должно быть NaN",
        s.n_second_coincident);
    }
  }

  if (s.have_point_time) {
    sweep_.push_back(s.t_max - s.t_min);
    t_first_.push_back(s.t_min);
    if (s.t_min < -0.001) {
      warn("point_before_stamp", "M4: время точек раньше header.stamp (stamp -- не начало развёртки)");
    }
  } else {
    ++no_time_frames_;
    warn("no_point_time", "M4: нет времени точки (t_offset_ns / timestamp)");
  }

  if (s.n_az) {
    az_min_.push_back(s.az_min);
    az_max_.push_back(s.az_max);
    az_center_.push_back(s.az_center);
    if (std::fabs(s.az_center) > cfg_.axes_warn_deg) {
      warn("axes", "оси: середина сектора в " + num(s.az_center, 0) + " град от направления вперёд -- "
        "проверьте input_axes (сейчас " + to_string(s.axes) + "); оси не переключаются");
    }
  }
  evaluate_late();
}

void InputMonitor::evaluate_late()
{
  if (frames_ < cfg_.grace_frames) {return;}
  if (!tf_found_) {
    warn("no_tf", "M3: нет /tf_static base_link -> '" + frame_id_ + "' (высота и наклон лидара неизвестны)");
  }
  if (cfg_.description_expected && !description_) {
    warn("no_description", "M6: нет описания датчика");
  }
  if (cfg_.speed_expected && speed_msgs_ == 0) {
    warn("no_speed", "S1: speed_topic задан, но скорость не приходит");
  }
  if (frames_ == cfg_.grace_frames && !az_min_.empty()) {
    const double lo = median(az_min_), hi = median(az_max_);
    const double need = cfg_.min_half_sector_deg - cfg_.sector_tol_deg;
    if (lo > -need || hi < need) {
      warn("sector", "M5: сектор " + num(lo, 0) + ".." + num(hi, 0) + " град, нужно не меньше +-50 град");
    }
  }
}

void InputMonitor::on_tf(const std::string & parent, const std::string & child, const double xyz[3],
  const double rpy_deg[3])
{
  if (std::find(tf_children_.begin(), tf_children_.end(), child) == tf_children_.end()) {
    tf_children_.push_back(child);
  }
  tf_found_ = true;
  tf_parent_ = parent;
  for (int i = 0; i < 3; ++i) {tf_xyz_[i] = xyz[i]; tf_rpy_[i] = rpy_deg[i];}
}

void InputMonitor::on_speed(double stamp_s, double speed_mps)
{
  if (speed_msgs_ == 0) {speed_first_ = stamp_s;}
  ++speed_msgs_;
  speed_last_t_ = stamp_s;
  speed_last_ = speed_mps;
}

void InputMonitor::on_description(const std::string & text)
{
  description_ = true;
  description_text_ = text;
}

std::vector<Check> InputMonitor::checks() const
{
  std::vector<Check> out;
  const bool any = frames_ > rejected_;

  {
    Check c{"format", "распознанный формат и оси", Level::Unknown, "", {}};
    if (!frames_) {
      c.message = "кадров ещё нет";
    } else {
      std::string fm;
      uint64_t best = 0;
      for (auto & kv : formats_) {
        if (kv.second > best) {best = kv.second; fm = kv.first;}
        c.values.push_back(nv("frames:" + kv.first, num(kv.second)));
      }
      c.level = fm.rfind("generic", 0) == 0 ? Level::Warn : Level::Ok;
      if (rejected_) {c.level = Level::Error;}
      c.message = fm.empty() ? "нет разобранных кадров" : fm;
      c.values.push_back(nv("frames", num(frames_)));
      c.values.push_back(nv("rejected", num(rejected_)));
      if (!last_error_.empty()) {c.values.push_back(sv("last_error", last_error_));}
      c.values.push_back(sv("frame_id", frame_id_));
      c.values.push_back(nv("height", num(static_cast<uint64_t>(height_))));
      c.values.push_back(nv("width", num(static_cast<uint64_t>(width_))));
      c.values.push_back(nv("frame_id_changes", num(frame_id_changes_)));
      c.values.push_back(nv("layout_changes", num(layout_changes_)));
      c.values.push_back(nv("size_changes", num(width_changes_)));
      if (frame_id_changes_ || layout_changes_ || width_changes_) {c.level = worst(c.level, Level::Warn);}
      if (!az_center_.empty()) {
        const double ctr = median(az_center_);
        c.values.push_back(nv("sector_center_deg", num(ctr, 1)));
        if (std::fabs(ctr) > cfg_.axes_warn_deg) {
          c.level = worst(c.level, Level::Warn);
          c.message += "; середина сектора не впереди -- проверьте input_axes";
        }
      }
    }
    out.push_back(c);
  }

  {
    Check c{"M1", "упорядоченное облако, NaN для пустых лучей", Level::Unknown, "", {}};
    if (any) {
      const double zero = n_in_ ? double(n_zero_) / n_in_ : 0.0;
      const double nonf = n_in_ ? double(n_nonfinite_) / n_in_ : 0.0;
      std::vector<std::string> bad;
      if (unordered_frames_) {bad.push_back("неупорядоченное (height=" + std::to_string(height_) + ")");}
      if (n_zero_) {bad.push_back("нули вместо NaN");}
      if (ring_bad_frames_) {bad.push_back("ring не по строкам");}
      if (dense_lies_) {bad.push_back("is_dense=true при пустых лучах");}
      c.level = bad.empty() ? Level::Ok : Level::Warn;
      for (size_t i = 0; i < bad.size(); ++i) {c.message += (i ? "; " : "") + bad[i];}
      if (bad.empty()) {c.message = "соответствует";}
      c.values = {nv("unordered_frames", num(unordered_frames_)), nv("empty_share", num(zero + nonf, 3)),
        nv("zero_share", num(zero, 3)), nv("nan_share", num(nonf, 3)),
        nv("ring_checked_frames", num(ring_frames_)), nv("ring_mismatch_frames", num(ring_bad_frames_)),
        nv("dense_mismatch_frames", num(dense_lies_))};
    }
    out.push_back(c);
  }

  {
    Check c{"M2", "режим отражений", Level::Unknown, "", {}};
    if (any) {
      if (dedupe_frames_) {
        const double share = pairs_valid_ ? double(pairs_coincident_) / pairs_valid_ : 0.0;
        c.level = Level::Warn;
        c.message = "dual return без return_id, дубли по расстоянию; совпало " + num(100 * share, 1) + "% пар";
        c.values = {sv("mode", "dual_by_distance"), nv("pair_coincident_share", num(share, 4)),
          nv("frames", num(dedupe_frames_))};
      } else if (rid_frames_) {
        const double nan_share = n_second_ ? 1.0 - double(n_second_valid_) / n_second_ : 0.0;
        c.level = n_second_coincident_ ? Level::Warn : Level::Ok;
        c.message = n_second_coincident_ ? "второе отражение совпадает с первым, а не NaN" :
          "dual return по return_id";
        c.values = {sv("mode", "return_id"), nv("second_nan_share", num(nan_share, 4)),
          nv("second_coincident_points", num(n_second_coincident_))};
      } else {
        c.level = Level::Ok;
        c.message = "single return";
        c.values = {sv("mode", "single")};
      }
    }
    out.push_back(c);
  }

  {
    Check c{"M3", "оси и /tf_static", Level::Unknown, "", {}};
    if (tf_found_) {
      c.level = Level::Ok;
      c.message = tf_parent_ + " -> " + frame_id_;
      c.values = {sv("parent", tf_parent_), nv("x", num(tf_xyz_[0])), nv("y", num(tf_xyz_[1])),
        nv("z", num(tf_xyz_[2])), nv("roll_deg", num(tf_rpy_[0], 2)), nv("pitch_deg", num(tf_rpy_[1], 2)),
        nv("yaw_deg", num(tf_rpy_[2], 2))};
    } else if (frames_) {
      c.level = frames_ >= cfg_.grace_frames ? Level::Warn : Level::Unknown;
      c.message = "нет /tf_static для '" + frame_id_ + "'";
      std::string seen;
      for (auto & ch : tf_children_) {seen += (seen.empty() ? "" : ",") + ch;}
      c.values = {sv("tf_children_seen", seen)};
    }
    if (frame_id_changes_) {
      c.level = worst(c.level, Level::Warn);
      c.message += "; frame_id менялся";
    }
    out.push_back(c);
  }

  {
    Check c{"M4", "время: синхронизация, шаг 100 +- 1 мс, время точек", Level::Unknown, "", {}};
    if (frames_) {
      const int y = year_of(first_stamp_);
      std::vector<std::string> bad;
      if (y < cfg_.min_year) {bad.push_back("часы не синхронизированы (" + std::to_string(y) + " год)");}
      if (nonmonotonic_) {bad.push_back(std::to_string(nonmonotonic_) + " немонотонных штампов");}
      if (gaps_) {bad.push_back(std::to_string(gaps_) + " разрывов, " + num(gap_total_s_, 1) + " с");}
      if (off_step_) {bad.push_back(std::to_string(off_step_) + " шагов вне 100 +- 1 мс");}
      if (no_time_frames_) {bad.push_back("нет времени точек");}
      c.level = bad.empty() ? Level::Ok : Level::Warn;
      for (size_t i = 0; i < bad.size(); ++i) {c.message += (i ? "; " : "") + bad[i];}
      if (bad.empty()) {c.message = "соответствует";}
      c.values = {nv("stamp_year", std::to_string(y)), nv("nonmonotonic", num(nonmonotonic_)),
        nv("gaps", num(gaps_)), nv("gap_total_s", num(gap_total_s_, 2)), nv("gap_max_s", num(gap_max_s_, 2)),
        nv("off_step", num(off_step_)), nv("step_median_ms", num(median(dts_) * 1e3, 2)),
        nv("sweep_ms_median", num(median(sweep_) * 1e3, 1)),
        nv("first_point_after_stamp_ms", num(median(t_first_) * 1e3, 2)),
        nv("frames_without_point_time", num(no_time_frames_))};
    }
    out.push_back(c);
  }

  {
    Check c{"M5", "сектор и частота", Level::Unknown, "", {}};
    if (!az_min_.empty()) {
      const double lo = median(az_min_), hi = median(az_max_);
      const double step = median(dts_);
      const double hz = step > 0 ? 1.0 / step : std::nan("");
      const double rx = last_recv_ > first_recv_ ? (frames_ - 1) / (last_recv_ - first_recv_) : std::nan("");
      const double need = cfg_.min_half_sector_deg - cfg_.sector_tol_deg;
      std::vector<std::string> bad;
      if (lo > -need || hi < need) {bad.push_back("сектор уже +-50 град");}
      if (std::isfinite(hz) && std::fabs(hz - 10.0) > cfg_.rate_tol_hz) {bad.push_back("частота не 10 Гц");}
      c.level = bad.empty() ? Level::Ok : Level::Warn;
      c.message = "сектор " + num(lo, 0) + ".." + num(hi, 0) + " град, " + num(hz, 1) + " Гц";
      for (auto & b : bad) {c.message += "; " + b;}
      c.values = {nv("az_min_deg", num(lo, 1)), nv("az_max_deg", num(hi, 1)),
        nv("sector_deg", num(hi - lo, 1)), nv("rate_hz_stamp", num(hz, 2)),
        nv("rate_hz_received", num(rx, 2))};
    }
    out.push_back(c);
  }

  {
    Check c{"M6", "описание датчика", Level::Unknown, "", {}};
    if (description_) {
      c.level = Level::Ok;
      c.message = "получено";
      c.values = {nv("bytes", num(static_cast<uint64_t>(description_text_.size())))};
    } else if (frames_ >= cfg_.grace_frames && cfg_.description_expected) {
      c.level = Level::Warn;
      c.message = "нет";
    }
    out.push_back(c);
  }

  {
    Check c{"S1", "скорость поезда", Level::Unknown, "", {}};
    if (speed_msgs_) {
      const double span = speed_last_t_ - speed_first_;
      const double hz = span > 0 ? (speed_msgs_ - 1) / span : std::nan("");
      c.level = std::isfinite(hz) && hz < 10.0 - cfg_.rate_tol_hz ? Level::Warn : Level::Ok;
      c.message = num(speed_last_, 2) + " м/с, " + num(hz, 1) + " Гц";
      c.values = {nv("messages", num(speed_msgs_)), nv("rate_hz", num(hz, 2)), nv("last_mps", num(speed_last_))};
    } else if (cfg_.speed_expected) {
      c.level = frames_ >= cfg_.grace_frames ? Level::Warn : Level::Unknown;
      c.message = "speed_topic задан, сообщений нет";
    } else {
      c.message = "speed_topic не задан";
    }
    out.push_back(c);
  }
  return out;
}

std::string InputMonitor::summary_json() const
{
  std::ostringstream o;
  const auto cs = checks();
  o << "{\"frames\":" << frames_ << ",\"rejected\":" << rejected_ << ",\"violations\":[";
  bool first = true;
  for (auto & c : cs) {
    if (c.level == Level::Warn || c.level == Level::Error) {
      o << (first ? "" : ",") << '"' << json_escape(c.id) << '"';
      first = false;
    }
  }
  o << "],\"checks\":{";
  for (size_t i = 0; i < cs.size(); ++i) {
    const auto & c = cs[i];
    o << (i ? "," : "") << '"' << json_escape(c.id) << "\":{\"level\":\"" << to_string(c.level)
      << "\",\"title\":\"" << json_escape(c.title) << "\",\"message\":\"" << json_escape(c.message) << '"';
    for (auto & v : c.values) {
      o << ",\"" << json_escape(v.key) << "\":";
      if (v.number) {o << v.text;} else {o << '"' << json_escape(v.text) << '"';}
    }
    o << '}';
  }
  o << "},\"warnings\":{";
  first = true;
  for (auto & kv : warn_counts_) {
    o << (first ? "" : ",") << '"' << json_escape(kv.first) << "\":{\"count\":" << kv.second
      << ",\"text\":\"" << json_escape(warn_text_.at(kv.first)) << "\"}";
    first = false;
  }
  o << "}}";
  return o.str();
}

std::string to_json(const ParseStats & s)
{
  std::ostringstream o;
  o << "{\"format\":\"" << to_string(s.format) << "\",\"axes\":\"" << to_string(s.axes)
    << "\",\"n_in\":" << s.n_in << ",\"n_nonfinite\":" << s.n_nonfinite << ",\"n_zero\":" << s.n_zero
    << ",\"n_dup\":" << s.n_dup << ",\"n_valid\":" << s.n_valid << ",\"n_cropped\":" << s.n_cropped
    << ",\"n_out\":" << s.n_out << ",\"dedupe_applied\":" << (s.dedupe_applied ? "true" : "false")
    << ",\"dedupe_rounded\":" << (s.dedupe_rounded ? "true" : "false") << ",\"n_dup_rounded\":" << s.n_dup_rounded
    << ",\"crop_applied\":" << (s.crop_applied ? "true" : "false")
    << ",\"crop_guard\":" << (s.crop_guard ? "true" : "false")
    << ",\"pairs_both_valid\":" << s.pairs_both_valid << ",\"pairs_coincident\":" << s.pairs_coincident
    << ",\"n_second\":" << s.n_second << ",\"n_second_valid\":" << s.n_second_valid
    << ",\"n_second_coincident\":" << s.n_second_coincident
    << ",\"ring_checked\":" << (s.ring_checked ? "true" : "false") << ",\"ring_mismatch\":" << s.ring_mismatch
    << ",\"t_min\":" << (s.have_point_time ? num(s.t_min, 6) : "null")
    << ",\"t_max\":" << (s.have_point_time ? num(s.t_max, 6) : "null")
    << ",\"az_min\":" << (s.n_az ? num(s.az_min, 2) : "null")
    << ",\"az_max\":" << (s.n_az ? num(s.az_max, 2) : "null")
    << ",\"az_center\":" << (s.n_az ? num(s.az_center, 2) : "null") << '}';
  return o.str();
}

}
