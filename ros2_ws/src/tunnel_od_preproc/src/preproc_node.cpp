#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <deque>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

#include "tunnel_od_preproc/canonical.hpp"
#include "tunnel_od_preproc/monitor.hpp"

namespace tunnel_od
{

namespace
{

using Clock = std::chrono::steady_clock;

int64_t wall_ns()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::system_clock::now().time_since_epoch()).count();
}

double ms_since(Clock::time_point t0)
{
  return std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
}

double percentile(std::vector<double> v, double q)
{
  if (v.empty()) {return std::nan("");}
  const size_t k = std::min(v.size() - 1, static_cast<size_t>(std::lround(q / 100.0 * (v.size() - 1))));
  std::nth_element(v.begin(), v.begin() + k, v.end());
  return v[k];
}

std::string num(double v, int prec = 3)
{
  if (!std::isfinite(v)) {return "null";}
  char buf[64];
  std::snprintf(buf, sizeof(buf), "%.*f", prec, v);
  return buf;
}

std::string strip_slash(const std::string & s)
{
  return !s.empty() && s[0] == '/' ? s.substr(1) : s;
}

struct Quat
{
  double x = 0, y = 0, z = 0, w = 1;
};

Quat mul(const Quat & a, const Quat & b)
{
  return {a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
    a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
    a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
    a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z};
}

void to_matrix(const Quat & q, double m[9])
{
  const double x = q.x, y = q.y, z = q.z, w = q.w;
  m[0] = 1 - 2 * (y * y + z * z); m[1] = 2 * (x * y - z * w);     m[2] = 2 * (x * z + y * w);
  m[3] = 2 * (x * y + z * w);     m[4] = 1 - 2 * (x * x + z * z); m[5] = 2 * (y * z - x * w);
  m[6] = 2 * (x * z - y * w);     m[7] = 2 * (y * z + x * w);     m[8] = 1 - 2 * (x * x + y * y);
}

void to_rpy_deg(const Quat & q, double rpy[3])
{
  const double r2d = 57.29577951308232;
  rpy[0] = std::atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y)) * r2d;
  rpy[1] = std::asin(std::max(-1.0, std::min(1.0, 2 * (q.w * q.y - q.z * q.x)))) * r2d;
  rpy[2] = std::atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)) * r2d;
}

const Quat kCoreInRep{0.0, 0.0, std::sqrt(0.5), std::sqrt(0.5)};

}

class PreprocNode : public rclcpp::Node
{
public:
  explicit PreprocNode(const rclcpp::NodeOptions & options)
  : Node("tunnel_od_preproc", options)
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "");
    output_topic_ = declare_parameter<std::string>("output_topic", "/tunnel_od/cloud");
    meta_topic_ = declare_parameter<std::string>("meta_topic", "/tunnel_od/input_meta");
    diag_topic_ = declare_parameter<std::string>("diagnostics_topic", "/tunnel_od/input_diagnostics");
    const auto fmt = declare_parameter<std::string>("input_format", "auto");
    const auto axes = declare_parameter<std::string>("input_axes", "auto");
    if (!parse_format(fmt, &opt_.format)) {
      throw std::invalid_argument("input_format: auto|legacy_hesai|contract_v1|generic, получено " + fmt);
    }
    if (!parse_axes(axes, &opt_.axes)) {
      throw std::invalid_argument("input_axes: auto|legacy|rep103, получено " + axes);
    }
    opt_.dedupe_dual_return = declare_parameter<bool>("dedupe_dual_return", true);
    opt_.crop.enabled = declare_parameter<bool>("crop_enabled", true);
    opt_.crop.fwd_min = declare_parameter<double>("crop_fwd_min", 2.0);
    opt_.crop.fwd_max = declare_parameter<double>("crop_fwd_max", 250.0);
    opt_.collect_stats = true;
    apply_mount_tf_ = declare_parameter<bool>("apply_mount_tf", false);
    base_frame_ = strip_slash(declare_parameter<std::string>("base_frame", "base_link"));
    core_suffix_ = declare_parameter<std::string>("core_frame_suffix", "_tunnel_od");
    speed_topic_ = declare_parameter<std::string>("speed_topic", "");
    speed_type_ = declare_parameter<std::string>("speed_type", "auto");
    speed_max_age_s_ = declare_parameter<double>("speed_max_age_s", 1.0);
    description_topic_ = declare_parameter<std::string>("description_topic", "/lidar/description");
    stats_period_ = declare_parameter<double>("stats_period_s", 5.0);
    stats_file_ = declare_parameter<std::string>("stats_file", "");
    const auto rel = declare_parameter<std::string>("qos_reliability", "reliable");
    const auto depth = declare_parameter<int>("qos_depth", 5);

    MonitorConfig mc;
    mc.configured_axes = opt_.axes;
    mc.configured_format = opt_.format;
    mc.speed_expected = !speed_topic_.empty();
    mc.description_expected = !description_topic_.empty();
    monitor_ = InputMonitor(mc);

    qos_ = rclcpp::QoS(rclcpp::KeepLast(depth));
    if (rel == "best_effort") {qos_.best_effort();} else {qos_.reliable();}

    pub_cloud_ = create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, rclcpp::QoS(5).reliable());
    pub_meta_ = create_publisher<std_msgs::msg::String>(meta_topic_, rclcpp::QoS(20).reliable());
    pub_diag_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(diag_topic_, 10);
    pub_tf_static_ = create_publisher<tf2_msgs::msg::TFMessage>(
      "/tf_static", rclcpp::QoS(10).reliable().transient_local());

    sub_tf_ = create_subscription<tf2_msgs::msg::TFMessage>(
      "/tf_static", rclcpp::QoS(100).reliable().transient_local(),
      [this](tf2_msgs::msg::TFMessage::ConstSharedPtr m) {on_tf(*m);});
    if (!description_topic_.empty()) {
      sub_desc_ = create_subscription<std_msgs::msg::String>(
        description_topic_, rclcpp::QoS(1).reliable().transient_local(),
        [this](std_msgs::msg::String::ConstSharedPtr m) {monitor_.on_description(m->data);});
    }

    if (input_topic_.empty()) {
      discover_timer_ = create_wall_timer(std::chrono::milliseconds(500), [this] {discover();});
      RCLCPP_INFO(get_logger(), "input_topic не задан: жду первый топик sensor_msgs/msg/PointCloud2");
    } else {
      subscribe(input_topic_);
    }
    if (!speed_topic_.empty()) {
      speed_timer_ = create_wall_timer(std::chrono::milliseconds(500), [this] {subscribe_speed();});
    }
    stats_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(stats_period_)),
      [this] {report();});
    RCLCPP_INFO(get_logger(),
      "формат %s, оси %s, dual return %s, обрезка %s [%.1f, %.1f] м, apply_mount_tf %s, выход %s",
      to_string(opt_.format), to_string(opt_.axes), opt_.dedupe_dual_return ? "удалять" : "оставлять",
      opt_.crop.enabled ? "да" : "нет", opt_.crop.fwd_min, opt_.crop.fwd_max,
      apply_mount_tf_ ? "да" : "нет", output_topic_.c_str());
  }

  ~PreprocNode() override
  {
    write_stats();
  }

private:
  void discover()
  {
    for (const auto & kv : get_topic_names_and_types()) {
      if (kv.first.rfind("/tunnel_od/", 0) == 0) {continue;}
      if (count_publishers(kv.first) == 0) {continue;}
      for (const auto & t : kv.second) {
        if (t == "sensor_msgs/msg/PointCloud2") {
          discover_timer_->cancel();
          subscribe(kv.first);
          return;
        }
      }
    }
  }

  void subscribe(const std::string & topic)
  {
    input_topic_ = topic;
    sub_cloud_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      topic, qos_,
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg, const rclcpp::MessageInfo & info) {
        on_cloud(*msg, info);
      });
    RCLCPP_INFO(get_logger(), "подписка на %s", topic.c_str());
  }

  void subscribe_speed()
  {
    std::string type = speed_type_;
    if (type == "auto") {
      for (const auto & kv : get_topic_names_and_types()) {
        if (kv.first != speed_topic_) {continue;}
        for (const auto & t : kv.second) {
          if (t == "geometry_msgs/msg/TwistStamped") {type = "twist";}
          if (t == "nav_msgs/msg/Odometry") {type = "odometry";}
        }
      }
      if (type == "auto") {return;}
    }
    speed_timer_->cancel();
    if (type == "twist") {
      sub_twist_ = create_subscription<geometry_msgs::msg::TwistStamped>(
        speed_topic_, rclcpp::QoS(20),
        [this](geometry_msgs::msg::TwistStamped::ConstSharedPtr m) {
          on_speed(m->header.stamp, m->twist.linear.x);
        });
    } else if (type == "odometry") {
      sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
        speed_topic_, rclcpp::QoS(20),
        [this](nav_msgs::msg::Odometry::ConstSharedPtr m) {
          on_speed(m->header.stamp, m->twist.twist.linear.x);
        });
    } else {
      RCLCPP_ERROR(get_logger(), "speed_type: auto|twist|odometry, получено %s", type.c_str());
      return;
    }
    RCLCPP_INFO(get_logger(), "скорость поезда (S1): %s (%s), в детекции не используется",
      speed_topic_.c_str(), type.c_str());
  }

  void on_speed(const builtin_interfaces::msg::Time & st, double v)
  {
    const double t = st.sec + st.nanosec * 1e-9;
    speed_ = v;
    speed_stamp_ = t;
    speed_recv_ = Clock::now();
    have_speed_ = true;
    monitor_.on_speed(t, v);
  }

  void on_tf(const tf2_msgs::msg::TFMessage & m)
  {
    for (const auto & t : m.transforms) {
      const std::string child = strip_slash(t.child_frame_id);
      if (child.size() > core_suffix_.size() &&
        child.compare(child.size() - core_suffix_.size(), core_suffix_.size(), core_suffix_) == 0)
      {
        continue;
      }
      Quat q{t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w};
      tf_[child] = {strip_slash(t.header.frame_id), q,
        {t.transform.translation.x, t.transform.translation.y, t.transform.translation.z}};
    }
  }

  void on_cloud(const sensor_msgs::msg::PointCloud2 & msg, const rclcpp::MessageInfo & info)
  {
    const auto t0 = Clock::now();
    const int64_t recv_ns = wall_ns();
    const auto & mi = info.get_rmw_message_info();
    ++received_;
    count_lost(mi);

    CloudDesc c;
    c.height = msg.height;
    c.width = msg.width;
    c.point_step = msg.point_step;
    c.row_step = msg.row_step;
    c.is_bigendian = msg.is_bigendian;
    c.is_dense = msg.is_dense;
    c.data_size = msg.data.size();
    if (msg.fields.size() != fields_cache_.size() || !same_fields(msg)) {
      std::vector<Field> fv;
      for (const auto & f : msg.fields) {fv.push_back(Field{f.name, f.offset, f.datatype, f.count});}
      layout_ = make_layout(fv);
      fields_cache_ = msg.fields;
    }
    const std::string frame = strip_slash(msg.header.frame_id);
    const int64_t stamp_ns = static_cast<int64_t>(msg.header.stamp.sec) * 1000000000LL + msg.header.stamp.nanosec;

    ParseOptions opt = opt_;
    auto it = tf_.find(frame);
    if (it != tf_.end() && !tf_reported_[frame]) {
      double rpy[3];
      to_rpy_deg(it->second.q, rpy);
      monitor_.on_tf(it->second.parent, frame, it->second.t.data(), rpy);
      tf_reported_[frame] = true;
      RCLCPP_INFO(get_logger(), "M3: /tf_static %s -> %s: xyz (%.3f, %.3f, %.3f) м, крен %.2f, тангаж %.2f, "
        "рыскание %.2f град%s", it->second.parent.c_str(), frame.c_str(), it->second.t[0], it->second.t[1],
        it->second.t[2], rpy[0], rpy[1], rpy[2], apply_mount_tf_ ? " -- применяется" : " -- не применяется");
    }
    bool rotated = false;
    if (apply_mount_tf_) {
      if (it != tf_.end() && it->second.parent == base_frame_) {
        opt.rotation.enabled = true;
        to_matrix(it->second.q, opt.rotation.m);
        opt.rotation.out_axes = Axes::Rep103;
        rotated = true;
      } else {
        ++unrotated_;
      }
    }

    ParseStats st;
    std::string err;
    const size_t cap = c.n_points() * 3;
    if (buf_.size() < cap) {buf_.resize(cap);}
    const double stamp_s = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9;
    const size_t n = parse_cloud(msg.data.data(), c, layout_, opt, stamp_s,
        buf_.data(), buf_.data() + 1, buf_.data() + 2, 3, &st, &err);

    FrameInfo fi;
    fi.stamp_ns = stamp_ns;
    fi.frame_id = frame;
    fi.cloud = c;
    fi.layout_signature = layout_.signature;
    fi.stats = st;
    fi.recv_s = std::chrono::duration<double>(t0.time_since_epoch()).count();
    fi.error = err;
    monitor_.on_frame(fi);
    for (const auto & w : monitor_.take_new_warnings()) {
      RCLCPP_WARN(get_logger(), "вход: %s", w.c_str());
    }
    if (!err.empty()) {
      ++errors_;
      return;
    }
    if (first_frame_) {
      first_frame_ = false;
      RCLCPP_INFO(get_logger(), "первый кадр: %s, %ux%u, point_step %u, fields [%s] -> формат %s, оси %s, "
        "%zu -> %zu точек", input_topic_.c_str(), c.height, c.width, c.point_step, layout_.signature.c_str(),
        to_string(st.format), to_string(st.axes), static_cast<size_t>(st.n_in), n);
    }

    auto out = std::make_unique<sensor_msgs::msg::PointCloud2>();
    out->header = msg.header;
    if (rotated || st.axes == Axes::Rep103) {
      out->header.frame_id = frame + core_suffix_;
      publish_core_frame(frame, rotated ? it->second.q : Quat{});
    }
    out->height = 1;
    out->width = static_cast<uint32_t>(n);
    out->fields.resize(3);
    const char * names[3] = {"x", "y", "z"};
    for (int i = 0; i < 3; ++i) {
      out->fields[i].name = names[i];
      out->fields[i].offset = 4 * i;
      out->fields[i].datatype = sensor_msgs::msg::PointField::FLOAT32;
      out->fields[i].count = 1;
    }
    out->point_step = 12;
    out->row_step = static_cast<uint32_t>(12 * n);
    out->is_bigendian = false;
    out->is_dense = true;
    const auto * bytes = reinterpret_cast<const uint8_t *>(buf_.data());
    out->data.assign(bytes, bytes + 12 * n);
    const double parse_ms = ms_since(t0);

    const bool speed_fresh = have_speed_ &&
      std::chrono::duration<double>(Clock::now() - speed_recv_).count() <= speed_max_age_s_;
    std::ostringstream o;
    o << "{\"stamp_sec\":" << msg.header.stamp.sec << ",\"stamp_nanosec\":" << msg.header.stamp.nanosec
      << ",\"frame\":" << published_ << ",\"source_ts_ns\":" << mi.source_timestamp
      << ",\"recv_ts_ns\":" << recv_ns << ",\"pub_ts_ns\":" << wall_ns()
      << ",\"transport_ms\":" << num((recv_ns - mi.source_timestamp) * 1e-6, 3)
      << ",\"preproc_ms\":" << num(parse_ms, 3)
      << ",\"train_speed_mps\":" << (speed_fresh ? num(speed_, 3) : "null")
      << ",\"train_speed_stamp\":" << (speed_fresh ? num(speed_stamp_, 3) : "null")
      << ",\"frame_id\":\"" << json_escape(out->header.frame_id) << '"'
      << ",\"parse\":" << to_json(st)
      << ",\"counters\":" << counters_json();
    if (published_ % meta_summary_every_ == 0) {o << ",\"input\":" << monitor_.summary_json();}
    o << '}';
    std_msgs::msg::String meta;
    meta.data = o.str();
    pub_meta_->publish(meta);
    pub_cloud_->publish(std::move(out));
    ++published_;

    const double total_ms = ms_since(t0);
    preproc_ms_.push_back(total_ms);
    transport_ms_.push_back((recv_ns - mi.source_timestamp) * 1e-6);
    period_ms_.push_back(total_ms);
  }

  bool same_fields(const sensor_msgs::msg::PointCloud2 & msg) const
  {
    for (size_t i = 0; i < msg.fields.size(); ++i) {
      const auto & a = msg.fields[i];
      const auto & b = fields_cache_[i];
      if (a.name != b.name || a.offset != b.offset || a.datatype != b.datatype) {return false;}
    }
    return true;
  }

  void count_lost(const rmw_message_info_t & mi)
  {
    if (mi.publication_sequence_number == RMW_MESSAGE_INFO_SEQUENCE_NUMBER_UNSUPPORTED) {
      seq_supported_ = false;
      return;
    }
    std::array<uint8_t, RMW_GID_STORAGE_SIZE> gid;
    std::copy(std::begin(mi.publisher_gid.data), std::end(mi.publisher_gid.data), gid.begin());
    auto it = last_seq_.find(gid);
    if (it != last_seq_.end() && mi.publication_sequence_number > it->second + 1) {
      dropped_ += mi.publication_sequence_number - it->second - 1;
    }
    last_seq_[gid] = mi.publication_sequence_number;
  }

  void publish_core_frame(const std::string & frame, const Quat & mount)
  {
    const Quat conj{-mount.x, -mount.y, -mount.z, mount.w};
    const Quat q = mul(conj, kCoreInRep);
    auto & last = core_tf_sent_[frame];
    if (last.first && std::fabs(last.second.x - q.x) + std::fabs(last.second.y - q.y) +
      std::fabs(last.second.z - q.z) + std::fabs(last.second.w - q.w) < 1e-9)
    {
      return;
    }
    last = {true, q};
    tf2_msgs::msg::TFMessage m;
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = now();
    t.header.frame_id = frame;
    t.child_frame_id = frame + core_suffix_;
    t.transform.rotation.x = q.x;
    t.transform.rotation.y = q.y;
    t.transform.rotation.z = q.z;
    t.transform.rotation.w = q.w;
    m.transforms.push_back(t);
    pub_tf_static_->publish(m);
    RCLCPP_INFO(get_logger(), "оси входа не совпадают с осями ядра: облако публикуется в %s "
      "(/tf_static %s -> %s)", t.child_frame_id.c_str(), frame.c_str(), t.child_frame_id.c_str());
  }

  std::string counters_json() const
  {
    std::ostringstream o;
    o << "{\"received\":" << received_ << ",\"published\":" << published_ << ",\"dropped\":" << dropped_
      << ",\"errors\":" << errors_ << ",\"seq_supported\":" << (seq_supported_ ? "true" : "false")
      << ",\"unrotated\":" << unrotated_ << '}';
    return o.str();
  }

  std::string stats_json() const
  {
    std::ostringstream o;
    o << "{\"input_topic\":\"" << json_escape(input_topic_) << "\",\"counters\":" << counters_json()
      << ",\"preproc_ms_p50\":" << num(percentile(preproc_ms_, 50)) << ",\"preproc_ms_p95\":"
      << num(percentile(preproc_ms_, 95)) << ",\"preproc_ms_max\":"
      << num(preproc_ms_.empty() ? std::nan("") : *std::max_element(preproc_ms_.begin(), preproc_ms_.end()))
      << ",\"transport_ms_p50\":" << num(percentile(transport_ms_, 50)) << ",\"transport_ms_p95\":"
      << num(percentile(transport_ms_, 95)) << ",\"input\":" << monitor_.summary_json() << '}';
    return o.str();
  }

  void write_stats() const
  {
    if (stats_file_.empty()) {return;}
    const std::string tmp = stats_file_ + ".tmp";
    FILE * f = std::fopen(tmp.c_str(), "w");
    if (!f) {return;}
    const std::string s = stats_json();
    std::fwrite(s.data(), 1, s.size(), f);
    std::fclose(f);
    std::rename(tmp.c_str(), stats_file_.c_str());
  }

  void report()
  {
    diagnostic_msgs::msg::DiagnosticArray arr;
    arr.header.stamp = now();
    for (const auto & ch : monitor_.checks()) {
      diagnostic_msgs::msg::DiagnosticStatus s;
      s.level = static_cast<uint8_t>(ch.level);
      s.name = "tunnel_od_preproc: " + ch.id + " " + ch.title;
      s.message = ch.message;
      s.hardware_id = input_topic_;
      for (const auto & v : ch.values) {
        diagnostic_msgs::msg::KeyValue kv;
        kv.key = v.key;
        kv.value = v.text;
        s.values.push_back(kv);
      }
      arr.status.push_back(s);
    }
    pub_diag_->publish(arr);
    write_stats();
    if (!period_ms_.empty()) {
      RCLCPP_INFO(get_logger(), "%.1f кадр/с, разбор p50 %.1f / p95 %.1f мс; всего: принято %lu, "
        "опубликовано %lu, потеряно до узла %lu%s, ошибок %lu", period_ms_.size() / stats_period_,
        percentile(period_ms_, 50), percentile(period_ms_, 95), static_cast<unsigned long>(received_),
        static_cast<unsigned long>(published_), static_cast<unsigned long>(dropped_),
        seq_supported_ ? "" : " (номера кадров DDS недоступны)", static_cast<unsigned long>(errors_));
    }
    period_ms_.clear();
  }

  struct Tf
  {
    std::string parent;
    Quat q;
    std::array<double, 3> t;
  };

  std::string input_topic_, output_topic_, meta_topic_, diag_topic_, base_frame_, core_suffix_;
  std::string speed_topic_, speed_type_, description_topic_, stats_file_;
  ParseOptions opt_;
  bool apply_mount_tf_ = false;
  double stats_period_ = 5.0, speed_max_age_s_ = 1.0;
  rclcpp::QoS qos_{5};
  InputMonitor monitor_;
  Layout layout_;
  std::vector<sensor_msgs::msg::PointField> fields_cache_;
  std::vector<float> buf_;
  std::map<std::string, Tf> tf_;
  std::map<std::string, bool> tf_reported_;
  std::map<std::string, std::pair<bool, Quat>> core_tf_sent_;
  std::map<std::array<uint8_t, RMW_GID_STORAGE_SIZE>, uint64_t> last_seq_;
  bool first_frame_ = true, seq_supported_ = true, have_speed_ = false;
  double speed_ = 0.0, speed_stamp_ = 0.0;
  Clock::time_point speed_recv_;
  uint64_t received_ = 0, published_ = 0, dropped_ = 0, errors_ = 0, unrotated_ = 0;
  const uint64_t meta_summary_every_ = 1;
  std::vector<double> preproc_ms_, transport_ms_, period_ms_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr sub_tf_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_desc_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_twist_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_meta_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr pub_diag_;
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr pub_tf_static_;
  rclcpp::TimerBase::SharedPtr discover_timer_, speed_timer_, stats_timer_;
};

}

RCLCPP_COMPONENTS_REGISTER_NODE(tunnel_od::PreprocNode)
