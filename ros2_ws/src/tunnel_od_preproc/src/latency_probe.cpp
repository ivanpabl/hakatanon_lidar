#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <map>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialization.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/string.hpp"

namespace
{

double pct(std::vector<double> v, double q)
{
  if (v.empty()) {return std::nan("");}
  std::sort(v.begin(), v.end());
  const double pos = q / 100.0 * (v.size() - 1);
  const size_t lo = static_cast<size_t>(pos);
  const size_t hi = std::min(lo + 1, v.size() - 1);
  return v[lo] + (pos - lo) * (v[hi] - v[lo]);
}

}

class LatencyProbe : public rclcpp::Node
{
public:
  LatencyProbe()
  : Node("tunnel_od_latency_probe")
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "");
    result_topic_ = declare_parameter<std::string>("result_topic", "/tunnel_od/result");
    out_file_ = declare_parameter<std::string>("out_file", "");
    sub_res_ = create_subscription<std_msgs::msg::String>(
      result_topic_, rclcpp::QoS(50).reliable(),
      [this](std_msgs::msg::String::ConstSharedPtr m, const rclcpp::MessageInfo & info) {
        on_result(*m, info.get_rmw_message_info().source_timestamp);
      });
    if (input_topic_.empty()) {
      timer_ = create_wall_timer(std::chrono::milliseconds(300), [this] {discover();});
    } else {
      subscribe(input_topic_);
    }
  }

  ~LatencyProbe() override {write();}

private:
  void discover()
  {
    for (const auto & kv : get_topic_names_and_types()) {
      if (kv.first.rfind("/tunnel_od/", 0) == 0) {continue;}
      if (count_publishers(kv.first) == 0) {continue;}
      for (const auto & t : kv.second) {
        if (t == "sensor_msgs/msg/PointCloud2") {
          timer_->cancel();
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
      topic, rclcpp::QoS(5).reliable(),
      [this](std::shared_ptr<const rclcpp::SerializedMessage> m, const rclcpp::MessageInfo & info) {
        on_cloud(*m, info.get_rmw_message_info().source_timestamp);
      });
    RCLCPP_INFO(get_logger(), "замер задержки: %s -> %s", topic.c_str(), result_topic_.c_str());
  }

  void on_cloud(const rclcpp::SerializedMessage & m, int64_t src_ns)
  {
    const auto & raw = m.get_rcl_serialized_message();
    if (raw.buffer_length < 12) {return;}
    int32_t sec;
    uint32_t nsec;
    std::memcpy(&sec, raw.buffer + 4, 4);
    std::memcpy(&nsec, raw.buffer + 8, 4);
    const int64_t key = static_cast<int64_t>(sec) * 1000000000LL + nsec;
    clouds_[key] = src_ns;
    ++n_clouds_;
  }

  void on_result(const std_msgs::msg::String & m, int64_t src_ns)
  {
    ++n_results_;
    const auto pos = m.data.find("\"stamp\":");
    if (pos == std::string::npos) {return;}
    const double stamp = std::strtod(m.data.c_str() + pos + 8, nullptr);
    const int64_t approx = static_cast<int64_t>(std::llround(stamp * 1e9));
    auto it = clouds_.lower_bound(approx - 2000);
    if (it == clouds_.end() || std::llabs(it->first - approx) > 2000) {
      ++unmatched_;
      return;
    }
    latency_.push_back((src_ns - it->second) * 1e-6);
  }

  void write() const
  {
    char buf[512];
    std::snprintf(buf, sizeof(buf),
      "{\"input_topic\":\"%s\",\"clouds\":%lu,\"results\":%lu,\"matched\":%zu,\"unmatched\":%lu,"
      "\"e2e_ms_p50\":%.2f,\"e2e_ms_p95\":%.2f,\"e2e_ms_max\":%.2f,\"e2e_ms\":[",
      input_topic_.c_str(), static_cast<unsigned long>(n_clouds_), static_cast<unsigned long>(n_results_),
      latency_.size(), static_cast<unsigned long>(unmatched_), pct(latency_, 50), pct(latency_, 95),
      latency_.empty() ? std::nan("") : *std::max_element(latency_.begin(), latency_.end()));
    std::string s = buf;
    for (size_t i = 0; i < latency_.size(); ++i) {
      std::snprintf(buf, sizeof(buf), "%s%.3f", i ? "," : "", latency_[i]);
      s += buf;
    }
    s += "]}";
    std::fprintf(stderr, "[latency_probe] кадров %lu, результатов %lu, сопоставлено %zu: e2e p50 %.1f / "
      "p95 %.1f / max %.1f мс\n", static_cast<unsigned long>(n_clouds_), static_cast<unsigned long>(n_results_),
      latency_.size(), pct(latency_, 50), pct(latency_, 95),
      latency_.empty() ? std::nan("") : *std::max_element(latency_.begin(), latency_.end()));
    if (out_file_.empty()) {return;}
    if (FILE * f = std::fopen(out_file_.c_str(), "w")) {
      std::fwrite(s.data(), 1, s.size(), f);
      std::fclose(f);
    }
  }

  std::string input_topic_, result_topic_, out_file_;
  std::map<int64_t, int64_t> clouds_;
  std::vector<double> latency_;
  uint64_t n_clouds_ = 0, n_results_ = 0, unmatched_ = 0;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_res_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LatencyProbe>();
  rclcpp::spin(node);
  node.reset();
  rclcpp::shutdown();
  return 0;
}
