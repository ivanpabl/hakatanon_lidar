#include "tunnel_od_preproc/c_api.h"

#include <algorithm>
#include <cstring>
#include <string>
#include <vector>

#include "tunnel_od_preproc/canonical.hpp"
#include "tunnel_od_preproc/monitor.hpp"

namespace
{

void copy_out(const std::string & s, char * buf, size_t len)
{
  if (!buf || !len) {return;}
  const size_t n = std::min(s.size(), len - 1);
  std::memcpy(buf, s.data(), n);
  buf[n] = '\0';
}

// неполученные предупреждения отдельного вызова tod_monitor_new_warnings
struct Monitor
{
  tunnel_od::InputMonitor mon;
  std::vector<std::string> pending;
  explicit Monitor(tunnel_od::MonitorConfig c)
  : mon(c) {}
};

}  // namespace

extern "C" {

int32_t tod_api_version(void) {return TOD_API_VERSION;}

int64_t tod_parse(
  const uint8_t * data, size_t data_size, const tod_cloud * cloud,
  const tod_field * fields, size_t n_fields, const tod_options * opt,
  float * out_x, float * out_y, float * out_z, size_t out_stride,
  void * monitor, double recv_s, char * stats_json, size_t stats_len, char * err, size_t err_len)
{
  using namespace tunnel_od;
  std::vector<Field> fv;
  fv.reserve(n_fields);
  for (size_t i = 0; i < n_fields; ++i) {
    fv.push_back(Field{fields[i].name ? fields[i].name : "", fields[i].offset, fields[i].datatype,
        fields[i].count});
  }
  const Layout layout = make_layout(fv);
  CloudDesc c;
  c.height = cloud->height;
  c.width = cloud->width;
  c.point_step = cloud->point_step;
  c.row_step = cloud->row_step;
  c.is_bigendian = cloud->is_bigendian != 0;
  c.is_dense = cloud->is_dense != 0;
  c.data_size = data_size;

  ParseOptions o;
  o.format = static_cast<Format>(opt->format);
  o.axes = static_cast<Axes>(opt->axes);
  o.dedupe_dual_return = opt->dedupe != 0;
  o.crop.enabled = opt->crop_enabled != 0;
  o.crop.fwd_min = opt->crop_min;
  o.crop.fwd_max = opt->crop_max;
  o.collect_stats = opt->collect_stats != 0 || monitor != nullptr || stats_json != nullptr;

  ParseStats st;
  std::string error;
  const double stamp_s = static_cast<double>(cloud->stamp_ns / 1000000000LL) +
    static_cast<double>(cloud->stamp_ns % 1000000000LL) * 1e-9;
  const size_t n = parse_cloud(data, c, layout, o, stamp_s, out_x, out_y, out_z, out_stride, &st, &error);
  if (monitor) {
    FrameInfo fi;
    fi.stamp_ns = cloud->stamp_ns;
    fi.frame_id = cloud->frame_id ? cloud->frame_id : "";
    fi.cloud = c;
    fi.layout_signature = layout.signature;
    fi.stats = st;
    fi.recv_s = recv_s;
    fi.error = error;
    auto * m = static_cast<Monitor *>(monitor);
    m->mon.on_frame(fi);
    for (auto & w : m->mon.take_new_warnings()) {m->pending.push_back(w);}
  }
  if (!error.empty()) {
    copy_out(error, err, err_len);
    return -1;
  }
  if (stats_json) {copy_out(to_json(st), stats_json, stats_len);}
  return static_cast<int64_t>(n);
}

void * tod_monitor_new(int32_t speed_expected, int32_t description_expected)
{
  tunnel_od::MonitorConfig cfg;
  cfg.speed_expected = speed_expected != 0;
  cfg.description_expected = description_expected != 0;
  return new Monitor(cfg);
}

void tod_monitor_free(void * monitor) {delete static_cast<Monitor *>(monitor);}

void tod_monitor_tf(void * monitor, const char * parent, const char * child, const double * xyz,
  const double * rpy_deg)
{
  static_cast<Monitor *>(monitor)->mon.on_tf(parent ? parent : "", child ? child : "", xyz, rpy_deg);
}

void tod_monitor_speed(void * monitor, double stamp_s, double speed_mps)
{
  static_cast<Monitor *>(monitor)->mon.on_speed(stamp_s, speed_mps);
}

void tod_monitor_description(void * monitor, const char * text)
{
  static_cast<Monitor *>(monitor)->mon.on_description(text ? text : "");
}

size_t tod_monitor_summary(void * monitor, char * buf, size_t len)
{
  const std::string s = static_cast<Monitor *>(monitor)->mon.summary_json();
  copy_out(s, buf, len);
  return s.size() + 1;
}

size_t tod_monitor_new_warnings(void * monitor, char * buf, size_t len)
{
  auto * m = static_cast<Monitor *>(monitor);
  std::string s;
  for (auto & w : m->pending) {s += w + "\n";}
  if (buf && len > s.size()) {m->pending.clear();}
  copy_out(s, buf, len);
  return s.size() + 1;
}

}  // extern "C"
