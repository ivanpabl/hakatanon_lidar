#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "tunnel_od_preproc/monitor.hpp"

using tunnel_od::Check;
using tunnel_od::FrameInfo;
using tunnel_od::InputMonitor;
using tunnel_od::Level;

namespace
{

Level level_of(const std::vector<Check> & cs, const std::string & id)
{
  for (const auto & c : cs) {
    if (c.id == id) {return c.level;}
  }
  return Level::Unknown;
}

FrameInfo legacy_frame(int64_t stamp_ns)
{
  FrameInfo f;
  f.stamp_ns = stamp_ns;
  f.frame_id = "hesai_lidar";
  f.cloud.height = 1;
  f.cloud.width = 307200;
  f.layout_signature = "x:0:7,y:4:7,z:8:7,intensity:12:7,ring:16:4,timestamp:18:8,";
  auto & s = f.stats;
  s.format = tunnel_od::Format::LegacyHesai;
  s.axes = tunnel_od::Axes::Legacy;
  s.n_in = 307200;
  s.n_zero = 120000;
  s.dedupe_applied = true;
  s.pairs_both_valid = 90000;
  s.pairs_coincident = 88000;
  s.ring_checked = true;
  s.have_point_time = true;
  s.t_min = 0.0006;
  s.t_max = 0.0327;
  s.n_az = 100;
  s.az_min = -50.0;
  s.az_max = 50.0;
  return f;
}

FrameInfo contract_frame(int64_t stamp_ns)
{
  FrameInfo f;
  f.stamp_ns = stamp_ns;
  f.frame_id = "lidar";
  f.cloud.height = 128;
  f.cloud.width = 2400;
  f.layout_signature = "x:0:7,y:4:7,z:8:7,intensity:12:2,return_id:13:2,ring:14:4,t_offset_ns:16:6,";
  auto & s = f.stats;
  s.format = tunnel_od::Format::ContractV1;
  s.axes = tunnel_od::Axes::Rep103;
  s.n_in = 307200;
  s.n_nonfinite = 150000;
  s.n_second = 153600;
  s.n_second_valid = 3000;
  s.ring_checked = true;
  s.have_point_time = true;
  s.t_min = 0.0;
  s.t_max = 0.032;
  s.n_az = 100;
  s.az_min = -57.0;
  s.az_max = 57.0;
  return f;
}

}

TEST(Monitor, LegacyStreamViolations)
{
  InputMonitor m;
  const int64_t t0 = 946687297LL * 1000000000LL;
  int64_t t = t0;
  for (int i = 0; i < 40; ++i) {
    t += (i == 20 ? 700 : 100) * 1000000LL;
    m.on_frame(legacy_frame(t));
  }
  const auto cs = m.checks();
  EXPECT_EQ(level_of(cs, "format"), Level::Ok);
  EXPECT_EQ(level_of(cs, "M1"), Level::Warn);
  EXPECT_EQ(level_of(cs, "M2"), Level::Warn);
  EXPECT_EQ(level_of(cs, "M3"), Level::Warn);
  EXPECT_EQ(level_of(cs, "M4"), Level::Warn);
  EXPECT_EQ(level_of(cs, "M5"), Level::Ok);
  EXPECT_EQ(level_of(cs, "M6"), Level::Warn);
  const auto j = m.summary_json();
  EXPECT_NE(j.find("\"gaps\":1"), std::string::npos);
  EXPECT_NE(j.find("\"stamp_year\":2000"), std::string::npos);
  const auto w = m.take_new_warnings();
  EXPECT_GE(w.size(), 5u);
  m.on_frame(legacy_frame(t + 100000000LL));
  EXPECT_TRUE(m.take_new_warnings().empty());
}

TEST(Monitor, ContractStreamIsClean)
{
  InputMonitor m;
  const double xyz[3] = {1.2, 0.0, 3.1}, rpy[3] = {0.1, -0.8, 0.0};
  m.on_tf("base_link", "lidar", xyz, rpy);
  m.on_description("{\"model\": \"Pandar128E3X\"}");
  int64_t t = 1790000000LL * 1000000000LL;
  for (int i = 0; i < 40; ++i) {
    t += 100000000LL;
    m.on_frame(contract_frame(t));
  }
  const auto cs = m.checks();
  for (const char * id : {"format", "M1", "M2", "M3", "M4", "M5", "M6"}) {
    EXPECT_EQ(level_of(cs, id), Level::Ok) << id << ": " << m.summary_json();
  }
  EXPECT_NE(m.summary_json().find("\"violations\":[]"), std::string::npos);
  EXPECT_TRUE(m.take_new_warnings().empty());
}

TEST(Monitor, LayoutAndFrameChangesAndAxesHeuristic)
{
  InputMonitor m;
  int64_t t = 1790000000LL * 1000000000LL;
  for (int i = 0; i < 25; ++i) {
    t += 100000000LL;
    auto f = contract_frame(t);
    if (i >= 10) {f.frame_id = "other";}
    if (i >= 15) {f.cloud.width = 1200;}
    if (i >= 20) {f.stats.az_center = 90.0;}
    m.on_frame(f);
  }
  const auto j = m.summary_json();
  EXPECT_NE(j.find("\"frame_id_changes\":1"), std::string::npos);
  EXPECT_NE(j.find("\"size_changes\":1"), std::string::npos);
  EXPECT_NE(j.find("\"axes\":{\"count\""), std::string::npos);
}

TEST(Monitor, RejectedFramesAndSpeed)
{
  tunnel_od::MonitorConfig cfg;
  cfg.speed_expected = true;
  InputMonitor m(cfg);
  auto f = contract_frame(1790000000LL * 1000000000LL);
  f.error = "нет полей x, y, z";
  m.on_frame(f);
  for (int i = 0; i < 30; ++i) {m.on_speed(1790000000.0 + 0.05 * i, 12.5);}
  const auto cs = m.checks();
  EXPECT_EQ(level_of(cs, "format"), Level::Error);
  EXPECT_EQ(level_of(cs, "S1"), Level::Ok);
}
