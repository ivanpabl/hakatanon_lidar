#include <gtest/gtest.h>

#include <cmath>
#include <cstring>
#include <limits>
#include <vector>

#include "tunnel_od_preproc/canonical.hpp"

using tunnel_od::Axes;
using tunnel_od::CloudDesc;
using tunnel_od::Field;
using tunnel_od::Format;
using tunnel_od::Layout;
using tunnel_od::ParseOptions;
using tunnel_od::ParseStats;
using tunnel_od::kColumnHeight;

namespace
{

const float kNaN = std::numeric_limits<float>::quiet_NaN();

struct P
{
  float x, y, z;
};

template<typename T>
void put(std::vector<uint8_t> & d, size_t off, T v)
{
  std::memcpy(d.data() + off, &v, sizeof(T));
}

struct Legacy
{
  std::vector<uint8_t> data;
  CloudDesc c;
  Layout l;
  explicit Legacy(const std::vector<P> & pts, double stamp = 100.0)
  {
    data.assign(pts.size() * 26, 0);
    for (size_t i = 0; i < pts.size(); ++i) {
      put(data, i * 26 + 0, pts[i].x);
      put(data, i * 26 + 4, pts[i].y);
      put(data, i * 26 + 8, pts[i].z);
      put(data, i * 26 + 16, static_cast<uint16_t>(i % kColumnHeight));
      put(data, i * 26 + 18, stamp + 1e-5 * static_cast<double>(i));
    }
    c.height = 1;
    c.width = static_cast<uint32_t>(pts.size());
    c.point_step = 26;
    c.row_step = c.width * 26;
    c.data_size = data.size();
    l = tunnel_od::make_layout({{"x", 0, 7, 1}, {"y", 4, 7, 1}, {"z", 8, 7, 1}, {"intensity", 12, 7, 1},
      {"ring", 16, 4, 1}, {"timestamp", 18, 8, 1}});
  }
};

std::vector<P> parse(const uint8_t * d, const CloudDesc & c, const Layout & l, ParseOptions o = {},
  ParseStats * st = nullptr, double stamp = 100.0)
{
  std::vector<float> out(c.n_points() * 3 + 3);
  std::string err;
  const size_t n = tunnel_od::parse_cloud(d, c, l, o, stamp, out.data(), out.data() + 1, out.data() + 2, 3,
      st, &err);
  EXPECT_EQ(err, "");
  std::vector<P> r(n);
  for (size_t i = 0; i < n; ++i) {r[i] = {out[3 * i], out[3 * i + 1], out[3 * i + 2]};}
  return r;
}

}

TEST(Canonical, LegacyDropsZerosAndDualReturnDuplicates)
{
  const size_t n_pairs = 4, n = 2 * n_pairs * kColumnHeight;
  std::vector<P> pts(n);
  for (size_t i = 0; i < n; ++i) {pts[i] = {1.0f + 0.01f * i, -5.0f, 0.0f};}
  for (size_t b = 0; b < n_pairs; ++b) {
    for (size_t r = 0; r < kColumnHeight; ++r) {
      pts[(2 * b + 1) * kColumnHeight + r] = pts[2 * b * kColumnHeight + r];
    }
  }
  for (size_t r = 0; r < 10; ++r) {pts[r] = pts[kColumnHeight + r] = {0, 0, 0};}
  Legacy L(pts);
  ASSERT_EQ(tunnel_od::detect_format(L.c, L.l), Format::LegacyHesai);
  ParseStats st;
  const auto out = parse(L.data.data(), L.c, L.l, {}, &st);
  EXPECT_EQ(out.size(), n_pairs * kColumnHeight - 10);
  EXPECT_FLOAT_EQ(out[0].x, pts[10].x);
  EXPECT_FLOAT_EQ(out[kColumnHeight - 10].x, pts[2 * kColumnHeight].x);
  EXPECT_EQ(st.n_zero, 20u);
  EXPECT_EQ(st.n_dup, n_pairs * kColumnHeight - 10);
  EXPECT_TRUE(st.dedupe_applied);
  EXPECT_EQ(st.ring_mismatch, 0u);
  EXPECT_EQ(st.pairs_both_valid, st.pairs_coincident);
}

TEST(Canonical, DuplicateThresholdIsStrictInFloat32)
{
  std::vector<P> pts(2 * kColumnHeight, P{1.0f, -5.0f, 0.5f});
  for (size_t r = 0; r < kColumnHeight; ++r) {pts[kColumnHeight + r].x = 1.0f + 0.02f;}
  pts[kColumnHeight].x = 1.0f + 0.005f;
  Legacy L(pts);
  const auto out = parse(L.data.data(), L.c, L.l);
  EXPECT_EQ(out.size(), 2 * kColumnHeight - 1);
}

TEST(Canonical, NoDedupeWhenWidthNotMultipleOf256)
{
  std::vector<P> pts(2 * kColumnHeight + 5, P{2.0f, -3.0f, 1.0f});
  Legacy L(pts);
  EXPECT_EQ(tunnel_od::detect_format(L.c, L.l), Format::Generic);
  ParseOptions o;
  o.format = Format::LegacyHesai;
  ParseStats st;
  EXPECT_EQ(parse(L.data.data(), L.c, L.l, o, &st).size(), pts.size());
  EXPECT_FALSE(st.dedupe_applied);
}

TEST(Canonical, DropsNanInfAndNegativeZero)
{
  std::vector<P> pts(2 * kColumnHeight, P{1.0f, -1.0f, 1.0f});
  for (size_t r = 0; r < kColumnHeight; ++r) {pts[kColumnHeight + r] = {5.0f, -5.0f, 5.0f};}
  pts[1] = {kNaN, 1, 1};
  pts[2] = {1, std::numeric_limits<float>::infinity(), 1};
  pts[3] = {-0.0f, 0.0f, -0.0f};
  Legacy L(pts);
  ParseStats st;
  const auto out = parse(L.data.data(), L.c, L.l, {}, &st);
  EXPECT_EQ(st.n_nonfinite, 2u);
  EXPECT_EQ(st.n_zero, 1u);
  EXPECT_EQ(out.size(), 2 * kColumnHeight - 3);
}

TEST(Canonical, GenericKeepsDuplicates)
{
  std::vector<P> pts(2 * kColumnHeight, P{1.0f, -1.0f, 1.0f});
  Legacy L(pts);
  Layout l = tunnel_od::make_layout({{"x", 0, 7, 1}, {"y", 4, 7, 1}, {"z", 8, 7, 1}});
  EXPECT_EQ(tunnel_od::detect_format(L.c, l), Format::Generic);
  EXPECT_EQ(parse(L.data.data(), L.c, l).size(), pts.size());
}

struct Contract
{
  std::vector<uint8_t> data;
  CloudDesc c;
  Layout l;
  Contract(size_t W, const std::vector<P> & col_major_rep, const std::vector<uint8_t> & rid)
  {
    const uint32_t ps = 24;
    data.assign(kColumnHeight * W * ps, 0);
    for (size_t col = 0; col < W; ++col) {
      for (size_t r = 0; r < kColumnHeight; ++r) {
        const size_t o = (r * W + col) * ps;
        const P & p = col_major_rep[col * kColumnHeight + r];
        put(data, o + 0, p.x);
        put(data, o + 4, p.y);
        put(data, o + 8, p.z);
        put(data, o + 13, rid[col]);
        put(data, o + 14, static_cast<uint16_t>(r));
        put(data, o + 16, static_cast<uint32_t>(1000 * col));
      }
    }
    c.height = kColumnHeight;
    c.width = static_cast<uint32_t>(W);
    c.point_step = ps;
    c.row_step = static_cast<uint32_t>(W * ps);
    c.data_size = data.size();
    l = tunnel_od::make_layout({{"t_offset_ns", 16, 6, 1}, {"ring", 14, 4, 1}, {"return_id", 13, 2, 1},
      {"intensity", 12, 2, 1}, {"z", 8, 7, 1}, {"y", 4, 7, 1}, {"x", 0, 7, 1}});
  }
};

TEST(Canonical, ContractTransposesToColumnOrderAndRotatesAxes)
{
  const size_t W = 4;
  std::vector<P> rep(W * kColumnHeight);
  for (size_t i = 0; i < rep.size(); ++i) {
    rep[i] = {10.0f + i, 0.5f * i, 0.25f};
  }
  rep[kColumnHeight + 7] = {kNaN, kNaN, kNaN};
  Contract K(W, rep, {0, 1, 0, 1});
  ASSERT_EQ(tunnel_od::detect_format(K.c, K.l), Format::ContractV1);
  ParseStats st;
  const auto out = parse(K.data.data(), K.c, K.l, {}, &st);
  ASSERT_EQ(out.size(), rep.size() - 1);
  EXPECT_EQ(st.axes, Axes::Rep103);
  EXPECT_FLOAT_EQ(out[0].x, rep[0].y);
  EXPECT_FLOAT_EQ(out[0].y, -rep[0].x);
  EXPECT_FLOAT_EQ(out[1].x, rep[1].y);
  EXPECT_FLOAT_EQ(out[kColumnHeight].y, -rep[kColumnHeight].x);
  EXPECT_EQ(st.ring_mismatch, 0u);
  EXPECT_EQ(st.n_second, 2 * kColumnHeight);
  EXPECT_EQ(st.n_second_valid, 2 * kColumnHeight - 1);
  EXPECT_EQ(st.n_second_coincident, 0u);
  ASSERT_TRUE(st.have_point_time);
  EXPECT_NEAR(st.t_max, 3e-6, 1.5e-6);
  EXPECT_NEAR(st.az_center, 0.0, 45.0);
}

TEST(Canonical, ContractFlagsSecondReturnThatIsNotNaN)
{
  const size_t W = 2;
  std::vector<P> rep(W * kColumnHeight, P{10.0f, 1.0f, 0.0f});
  Contract K(W, rep, {0, 1});
  ParseStats st;
  parse(K.data.data(), K.c, K.l, {}, &st);
  EXPECT_EQ(st.n_second_coincident, kColumnHeight);
}

TEST(Canonical, LegacyAxesRoundTripThroughRep103)
{
  std::vector<P> pts(2 * kColumnHeight);
  for (size_t i = 0; i < pts.size(); ++i) {pts[i] = {0.3f * i - 7.0f, -1.7f * i - 2.0f, 0.1f * i};}
  Legacy L(pts);
  std::vector<P> rep(pts.size());
  for (size_t i = 0; i < pts.size(); ++i) {rep[i] = {-pts[i].y, pts[i].x, pts[i].z};}
  Contract K(2, rep, {0, 0});
  const auto a = parse(L.data.data(), L.c, L.l);
  const auto b = parse(K.data.data(), K.c, K.l);
  ASSERT_EQ(a.size(), b.size());
  EXPECT_EQ(0, std::memcmp(a.data(), b.data(), a.size() * sizeof(P)));
}

TEST(Canonical, CropKeepsOnlyForwardRangeWithFloorGuard)
{
  std::vector<P> pts(2 * kColumnHeight);
  for (size_t i = 0; i < pts.size(); ++i) {
    const float fwd = -10.0f + 1.5f * static_cast<float>(i % 200);
    pts[i] = {0.1f * (i % 7), -fwd, -1.0f + 0.001f * i};
  }
  Legacy L(pts);
  ParseOptions o;
  o.dedupe_dual_return = false;
  o.crop.enabled = true;
  ParseStats st;
  const auto out = parse(L.data.data(), L.c, L.l, o, &st);
  EXPECT_TRUE(st.crop_guard);
  EXPECT_EQ(out.size(), pts.size());

  std::vector<P> many;
  for (int k = 0; k < 4; ++k) {many.insert(many.end(), pts.begin(), pts.end());}
  for (size_t i = 0; i < 4 * kColumnHeight; ++i) {many[i] = {0.5f, -5.0f - 0.01f * i, -1.5f};}
  Legacy M(many);
  const auto cut = parse(M.data.data(), M.c, M.l, o, &st);
  EXPECT_FALSE(st.crop_guard);
  EXPECT_TRUE(st.crop_applied);
  EXPECT_GT(st.n_cropped, 0u);
  for (const auto & p : cut) {
    EXPECT_GE(-p.y, 2.0f);
    EXPECT_LE(-p.y, 250.0f);
  }
  EXPECT_EQ(cut.size() + st.n_cropped, many.size());
}

TEST(Canonical, FormatDetection)
{
  CloudDesc c;
  c.height = 1;
  c.width = 256 * 3;
  Layout legacy = tunnel_od::make_layout({{"x", 0, 7, 1}, {"y", 4, 7, 1}, {"z", 8, 7, 1},
    {"timestamp", 18, 8, 1}});
  EXPECT_EQ(tunnel_od::detect_format(c, legacy), Format::LegacyHesai);
  Layout rid = tunnel_od::make_layout({{"x", 0, 7, 1}, {"y", 4, 7, 1}, {"z", 8, 7, 1},
    {"timestamp", 18, 8, 1}, {"return_id", 12, 2, 1}});
  EXPECT_EQ(tunnel_od::detect_format(c, rid), Format::ContractV1);
  c.height = 128;
  c.width = 100;
  EXPECT_EQ(tunnel_od::detect_format(c, tunnel_od::make_layout({{"x", 0, 7, 1}, {"y", 4, 7, 1},
    {"z", 8, 7, 1}})), Format::ContractV1);
  c.height = 16;
  EXPECT_EQ(tunnel_od::detect_format(c, legacy), Format::Generic);
  EXPECT_EQ(tunnel_od::default_axes(Format::ContractV1), Axes::Rep103);
  EXPECT_EQ(tunnel_od::default_axes(Format::LegacyHesai), Axes::Legacy);
}

TEST(Canonical, RejectsUnusableClouds)
{
  std::vector<uint8_t> d(100, 0);
  CloudDesc c;
  c.height = 1;
  c.width = 10;
  c.point_step = 12;
  c.row_step = 120;
  c.data_size = d.size();
  Layout l = tunnel_od::make_layout({{"x", 0, 7, 1}, {"y", 4, 7, 1}, {"z", 8, 7, 1}});
  EXPECT_NE(tunnel_od::validate(c, l), "");
  c.width = 8;
  c.row_step = 96;
  EXPECT_EQ(tunnel_od::validate(c, l), "");
  Layout f64 = tunnel_od::make_layout({{"x", 0, 8, 1}, {"y", 8, 8, 1}, {"z", 16, 8, 1}});
  EXPECT_NE(tunnel_od::validate(c, f64), "");
}
