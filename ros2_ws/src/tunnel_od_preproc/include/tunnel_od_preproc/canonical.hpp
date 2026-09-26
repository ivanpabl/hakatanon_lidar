// Разбор входного PointCloud2 в каноническое облако ядра tunnel_od. Без ROS 2:
// тот же исходник собирается в узел (preproc_node), в gtest и в библиотеку для
// Python (ctypes, c_api.h).
//
// Каноническое облако -- то, что ждёт ядро (core/tunnel_od):
//   x -- вбок, вперёд = -y, z -- вверх (оси записей Hesai, "legacy");
//   только точки с отражением (без NaN/inf и нулей (0,0,0));
//   без дублей dual return;
//   порядок -- как у parse_pointcloud2 на записях: столбец за столбцом, внутри
//   столбца по ring.
// На legacy-данных результат побитно совпадает с parse_pointcloud2 (Python).
#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace tunnel_od
{

constexpr uint32_t kColumnHeight = 128;          // каналов в столбце (Pandar128)
constexpr float kDualReturnDupM = 0.01f;         // pointcloud.DUAL_RETURN_DUP_M

// sensor_msgs/PointField::datatype
enum PointFieldType : uint8_t
{
  kInt8 = 1, kUint8 = 2, kInt16 = 3, kUint16 = 4, kInt32 = 5, kUint32 = 6, kFloat32 = 7, kFloat64 = 8
};

enum class Format { Auto, LegacyHesai, ContractV1, Generic };
enum class Axes { Auto, Legacy, Rep103 };

const char * to_string(Format f);
const char * to_string(Axes a);
bool parse_format(const std::string & s, Format * out);
bool parse_axes(const std::string & s, Axes * out);

struct Field
{
  std::string name;
  uint32_t offset = 0;
  uint8_t datatype = 0;
  uint32_t count = 1;
};

// Смещение и тип поля; offset < 0 -- поля нет.
struct FieldRef
{
  int32_t offset = -1;
  uint8_t type = 0;
  bool present() const {return offset >= 0;}
};

struct Layout
{
  FieldRef x, y, z, intensity, ring, return_id, t_offset_ns, timestamp;
  std::string signature;       // "имя:смещение:тип,..." -- для отслеживания смены раскладки
};

struct CloudDesc
{
  uint32_t height = 0;
  uint32_t width = 0;
  uint32_t point_step = 0;
  uint32_t row_step = 0;
  bool is_bigendian = false;
  bool is_dense = false;
  size_t data_size = 0;
  size_t n_points() const {return static_cast<size_t>(height) * width;}
};

Layout make_layout(const std::vector<Field> & fields);

// Пустая строка -- облако можно разобрать; иначе причина, почему нельзя.
std::string validate(const CloudDesc & cloud, const Layout & layout);

// Формат по fields, height и width (не по размеру кадра):
//   legacy_hesai: height == 1, есть timestamp (float64), нет return_id, width % 256 == 0;
//   contract_v1:  height == 128 или есть return_id / t_offset_ns;
//   иначе generic.
Format detect_format(const CloudDesc & cloud, const Layout & layout);

// Оси по умолчанию для формата: rep103 для contract_v1, legacy для остальных.
Axes default_axes(Format f);

// Обрезка по дальности вперёд (fwd = -y в осях ядра). Режется только то, что ядро
// отбрасывает само (см. docs/INPUT_FORMAT.md, "Обрезка"). Если в кадре меньше
// kFloorGuardMin точек в окне оценки пола (bed.estimate_floor_z), кадр не режется:
// тогда запасная оценка пола берёт перцентиль по всем точкам.
struct CropParams
{
  bool enabled = false;
  double fwd_min = 2.0;
  double fwd_max = 250.0;
};
constexpr float kFloorGuardNear = 2.0f;         // bed.estimate_floor_z: near
constexpr float kFloorGuardFar = 15.0f;         //                       far
constexpr float kFloorGuardHalfWidth = 3.0f;    //                       half_width
constexpr uint64_t kFloorGuardMin = 50;         //                       m.sum() < 50

// Поворот облака (apply_mount_tf): p' = R * p в осях входа, затем перевод в оси ядра.
struct Rotation
{
  bool enabled = false;
  double m[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};    // по строкам
  Axes out_axes = Axes::Rep103;                  // оси после поворота (base_link -- REP-103)
};

struct ParseOptions
{
  Format format = Format::Auto;
  Axes axes = Axes::Auto;
  bool dedupe_dual_return = true;
  CropParams crop;
  Rotation rotation;
  bool collect_stats = true;
};

// Что увидели в кадре (для проверки контракта). Считается в том же проходе.
struct ParseStats
{
  Format format = Format::Generic;
  Axes axes = Axes::Legacy;
  uint64_t n_in = 0;              // точек во входе (height * width)
  uint64_t n_nonfinite = 0;       // NaN/inf хотя бы в одной из x, y, z
  uint64_t n_zero = 0;            // (0, 0, 0)
  uint64_t n_dup = 0;             // удалено как дубль dual return (legacy)
  uint64_t n_valid = 0;           // после удаления пустых и дублей
  uint64_t n_cropped = 0;         // отрезано по дальности
  uint64_t n_out = 0;             // в выходе
  bool dedupe_applied = false;
  bool crop_applied = false;
  bool crop_guard = false;        // обрезка отменена: мало точек пола (kFloorGuardMin)
  // dual return
  uint64_t pairs_both_valid = 0;  // legacy: пары (2k, 2k+1), где обе точки с отражением
  uint64_t pairs_coincident = 0;  //         ... и совпадают (< 1 см)
  uint64_t n_second = 0;          // contract: точек с return_id != 0
  uint64_t n_second_valid = 0;    //           ... с отражением
  uint64_t n_second_coincident = 0;  //        ... совпадающих с первым отражением (нарушение M2)
  // порядок каналов
  bool ring_checked = false;
  uint64_t ring_mismatch = 0;     // ring != номер строки (contract) / i % 128 (legacy)
  // время точек, с от header.stamp (legacy: timestamp - stamp; contract: t_offset_ns)
  bool have_point_time = false;
  double t_min = 0.0, t_max = 0.0;
  // азимут в осях ядра, град (atan2(x, -y)), по выборке точек с отражением
  uint64_t n_az = 0;
  double az_min = 0.0, az_max = 0.0, az_center = 0.0;
};

// Разбор. out_x/out_y/out_z -- куда писать (ёмкость не меньше cloud.n_points()),
// шаг out_stride во float (3 -- xyz подряд, 1 -- три отдельных массива).
// stamp_s -- header.stamp в секундах (для времени точек legacy).
// Возвращает число точек в выходе; при ошибке -- 0 и *error.
size_t parse_cloud(
  const uint8_t * data, const CloudDesc & cloud, const Layout & layout,
  const ParseOptions & opt, double stamp_s,
  float * out_x, float * out_y, float * out_z, size_t out_stride,
  ParseStats * stats, std::string * error);

}  // namespace tunnel_od
