/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2025, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <mrpt/maps/CColouredPointsMap.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/ros2bridge/point_cloud2.h>
#include <mrpt/ros2bridge/time.h>
#include <mrpt/version.h>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
//
#include <mrpt/config.h>  // MRPT_IS_BIG_ENDIAN

#include <cstdlib>

using namespace mrpt::maps;

namespace
{
bool check_field(
    const sensor_msgs::msg::PointField& input_field,
    std::string check_name,
    const sensor_msgs::msg::PointField** output)
{
  bool coherence_error = false;
  if (input_field.name == check_name)
  {
    if (input_field.datatype != sensor_msgs::msg::PointField::FLOAT32 &&
        input_field.datatype != sensor_msgs::msg::PointField::FLOAT64 &&
        input_field.datatype != sensor_msgs::msg::PointField::UINT16 &&
        input_field.datatype != sensor_msgs::msg::PointField::UINT32 &&
        input_field.datatype != sensor_msgs::msg::PointField::UINT8)
    {
      *output = nullptr;
      coherence_error = true;
    }
    else
    {
      *output = &input_field;
    }
  }
  return coherence_error;
}

void get_float_from_field(
    const sensor_msgs::msg::PointField* field, const unsigned char* data, float& output)
{
  if (field != nullptr)
  {
    if (field->datatype == sensor_msgs::msg::PointField::FLOAT32)
    {
      output = *(reinterpret_cast<const float*>(&data[field->offset]));
    }
    else if (field->datatype == sensor_msgs::msg::PointField::FLOAT64)
    {
      output = static_cast<float>(*(reinterpret_cast<const double*>(&data[field->offset])));
    }
  }
  else
  {
    output = 0.0;
  }
}

void get_double_from_field(
    const sensor_msgs::msg::PointField* field, const unsigned char* data, double& output)
{
  if (field != nullptr)
  {
    if (field->datatype == sensor_msgs::msg::PointField::FLOAT32)
    {
      output = static_cast<double>(*(reinterpret_cast<const float*>(&data[field->offset])));
    }
    else if (field->datatype == sensor_msgs::msg::PointField::FLOAT64)
    {
      output = *(reinterpret_cast<const double*>(&data[field->offset]));
    }
  }
  else
  {
    output = 0.0;
  }
}

void get_uint16_from_field(
    const sensor_msgs::msg::PointField* field, const unsigned char* data, uint16_t& output)
{
  if (field != nullptr)
  {
    if (field->datatype == sensor_msgs::msg::PointField::UINT16)
    {
      output = *(reinterpret_cast<const uint16_t*>(&data[field->offset]));
    }
    else if (field->datatype == sensor_msgs::msg::PointField::UINT8)
    {
      output = *(reinterpret_cast<const uint8_t*>(&data[field->offset]));
    }
  }
  else
  {
    output = 0;
  }
}
void get_uint32_from_field(
    const sensor_msgs::msg::PointField* field, const unsigned char* data, uint32_t& output)
{
  if (field != nullptr)
  {
    if (field->datatype == sensor_msgs::msg::PointField::UINT32)
    {
      output = *(reinterpret_cast<const uint32_t*>(&data[field->offset]));
    }
  }
  else
  {
    output = 0;
  }
}
}  // namespace

std::set<std::string> mrpt::ros2bridge::extractFields(const sensor_msgs::msg::PointCloud2& msg)
{
  std::set<std::string> lst;
  for (const auto& f : msg.fields)
  {
    lst.insert(f.name);
  }
  return lst;
}

/** Convert sensor_msgs/PointCloud2 -> mrpt::slam::CSimplePointsMap
 *
 * \return true on successful conversion, false on any error.
 */
bool mrpt::ros2bridge::fromROS(const sensor_msgs::msg::PointCloud2& msg, CSimplePointsMap& obj)
{
  // Copy point data
  unsigned int num_points = msg.width * msg.height;
  obj.clear();
  obj.reserve(num_points);

  bool incompatible = false;
  const sensor_msgs::msg::PointField *x_field = nullptr, *y_field = nullptr, *z_field = nullptr;

  for (unsigned int i = 0; i < msg.fields.size() && !incompatible; i++)
  {
    incompatible |= check_field(msg.fields[i], "x", &x_field);
    incompatible |= check_field(msg.fields[i], "y", &y_field);
    incompatible |= check_field(msg.fields[i], "z", &z_field);
  }

  if (incompatible || (!x_field || !y_field || !z_field))
  {
    return false;
  }

  // If not, memcpy each group of contiguous fields separately
  for (std::size_t row = 0; row < msg.height; ++row)
  {
    const unsigned char* row_data = &msg.data[row * msg.row_step];
    for (std::size_t col = 0; col < msg.width; ++col)
    {
      const unsigned char* msg_data = row_data + col * msg.point_step;

      float x = 0, y = 0, z = 0;
      get_float_from_field(x_field, msg_data, x);
      get_float_from_field(y_field, msg_data, y);
      get_float_from_field(z_field, msg_data, z);
      obj.insertPoint(x, y, z);
    }
  }

  return true;
}

bool mrpt::ros2bridge::fromROS(const sensor_msgs::msg::PointCloud2& msg, CPointsMapXYZI& obj)
{
  // Copy point data
  unsigned int num_points = msg.width * msg.height;
  obj.clear();
  obj.reserve(num_points);

  bool incompatible = false;
  const sensor_msgs::msg::PointField *x_field = nullptr, *y_field = nullptr, *z_field = nullptr,
                                     *i_field = nullptr;

  for (unsigned int i = 0; i < msg.fields.size() && !incompatible; i++)
  {
    incompatible |= check_field(msg.fields[i], "x", &x_field);
    incompatible |= check_field(msg.fields[i], "y", &y_field);
    incompatible |= check_field(msg.fields[i], "z", &z_field);
    incompatible |= check_field(msg.fields[i], "intensity", &i_field);
  }

  if (incompatible || (!x_field || !y_field || !z_field || !i_field))
  {
    return false;
  }

#if MRPT_VERSION >= 0x20f00  // 2.15.0
  auto* Is = obj.getPointsBufferRef_float_field(CPointsMapXYZI::POINT_FIELD_INTENSITY);
  ASSERT_(Is);
#endif

  for (std::size_t row = 0; row < msg.height; ++row)
  {
    const unsigned char* row_data = &msg.data[row * msg.row_step];
    for (std::size_t col = 0; col < msg.width; ++col)
    {
      const unsigned char* msg_data = row_data + col * msg.point_step;

      float x = 0, y = 0, z = 0, i = 0;
      get_float_from_field(x_field, msg_data, x);
      get_float_from_field(y_field, msg_data, y);
      get_float_from_field(z_field, msg_data, z);
      get_float_from_field(i_field, msg_data, i);
      obj.insertPoint(x, y, z);

#if MRPT_VERSION >= 0x20f00  // 2.15.0
      Is->push_back(i);
#else
      obj.insertPointField_Intensity(i);
#endif
    }
  }
  return true;
}

bool mrpt::ros2bridge::fromROS(const sensor_msgs::msg::PointCloud2& msg, CPointsMapXYZIRT& obj)
{
  // Copy point data
  unsigned int num_points = msg.width * msg.height;

  bool incompatible = false;
  const sensor_msgs::msg::PointField *x_field = nullptr, *y_field = nullptr, *z_field = nullptr,
                                     *i_field = nullptr, *r_field = nullptr, *t_field = nullptr;

  for (unsigned int i = 0; i < msg.fields.size() && !incompatible; i++)
  {
    incompatible |= check_field(msg.fields[i], "x", &x_field);
    incompatible |= check_field(msg.fields[i], "y", &y_field);
    incompatible |= check_field(msg.fields[i], "z", &z_field);
    incompatible |= check_field(msg.fields[i], "intensity", &i_field);
    incompatible |= check_field(msg.fields[i], "ring", &r_field);

    incompatible |= check_field(msg.fields[i], "timestamp", &t_field);
    incompatible |= check_field(msg.fields[i], "time", &t_field);
    incompatible |= check_field(msg.fields[i], "t", &t_field);
  }

  if (incompatible || (!x_field || !y_field || !z_field))
  {
    return false;
  }

  obj.resize_XYZIRT(num_points, !!i_field, !!r_field, !!t_field);

  unsigned int idx = 0;
  std::optional<double> baseTimeStamp;
  for (std::size_t row = 0; row < msg.height; ++row)
  {
    const unsigned char* row_data = &msg.data[row * msg.row_step];
    for (std::size_t col = 0; col < msg.width; ++col, ++idx)
    {
      const unsigned char* msg_data = row_data + col * msg.point_step;

      float x = 0, y = 0, z = 0;
      get_float_from_field(x_field, msg_data, x);
      get_float_from_field(y_field, msg_data, y);
      get_float_from_field(z_field, msg_data, z);
      obj.setPointFast(idx, x, y, z);

      if (i_field)
      {
        float i = 0;
        get_float_from_field(i_field, msg_data, i);
        obj.setPointIntensity(idx, i);
      }
      if (r_field)
      {
        uint16_t ring_id = 0;
        get_uint16_from_field(r_field, msg_data, ring_id);
        obj.setPointRing(idx, ring_id);
      }
      if (t_field)
      {
        double t = 0;

        if (t_field->datatype == sensor_msgs::msg::PointField::FLOAT32 ||
            t_field->datatype == sensor_msgs::msg::PointField::FLOAT64)
        {
          get_double_from_field(t_field, msg_data, t);
        }
        else
        {
          uint32_t tim = 0;

          get_uint32_from_field(t_field, msg_data, tim);

          // Convention: they seem to be nanoseconds:
          t = tim * 1e-9;
        }

        // If the sensor uses absolute timestamp, convert them to relative
        // since otherwise precision is lost in the double->float conversion:
        if (std::abs(t) > 5.0)
        {
          // It looks like absolute timestamps, convert to relative:
          if (!baseTimeStamp)
          {
            baseTimeStamp = t;
          }
          obj.setPointTime(idx, static_cast<float>(t - *baseTimeStamp));
        }
        else
        {
          // It looks relative timestamps:
          obj.setPointTime(idx, static_cast<float>(t));
        }
      }
    }
  }
  return true;
}

#if MRPT_VERSION >= 0x20f00  // 2.15.0
bool mrpt::ros2bridge::fromROS(
    const sensor_msgs::msg::PointCloud2& msg, mrpt::maps::CGenericPointsMap& obj)
{
  // Copy point data
  unsigned int num_points = msg.width * msg.height;

  bool incompatible = false;
  const sensor_msgs::msg::PointField* x_field = nullptr;
  const sensor_msgs::msg::PointField* y_field = nullptr;
  const sensor_msgs::msg::PointField* z_field = nullptr;
  const sensor_msgs::msg::PointField* t_field = nullptr;
  std::map<std::string, const sensor_msgs::msg::PointField*> other_fields_float;
  std::map<std::string, const sensor_msgs::msg::PointField*> other_fields_uint;

  for (const auto& field : msg.fields)
  {
    if (field.name == "x" || field.name == "y" || field.name == "z")
    {
      incompatible |= check_field(field, "x", &x_field);
      incompatible |= check_field(field, "y", &y_field);
      incompatible |= check_field(field, "z", &z_field);
      continue;
    }

    // Timestamp per point must be handled specially to handle different conventions:
    if (field.name == "timestamp" || field.name == "time" || field.name == "t")
    {
      incompatible |= check_field(field, "timestamp", &t_field);
      incompatible |= check_field(field, "time", &t_field);
      incompatible |= check_field(field, "t", &t_field);
      continue;
    }

    if (field.datatype == sensor_msgs::msg::PointField::FLOAT32 ||
        field.datatype == sensor_msgs::msg::PointField::FLOAT64)
    {
      other_fields_float[field.name] = &field;
    }
    else if (
        field.datatype == sensor_msgs::msg::PointField::UINT16 ||
        field.datatype == sensor_msgs::msg::PointField::UINT32 ||
        field.datatype == sensor_msgs::msg::PointField::UINT8)
    {
      other_fields_uint[field.name] = &field;
    }
  }

  if (incompatible || (!x_field || !y_field || !z_field))
  {
    return false;
  }

  for (const auto& [name, _] : other_fields_float)
  {
    obj.registerField_float(name);
  }
  for (const auto& [name, _] : other_fields_uint)
  {
    obj.registerField_uint16(name);
  }
  if (t_field)
  {
    obj.registerField_float("t");
  }

  obj.resize(num_points);

  unsigned int idx = 0;
  std::optional<double> baseTimeStamp;
  for (std::size_t row = 0; row < msg.height; ++row)
  {
    const unsigned char* row_data = &msg.data[static_cast<std::size_t>(row) * msg.row_step];
    for (std::size_t col = 0; col < msg.width; ++col, ++idx)
    {
      const unsigned char* msg_data = row_data + static_cast<std::size_t>(col) * msg.point_step;

      float x = 0, y = 0, z = 0;
      get_float_from_field(x_field, msg_data, x);
      get_float_from_field(y_field, msg_data, y);
      get_float_from_field(z_field, msg_data, z);
      obj.setPointFast(idx, x, y, z);

      for (const auto& [name, field_ptr] : other_fields_float)
      {
        float val = 0;
        get_float_from_field(field_ptr, msg_data, val);
        obj.setPointField_float(idx, name, val);
      }
      for (const auto& [name, field_ptr] : other_fields_uint)
      {
        uint16_t val = 0;
        get_uint16_from_field(field_ptr, msg_data, val);
        obj.setPointField_uint16(idx, name, val);
      }

      if (t_field)
      {
        double t = 0;

        if (t_field->datatype == sensor_msgs::msg::PointField::FLOAT32 ||
            t_field->datatype == sensor_msgs::msg::PointField::FLOAT64)
        {
          get_double_from_field(t_field, msg_data, t);
        }
        else
        {
          uint32_t tim = 0;

          get_uint32_from_field(t_field, msg_data, tim);

          // Convention: they seem to be nanoseconds:
          t = tim * 1e-9;
        }

        // If the sensor uses absolute timestamp, convert them to relative
        // since otherwise precision is lost in the double->float conversion:
        if (std::abs(t) > 5.0)
        {
          // It looks like absolute timestamps, convert to relative:
          if (!baseTimeStamp)
          {
            baseTimeStamp = t;
          }
          obj.setPointField_float(idx, "t", static_cast<float>(t - *baseTimeStamp));
        }
        else
        {
          // It looks relative timestamps:
          obj.setPointField_float(idx, "t", static_cast<float>(t));
        }
      }
    }
  }
  return true;
}

bool mrpt::ros2bridge::toROS(
    const mrpt::maps::CGenericPointsMap& obj,
    const std_msgs::msg::Header& msg_header,
    sensor_msgs::msg::PointCloud2& msg)
{
  //
  {
    msg.header = msg_header;

    // 2D structure of the point cloud. If the cloud is unordered, height is
    //  1 and width is the length of the point cloud.
    msg.height = 1;
    msg.width = obj.size();

    // Basic XYZ fields:
    std::vector<std::string> names = {"x", "y", "z"};
    std::vector<size_t> offsets = {0, sizeof(float) * 1, sizeof(float) * 2};
    size_t point_step = sizeof(float) * 3;

    // Gather additional registered fields in the generic map:
    // Float fields (including "t" if present) and unsigned integer fields (uint16)
    std::vector<std::string_view> float_fields;
    std::vector<std::string_view> uint16_fields;

    // The following two calls assume CGenericPointsMap exposes methods to list
    // registered fields. Adjust these method names to the actual API if needed.
    float_fields = obj.getPointFieldNames_float();
    uint16_fields = obj.getPointFieldNames_uint16();

    // Remove x,y,z from the registered lists if present:
    auto remove_name = [](std::vector<std::string_view>& vec, const std::string& n)
    { vec.erase(std::remove(vec.begin(), vec.end(), n), vec.end()); };
    remove_name(float_fields, "x");
    remove_name(float_fields, "y");
    remove_name(float_fields, "z");
    remove_name(uint16_fields, "x");
    remove_name(uint16_fields, "y");
    remove_name(uint16_fields, "z");

    // Append float fields
    for (const auto& fn : float_fields)
    {
      names.push_back(std::string(fn));
      offsets.push_back(point_step);
      point_step += sizeof(float);
    }

    // Append uint16 fields
    for (const auto& un : uint16_fields)
    {
      names.push_back(std::string(un));
      offsets.push_back(point_step);
      point_step += sizeof(uint16_t);
    }

    // Build msg.fields
    msg.fields.resize(names.size());
    for (size_t i = 0; i < names.size(); ++i)
    {
      auto& f = msg.fields[i];
      f.count = 1;
      f.offset = static_cast<uint32_t>(offsets[i]);
      // ring-like uint16 fields:
      if (std::find(uint16_fields.begin(), uint16_fields.end(), names[i]) != uint16_fields.end())
      {
        f.datatype = sensor_msgs::msg::PointField::UINT16;
      }
      else
      {
        f.datatype = sensor_msgs::msg::PointField::FLOAT32;
      }
      f.name = names[i];
    }

#if MRPT_IS_BIG_ENDIAN
    msg.is_bigendian = true;
#else
    msg.is_bigendian = false;
#endif

    msg.point_step = static_cast<uint32_t>(point_step);
    msg.row_step = msg.width * msg.point_step;

    // Resize data buffer
    msg.data.resize(static_cast<std::size_t>(msg.row_step) * msg.height);

    // Access base point coordinate buffers:
    const auto& xs = obj.getPointsBufferRef_x();
    const auto& ys = obj.getPointsBufferRef_y();
    const auto& zs = obj.getPointsBufferRef_z();
    const size_t N = xs.size();
    ASSERT_EQUAL_(ys.size(), N);
    ASSERT_EQUAL_(zs.size(), N);
    ASSERT_EQUAL_(msg.width, N);

    // Prepare pointers to additional fields buffers:
    std::map<std::string_view, const mrpt::aligned_std_vector<float>*> float_bufs;
    std::map<std::string_view, const mrpt::aligned_std_vector<uint16_t>*> uint16_bufs;

    for (const auto& fn : float_fields)
    {
      const auto* v = obj.getPointsBufferRef_float_field(fn);
      ASSERT_(v);
      ASSERT_EQUAL_(v->size(), N);
      float_bufs[fn] = v;
    }
    for (const auto& un : uint16_fields)
    {
      const auto* v = obj.getPointsBufferRef_uint_field(un);
      ASSERT_(v);
      ASSERT_EQUAL_(v->size(), N);
      uint16_bufs[un] = v;
    }

    // Fill data per point
    uint8_t* dst = msg.data.data();
    for (size_t i = 0; i < N; ++i)
    {
      // x,y,z
      memcpy(dst + offsets[0], &xs[i], sizeof(float));
      memcpy(dst + offsets[1], &ys[i], sizeof(float));
      memcpy(dst + offsets[2], &zs[i], sizeof(float));

      // float fields
      for (size_t fi = 0; fi < float_fields.size(); ++fi)
      {
        const auto& name = float_fields[fi];
        const auto* buf = float_bufs[name];
        memcpy(dst + offsets[3 + fi], &(*buf)[i], sizeof(float));
      }

      // uint16 fields
      for (size_t ui = 0; ui < uint16_fields.size(); ++ui)
      {
        const auto& name = uint16_fields[ui];
        const auto* buf = uint16_bufs[name];
        memcpy(dst + offsets[3 + float_fields.size() + ui], &(*buf)[i], sizeof(uint16_t));
      }

      dst += msg.point_step;
    }

    return true;
  }
}

#endif

bool mrpt::ros2bridge::toROS(
    const CSimplePointsMap& obj,
    const std_msgs::msg::Header& msg_header,
    sensor_msgs::msg::PointCloud2& msg)
{
  msg.header = msg_header;

  // 2D structure of the point cloud. If the cloud is unordered, height is
  //  1 and width is the length of the point cloud.
  msg.height = 1;
  msg.width = obj.size();

  std::array<std::string, 3> names = {"x", "y", "z"};
  std::array<size_t, 3> offsets = {0, sizeof(float) * 1, sizeof(float) * 2};

  msg.fields.resize(3);
  for (size_t i = 0; i < 3; i++)
  {
    auto& f = msg.fields.at(i);

    f.count = 1;
    f.offset = offsets[i];
    f.datatype = sensor_msgs::msg::PointField::FLOAT32;
    f.name = names[i];
  }

#if MRPT_IS_BIG_ENDIAN
  msg.is_bigendian = true;
#else
  msg.is_bigendian = false;
#endif

  msg.point_step = sizeof(float) * 3;
  msg.row_step = msg.width * msg.point_step;

  // data:
  msg.data.resize(msg.row_step * msg.height);

  const auto& xs = obj.getPointsBufferRef_x();
  const auto& ys = obj.getPointsBufferRef_y();
  const auto& zs = obj.getPointsBufferRef_z();

  float* pointDest = reinterpret_cast<float*>(msg.data.data());
  for (size_t i = 0; i < xs.size(); i++)
  {
    *pointDest++ = xs[i];
    *pointDest++ = ys[i];
    *pointDest++ = zs[i];
  }

  return true;
}

bool mrpt::ros2bridge::toROS(
    const CPointsMapXYZI& obj,
    const std_msgs::msg::Header& msg_header,
    sensor_msgs::msg::PointCloud2& msg)
{
  msg.header = msg_header;

  // 2D structure of the point cloud. If the cloud is unordered, height is
  //  1 and width is the length of the point cloud.
  msg.height = 1;
  msg.width = obj.size();

  std::array<std::string, 4> names = {"x", "y", "z", "intensity"};
  std::array<size_t, 4> offsets = {0, sizeof(float) * 1, sizeof(float) * 2, sizeof(float) * 3};

  msg.fields.resize(4);
  for (size_t i = 0; i < 4; i++)
  {
    auto& f = msg.fields.at(i);

    f.count = 1;
    f.offset = offsets[i];
    f.datatype = sensor_msgs::msg::PointField::FLOAT32;
    f.name = names[i];
  }

#if MRPT_IS_BIG_ENDIAN
  msg.is_bigendian = true;
#else
  msg.is_bigendian = false;
#endif

  msg.point_step = sizeof(float) * 4;
  msg.row_step = msg.width * msg.point_step;

  // data:
  msg.data.resize(static_cast<std::size_t>(msg.row_step) * msg.height);

  const auto& xs = obj.getPointsBufferRef_x();
  const auto& ys = obj.getPointsBufferRef_y();
  const auto& zs = obj.getPointsBufferRef_z();

#if MRPT_VERSION >= 0x20f00  // 2.15.0
  const auto* Is = obj.getPointsBufferRef_float_field(CPointsMapXYZI::POINT_FIELD_INTENSITY);
#else
  const auto* Is = obj.getPointsBufferRef_intensity();
#endif
  ASSERT_(Is);

  float* pointDest = reinterpret_cast<float*>(msg.data.data());
  for (size_t i = 0; i < xs.size(); i++)
  {
    *pointDest++ = xs[i];
    *pointDest++ = ys[i];
    *pointDest++ = zs[i];
    *pointDest++ = (*Is)[i];
  }

  return true;
}

bool mrpt::ros2bridge::toROS(
    const CPointsMapXYZIRT& obj,
    const std_msgs::msg::Header& msg_header,
    sensor_msgs::msg::PointCloud2& msg)
{
  msg.header = msg_header;

  // 2D structure of the point cloud. If the cloud is unordered, height is
  //  1 and width is the length of the point cloud.
  msg.height = 1;
  msg.width = obj.size();

  std::vector<std::string> names = {"x", "y", "z"};
  std::vector<size_t> offsets = {0, sizeof(float) * 1, sizeof(float) * 2};

  msg.point_step = sizeof(float) * 3;

#if MRPT_VERSION >= 0x20f00  // 2.15.0
  const auto* Is = obj.getPointsBufferRef_float_field(CPointsMapXYZIRT::POINT_FIELD_INTENSITY);
  const auto* Rs = obj.getPointsBufferRef_uint_field(CPointsMapXYZIRT::POINT_FIELD_RING_ID);
  const auto* Ts = obj.getPointsBufferRef_float_field(CPointsMapXYZIRT::POINT_FIELD_TIMESTAMP);
#else
  const auto* Is = obj.getPointsBufferRef_intensity();
  const auto* Rs = obj.getPointsBufferRef_ring();
  const auto* Ts = obj.getPointsBufferRef_timestamp();
#endif

  if (obj.hasIntensityField())
  {
    ASSERT_(Is);
    ASSERT_EQUAL_(Is->size(), obj.size());
    names.push_back("intensity");
    offsets.push_back(msg.point_step);
    msg.point_step += sizeof(float);
  }
  if (obj.hasTimeField())
  {
    ASSERT_(Ts);
    ASSERT_EQUAL_(Ts->size(), obj.size());
    names.push_back("time");
    offsets.push_back(msg.point_step);
    msg.point_step += sizeof(float);
  }
  if (obj.hasRingField())
  {
    ASSERT_(Rs);
    ASSERT_EQUAL_(Rs->size(), obj.size());
    names.push_back("ring");
    offsets.push_back(msg.point_step);
    msg.point_step += sizeof(uint16_t);
  }

  msg.fields.resize(names.size());
  for (size_t i = 0; i < names.size(); i++)
  {
    auto& f = msg.fields.at(i);

    f.count = 1;
    f.offset = offsets[i];
    f.datatype = (names[i] == "ring") ? sensor_msgs::msg::PointField::UINT16
                                      : sensor_msgs::msg::PointField::FLOAT32;
    f.name = names[i];
  }

#if MRPT_IS_BIG_ENDIAN
  msg.is_bigendian = true;
#else
  msg.is_bigendian = false;
#endif

  msg.row_step = msg.width * msg.point_step;

  // data:
  msg.data.resize(static_cast<std::size_t>(msg.row_step) * msg.height);

  const auto& xs = obj.getPointsBufferRef_x();
  const auto& ys = obj.getPointsBufferRef_y();
  const auto& zs = obj.getPointsBufferRef_z();

  uint8_t* pointDest = msg.data.data();
  for (size_t i = 0; i < xs.size(); i++)
  {
    int f = 0;
    memcpy(pointDest + offsets[f++], &xs[i], sizeof(float));
    memcpy(pointDest + offsets[f++], &ys[i], sizeof(float));
    memcpy(pointDest + offsets[f++], &zs[i], sizeof(float));

    if (obj.hasIntensityField())
    {
      memcpy(pointDest + offsets[f++], &(*Is)[i], sizeof(float));
    }

    if (obj.hasTimeField())
    {
      memcpy(pointDest + offsets[f++], &(*Ts)[i], sizeof(float));
    }

    if (obj.hasRingField())
    {
      memcpy(pointDest + offsets[f++], &(*Rs)[i], sizeof(uint16_t));
    }

    pointDest += msg.point_step;
  }

  return true;
}

/** Convert sensor_msgs/PointCloud2 -> mrpt::obs::CObservationRotatingScan */
bool mrpt::ros2bridge::fromROS(
    const sensor_msgs::msg::PointCloud2& msg,
    mrpt::obs::CObservationRotatingScan& obj,
    const mrpt::poses::CPose3D& sensorPoseOnRobot,
    unsigned int num_azimuth_divisions,
    float max_intensity)
{
  // Copy point data
  obj.timestamp = mrpt::ros2bridge::fromROS(msg.header.stamp);
  obj.originalReceivedTimestamp = obj.timestamp;

  bool incompatible = false;
  const sensor_msgs::msg::PointField *x_field = nullptr, *y_field = nullptr, *z_field = nullptr,
                                     *i_field = nullptr, *ring_field = nullptr;

  for (unsigned int i = 0; i < msg.fields.size() && !incompatible; i++)
  {
    incompatible |= check_field(msg.fields[i], "x", &x_field);
    incompatible |= check_field(msg.fields[i], "y", &y_field);
    incompatible |= check_field(msg.fields[i], "z", &z_field);
    incompatible |= check_field(msg.fields[i], "ring", &ring_field);
    check_field(msg.fields[i], "intensity", &i_field);
  }

  if (incompatible || (!x_field || !y_field || !z_field || !ring_field))
  {
    return false;
  }

  // 1st: go through the scan and find ring count:
  uint16_t ring_min = 0, ring_max = 0;

  for (std::size_t row = 0; row < msg.height; ++row)
  {
    const unsigned char* row_data = &msg.data[row * msg.row_step];
    for (std::size_t col = 0; col < msg.width; ++col)
    {
      const unsigned char* msg_data = row_data + col * msg.point_step;
      uint16_t ring_id = 0;
      get_uint16_from_field(ring_field, msg_data, ring_id);

      mrpt::keep_min(ring_min, ring_id);
      mrpt::keep_max(ring_max, ring_id);
    }
  }
  ASSERT_NOT_EQUAL_(ring_min, ring_max);

  obj.rowCount = ring_max - ring_min + 1;

  const bool inputCloudIsOrganized = msg.height != 1;

  if (!num_azimuth_divisions)
  {
    if (inputCloudIsOrganized)
    {
      ASSERT_GT_(msg.width, 0U);
      num_azimuth_divisions = msg.width;
    }
    else
    {
      THROW_EXCEPTION(
          "An explicit value for num_azimuth_divisions must be given if "
          "the input cloud is not 'organized'");
    }
  }

  obj.columnCount = num_azimuth_divisions;

  obj.rangeImage.resize(obj.rowCount, obj.columnCount);
  obj.rangeImage.fill(0);

  obj.sensorPose = sensorPoseOnRobot;

  // Default unit: 1cm
  if (obj.rangeResolution == 0)
  {
    obj.rangeResolution = 1e-2;
  }

  if (i_field)
  {
    obj.intensityImage.resize(obj.rowCount, obj.columnCount);
    obj.intensityImage.fill(0);
  }
  else
  {
    obj.intensityImage.resize(0, 0);
  }

  if (inputCloudIsOrganized)
  {
    obj.organizedPoints.resize(obj.rowCount, obj.columnCount);
  }

  // If not, memcpy each group of contiguous fields separately
  for (std::size_t row = 0; row < msg.height; ++row)
  {
    const unsigned char* row_data = &msg.data[row * msg.row_step];
    for (std::size_t col = 0; col < msg.width; ++col)
    {
      const unsigned char* msg_data = row_data + col * msg.point_step;

      float x = 0, y = 0, z = 0;
      uint16_t ring_id = 0;
      get_float_from_field(x_field, msg_data, x);
      get_float_from_field(y_field, msg_data, y);
      get_float_from_field(z_field, msg_data, z);
      get_uint16_from_field(ring_field, msg_data, ring_id);

      const mrpt::math::TPoint3D localPt = {x, y, z};

      unsigned int az_idx;
      if (inputCloudIsOrganized)
      {
        // "azimuth index" is just the "column":
        az_idx = col;
      }
      else
      {
        // Recover "azimuth index" from trigonometry:
        const double azimuth = std::atan2(localPt.y, localPt.x);

        az_idx = lround((num_azimuth_divisions - 1) * (azimuth + M_PI) / (2 * M_PI));
        ASSERT_LE_(az_idx, num_azimuth_divisions - 1);
      }

      // Store in matrix form:
      obj.rangeImage(ring_id, az_idx) = lround(localPt.norm() / obj.rangeResolution);

      if (i_field)
      {
        float intensity = 0;
        get_float_from_field(i_field, msg_data, intensity);
        obj.intensityImage(ring_id, az_idx) = lround(255 * intensity / max_intensity);
      }

      if (inputCloudIsOrganized)
      {
        obj.organizedPoints(ring_id, az_idx) = localPt;
      }
    }
  }

  return true;
}
