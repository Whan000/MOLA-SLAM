/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2025, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/maps/CColouredPointsMap.h>
#include <mrpt/maps/CPointsMapXYZI.h>
#include <mrpt/maps/CPointsMapXYZIRT.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/obs/CObservationRotatingScan.h>
#include <mrpt/version.h>

#if MRPT_VERSION >= 0x20f00  // 2.15.0
#include <mrpt/maps/CGenericPointsMap.h>
#endif

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <set>
#include <string>

namespace mrpt::ros2bridge
{
/** \addtogroup mrpt_ros2bridge_grp
 * @{ */

/** @name sensor_msgs::msg::PointCloud2: ROS <-> MRPT
 *  @{ */

/** Convert sensor_msgs/PointCloud2 -> mrpt::maps::CSimplePointsMap
 *  Only (x,y,z) data is converted. To use the intensity channel, see
 * the alternative signatures for CPointsMapXYZI or CPointsMapXYZIRT
 * Requires point cloud fields: x,y,z.
 * \return true on successful conversion, false on any error.
 * \sa toROS
 */
bool fromROS(const sensor_msgs::msg::PointCloud2& msg, mrpt::maps::CSimplePointsMap& obj);

/** \overload For (x,y,z,intensity) channels.
 * Requires point cloud fields: x,y,z,intensity
 */
bool fromROS(const sensor_msgs::msg::PointCloud2& msg, mrpt::maps::CPointsMapXYZI& obj);

/** \overload For (x,y,z,intensity,ring,time) channels.
 * Requires point cloud fields: x,y,z,intensity,ring,time
 */
bool fromROS(const sensor_msgs::msg::PointCloud2& msg, mrpt::maps::CPointsMapXYZIRT& obj);

#if MRPT_VERSION >= 0x20f00  // 2.15.0

/** Convert sensor_msgs/PointCloud2 to mrpt::maps::CGenericPointsMap.
 *
 * Conventions for ROS2 input PointCloud2 message fields:
 *  - Mandatory: x,y,z (float32)
 *  - Optional: timestamp/time/t (float32 or float64 [seconds], or uint32/uint64 [nanoseconds])
 *  - Other fields: float32, float64 (-> will be converted to float32), uint8, uint16, uint32 (->
 *    will be converted to uint16).
 *
 * @param msg Input ROS2 PointCloud2 message
 * @param obj Output MRPT CGenericPointsMap object. Note that its contents will NOT be cleared
 *            intentionally, so this function could be used to append data from multiple messages.
 *            In that case, all messages must have the same fields.
 *
 * \return true on successful conversion, false on any error.
 */
bool fromROS(const sensor_msgs::msg::PointCloud2& msg, mrpt::maps::CGenericPointsMap& obj);

#endif

/** Convert sensor_msgs/PointCloud2 -> mrpt::obs::CObservationRotatingScan.
 * Requires point cloud fields: x,y,z,ring[,intensity][,time]
 *
 * If num_azimuth_divisions=0, it will be taken from the point cloud "width"
 * field.
 *
 * Points are supposed to be given in the sensor frame of reference.
 *
 */
bool fromROS(
    const sensor_msgs::msg::PointCloud2& m,
    mrpt::obs::CObservationRotatingScan& o,
    const mrpt::poses::CPose3D& sensorPoseOnRobot,
    unsigned int num_azimuth_divisions = 0,
    float max_intensity = 1000.0f);

/** Extract a list of fields found in the point cloud.
 * Typically: {"x","y","z","intensity"}
 */
std::set<std::string> extractFields(const sensor_msgs::msg::PointCloud2& msg);

/** Convert mrpt::slam::CSimplePointsMap -> sensor_msgs/PointCloud2
 *  The user must supply the "msg_header" field to be copied into the output
 * message object, since that part does not appear in MRPT classes.
 *
 * Generated sensor_msgs::PointCloud2::channels: `x`, `y`, `z`.
 *
 * \return true on successful conversion, false on any error.
 * \sa fromROS
 */
bool toROS(
    const mrpt::maps::CSimplePointsMap& obj,
    const std_msgs::msg::Header& msg_header,
    sensor_msgs::msg::PointCloud2& msg);

/** \overload With these fields: `x`, `y`, `z`, `intensity`
 * \return true on successful conversion, false on any error.
 * \sa fromROS
 */
bool toROS(
    const mrpt::maps::CPointsMapXYZI& obj,
    const std_msgs::msg::Header& msg_header,
    sensor_msgs::msg::PointCloud2& msg);

/** \overload With these fields: `x`, `y`, `z`, `intensity`, `ring`, `timestamp`
 * \return true on successful conversion, false on any error.
 * \sa fromROS
 */
bool toROS(
    const mrpt::maps::CPointsMapXYZIRT& obj,
    const std_msgs::msg::Header& msg_header,
    sensor_msgs::msg::PointCloud2& msg);

#if MRPT_VERSION >= 0x20f00  // 2.15.0

/** Convert mrpt::maps::CGenericPointsMap with arbitrary per-point-fields to sensor_msgs/PointCloud2
 * \return true on successful conversion, false on any error.
 */
bool toROS(
    const mrpt::maps::CGenericPointsMap& obj,
    const std_msgs::msg::Header& msg_header,
    sensor_msgs::msg::PointCloud2& msg);

#endif

/** @} */
/** @} */

}  // namespace mrpt::ros2bridge
