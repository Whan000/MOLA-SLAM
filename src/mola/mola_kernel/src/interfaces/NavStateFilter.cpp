/* -------------------------------------------------------------------------
 *   A Modular Optimization framework for Localization and mApping  (MOLA)
 *
 * Copyright (C) 2018-2025 Jose Luis Blanco, University of Almeria
 * Licensed under the GNU GPL v3 for non-commercial applications.
 *
 * This file is part of MOLA.
 * MOLA is free software: you can redistribute it and/or modify it under the
 * terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * MOLA is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE. See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * MOLA. If not, see <https://www.gnu.org/licenses/>.
 * ------------------------------------------------------------------------- */
/**
 * @file   NavStateFilter.cpp
 * @brief  State vector for SE(3) pose + velocity
 * @author Jose Luis Blanco Claraco
 * @date   Jan 22, 2024
 */

#include <mola_kernel/interfaces/NavStateFilter.h>
#include <mola_kernel/interfaces/RawDataSourceBase.h>

#include <Eigen/Dense>
#include <sstream>

using namespace mola;

NavStateFilter::NavStateFilter()
{
  this->mrpt::system::COutputLogger::setLoggerName("NavStateFilter");
}

std::string NavState::asString() const
{
  std::ostringstream ss;
  ss << "pose  : " << pose;
  ss << "twist : " << twist.asString() << "\n";
  ss << "twist inv_cov diagonal: " << twist_inv_cov.asEigen().diagonal().transpose() << "\n";

  return ss.str();
}

void NavStateFilter::initialize(const Yaml& cfg)
{
  MRPT_TRY_START

  if (cfg.has("raw_data_source"))
  {
    auto ds_source = cfg["raw_data_source"];

    if (cfg.isSequence())
    {
      for (const auto& v : cfg.asSequence())
      {
        navstate_source_names_.insert(v.as<std::string>());
      }
    }
    else
    {
      navstate_source_names_.insert(ds_source.as<std::string>());
    }

    for (const auto& src_name : navstate_source_names_)
    {
      MRPT_LOG_DEBUG_STREAM(
          "NavStateFilter::initialize(): subscribing to '" << src_name
                                                           << "' for raw sensor input.");

      ASSERT_(this->nameServer_);

      auto data_src = nameServer_(src_name);
      if (!data_src)
      {
        THROW_EXCEPTION_FMT("Cannot find data source module named `%s`", src_name.c_str());
      }

      auto rdsb = std::dynamic_pointer_cast<RawDataSourceBase>(data_src);
      if (!rdsb)
      {
        THROW_EXCEPTION_FMT(
            "Could not cast data source module named `%s` to RawDataSourceBase", src_name.c_str());
      }

      // Subscribe:
      rdsb->attachToDataConsumer(*this);
    }
  }
  else
  {
    // This may be OK if we are running outside of the context of mola-cli
    MRPT_LOG_DEBUG_STREAM(
        "No 'raw_data_source' entry found in the YAML definition for a "
        "NavStateFilter: YAML contents:\n"
        << cfg << "\n");
  }

  MRPT_TRY_END
}
