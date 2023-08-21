/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <AzCore/Component/ComponentBus.h>
#include <AzCore/IO/SystemFile.h>
#include <AzCore/Math/Transform.h>
#include <AzCore/std/containers/unordered_map.h>
#include <AzCore/std/containers/unordered_set.h>
#include <AzCore/std/containers/vector.h>
#include <AzCore/std/function/function_template.h>
#include <AzCore/std/string/string.h>
#include <RobotImporter/URDF/UrdfParser.h>

#include <sdf/sdf.hh>

namespace sdf
{
    inline namespace v13
    {
        class Error;
    } // namespace v13
} // namespace sdf


namespace AZStd
{
    // Allow std::vector<sdf::Error> to meet the requirements of contiguous iterator in C++17
    // This allows constructing an AZStd::span from a std::vector
    template<>
    struct iterator_traits<typename std::vector<sdf::v13::Error>::iterator>
    {
        // Use the standard library iterator traits for all traits except for the iterator_concept
        using difference_type = typename std::iterator_traits<typename std::vector<sdf::v13::Error>::iterator>::difference_type;
        using value_type = typename std::iterator_traits<typename std::vector<sdf::v13::Error>::iterator>::value_type;
        using pointer = typename std::iterator_traits<typename std::vector<sdf::v13::Error>::iterator>::pointer;
        using reference = typename std::iterator_traits<typename std::vector<sdf::v13::Error>::iterator>::reference;
        using iterator_category = typename std::iterator_traits<typename std::vector<sdf::v13::Error>::iterator>::iterator_category;
        using iterator_concept = contiguous_iterator_tag;
    };

    template<>
    struct iterator_traits<typename std::vector<sdf::v13::Error>::const_iterator>
    {
        // Use the standard library iterator traits for all traits except for the iterator_concept
        using difference_type = typename std::iterator_traits<typename std::vector<sdf::v13::Error>::const_iterator>::difference_type;
        using value_type = typename std::iterator_traits<typename std::vector<sdf::v13::Error>::const_iterator>::value_type;
        using pointer = typename std::iterator_traits<typename std::vector<sdf::v13::Error>::const_iterator>::pointer;
        using reference = typename std::iterator_traits<typename std::vector<sdf::v13::Error>::const_iterator>::reference;
        using iterator_category = typename std::iterator_traits<typename std::vector<sdf::v13::Error>::const_iterator>::iterator_category;
        using iterator_concept = contiguous_iterator_tag;
    };
}

namespace ROS2::Utils
{
    //! Converts each sdf::Error from an sdf::Errors vector into an AZStd::string
    //! @param sdfErrors A span of sdf::Error objects
    //! @return AZStd::string which contains each error formatted for human readable output
    AZStd::string JoinSdfErrorsToString(AZStd::span<const sdf::Error> sdfErrors);
    //! Converts each sdf::Error from an sdf::Errors vector into an AZStd::string
    //! @param outputResult Appends to output string each sdf error
    //! @param sdfErrors A span of sdf::Error objects
    void AppendSdfErrorsToString(AZStd::string& outputResult, AZStd::span<const sdf::Error> sdfErrors);
} // namespace ROS2::Utils
