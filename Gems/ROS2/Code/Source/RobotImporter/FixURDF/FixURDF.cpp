/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "FixURDF.h"
#include <AzCore/XML/rapidxml_print.h>
#include <AzCore/std/containers/set.h>
#include <iostream>
namespace ROS2::Utils
{
    AZStd::vector<AZStd::string> AddMissingInertiaToLink(AZ::rapidxml::xml_node<>* urdf)
    {
        AZStd::vector<AZStd::string> modifiedLinks;
        using namespace AZ::rapidxml;
        AZStd::vector<xml_node<>*> links;
        for (xml_node<>* link = urdf->first_node("link"); link; link = link->next_sibling("link"))
        {
            if (!link->first_node("inertial"))
            {
                links.push_back(link);
            }
        }

        for (auto* link : links)
        {
            xml_node<>* inertial = urdf->document()->allocate_node(node_element, "inertial");

            xml_node<>* mass = urdf->document()->allocate_node(node_element, "mass");
            mass->append_attribute(urdf->document()->allocate_attribute("value", "1."));
            inertial->append_node(mass);
            xml_node<>* inertia = urdf->document()->allocate_node(node_element, "inertia");

            inertia->append_attribute(urdf->document()->allocate_attribute("ixx", "1."));
            inertia->append_attribute(urdf->document()->allocate_attribute("ixy", "0."));
            inertia->append_attribute(urdf->document()->allocate_attribute("ixz", "0."));

            inertia->append_attribute(urdf->document()->allocate_attribute("iyx", "0."));
            inertia->append_attribute(urdf->document()->allocate_attribute("iyy", "1."));
            inertia->append_attribute(urdf->document()->allocate_attribute("iyz", "0."));

            inertia->append_attribute(urdf->document()->allocate_attribute("izx", "0."));
            inertia->append_attribute(urdf->document()->allocate_attribute("izy", "0."));
            inertia->append_attribute(urdf->document()->allocate_attribute("izz", "1."));

            inertial->append_node(inertia);

            xml_node<>* origin = urdf->document()->allocate_node(node_element, "origin");
            origin->append_attribute(urdf->document()->allocate_attribute("xyz", "0. 0. 0."));
            inertial->append_node(origin);

            auto* name = link->first_attribute("name");
            if (name)
            {
                modifiedLinks.push_back(name->value());
            }
            link->append_node(inertial);
        }

        return modifiedLinks;
    }

    AZStd::vector<AZStd::string> ChangeDuplications(AZ::rapidxml::xml_node<>* urdf)
    {
        using namespace AZ::rapidxml;
        AZStd::vector<AZStd::string> modifiedLinks;
        AZStd::set<AZStd::string> linkAndJointsName;
        for (xml_node<>* link = urdf->first_node("link"); link; link = link->next_sibling("link"))
        {
            auto* name = link->first_attribute("name");
            if (name)
            {
                linkAndJointsName.insert(name->value());
            }
        }
        for (xml_node<>* joint = urdf->first_node("joint"); joint; joint = joint->next_sibling("joint"))
        {
            auto* name = joint->first_attribute("name");
            if (name)
            {
                if (linkAndJointsName.contains(name->value()))
                {
                    auto newName = AZStd::string(name->value()) + "_joint_dup";
                    name->value(urdf->document()->allocate_string(newName.c_str()));
                    linkAndJointsName.insert(newName);
                    modifiedLinks.push_back(newName);
                }
                else
                {
                    linkAndJointsName.insert(name->value());
                }
            }
        }
        return modifiedLinks;
    }

    AZStd::pair<std::string, AZStd::vector<AZStd::string>> ModifyURDFInMemory(const std::string& data)
    {
        AZStd::vector<AZStd::string> modifiedElements;
        using namespace AZ::rapidxml;
        xml_document<> doc;
        doc.parse<0>(const_cast<char*>(data.c_str()));
        xml_node<>* urdf = doc.first_node("robot");
        auto links = AddMissingInertiaToLink(urdf);

        if (links.size())
        {
            AZ_Warning("ROS2", false, "Added missing inertia to links: ");
            for (auto& link : links)
            {
                modifiedElements.push_back(link);
                AZ_Warning("ROS2", false, "  -> %s", link.c_str());
            }
        }

        auto renames = ChangeDuplications(urdf);
        if (renames.size())
        {
            AZ_Warning("ROS2", false, "Renamed duplicated links and joints: ");
            for (auto& dup : renames)
            {
                modifiedElements.push_back(dup);
                AZ_Warning("ROS2", false, "  -> %s", dup.c_str());
            }
        }

        std::string xmlDocString;
        AZ::rapidxml::print(std::back_inserter(xmlDocString), *urdf, 0);

        return { xmlDocString, modifiedElements };
    }
} // namespace ROS2::Utils
