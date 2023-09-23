/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "FixURDF.h"
#include <AzCore/XML/rapidxml.h>
#include <AzCore/XML/rapidxml_print.h>
#include <AzCore/std/containers/unordered_map.h>
#include <iostream>

namespace ROS2::Utils
{
    //! Modifies a parsed URDF in memory to add missing inertia to links, which prevents SDF error 19.
    //! @param urdf URDF to modify.
    //! @returns a list of names of links that were modified.
    AZStd::vector<AZStd::string> AddMissingInertiaToLinks(AZ::rapidxml::xml_node<>* urdf)
    {
        AZStd::vector<AZStd::string> modifiedLinks;
        using namespace AZ::rapidxml;

        for (xml_node<>* link = urdf->first_node("link"); link; link = link->next_sibling("link"))
        {
            bool modified = false;
            bool inertiaPresent = true;

            auto name_xml = link->first_attribute("name");
            AZStd::string name = "unknown_link";
            if (name_xml)
            {
                name = name_xml->value();
            }

            auto inertial = link->first_node("inertial");
            if (!inertial)
            {
                AZ_Warning("URDF", false, "Missing inertial tag in link %s, applying default tag.", name.c_str());
                inertial = urdf->document()->allocate_node(node_element, "inertial");
                link->append_node(inertial);
                modified = true;
                inertiaPresent = false;
            }

            auto mass = inertial->first_node("mass");
            if (!mass)
            {
                AZ_Warning("URDF", !inertiaPresent, "Missing mass tag inside the inertial tag link %s, applying default mass tag.", name.c_str());
                mass = urdf->document()->allocate_node(node_element, "mass");
                mass->append_attribute(urdf->document()->allocate_attribute("value", "1."));
                inertial->append_node(mass);
                modified = true;
            }

            auto inertia = inertial->first_node("inertia");
            if (!inertia)
            {
                AZ_Warning("URDF", !inertiaPresent, "Missing inertia tag inside the inertial tag link %s, applying default inertia tag.", name.c_str());
                inertia = urdf->document()->allocate_node(node_element, "inertia");

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
                modified = true;
            }

            if (modified)
            {
                modifiedLinks.push_back(name);
            }
        }

        return modifiedLinks;
    }

    //! Handles a case of multiple joints and the link sharing a common names which causes SDF error2 (but is fine in URDF)
    //! Function will add a suffix "_dup" to the name of the joint if it is also the name of a link.
    //! If there are name collisions in links, this will not be able to fix it, the SDF parser will throw an error.
    //! @param urdf URDF to modify.
    //! @returns a list of links that were modified
    AZStd::vector<AZStd::string> RenameDuplicatedJoints(AZ::rapidxml::xml_node<>* urdf)
    {
        using namespace AZ::rapidxml;
        AZStd::vector<AZStd::string> modifiedLinks;
        AZStd::unordered_map<AZStd::string, unsigned int> linkAndJointsName;
        for (xml_node<>* link = urdf->first_node("link"); link; link = link->next_sibling("link"))
        {
            auto* name = link->first_attribute("name");
            if (name)
            {
                linkAndJointsName.insert(AZStd::make_pair(name->value(), 0));
            }
        }
        for (xml_node<>* joint = urdf->first_node("joint"); joint; joint = joint->next_sibling("joint"))
        {
            auto* name = joint->first_attribute("name");
            if (name)
            {
                if (linkAndJointsName.contains(name->value()))
                {
                    unsigned int& count = linkAndJointsName[name->value()];
                    auto newName = AZStd::string::format("%s_dup%u", name->value(), count);
                    name->value(urdf->document()->allocate_string(newName.c_str()));
                    count++;
                    modifiedLinks.push_back(AZStd::move(newName));
                }
                else
                {
                    linkAndJointsName.insert(AZStd::make_pair(name->value(), 0));
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
        if (!urdf)
        {
            AZ_Warning("URDF", false, "No robot tag in URDF");
            return {data, modifiedElements};
        }
        auto links = AddMissingInertiaToLinks(urdf);
        modifiedElements.insert(modifiedElements.end(), AZStd::make_move_iterator(links.begin()), AZStd::make_move_iterator(links.end()));

        auto renames = RenameDuplicatedJoints(urdf);
        modifiedElements.insert(
            modifiedElements.end(), AZStd::make_move_iterator(renames.begin()), AZStd::make_move_iterator(renames.end()));

        std::string xmlDocString;
        AZ::rapidxml::print(std::back_inserter(xmlDocString), *urdf, 0);
        return { xmlDocString, modifiedElements };
    }
} // namespace ROS2::Utils
