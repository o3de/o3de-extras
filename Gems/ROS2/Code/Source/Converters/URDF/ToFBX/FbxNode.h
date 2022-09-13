/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <any>

#include <AzCore/Memory/SystemAllocator.h>
#include <AzCore/std/containers/vector.h>
#include <AzCore/std/string/string.h>

#include "UniqueIdGenerator.h"

namespace ROS2
{
    namespace Fbx
    {
        //! The property of the FBX node.
        using Property = std::any;
        //! The collection of properties of the FBX node.
        using Properties = AZStd::vector<Property>;
        //! The collection of the FBX ndoes.
        using Nodes = AZStd::vector<class Node>;

        //! Represents raw string without quotation marks for Node properties purposes.
        //! String values are quoted by default when added as node properties.
        //! But in very rare cases it's necessary to add field without quotation marks.
        struct RawString
        {
            RawString(const AZStd::string& str)
                : data(str)
            {
            }
            AZStd::string data;
        };

        //! A node in the FBX file tree structure.
        //! Each named node could contain children nodes (subnodes) and multiple properties.
        class Node
        {
        public:
            AZ_CLASS_ALLOCATOR(Node, AZ::SystemAllocator, 0);

            Node(const AZStd::string& name, const Properties& properties = {}, const Nodes& children = {});

            //! Get name of the node.
            //! @return A name of the node.
            AZStd::string GetName() const;
            //! Get children of the node.
            //! @return All direct children nodes of the node.
            Nodes GetChildren() const;
            //! Get properties of the node.
            //! @return All properties of the node.
            Properties GetProperties() const;
            //! Check whether the node has children.
            bool HasChildren() const;
            //! Check whether the node has properties.
            bool HasProperties() const;

            //! Add new property to existing node
            //! @param property A property that will be added to node.
            void AddProperty(const Property& property);

            //! Add new child node to existing node.
            void AddChild(const Node& child);
            //! Add new child node to existing node.
            void AddChild(const AZStd::string& name, const Property& property);
            //! Add new child node to existing node.
            void AddChild(const Node&& child);

            //! Convert the node to string (ASCII Fbx).
            //! @param nodeDepth A node depth in FBX tree structure.
            //! Used to calculate offset in text version of the node.
            //! @return A string that contains text version of the node.
            AZStd::string ToString(int nodeDepth = 0) const;

        private:
            AZStd::string m_name;
            Nodes m_children;
            Properties m_properties;
        };

        //! A node with unique id
        struct NodeWithId
        {
            NodeWithId(Id id, const Node& node)
                : id(id)
                , node(node)
            {
            }

            NodeWithId(Id id, const Node&& node)
                : id(id)
                , node(std::move(node))
            {
            }

            Id id;
            Node node;
        };

    } // namespace Fbx
} // namespace ROS2