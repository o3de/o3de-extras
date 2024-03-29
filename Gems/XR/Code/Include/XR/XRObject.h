/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <AzCore/std/smart_ptr/intrusive_base.h>
#include <AzCore/Name/Name.h>

namespace XR
{
    //! All objects have an explicit Init / Shutdown path in addition to
    //! creation / deletion. The user is expected to Init the derived variant
    //! in order to use it. This extension allows the user to forgo shutdown
    //! explicitly and depend on the Ptr ref-counting if necessary.
    //! This requires that Shutdown properly account for being called multiple times.
    struct ObjectDeleter
        : public AZStd::intrusive_default_delete
    {
        template <typename U>
        void operator () (U* p) const;
    };

    //! Base class for any persistent resource. Provides a name, reference
    //! counting, and a common RTTI base class for all objects .
    class Object
        : public AZStd::intrusive_refcount<AZStd::atomic_uint, ObjectDeleter>
    {
    public:
        AZ_RTTI(Object, "{74FCA8BF-CBDA-43EB-A378-89F752ED2FCA}");
        ~Object() override = default;

        //! Sets the name of the object.
        void SetName(const AZ::Name& name)
        {
            m_name = name;
            SetNameInternal(m_name.GetStringView());
        }

        //! Returns the name set on the object by SetName
        const AZ::Name& GetName() const { return m_name; }

    protected:
        Object() = default;

    private:
        // Friended to allow private call to Shutdown.
        friend struct ObjectDeleter;

        //! Shuts down the object. Derived classes can make this public if it fits
        //! with their lifecycle model (i.e. if they use an explicit Init / Shutdown).
        //! By default, it is private in order to maintain consistency with a simpler RAII lifecycle.
        virtual void Shutdown() {};

        virtual void SetNameInternal([[maybe_unused]] const AZStd::string_view& name) {}

        //! Object name. Very helpful for debugging
        AZ::Name m_name;
    };

    template <typename U>
    void ObjectDeleter::operator () (U* p) const
    {
        //! Recover the mutable parent object pointer from the refcount base class.
        Object* object = const_cast<Object*>(static_cast<const Object*>(p));

        //! Shuts down the object before deletion.
        object->Shutdown();

        //! Then invoke the base delete policy.
        AZStd::intrusive_default_delete::operator () (object);
    }
}

