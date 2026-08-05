/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <RobotImporter/Sdf/SdfVisitors.h>

namespace ROS2RobotImporter::Utils
{
    void VisitLinks(const sdf::Model& sdfModel, const LinkVisitorCallback& linkVisitorCB, bool visitNestedModelLinks)
    {
        // Function object which can visit all links of a model
        // Optionally it supports recursing nested models to visit their links as well
        struct VisitLinksForNestedModels_fn
        {
            VisitModelResponse operator()(const sdf::Model& model)
            {
                m_modelStack.push_back(model);
                VisitModelResponse visitResponse = VisitLinksForModel(model);
                if (m_recurseModels && visitResponse == VisitModelResponse::VisitNestedAndSiblings)
                {
                    // Nested model link are only visited if the joint visitor returns true
                    for (uint64_t modelIndex{}; modelIndex < model.ModelCount(); ++modelIndex)
                    {
                        if (const sdf::Model* nestedModel = model.ModelByIndex(modelIndex); nestedModel != nullptr)
                        {
                            if (VisitModelResponse nestedVisitResponse = operator()(*nestedModel);
                                nestedVisitResponse >= VisitModelResponse::Stop)
                            {
                                // Sibling nested model links are only visited
                                // if the joint visitor returns true
                                break;
                            }
                        }
                    }
                }
                m_modelStack.pop_back();

                return visitResponse;
            }

        private:
            //! Returns success by default
            //! But an invoked visitor can return false to halt further iteration
            VisitModelResponse VisitLinksForModel(const sdf::Model& currentModel)
            {
                for (uint64_t linkIndex{}; linkIndex < currentModel.LinkCount(); ++linkIndex)
                {
                    if (const sdf::Link* link = currentModel.LinkByIndex(linkIndex); link != nullptr)
                    {
                        if (!m_linkVisitorCB(*link, m_modelStack))
                        {
                            return VisitModelResponse::Stop;
                        }
                    }
                }

                return VisitModelResponse::VisitNestedAndSiblings;
            }

        public:
            LinkVisitorCallback m_linkVisitorCB;
            bool m_recurseModels{};

        private:
            // Stack storing the current composition of models visited so far
            ModelStack m_modelStack;
        };

        VisitLinksForNestedModels_fn VisitLinksForNestedModels{};
        VisitLinksForNestedModels.m_linkVisitorCB = linkVisitorCB;
        VisitLinksForNestedModels.m_recurseModels = visitNestedModelLinks;
        VisitLinksForNestedModels(sdfModel);
    }

    void VisitJoints(const sdf::Model& sdfModel, const JointVisitorCallback& jointVisitorCB, bool visitNestedModelJoints)
    {
        // Function object which can visit all joints of a model
        // Optionally it supports recursing nested models to visit their joints as well
        struct VisitJointsForNestedModels_fn
        {
            VisitModelResponse operator()(const sdf::Model& model)
            {
                m_modelStack.push_back(model);
                VisitModelResponse visitResponse = VisitJointsForModel(model);
                if (m_recurseModels && visitResponse == VisitModelResponse::VisitNestedAndSiblings)
                {
                    // Nested model joints are only visited if the joint visitor returns true
                    for (uint64_t modelIndex{}; modelIndex < model.ModelCount(); ++modelIndex)
                    {
                        if (const sdf::Model* nestedModel = model.ModelByIndex(modelIndex); nestedModel != nullptr)
                        {
                            if (VisitModelResponse nestedVisitResponse = operator()(*nestedModel);
                                nestedVisitResponse >= VisitModelResponse::Stop)
                            {
                                // Sibling nested model joints are only visited
                                // if the joint visitor returns true
                                break;
                            }
                        }
                    }
                }
                m_modelStack.pop_back();

                return visitResponse;
            }

        private:
            VisitModelResponse VisitJointsForModel(const sdf::Model& currentModel)
            {
                for (uint64_t jointIndex{}; jointIndex < currentModel.JointCount(); ++jointIndex)
                {
                    const sdf::Joint* joint = currentModel.JointByIndex(jointIndex);
                    // Skip any joints whose parent and child link references
                    // don't have an actual sdf::link in the parsed model
                    if (joint != nullptr && currentModel.LinkNameExists(joint->ParentName()) && currentModel.LinkByName(joint->ChildName()))
                    {
                        if (!m_jointVisitorCB(*joint, m_modelStack))
                        {
                            return VisitModelResponse::Stop;
                        }
                    }
                }

                return VisitModelResponse::VisitNestedAndSiblings;
            }

        public:
            JointVisitorCallback m_jointVisitorCB;
            bool m_recurseModels{};

        private:
            // Stack storing the current composition of models visited so far
            ModelStack m_modelStack;
        };

        VisitJointsForNestedModels_fn VisitJointsForNestedModels{};
        VisitJointsForNestedModels.m_jointVisitorCB = jointVisitorCB;
        VisitJointsForNestedModels.m_recurseModels = visitNestedModelJoints;
        VisitJointsForNestedModels(sdfModel);
    }
    struct VisitModelsForNestedModels_fn
    {
        VisitModelResponse operator()(const sdf::Model& model)
        {
            // The VisitModelResponse enum value is used to filter out
            // less callbacks the higher the value grows.
            // So any values above VisitNestedAndSiblings will not visit nested models
            VisitModelResponse visitResponse = m_modelVisitorCB(model, m_modelStack);

            if (m_recurseModels && visitResponse == VisitModelResponse::VisitNestedAndSiblings)
            {
                // Nested models are only visited if the model visitor returns VisitNestedAndSiblings
                m_modelStack.push_back(model);
                for (uint64_t modelIndex{}; modelIndex < model.ModelCount(); ++modelIndex)
                {
                    if (const sdf::Model* nestedModel = model.ModelByIndex(modelIndex); nestedModel != nullptr)
                    {
                        if (VisitModelResponse nestedVisitResponse = operator()(*nestedModel);
                            nestedVisitResponse >= VisitModelResponse::Stop)
                        {
                            // Visiting of the nested model has returned Stop, so halt any sibling model visitation
                            break;
                        }
                    }
                }
                m_modelStack.pop_back();
            }

            return visitResponse;
        }

        VisitModelResponse operator()(const sdf::World& world)
        {
            // Nested model are only visited if the model visitor returns true
            for (uint64_t modelIndex{}; modelIndex < world.ModelCount(); ++modelIndex)
            {
                if (const sdf::Model* model = world.ModelByIndex(modelIndex); model != nullptr)
                {
                    // Delegate to the sdf::Model call operator overload to visit nested models
                    // Stop visiting the world's <model> children if any children return Stop
                    if (VisitModelResponse visitResponse = operator()(*model); visitResponse >= VisitModelResponse::Stop)
                    {
                        return visitResponse;
                    }
                }
            }

            // By default visit any sibling worlds' models if visitation doesn't return Stop
            return VisitModelResponse::VisitNestedAndSiblings;
        }

        void operator()(const sdf::Root& root)
        {
            // Visit all <model> tags at the root of the SDF
            VisitModelResponse modelVisitResponse = VisitModelResponse::VisitNestedAndSiblings;
            if (const sdf::Model* model = root.Model(); model != nullptr)
            {
                modelVisitResponse = operator()(*model);
            }

            // If the root <model> indicated that visitation should stop, then return
            if (modelVisitResponse >= VisitModelResponse::Stop)
            {
                return;
            }
            // Next visit any <world> tags in the SDF
            for (uint64_t worldIndex{}; worldIndex < root.WorldCount(); ++worldIndex)
            {
                if (const sdf::World* world = root.WorldByIndex(worldIndex); world != nullptr)
                {
                    // Delegate to the sdf::World call operator overload to visit any <model> tags in the World
                    if (VisitModelResponse worldVisitResponse = operator()(*world); worldVisitResponse >= VisitModelResponse::Stop)
                    {
                        break;
                    }
                }
            }
        }

    public:
        ModelVisitorCallback m_modelVisitorCB;
        bool m_recurseModels{};

    private:
        // Stack storing the current composition of models visited so far
        ModelStack m_modelStack;
    };

    void VisitModels(const sdf::Root& sdfRoot, const ModelVisitorCallback& modelVisitorCB, bool visitNestedModels)
    {
        VisitModelsForNestedModels_fn VisitModelsForNestedModels{};
        VisitModelsForNestedModels.m_modelVisitorCB = modelVisitorCB;
        VisitModelsForNestedModels.m_recurseModels = visitNestedModels;
        VisitModelsForNestedModels(sdfRoot);
    }

    void VisitModels(const sdf::World& sdfWorld, const ModelVisitorCallback& modelVisitorCB, bool visitNestedModels)
    {
        VisitModelsForNestedModels_fn VisitModelsForNestedModels{};
        VisitModelsForNestedModels.m_modelVisitorCB = modelVisitorCB;
        VisitModelsForNestedModels.m_recurseModels = visitNestedModels;
        VisitModelsForNestedModels(sdfWorld);
    }

    void VisitModels(const sdf::Model& sdfModel, const ModelVisitorCallback& modelVisitorCB, bool visitNestedModels)
    {
        VisitModelsForNestedModels_fn VisitModelsForNestedModels{};
        VisitModelsForNestedModels.m_modelVisitorCB = modelVisitorCB;
        VisitModelsForNestedModels.m_recurseModels = visitNestedModels;
        VisitModelsForNestedModels(sdfModel);
    }
} // namespace ROS2RobotImporter::Utils
