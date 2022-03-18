#include "ROS2LidarSensorComponent.h"

using namespace ROS2;

void ROS2LidarSensorComponent::Init()
{
    //ROS2Requests* requests = ROS2Interface::Get();
    auto ros2_node = ROS2Interface::Get()->GetNode();
    //EBUS_EVENT_RESULT(ros2_node, ROS2RequestBus, GetNode); // ...in case of no response
    m_pointCloudPublisher = ros2_node->create_publisher<sensor_msgs::msg::PointCloud2>("point_cloud", 10);
}

void ROS2LidarSensorComponent::Activate()
{
    // TODO - add range validation (Attributes?)
    m_frameTime = m_hz == 0 ? 1 : 1 / m_hz;
    AZ::TickBus::Handler::BusConnect();
}

void ROS2LidarSensorComponent::Deactivate()
{
    AZ::TickBus::Handler::BusDisconnect();
}

void ROS2LidarSensorComponent::Reflect(AZ::ReflectContext* context)
{
    if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
    {
        serialize->Class<ROS2LidarSensorComponent, AZ::Component>()
            ->Version(1)
            ->Field("hz", &ROS2LidarSensorComponent::m_hz)
            ->Field("frameName", &ROS2LidarSensorComponent::m_frameName) //TODO - serializer?
            ;

        if (AZ::EditContext* ec = serialize->GetEditContext())
        {
            ec->Class<ROS2LidarSensorComponent>("Lidar Sensor", "[Simple Lidar component]")
                ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC("Game"))
                ->DataElement(AZ::Edit::UIHandlers::Default, &ROS2LidarSensorComponent::m_hz, "Hz", "Lidar data acquisition and publish frequency")
                ->DataElement(AZ::Edit::UIHandlers::Default, &ROS2LidarSensorComponent::m_frameName, "Frame Name", "Lidar ros2 message frame")
                ;
        }
    }
}

/*
void ROS2LidarSensorComponent::GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required)
{
    required.push_back(AZ_CRC("ROS2Sevice"));
}
*/

void ROS2LidarSensorComponent::OnTick([[maybe_unused]] float deltaTime, [[maybe_unused]] AZ::ScriptTimePoint time)
{
    static float elapsed = 0;
    elapsed += deltaTime;
    if (elapsed < m_frameTime)
        return;

    elapsed -= m_frameTime;
    if (deltaTime > m_frameTime)
    {   // Frequency higher than possible, not catching up, just keep going with each frame.
        elapsed = 0;
    }

    auto message = sensor_msgs::msg::PointCloud2();
    message.header.frame_id = m_frameName.data();
    message.height = 0;
    message.width = 0;
    message.point_step = 0;
    message.row_step = 0;

    m_pointCloudPublisher->publish(message);
}

