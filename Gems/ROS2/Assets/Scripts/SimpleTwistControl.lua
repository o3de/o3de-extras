--[[
    This is an example of control implementation which sets the desired velocity on a single body.
    To imitate the steering, current linear and angular velocities of a single rigidbody are forcefully overwritten
    with the desired control.
    TODO: Control the robot with forces applied to the wheels instead of directly setting up body velocity.
--]]

local SimpleTwistControl = {
    Properties = {}
}

function SimpleTwistControl:OnActivate()
    self.twistNotificationBus = TwistNotificationBus.Connect(self);
end

function SimpleTwistControl:TwistReceived(linear, angular)
    -- Get current linear velocity
    local currentLinearVelocity = RigidBodyRequestBus.Event.GetLinearVelocity(self.entityId)

    -- Convert local steering to world frame
    local worldTM = TransformBus.Event.GetWorldTM(self.entityId)
    local linearVelocityTransformed = Transform.TransformVector(worldTM, linear)

    -- Overwrite control velocities on two axis
    currentLinearVelocity["x"] = linearVelocityTransformed["x"]
    currentLinearVelocity["y"] = linearVelocityTransformed["y"]

    -- Reapply altered velocities for the rigidbody
    RigidBodyRequestBus.Event.SetAngularVelocity(self.entityId, angular)
    RigidBodyRequestBus.Event.SetLinearVelocity(self.entityId, currentLinearVelocity)
end

function SimpleTwistControl:OnDeactivate()
    self.twistNotificationBus:Disconnect()
end

return SimpleTwistControl
