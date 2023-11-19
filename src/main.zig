const std = @import("std");

const c = @cImport({
    @cInclude("box2d/debug_draw.h");
    @cInclude("box2d/box2d.h");
});

const Timer = struct {
    id: c.b2Timer,

    pub fn create() Timer {
        return .{ .id = c.b2CreateTimer() };
    }

    pub fn getTicks(self: *Timer) i64 {
        return c.b2GetTicks(self.id);
    }

    pub fn getMilliseconds(self: *Timer) f32 {
        return c.b2GetMilliseconds(self.id);
    }

    pub fn getMillisecondsAndReset(self: *Timer) f32 {
        return c.b2GetMillisecondsAndReset(self.id);
    }
};

pub fn sleepMilliseconds(ms: f32) void {
    c.b2SleepMilliseconds(ms);
}

const World = struct {
    id: c.b2WorldId,

    pub fn create(def: *c.b2WorldDef) World {
        const id = c.b2CreateWorld(def);
        return .{
            .id = id,
        };
    }

    pub fn defaultDef() c.b2WorldDef {
        return c.b2DefaultWorldDef();
    }

    pub fn destroy(self: *World) void {
        c.b2DestroyWorld(self.id);
    }

    pub fn step(self: World, timeStep: f32, velocityIterations: i32, relaxIterations: i32) void {
        c.b2World_Step(self.id, timeStep, velocityIterations, relaxIterations);
    }

    pub fn draw(self: World, debugDraw: anytype) void {
        c.b2World_Draw(self.id, debugDraw);
    }

    pub fn createBody(self: World, def: *c.b2BodyDef) Body {
        return .{
            .id = c.b2World_CreateBody(self.id, def),
        };
    }

    pub fn createDistanceJoint(self: World, def: c.b2DistanceJointDef) DistanceJoint {
        return .{
            .joint = .{
                .id = c.b2World_CreateDistanceJoint(self.id, &def),
            },
        };
    }

    pub fn createMouseJoint(self: World, def: c.b2MouseJointDef) MouseJoint {
        return .{
            .joint = .{
                .id = c.b2World_CreateMouseJoint(self.id, &def),
            },
        };
    }

    pub fn createPrismaticJoint(self: World, def: c.b2PrismaticJointDef) Joint {
        return .{
            .id = c.b2World_CreatePrismaticJoint(self.id, &def),
        };
    }

    pub fn createRevoluteJoint(self: World, def: c.b2RevoluteJointDef) RevoluteJoint {
        return .{
            .joint = .{
                .id = c.b2World_CreateRevoluteJoint(self.id, &def),
            },
        };
    }

    pub fn createWeldJoint(self: World, def: c.b2WeldJointDef) Joint {
        return .{
            .id = c.b2World_CreateWeldJoint(self.id, &def),
        };
    }

    pub fn queryAABB(self: World, aabb: c.b2AABB, fcn: c.b2QueryCallbackFcn, context: ?*anyopaque) void {
        c.b2World_QueryAABB(self.id, aabb, fcn, context);
    }

    pub fn enableSleeping(self: World, flag: bool) void {
        c.b2World_EnableSleeping(self.id, flag);
    }

    pub fn enableWarmStarting(self: World, flag: bool) void {
        c.b2World_EnableWarmStarting(self.id, flag);
    }

    pub fn enableContinuous(self: World, flag: bool) void {
        c.b2World_EnableContinuous(self.id, flag);
    }

    pub fn setResitutionThreshold(self: World, value: f32) void {
        c.b2World_SetRestitutionThreshold(self.id, value);
    }

    pub fn setContactTuning(self: World, hertz: f32, dampingRatio: f32, pushVelocity: f32) void {
        c.b2World_SetContactTuning(self.id, hertz, dampingRatio, pushVelocity);
    }

    pub fn getProfile(self: World) c.struct_b2Profile {
        return c.b2World_GetProfile(self.id);
    }

    pub fn getStatistics(self: World) c.struct_b2Statistics {
        return c.b2World_GetStatistics(self.id);
    }
};

pub const Joint = struct {
    id: c.b2JointId,

    pub fn destroy(self: *Joint) void {
        c.b2World_DestroyJoint(self.id);
    }

    pub fn getBodyA(self: Joint) Body {
        return .{
            .id = c.b2Joint_GetBodyA(self.id),
        };
    }

    pub fn getBodyB(self: Joint) Body {
        return .{
            .id = c.b2Joint_GetBodyB(self.id),
        };
    }
};

pub const DistanceJoint = struct {
    joint: Joint,

    pub fn destroy(self: *DistanceJoint) void {
        self.joint.destroy();
    }

    pub fn getBodyA(self: DistanceJoint) Body {
        return self.joint.getBodyA();
    }

    pub fn getBodyB(self: DistanceJoint) Body {
        return self.joint.getBodyB();
    }

    pub fn getConstraintForce(self: DistanceJoint, timeStep: f32) f32 {
        return c.b2DistanceJoint_GetConstraintForce(self.joint.id, timeStep);
    }

    pub fn setLength(self: DistanceJoint, length: f32, minLength: f32, maxLength: f32) void {
        c.b2DistanceJoint_SetLength(self.joint.id, length, minLength, maxLength);
    }

    pub fn getCurrentLength(self: DistanceJoint) f32 {
        return c.b2DistanceJoint_GetCurrentLength(self.joint.id);
    }

    pub fn setTuning(self: DistanceJoint, hertz: f32, dampingRatio: f32) void {
        c.b2DistanceJoint_SetTuning(self.joint.id, hertz, dampingRatio);
    }
};

pub const MouseJoint = struct {
    joint: Joint,

    pub fn destroy(self: *DistanceJoint) void {
        self.joint.destroy();
    }

    pub fn getBodyA(self: DistanceJoint) Body {
        return self.joint.getBodyA();
    }

    pub fn getBodyB(self: DistanceJoint) Body {
        return self.joint.getBodyB();
    }

    pub fn setTarget(self: DistanceJoint, target: c.b2Vec2) void {
        c.b2MouseJoint_SetTarget(self.joint.id, target);
    }
};

pub const RevoluteJoint = struct {
    joint: Joint,

    pub fn destroy(self: *RevoluteJoint) void {
        self.joint.destroy();
    }

    pub fn getBodyA(self: RevoluteJoint) Body {
        return self.joint.getBodyA();
    }

    pub fn getBodyB(self: RevoluteJoint) Body {
        return self.joint.getBodyB();
    }

    pub fn enableLimit(self: RevoluteJoint, enable: bool) void {
        c.b2RevoluteJoint_EnableLimit(self.joint.id, enable);
    }

    pub fn enableMotor(self: RevoluteJoint, enable: bool) void {
        c.b2RevoluteJoint_EnableMotor(self.joint.id, enable);
    }

    pub fn setMotorSpeed(self: RevoluteJoint, speed: f32) void {
        c.b2RevoluteJoint_SetMotorSpeed(self.joint.id, speed);
    }

    pub fn getMotorTorque(self: RevoluteJoint, inverseTimeStep: f32) f32 {
        return c.b2RevoluteJoint_GetMotorTorque(self.joint.id, inverseTimeStep);
    }

    pub fn setMaxMotorTorque(self: RevoluteJoint, torque: f32) void {
        c.b2RevoluteJoint_SetMaxMotorTorque(self.joint.id, torque);
    }

    pub fn getConstraintForce(self: RevoluteJoint) c.b2Vec2 {
        return c.b2RevoluteJoint_GetConstraintForce(self.joint.id);
    }
};

pub const Body = struct {
    id: c.b2BodyId,

    pub fn destroy(self: *Body) void {
        c.b2World_DestroyBody(self.id);
    }

    pub fn defaultDef() c.b2BodyDef {
        return c.b2DefaultBodyDef();
    }

    pub fn getPosition(self: Body) c.b2Vec2 {
        return c.b2Body_GetPosition(self.id);
    }

    pub fn getAngle(self: Body) f32 {
        return c.b2Body_GetAngle(self.id);
    }

    pub fn setTransform(self: Body, position: c.b2Vec2, angle: f32) void {
        c.b2Body_SetTransform(self.id, position, angle);
    }

    pub fn getLocalPoint(self: Body, globalPoint: c.b2Vec2) c.b2Vec2 {
        return c.b2Body_GetLocalPoint(self.id, globalPoint);
    }

    pub fn getWorldPoint(self: Body, globalPoint: c.b2Vec2) c.b2Vec2 {
        return c.b2Body_GetWorldPoint(self.id, globalPoint);
    }

    pub fn getLinearVelocity(self: Body) c.b2Vec2 {
        return c.b2Body_GetLinearVelocity(self.id);
    }

    pub fn getAngularVelocity(self: Body) f32 {
        return c.b2Body_GetAngularVelocity(self.id);
    }

    pub fn setLinearVelocity(self: Body, linearVelocity: c.b2Vec2) void {
        c.b2Body_SetLinearVelocity(self.id, linearVelocity);
    }

    pub fn setAngularVelocity(self: Body, angularVelocity: f32) void {
        c.b2Body_SetAngularVelocity(self.id, angularVelocity);
    }

    pub fn getType(self: Body) Type {
        return @enumFromInt(c.b2Body_GetType(self.id));
    }

    pub fn setType(self: Body, type_: Type) void {
        c.b2Body_SetType(self.id, @intFromEnum(type_));
    }

    pub fn getMass(self: Body) f32 {
        return c.b2Body_GetMass(self.id);
    }

    pub fn getInertiaTensor(self: Body) f32 {
        return c.b2Body_GetInertiaTensor(self.id);
    }

    pub fn getLocalCenterOfMass(self: Body) c.b2Vec2 {
        return c.b2Body_GetLocalCenterOfMass(self.id);
    }

    pub fn getWorldCenterOfMass(self: Body) c.b2Vec2 {
        return c.b2Body_GetWorldCenterOfMass(self.id);
    }

    // pub fn setMassData(massData: c.b2MassData) void {
    //     c.b2Body_SetMassData(massData);
    // }

    // pub fn isAwake(self: Body) void {
    //     //FIXME: this should return bool?
    //     c.b2Body_IsAwake(self.id);
    // }

    pub fn wake(self: Body) void {
        c.b2Body_Wake(self.id);
    }

    pub fn isEnabled(self: Body) bool {
        return c.b2Body_IsEnabled(self.id);
    }

    pub fn disable(self: Body) void {
        c.b2Body_Disable(self.id);
    }

    pub fn enable(self: Body) void {
        c.b2Body_Enable(self.id);
    }

    pub fn createCircle(self: Body, def: c.b2ShapeDef, circle: c.b2Circle) Shape {
        return .{
            .id = c.b2Body_CreateCircle(self.id, &def, &circle),
        };
    }

    pub fn createSegment(self: Body, def: c.b2ShapeDef, segment: c.b2Segment) Shape {
        return .{
            .id = c.b2Body_CreateSegment(self.id, &def, &segment),
        };
    }

    pub fn createCapsule(self: Body, def: c.b2ShapeDef, capsule: c.b2Capsule) Shape {
        return .{
            .id = c.b2Body_CreateCapsule(self.id, &def, &capsule),
        };
    }

    pub fn createPolygon(self: Body, def: *c.b2ShapeDef, polygon: Polygon) Shape {
        return .{
            .id = c.b2Body_CreatePolygon(self.id, def, &polygon.polygon),
        };
    }
};

pub const Shape = struct {
    id: c.b2ShapeId,

    pub fn defaultDef() c.b2ShapeDef {
        return c.b2DefaultShapeDef();
    }

    pub fn getBody(self: Shape) Body {
        return .{
            .id = c.b2Shape_GetBody(self.id),
        };
    }

    pub fn testPoint(self: Shape, point: c.b2Vec2) bool {
        return c.b2Shape_TestPoint(self.id, point);
    }

    pub fn setFriction(self: Shape, friction: f32) void {
        c.b2Shape_SetFriction(self.id, friction);
    }
};

pub const Polygon = struct {
    polygon: c.b2Polygon,

    pub fn makeBox(hx: f32, hy: f32) Polygon {
        return .{
            .polygon = c.b2MakeBox(hx, hy),
        };
    }
};

pub const Type = enum(c_uint) {
    _,
};

test World {
    var worldDef = World.defaultDef();
    worldDef.gravity = .{ .x = 0.0, .y = -10.0 };

    var world = World.create(&worldDef);

    var groundBodyDef = Body.defaultDef();
    groundBodyDef.position = .{ .x = 0.0, .y = -10.0 };

    var groundBody = world.createBody(&groundBodyDef);

    const groundBox = Polygon.makeBox(50.0, 10.0);
    var groundShapeDef = Shape.defaultDef();

    _ = groundBody.createPolygon(&groundShapeDef, groundBox);

    var bodyDef = Body.defaultDef();
    bodyDef.type = c.b2_dynamicBody;
    bodyDef.position = .{ .x = 0.0, .y = 4.0 };

    var body = world.createBody(&bodyDef);

    var dynamicBox = Polygon.makeBox(1.0, 1.0);

    var shapeDef = Shape.defaultDef();

    shapeDef.density = 1.0;
    shapeDef.friction = 0.3;

    _ = body.createPolygon(&shapeDef, dynamicBox);

    for (0..60) |_| {
        const timeStep = 1.0 / 60.0;
        const velocityIterations = 6;
        const relaxIterations = 2;

        world.step(timeStep, velocityIterations, relaxIterations);
    }

    const position = body.getPosition();
    const angle = body.getAngle();

    try std.testing.expect(@abs(position.x) < 0.01);
    try std.testing.expect(@abs(position.y - 1.0) < 0.01);
    try std.testing.expect(@abs(angle) < 0.01);

    world.destroy();
}

test {
    std.testing.refAllDeclsRecursive(@This());
}
