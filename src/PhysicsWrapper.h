
#ifndef PHYSICSWRAPPER_H
#define PHYSICSWRAPPER_H

#include <box2d/box2d.h>
#include <memory>
#include <functional>
#include <unordered_map>
#include <vector>

struct ContactInfo;

class PhysicsWrapper : public b2ContactListener {
public:
    using ContactCallback = std::function<void(b2Fixture* fixtureA, b2Fixture* fixtureB)>;
    using PreSolveCallback = std::function<bool(b2Fixture* fixtureA, b2Fixture* fixtureB, const ContactInfo& info)>;
    using PostSolveCallback = std::function<void(b2Fixture* fixtureA, b2Fixture* fixtureB, const b2ContactImpulse* impulse)>;

    PhysicsWrapper(const b2Vec2& gravity = b2Vec2(0.0f, -9.8f));
    ~PhysicsWrapper();

    void Update(float deltaTime);

    b2Body* CreateBody(const b2BodyDef* def);
    void DestroyBody(b2Body* body);

    b2Fixture* CreatePolygonFixture(b2Body* body, const b2PolygonShape* shape, float density = 1.0f);
    b2Fixture* CreateCircleFixture(b2Body* body, const b2CircleShape* shape, float density = 1.0f);
    b2Fixture* CreateBoxFixture(b2Body* body, float halfWidth, float halfHeight, float density = 1.0f);

    void EnableCustomCollisionDetection(bool enable) { m_useCustomDetection = enable; }
    bool IsCustomCollisionDetectionEnabled() const { return m_useCustomDetection; }

    void SetCollisionFilter(b2Fixture* fixture, uint16_t category, uint16_t mask);

    void SetBeginContactCallback(ContactCallback callback) { m_beginContactCallback = callback; }
    void SetEndContactCallback(ContactCallback callback) { m_endContactCallback = callback; }
    void SetPreSolveCallback(PreSolveCallback callback) { m_preSolveCallback = callback; }
    void SetPostSolveCallback(PostSolveCallback callback) { m_postSolveCallback = callback; }

    b2World* GetWorld() { return m_world.get(); }
    const b2World* GetWorld() const { return m_world.get(); }

    void SetDebugDraw(b2Draw* debugDraw);
    void DrawDebugData();

    void QueryAABB(const b2AABB& aabb, std::vector<b2Fixture*>& fixtures);
    bool RayCast(const b2Vec2& start, const b2Vec2& end, b2RayCastOutput& output, b2Fixture** hitFixture = nullptr);

protected:
    void BeginContact(b2Contact* contact) override;
    void EndContact(b2Contact* contact) override;
    void PreSolve(b2Contact* contact, const b2Manifold* oldManifold) override;
    void PostSolve(b2Contact* contact, const b2ContactImpulse* impulse) override;

private:
    ContactInfo PerformCustomCollisionCheck(b2Fixture* fixtureA, b2Fixture* fixtureB, const b2Transform& xfA, const b2Transform& xfB);

    std::unique_ptr<b2World> m_world;
    bool m_useCustomDetection;

    ContactCallback m_beginContactCallback;
    ContactCallback m_endContactCallback;
    PreSolveCallback m_preSolveCallback;
    PostSolveCallback m_postSolveCallback;

    std::unordered_map<b2Contact*, ContactInfo> m_contactCache;

    int32_t m_velocityIterations;
    int32_t m_positionIterations;
};

class AABBQueryCallback : public b2QueryCallback {
public:
    std::vector<b2Fixture*>* fixtures;

    bool ReportFixture(b2Fixture* fixture) override {
        fixtures->push_back(fixture);
        return true; // Continuar la query
    }
};

class RayCastCallback : public b2RayCastCallback {
public:
    bool hasHit = false;
    b2Fixture* fixture = nullptr;
    b2Vec2 point;
    b2Vec2 normal;
    float fraction = 1.0f;

    float ReportFixture(b2Fixture* f, const b2Vec2& p, const b2Vec2& n, float fr) override {
        hasHit = true;
        fixture = f;
        point = p;
        normal = n;
        fraction = fr;
        return fr;
    }
};

#endif //PHYSICSWRAPPER_H