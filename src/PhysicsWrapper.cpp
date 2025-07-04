#include "PhysicsWrapper.h"
#include "Collision.h"
#include <iostream>

PhysicsWrapper::PhysicsWrapper(const b2Vec2& gravity)
    : m_useCustomDetection(true)
    , m_velocityIterations(6)
    , m_positionIterations(2) {

    m_world = std::make_unique<b2World>(gravity);
    m_world->SetContactListener(this);

    // Configuración adicional del mundo
    m_world->SetAllowSleeping(true);
    m_world->SetContinuousPhysics(true);
}

PhysicsWrapper::~PhysicsWrapper() {
}

void PhysicsWrapper::Update(float deltaTime) {
    m_contactCache.clear();

    m_world->Step(deltaTime, m_velocityIterations, m_positionIterations);

    // Opcional: limpiar cuerpos marcados para destrucción
    // (si implementas un sistema de destrucción diferida)
}

void PhysicsWrapper::BeginContact(b2Contact* contact) {
    b2Fixture* fixtureA = contact->GetFixtureA();
    b2Fixture* fixtureB = contact->GetFixtureB();

    // Log para debugging
    if (m_useCustomDetection) {
        std::cout << "[BeginContact] Contacto iniciado entre fixtures" << std::endl;
    }

    // Llamar callback si está configurado
    if (m_beginContactCallback) {
        m_beginContactCallback(fixtureA, fixtureB);
    }
}

void PhysicsWrapper::EndContact(b2Contact* contact) {
    b2Fixture* fixtureA = contact->GetFixtureA();
    b2Fixture* fixtureB = contact->GetFixtureB();

    // Log para debugging
    if (m_useCustomDetection) {
        std::cout << "[EndContact] Contacto terminado entre fixtures" << std::endl;
    }

    // Llamar callback si está configurado
    if (m_endContactCallback) {
        m_endContactCallback(fixtureA, fixtureB);
    }

    // Limpiar del cache si existe
    m_contactCache.erase(contact);
}

void PhysicsWrapper::PreSolve(b2Contact* contact, const b2Manifold* oldManifold) {
    if (!m_useCustomDetection) {
        return;
    }

    b2Fixture* fixtureA = contact->GetFixtureA();
    b2Fixture* fixtureB = contact->GetFixtureB();

    b2Body* bodyA = fixtureA->GetBody();
    b2Body* bodyB = fixtureB->GetBody();
    const b2Transform& xfA = bodyA->GetTransform();
    const b2Transform& xfB = bodyB->GetTransform();

    ContactInfo result;
    auto cacheIt = m_contactCache.find(contact);
    if (cacheIt != m_contactCache.end()) {
        result = cacheIt->second;
    } else {
        result = PerformCustomCollisionCheck(fixtureA, fixtureB, xfA, xfB);
        m_contactCache[contact] = result;
    }

    if (!result.hasCollision) {
        contact->SetEnabled(false);
        std::cout << "[PreSolve] Contacto deshabilitado por detección personalizada" << std::endl;
        return;
    }

    std::cout << "[PreSolve] Colisión confirmada - Profundidad: " << result.depth
              << " Normal: (" << result.normal.x << ", " << result.normal.y << ")"
              << std::endl;

    b2WorldManifold worldManifold;
    contact->GetWorldManifold(&worldManifold);

    if (m_preSolveCallback) {
        bool shouldContinue = m_preSolveCallback(fixtureA, fixtureB, result);
        if (!shouldContinue) {
            contact->SetEnabled(false);
            std::cout << "[PreSolve] Contacto deshabilitado por callback" << std::endl;
        }
    }
}

void PhysicsWrapper::PostSolve(b2Contact* contact, const b2ContactImpulse* impulse) {
    if (!m_postSolveCallback) {
        return;
    }

    b2Fixture* fixtureA = contact->GetFixtureA();
    b2Fixture* fixtureB = contact->GetFixtureB();

    // Calcular el impulso total
    float totalImpulse = 0.0f;
    for (int i = 0; i < impulse->count; ++i) {
        totalImpulse += impulse->normalImpulses[i];
    }

    // Log para colisiones fuertes
    if (totalImpulse > 10.0f) {
        std::cout << "[PostSolve] Colisión fuerte detectada! Impulso: " << totalImpulse << std::endl;
    }

    m_postSolveCallback(fixtureA, fixtureB, impulse);
}

ContactInfo PhysicsWrapper::PerformCustomCollisionCheck(b2Fixture* fixtureA, b2Fixture* fixtureB,
                                                        const b2Transform& xfA, const b2Transform& xfB) {
    ContactInfo result;

    b2Shape::Type typeA = fixtureA->GetType();
    b2Shape::Type typeB = fixtureB->GetType();

    if (typeA == b2Shape::e_circle && typeB == b2Shape::e_circle) {
        const b2CircleShape* circleA = static_cast<const b2CircleShape*>(fixtureA->GetShape());
        const b2CircleShape* circleB = static_cast<const b2CircleShape*>(fixtureB->GetShape());

        Circle cA = {
            b2Mul(xfA, circleA->m_p),
            circleA->m_radius
        };
        Circle cB = {
            b2Mul(xfB, circleB->m_p),
            circleB->m_radius
        };

        result = Collision::CheckCircleToCircle(cA, cB);
    }
    // Polígono vs Polígono
    else if (typeA == b2Shape::e_polygon && typeB == b2Shape::e_polygon) {
        const b2PolygonShape* polyA = static_cast<const b2PolygonShape*>(fixtureA->GetShape());
        const b2PolygonShape* polyB = static_cast<const b2PolygonShape*>(fixtureB->GetShape());

        Polygon pA, pB;

        pA.vertices.reserve(polyA->m_count);
        for (int i = 0; i < polyA->m_count; ++i) {
            pA.vertices.push_back(b2Mul(xfA, polyA->m_vertices[i]));
        }
        pA.ComputeNormals();

        pB.vertices.reserve(polyB->m_count);
        for (int i = 0; i < polyB->m_count; ++i) {
            pB.vertices.push_back(b2Mul(xfB, polyB->m_vertices[i]));
        }
        pB.ComputeNormals();

        result = Collision::CheckPolygonToPolygon(pA, pB);
    }
    else if (typeA == b2Shape::e_circle && typeB == b2Shape::e_polygon) {
        const b2CircleShape* circle = static_cast<const b2CircleShape*>(fixtureA->GetShape());
        const b2PolygonShape* poly = static_cast<const b2PolygonShape*>(fixtureB->GetShape());

        Circle c = {
            b2Mul(xfA, circle->m_p),
            circle->m_radius
        };

        Polygon p;
        p.vertices.reserve(poly->m_count);
        for (int i = 0; i < poly->m_count; ++i) {
            p.vertices.push_back(b2Mul(xfB, poly->m_vertices[i]));
        }
        p.ComputeNormals();

        result = Collision::CheckCircleToPolygon(c, p);
    }
    else if (typeA == b2Shape::e_polygon && typeB == b2Shape::e_circle) {
        const b2PolygonShape* poly = static_cast<const b2PolygonShape*>(fixtureA->GetShape());
        const b2CircleShape* circle = static_cast<const b2CircleShape*>(fixtureB->GetShape());

        Polygon p;
        p.vertices.reserve(poly->m_count);
        for (int i = 0; i < poly->m_count; ++i) {
            p.vertices.push_back(b2Mul(xfA, poly->m_vertices[i]));
        }
        p.ComputeNormals();

        Circle c = {
            b2Mul(xfB, circle->m_p),
            circle->m_radius
        };

        result = Collision::CheckCircleToPolygon(c, p);

        if (result.hasCollision) {
            result.normal = -result.normal;
        }
    }
    else {
        std::cout<<"BOX2D"<<std::endl;
        // Para otros tipos de formas (edge, chain), usar detección de Box2D
        result.hasCollision = true;
    }

    return result;
}

b2Body* PhysicsWrapper::CreateBody(const b2BodyDef* def) {
    return m_world->CreateBody(def);
}

void PhysicsWrapper::DestroyBody(b2Body* body) {
    if (body) {
        m_world->DestroyBody(body);
    }
}

b2Fixture* PhysicsWrapper::CreatePolygonFixture(b2Body* body, const b2PolygonShape* shape, float density) {
    b2FixtureDef fixtureDef;
    fixtureDef.shape = shape;
    fixtureDef.density = density;
    fixtureDef.friction = 0.3f;
    fixtureDef.restitution = 0.1f;

    return body->CreateFixture(&fixtureDef);
}

b2Fixture* PhysicsWrapper::CreateCircleFixture(b2Body* body, const b2CircleShape* shape, float density) {
    b2FixtureDef fixtureDef;
    fixtureDef.shape = shape;
    fixtureDef.density = density;
    fixtureDef.friction = 0.3f;
    fixtureDef.restitution = 0.1f;

    return body->CreateFixture(&fixtureDef);
}

b2Fixture* PhysicsWrapper::CreateBoxFixture(b2Body* body, float halfWidth, float halfHeight, float density) {
    b2PolygonShape box;
    box.SetAsBox(halfWidth, halfHeight);

    return CreatePolygonFixture(body, &box, density);
}

void PhysicsWrapper::SetCollisionFilter(b2Fixture* fixture, uint16_t category, uint16_t mask) {
    b2Filter filter;
    filter.categoryBits = category;
    filter.maskBits = mask;
    fixture->SetFilterData(filter);
}

void PhysicsWrapper::SetDebugDraw(b2Draw* debugDraw) {
    m_world->SetDebugDraw(debugDraw);
}

void PhysicsWrapper::DrawDebugData() {
    m_world->DebugDraw();
}

void PhysicsWrapper::QueryAABB(const b2AABB& aabb, std::vector<b2Fixture*>& fixtures) {
    AABBQueryCallback callback;
    callback.fixtures = &fixtures;
    m_world->QueryAABB(&callback, aabb);
}

bool PhysicsWrapper::RayCast(const b2Vec2& start, const b2Vec2& end, b2RayCastOutput& output, b2Fixture** hitFixture) {
    RayCastCallback callback;
    m_world->RayCast(&callback, start, end);

    if (callback.hasHit) {
        output.fraction = callback.fraction;
        output.normal = callback.normal;
        if (hitFixture) {
            *hitFixture = callback.fixture;
        }
        return true;
    }

    return false;
}