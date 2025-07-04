//
// Created by Benjamin Soto on 3/07/25.
//
#ifndef COLLISION_H
#define COLLISION_H

#include <box2d/box2d.h>
#include <vector>
#include <limits>
#include <algorithm>

struct Circle {
    b2Vec2 center;
    float radius;
};

struct Polygon {
    std::vector<b2Vec2> vertices;
    std::vector<b2Vec2> normals;

    // Constructor helper para calcular las normales automáticamente
    void ComputeNormals() {
        normals.clear();
        size_t count = vertices.size();
        normals.reserve(count);

        for (size_t i = 0; i < count; ++i) {
            b2Vec2 edge = vertices.at((i + 1) % count) - vertices.at(i);
            // Normal perpendicular (rotación 90 grados en sentido horario)
            b2Vec2 normal(-edge.y, edge.x);
            float length = normal.Length();
            if (length > 0.0f) {
                normal *= (1.0f / length);
            }
            normals.push_back(normal);
        }
    }

    // Obtener el centroide del polígono
    b2Vec2 GetCenter() const {
        b2Vec2 center(0.0f, 0.0f);
        for (const auto& v : vertices) {
            center += v;
        }
        return (1.0f / vertices.size()) * center;
    }
};

struct ContactInfo {
    bool hasCollision = false;
    b2Vec2 normal = b2Vec2_zero;
    float depth = 0.0f;
    b2Vec2 contactPoint = b2Vec2_zero; // Punto de contacto opcional
};

namespace Collision {
    ContactInfo CheckCircleToCircle(const Circle& a, const Circle& b);
    ContactInfo CheckPolygonToPolygon(const Polygon& a, const Polygon& b);
    ContactInfo CheckCircleToPolygon(const Circle& circle, const Polygon& polygon);

    // Funciones auxiliares
    void ProjectVertices(const std::vector<b2Vec2>& vertices, const b2Vec2& axis, float& min, float& max);
    void ProjectCircle(const Circle& circle, const b2Vec2& axis, float& min, float& max);
    b2Vec2 GetClosestPointOnEdge(const b2Vec2& point, const b2Vec2& edgeStart, const b2Vec2& edgeEnd);
}

#endif