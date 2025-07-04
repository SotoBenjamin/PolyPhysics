//
// Created by Benjamin Soto on 3/07/25.
//
#include "Collision.h"
#include <cmath>

namespace Collision {

    ContactInfo CheckCircleToCircle(const Circle& a, const Circle& b) {
        ContactInfo result;
        b2Vec2 toVector = b.center - a.center;
        float distSq = toVector.LengthSquared();
        float radiiSum = a.radius + b.radius;

        if (distSq < radiiSum * radiiSum) {
            result.hasCollision = true;
            float dist = std::sqrt(distSq);

            if (dist > 0.0f) {
                result.normal = (1.0f / dist) * toVector;
                result.depth = radiiSum - dist;
                // Punto de contacto en el borde del círculo A hacia B
                result.contactPoint = a.center + a.radius * result.normal;
            } else {
                // Círculos en la misma posición
                result.normal.Set(1.0f, 0.0f);
                result.depth = radiiSum;
                result.contactPoint = a.center;
            }
        }
        return result;
    }

    void ProjectVertices(const std::vector<b2Vec2>& vertices, const b2Vec2& axis, float& min, float& max) {
        min = std::numeric_limits<float>::max();
        max = -std::numeric_limits<float>::max();

        for (const auto& v : vertices) {
            float proj = b2Dot(v, axis);
            min = std::min(min, proj);
            max = std::max(max, proj);
        }
    }

    void ProjectCircle(const Circle& circle, const b2Vec2& axis, float& min, float& max) {
        float centerProj = b2Dot(circle.center, axis);
        min = centerProj - circle.radius;
        max = centerProj + circle.radius;
    }

    ContactInfo CheckPolygonToPolygon(const Polygon& a, const Polygon& b) {
        ContactInfo result;
        result.depth = std::numeric_limits<float>::max();

        // Verificar ejes de A
        for (const auto& axis : a.normals) {
            float minA, maxA, minB, maxB;
            ProjectVertices(a.vertices, axis, minA, maxA);
            ProjectVertices(b.vertices, axis, minB, maxB);

            if (maxA < minB || maxB < minA) {
                // Encontró un eje de separación
                return result; // hasCollision = false por defecto
            }

            float overlap = std::min(maxA, maxB) - std::max(minA, minB);
            if (overlap < result.depth) {
                result.depth = overlap;
                result.normal = axis;
            }
        }

        // Verificar ejes de B
        for (const auto& axis : b.normals) {
            float minA, maxA, minB, maxB;
            ProjectVertices(a.vertices, axis, minA, maxA);
            ProjectVertices(b.vertices, axis, minB, maxB);

            if (maxA < minB || maxB < minA) {
                return result;
            }

            float overlap = std::min(maxA, maxB) - std::max(minA, minB);
            if (overlap < result.depth) {
                result.depth = overlap;
                result.normal = axis;
            }
        }

        // Calcular dirección correcta de la normal
        b2Vec2 centerA = a.GetCenter();
        b2Vec2 centerB = b.GetCenter();
        b2Vec2 dir = centerB - centerA;

        if (b2Dot(dir, result.normal) < 0.0f) {
            result.normal = -result.normal;
        }

        result.hasCollision = true;
        return result;
    }

    b2Vec2 GetClosestPointOnEdge(const b2Vec2& point, const b2Vec2& edgeStart, const b2Vec2& edgeEnd) {
        b2Vec2 edge = edgeEnd - edgeStart;
        b2Vec2 toPoint = point - edgeStart;

        float edgeLengthSq = edge.LengthSquared();
        if (edgeLengthSq < 1e-6f) {
            return edgeStart; // Edge degenerado
        }

        float t = b2Dot(toPoint, edge) / edgeLengthSq;
        t = b2Clamp(t, 0.0f, 1.0f);

        return edgeStart + t * edge;
    }

    ContactInfo CheckCircleToPolygon(const Circle& circle, const Polygon& polygon) {
        ContactInfo result;

        // Verificar primero con SAT
        result.depth = std::numeric_limits<float>::max();

        // Verificar ejes del polígono
        for (const auto& axis : polygon.normals) {
            float minPoly, maxPoly, minCircle, maxCircle;
            ProjectVertices(polygon.vertices, axis, minPoly, maxPoly);
            ProjectCircle(circle, axis, minCircle, maxCircle);

            if (maxPoly < minCircle || maxCircle < minPoly) {
                return result; // No hay colisión
            }

            float overlap = std::min(maxPoly, maxCircle) - std::max(minPoly, minCircle);
            if (overlap < result.depth) {
                result.depth = overlap;
                result.normal = axis;
            }
        }

        // Encontrar el punto más cercano en el polígono
        float minDistSq = std::numeric_limits<float>::max();
        b2Vec2 closestPoint;

        for (size_t i = 0; i < polygon.vertices.size(); ++i) {
            b2Vec2 p1 = polygon.vertices.at(i);
            b2Vec2 p2 = polygon.vertices.at((i + 1) % polygon.vertices.size());

            b2Vec2 pointOnEdge = GetClosestPointOnEdge(circle.center, p1, p2);
            float distSq = (circle.center - pointOnEdge).LengthSquared();

            if (distSq < minDistSq) {
                minDistSq = distSq;
                closestPoint = pointOnEdge;
            }
        }

        // Verificar el eje desde el centro del círculo al punto más cercano
        b2Vec2 axis = circle.center - closestPoint;
        float distSq = axis.LengthSquared();

        if (distSq > 1e-6f) {
            float invDist = 1.0f / std::sqrt(distSq);
            axis = invDist * axis;

            float minPoly, maxPoly, minCircle, maxCircle;
            ProjectVertices(polygon.vertices, axis, minPoly, maxPoly);
            ProjectCircle(circle, axis, minCircle, maxCircle);

            if (maxPoly < minCircle || maxCircle < minPoly) {
                return result; // No hay colisión
            }

            float overlap = std::min(maxPoly, maxCircle) - std::max(minPoly, minCircle);
            if (overlap < result.depth) {
                result.depth = overlap;
                result.normal = axis;
            }
        }

        // Ajustar dirección de la normal
        b2Vec2 polyCenter = polygon.GetCenter();
        if (b2Dot(circle.center - polyCenter, result.normal) < 0.0f) {
            result.normal = -result.normal;
        }

        result.hasCollision = true;
        result.contactPoint = closestPoint;

        return result;
    }
}