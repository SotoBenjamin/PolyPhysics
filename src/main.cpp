#include <SFML/Graphics.hpp>
#include "PhysicsWrapper.h"
#include <vector>
#include <memory>
#include <iostream>
#include <cmath> // Para std::cos y std::sin

// --- Constants ---
const float SCREEN_WIDTH = 1280.f;
const float SCREEN_HEIGHT = 720.f;
const float SCALE = 30.f;

b2Vec2 pixelsToMeters(const sf::Vector2f& pixels) {
    return b2Vec2(pixels.x / SCALE, pixels.y / SCALE);
}

sf::Vector2f metersToPixels(const b2Vec2& meters) {
    return sf::Vector2f(meters.x * SCALE, meters.y * SCALE);
}

class Game {
public:
    enum GameState {
        PLAYING,
        WON
    };

    Game()
        : m_window(sf::VideoMode(SCREEN_WIDTH, SCREEN_HEIGHT), "Angry Birds - Estructura con Hexágono"),
          m_physics(b2Vec2(0.0f, 9.8f)),
          m_gameState(PLAYING)
    {
        m_window.setFramerateLimit(60);

        if (!m_font.loadFromFile("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf")) {
            if (!m_font.loadFromFile("C:/Windows/Fonts/Arial.ttf")) {
                std::cerr << "Error: No se pudo cargar la fuente." << std::endl;
            }
        }

        m_messageText.setFont(m_font);
        m_messageText.setCharacterSize(50);
        m_messageText.setFillColor(sf::Color::White);
        m_messageText.setStyle(sf::Text::Bold);

        createScene();
    }

    void run() {
        sf::Clock clock;
        while (m_window.isOpen()) {
            sf::Time dt = clock.restart();
            processEvents();
            update(dt.asSeconds());
            render();
        }
    }

private:
    void reset() {
        for (auto& body : m_bodies) {
            m_physics.DestroyBody(body);
        }
        m_bodies.clear();

        m_physics.DestroyBody(m_groundBody);
        m_physics.DestroyBody(m_leftWallBody);
        m_physics.DestroyBody(m_rightWallBody);
        m_physics.DestroyBody(m_ceilingBody);

        m_birdBody = nullptr;
        m_pigBody = nullptr;
        m_targetBody = nullptr;

        m_isDragging = false;
        m_isBirdLaunched = false;
        m_gameState = PLAYING;

        createScene();
    }

    void createScene() {
        // --- Muros y suelo ---
        b2BodyDef groundBodyDef;
        groundBodyDef.position.Set(SCREEN_WIDTH / 2.f / SCALE, SCREEN_HEIGHT / SCALE - (10.f / SCALE));
        m_groundBody = m_physics.CreateBody(&groundBodyDef);
        b2PolygonShape groundBox;
        groundBox.SetAsBox(SCREEN_WIDTH / 2.f / SCALE, 10.f / SCALE);
        m_groundBody->CreateFixture(&groundBox, 0.0f);

        // ... (código de los muros sin cambios)
        b2BodyDef leftWallDef;
        leftWallDef.position.Set(10.f / SCALE, SCREEN_HEIGHT / 2.f / SCALE);
        m_leftWallBody = m_physics.CreateBody(&leftWallDef);
        b2PolygonShape leftWallBox;
        leftWallBox.SetAsBox(10.f / SCALE, SCREEN_HEIGHT / 2.f / SCALE);
        m_leftWallBody->CreateFixture(&leftWallBox, 0.0f);

        b2BodyDef rightWallDef;
        rightWallDef.position.Set(SCREEN_WIDTH / SCALE - (10.f / SCALE), SCREEN_HEIGHT / 2.f / SCALE);
        m_rightWallBody = m_physics.CreateBody(&rightWallDef);
        b2PolygonShape rightWallBox;
        rightWallBox.SetAsBox(10.f / SCALE, SCREEN_HEIGHT / 2.f / SCALE);
        m_rightWallBody->CreateFixture(&rightWallBox, 0.0f);

        b2BodyDef ceilingDef;
        ceilingDef.position.Set(SCREEN_WIDTH / 2.f / SCALE, 10.f / SCALE);
        m_ceilingBody = m_physics.CreateBody(&ceilingDef);
        b2PolygonShape ceilingBox;
        ceilingBox.SetAsBox(SCREEN_WIDTH / 2.f / SCALE, 10.f / SCALE);
        m_ceilingBody->CreateFixture(&ceilingBox, 0.0f);


        // --- Estructura de Obstáculos ---
        createBox(950.f + 1 * 55.f, SCREEN_HEIGHT - 35.f, 25.f, 25.f);
        createBox(950.f + 2 * 55.f, SCREEN_HEIGHT - 35.f, 25.f, 25.f);
        createBox(977.f + 1 * 55.f, SCREEN_HEIGHT - 85.f, 25.f, 25.f);
        createBox(1032.f, SCREEN_HEIGHT - 135.f, 80.f, 10.f);
        createTriangle({1032.f, SCREEN_HEIGHT - 185.f}, 35.f);

        // Añadimos el nuevo hexágono a la estructura
        createHexagon({950.f, SCREEN_HEIGHT - 60.f}, 30.f);

        // --- Cerdo (Objetivo) ---
        b2BodyDef pigBodyDef;
        pigBodyDef.type = b2_dynamicBody;
        pigBodyDef.position.Set(1032.f / SCALE, (SCREEN_HEIGHT - 105.f) / SCALE);
        m_pigBody = m_physics.CreateBody(&pigBodyDef);
        b2CircleShape pigShape;
        pigShape.m_radius = 15.f / SCALE;
        m_physics.CreateCircleFixture(m_pigBody, &pigShape, 0.5f);
        m_bodies.push_back(m_pigBody);
        m_pigBody->SetSleepingAllowed(true);
        m_pigBody->SetAwake(false);
        m_targetBody = m_pigBody; // El cerdo sigue siendo el objetivo

        // --- Pájaro ---
        b2BodyDef birdBodyDef;
        birdBodyDef.type = b2_dynamicBody;
        birdBodyDef.position.Set(150.f / SCALE, (SCREEN_HEIGHT - 100.f) / SCALE);
        m_birdBody = m_physics.CreateBody(&birdBodyDef);
        b2CircleShape circleShape;
        circleShape.m_radius = 20.f / SCALE;
        m_physics.CreateCircleFixture(m_birdBody, &circleShape, 2.0f);
        m_bodies.push_back(m_birdBody);
        m_birdBody->SetSleepingAllowed(true);
        m_birdBody->SetAwake(false);
        m_isBirdLaunched = false;
    }
    
    // --- NUEVA FUNCIÓN PARA CREAR HEXÁGONOS ---
    b2Body* createHexagon(const sf::Vector2f& position, float radius) {
        b2BodyDef bodyDef;
        bodyDef.type = b2_dynamicBody;
        bodyDef.position = pixelsToMeters(position);
        b2Body* hexaBody = m_physics.CreateBody(&bodyDef);

        b2PolygonShape hexagonShape;
        b2Vec2 vertices[6];
        float angle = 0.0f;
        for (int i = 0; i < 6; i++) {
            vertices[i].Set(
                (radius / SCALE) * std::cos(angle),
                (radius / SCALE) * std::sin(angle)
            );
            angle += 60.0f * b2_pi / 180.0f; // 60 grados en radianes
        }
        hexagonShape.Set(vertices, 6);

        m_physics.CreatePolygonFixture(hexaBody, &hexagonShape, 1.2f); // Densidad media
        m_bodies.push_back(hexaBody);

        hexaBody->SetSleepingAllowed(true);
        hexaBody->SetAwake(false);

        return hexaBody;
    }

    b2Body* createTriangle(const sf::Vector2f& position, float size) {
        b2BodyDef bodyDef;
        bodyDef.type = b2_dynamicBody;
        bodyDef.position = pixelsToMeters(position);
        b2Body* triangleBody = m_physics.CreateBody(&bodyDef);
        b2PolygonShape triangleShape;
        b2Vec2 vertices[3];
        vertices[0].Set(0.0f, -size / SCALE);
        vertices[1].Set(size / SCALE, size / SCALE);
        vertices[2].Set(-size / SCALE, size / SCALE);
        triangleShape.Set(vertices, 3);
        m_physics.CreatePolygonFixture(triangleBody, &triangleShape, 1.5f);
        m_bodies.push_back(triangleBody);
        triangleBody->SetSleepingAllowed(true);
        triangleBody->SetAwake(false);
        return triangleBody;
    }

    b2Body* createBox(float x, float y, float halfWidth, float halfHeight) {
        b2BodyDef bodyDef;
        bodyDef.type = b2_dynamicBody;
        bodyDef.position.Set(x / SCALE, y / SCALE);
        b2Body* boxBody = m_physics.CreateBody(&bodyDef);
        m_physics.CreateBoxFixture(boxBody, halfWidth / SCALE, halfHeight / SCALE, 1.0f);
        m_bodies.push_back(boxBody);
        boxBody->SetSleepingAllowed(true);
        boxBody->SetAwake(false);
        return boxBody;
    }

    void processEvents() {
    sf::Event event;
    // Bucle correcto para procesar todos los eventos en la cola
    while (m_window.pollEvent(event)) {
        // Evento para cerrar la ventana
        if (event.type == sf::Event::Closed) {
            m_window.close();
        }

        // Evento para reiniciar el juego
        if (event.type == sf::Event::KeyPressed && event.key.code == sf::Keyboard::R) {
            reset();
        }

        if (m_gameState == PLAYING) {
            if (event.type == sf::Event::MouseButtonPressed) {
                if (event.mouseButton.button == sf::Mouse::Left) {
                    sf::Vector2f mousePos = m_window.mapPixelToCoords({event.mouseButton.x, event.mouseButton.y});
                    sf::Vector2f birdPos = metersToPixels(m_birdBody->GetPosition());
                    if (std::hypot(mousePos.x - birdPos.x, mousePos.y - birdPos.y) < 30.f && !m_isBirdLaunched) {
                        m_isDragging = true;
                        m_dragStartPos = mousePos;
                    }
                }
            }

            if (event.type == sf::Event::MouseButtonReleased) {
                if (event.mouseButton.button == sf::Mouse::Left && m_isDragging) {
                    m_isDragging = false;
                    m_isBirdLaunched = true;
                    m_birdBody->SetAwake(true);
                    sf::Vector2f dragEndPos = m_window.mapPixelToCoords({event.mouseButton.x, event.mouseButton.y});
                    sf::Vector2f launchVector = m_dragStartPos - dragEndPos;
                    float launchPower = 0.5f;
                    m_birdBody->SetLinearVelocity(b2Vec2(launchVector.x * launchPower, launchVector.y * launchPower));
                }
            }
        }
    }
}

    void update(float dt) {
        if (m_gameState == PLAYING) {
            m_physics.Update(dt);
            if (m_targetBody && m_targetBody->GetPosition().y > (SCREEN_HEIGHT - 40.f) / SCALE) {
                m_gameState = WON;
                m_messageText.setString("¡Ganaste!\nPresiona R para reiniciar");
                sf::FloatRect textRect = m_messageText.getLocalBounds();
                m_messageText.setOrigin(textRect.left + textRect.width / 2.0f, textRect.top + textRect.height / 2.0f);
                m_messageText.setPosition(sf::Vector2f(SCREEN_WIDTH / 2.0f, SCREEN_HEIGHT / 2.0f));
            }
        }
    }

    void render() {
        m_window.clear(sf::Color(135, 206, 235)); // Sky blue

        for (const auto& body : m_bodies) {
            b2Fixture* fixture = body->GetFixtureList();
            while (fixture) {
                sf::Vector2f pos = metersToPixels(body->GetPosition());
                float angle = body->GetAngle() * 180.f / b2_pi;

                if (fixture->GetType() == b2Shape::e_circle) {
                    sf::CircleShape circle(fixture->GetShape()->m_radius * SCALE);
                    circle.setOrigin(circle.getRadius(), circle.getRadius());
                    circle.setPosition(pos);
                    circle.setRotation(angle);
                    if (body == m_birdBody) {
                        circle.setFillColor(sf::Color::Red);
                    } else if (body == m_pigBody) {
                        circle.setFillColor(sf::Color::Green);
                    }
                    m_window.draw(circle);
                } else if (fixture->GetType() == b2Shape::e_polygon) {
                    b2PolygonShape* polyShape = (b2PolygonShape*)fixture->GetShape();
                    int vertexCount = polyShape->m_count;
                    sf::ConvexShape convex;
                    convex.setPointCount(vertexCount);
                    for (int i = 0; i < vertexCount; i++) {
                        sf::Vector2f point = metersToPixels(polyShape->m_vertices[i]);
                        convex.setPoint(i, point);
                    }
                    convex.setPosition(pos);
                    convex.setRotation(angle);
                    convex.setFillColor(sf::Color(139, 69, 19)); // Brown
                    convex.setOutlineColor(sf::Color::Black);
                    convex.setOutlineThickness(1.f);
                    m_window.draw(convex);
                }
                fixture = fixture->GetNext();
            }
        }

        sf::RectangleShape ground(sf::Vector2f(SCREEN_WIDTH, 20.f));
        ground.setPosition(0, SCREEN_HEIGHT - 20.f);
        ground.setFillColor(sf::Color(34, 139, 34)); // Dark green
        m_window.draw(ground);

        if (m_isDragging) {
            sf::Vertex line[] = {
                sf::Vertex(m_dragStartPos, sf::Color::Black),
                sf::Vertex(sf::Vector2f(sf::Mouse::getPosition(m_window)), sf::Color::Black)
            };
            m_window.draw(line, 2, sf::Lines);
        }

        if (m_gameState == WON) {
            m_window.draw(m_messageText);
        }

        m_window.display();
    }

    sf::RenderWindow m_window;
    PhysicsWrapper m_physics;
    std::vector<b2Body*> m_bodies;
    b2Body* m_birdBody = nullptr;
    b2Body* m_pigBody = nullptr;
    b2Body* m_targetBody = nullptr;
    b2Body* m_groundBody = nullptr;
    b2Body* m_leftWallBody = nullptr;
    b2Body* m_rightWallBody = nullptr;
    b2Body* m_ceilingBody = nullptr;

    bool m_isDragging = false;
    bool m_isBirdLaunched = false;
    sf::Vector2f m_dragStartPos;
    GameState m_gameState;

    sf::Font m_font;
    sf::Text m_messageText;
};

int main() {
    Game game;
    game.run();
    return 0;
}