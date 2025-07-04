#include <SFML/Graphics.hpp>
#include "PhysicsWrapper.h"
#include <vector>
#include <memory>
#include <iostream> // Para mensajes de error

// --- Constants ---
const float SCREEN_WIDTH = 1280.f;
const float SCREEN_HEIGHT = 720.f;

// Box2D works with meters, so we need a scale to convert pixels to meters
const float SCALE = 30.f;

// --- Helper Functions ---

// Convert SFML position (pixels) to Box2D position (meters)
b2Vec2 pixelsToMeters(const sf::Vector2f& pixels) {
    return b2Vec2(pixels.x / SCALE, pixels.y / SCALE);
}

// Convert Box2D position (meters) to SFML position (pixels)
sf::Vector2f metersToPixels(const b2Vec2& meters) {
    return sf::Vector2f(meters.x * SCALE, meters.y * SCALE);
}

// --- Main Game Class ---

class Game {
public:
    // Enum para controlar el estado del juego
    enum GameState {
        PLAYING,
        WON
    };

    Game()
        : m_window(sf::VideoMode(SCREEN_WIDTH, SCREEN_HEIGHT), "Angry Birds Prototype"),
          m_physics(b2Vec2(0.0f, 9.8f)), // Gravity pointing down
          m_gameState(PLAYING)
    {
        m_window.setFramerateLimit(60);
        
        // Cargar una fuente para los mensajes
        if (!m_font.loadFromFile("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf")) {
            // Fallback para Windows si la fuente anterior no se encuentra
            if (!m_font.loadFromFile("C:/Windows/Fonts/Arial.ttf")) {
                std::cerr << "Error: No se pudo cargar la fuente. Asegúrate de que Arial.ttf o DejaVuSans.ttf estén disponibles." << std::endl;
            }
        }
        
        // Configurar el texto del mensaje
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
        // Eliminar todos los cuerpos actuales
        for (auto& body : m_bodies) {
            m_physics.DestroyBody(body);
        }
        m_bodies.clear();

        // Eliminar los cuerpos de las paredes también
        m_physics.DestroyBody(m_groundBody);
        m_physics.DestroyBody(m_leftWallBody);
        m_physics.DestroyBody(m_rightWallBody);
        m_physics.DestroyBody(m_ceilingBody);
        
        m_birdBody = nullptr;
        m_targetBody = nullptr;
        
        // Reiniciar estado
        m_isDragging = false;
        m_isBirdLaunched = false;
        m_gameState = PLAYING;

        // Volver a crear la escena
        createScene();
    }

    void createScene() {
        // --- Crear el Suelo y las Paredes (CORREGIDO) ---
        // Cada pared/suelo es ahora su propio cuerpo estático para mayor estabilidad.
        
        // Suelo
        b2BodyDef groundBodyDef;
        groundBodyDef.position.Set(SCREEN_WIDTH / 2.f / SCALE, SCREEN_HEIGHT / SCALE - (10.f / SCALE));
        m_groundBody = m_physics.CreateBody(&groundBodyDef);
        b2PolygonShape groundBox;
        groundBox.SetAsBox(SCREEN_WIDTH / 2.f / SCALE, 10.f / SCALE);
        m_groundBody->CreateFixture(&groundBox, 0.0f);

        // Pared izquierda
        b2BodyDef leftWallDef;
        leftWallDef.position.Set(10.f / SCALE, SCREEN_HEIGHT / 2.f / SCALE);
        m_leftWallBody = m_physics.CreateBody(&leftWallDef);
        b2PolygonShape leftWallBox;
        leftWallBox.SetAsBox(10.f / SCALE, SCREEN_HEIGHT / 2.f / SCALE);
        m_leftWallBody->CreateFixture(&leftWallBox, 0.0f);

        // Pared derecha
        b2BodyDef rightWallDef;
        rightWallDef.position.Set(SCREEN_WIDTH / SCALE - (10.f / SCALE), SCREEN_HEIGHT / 2.f / SCALE);
        m_rightWallBody = m_physics.CreateBody(&rightWallDef);
        b2PolygonShape rightWallBox;
        rightWallBox.SetAsBox(10.f / SCALE, SCREEN_HEIGHT / 2.f / SCALE);
        m_rightWallBody->CreateFixture(&rightWallBox, 0.0f);

        // Techo
        b2BodyDef ceilingDef;
        ceilingDef.position.Set(SCREEN_WIDTH / 2.f / SCALE, 10.f / SCALE);
        m_ceilingBody = m_physics.CreateBody(&ceilingDef);
        b2PolygonShape ceilingBox;
        ceilingBox.SetAsBox(SCREEN_WIDTH / 2.f / SCALE, 10.f / SCALE);
        m_ceilingBody->CreateFixture(&ceilingBox, 0.0f);


        // --- Crear Estructura de Obstáculos ---
        // Base
        for (int i = 0; i < 3; ++i) {
            createBox(950.f + i * 55.f, SCREEN_HEIGHT - 35.f, 25.f, 25.f);
        }
        // Segundo nivel
        for (int i = 0; i < 2; ++i) {
            createBox(977.f + i * 55.f, SCREEN_HEIGHT - 85.f, 25.f, 25.f);
        }
        // Techo de la estructura
        createBox(1005.f, SCREEN_HEIGHT - 135.f, 80.f, 10.f);

        // --- Crear el Objetivo ---
        b2BodyDef targetDef;
        targetDef.type = b2_dynamicBody;
        targetDef.position.Set(1005.f / SCALE, (SCREEN_HEIGHT - 105.f) / SCALE);
        m_targetBody = m_physics.CreateBody(&targetDef);
        
        b2CircleShape targetShape;
        targetShape.m_radius = 15.f / SCALE;
        m_physics.CreateCircleFixture(m_targetBody, &targetShape, 0.5f); // Más ligero que las cajas
        m_bodies.push_back(m_targetBody);

        // Poner a dormir al objetivo
        m_targetBody->SetSleepingAllowed(true);
        m_targetBody->SetAwake(false);


        // --- Crear el Pájaro ---
        b2BodyDef birdBodyDef;
        birdBodyDef.type = b2_dynamicBody;
        birdBodyDef.position.Set(150.f / SCALE, (SCREEN_HEIGHT - 100.f) / SCALE);
        m_birdBody = m_physics.CreateBody(&birdBodyDef);

        b2CircleShape circleShape;
        circleShape.m_radius = 20.f / SCALE;
        m_physics.CreateCircleFixture(m_birdBody, &circleShape, 2.0f); // Pesado
        m_bodies.push_back(m_birdBody);
        
        m_birdBody->SetSleepingAllowed(true);
        m_birdBody->SetAwake(false);
        m_isBirdLaunched = false;
    }

    b2Body* createBox(float x, float y, float halfWidth, float halfHeight) {
        b2BodyDef bodyDef;
        bodyDef.type = b2_dynamicBody;
        bodyDef.position.Set(x / SCALE, y / SCALE);
        b2Body* boxBody = m_physics.CreateBody(&bodyDef);

        m_physics.CreateBoxFixture(boxBody, halfWidth / SCALE, halfHeight / SCALE, 1.0f);
        m_bodies.push_back(boxBody);
        
        // Poner a dormir las cajas de la estructura
        boxBody->SetSleepingAllowed(true);
        boxBody->SetAwake(false);
        
        return boxBody;
    }

    void processEvents() {
        sf::Event event;
        while (m_window.pollEvent(event)) {
            if (event.type == sf::Event::Closed) {
                m_window.close();
            }
            if (event.type == sf::Event::KeyPressed && event.key.code == sf::Keyboard::R) {
                reset();
            }

            // --- Launch Mechanic (only if playing) ---
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
                        m_birdBody->SetAwake(true); // Despertar el pájaro al lanzarlo

                        sf::Vector2f dragEndPos = m_window.mapPixelToCoords({event.mouseButton.x, event.mouseButton.y});
                        sf::Vector2f launchVector = m_dragStartPos - dragEndPos;

                        float launchPower = 0.5f; // Aumentado para más potencia
                        m_birdBody->SetLinearVelocity(b2Vec2(launchVector.x * launchPower, launchVector.y * launchPower));
                    }
                }
            }
        }
    }

    void update(float dt) {
        if (m_gameState == PLAYING) {
            m_physics.Update(dt);

            // Condición de victoria: si el objetivo cae al suelo
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

        // Render all physics bodies
        for (const auto& body : m_bodies) {
            b2Fixture* fixture = body->GetFixtureList();
            while (fixture) {
                b2Shape::Type shapeType = fixture->GetType();
                sf::Vector2f pos = metersToPixels(body->GetPosition());
                float angle = body->GetAngle() * 180.f / b2_pi;

                if (shapeType == b2Shape::e_circle) {
                    sf::CircleShape circle(fixture->GetShape()->m_radius * SCALE);
                    circle.setOrigin(circle.getRadius(), circle.getRadius());
                    circle.setPosition(pos);
                    circle.setRotation(angle);

                    // Colorear el pájaro y el objetivo de forma diferente
                    if (body == m_birdBody) {
                        circle.setFillColor(sf::Color::Red);
                    } else if (body == m_targetBody) {
                        circle.setFillColor(sf::Color::Green);
                    }
                    
                    m_window.draw(circle);
                } else if (shapeType == b2Shape::e_polygon) {
                    b2PolygonShape* poly = (b2PolygonShape*)fixture->GetShape();
                    // Asumimos que es una caja creada con SetAsBox
                    b2Vec2 halfSize = poly->m_vertices[2]; 

                    sf::RectangleShape rect(sf::Vector2f(halfSize.x * 2 * SCALE, halfSize.y * 2 * SCALE));
                    rect.setOrigin(rect.getSize().x / 2.f, rect.getSize().y / 2.f);
                    rect.setPosition(pos);
                    rect.setRotation(angle);
                    rect.setFillColor(sf::Color(139, 69, 19)); // Brown
                    rect.setOutlineColor(sf::Color::Black);
                    rect.setOutlineThickness(1.f);
                    m_window.draw(rect);
                }

                fixture = fixture->GetNext();
            }
        }
        
        // Dibujar el suelo (es visual, el cuerpo físico es invisible)
        sf::RectangleShape ground(sf::Vector2f(SCREEN_WIDTH, 20.f));
        ground.setPosition(0, SCREEN_HEIGHT - 20.f);
        ground.setFillColor(sf::Color(34, 139, 34)); // Verde oscuro
        m_window.draw(ground);
        
        // Dibujar línea de lanzamiento
        if (m_isDragging) {
            sf::Vertex line[] = {
                sf::Vertex(m_dragStartPos, sf::Color::Black),
                sf::Vertex(sf::Vector2f(sf::Mouse::getPosition(m_window)), sf::Color::Black)
            };
            m_window.draw(line, 2, sf::Lines);
        }

        // Dibujar mensaje de victoria
        if (m_gameState == WON) {
            m_window.draw(m_messageText);
        }

        m_window.display();
    }

    sf::RenderWindow m_window;
    PhysicsWrapper m_physics;
    
    // Punteros a los cuerpos
    std::vector<b2Body*> m_bodies;
    b2Body* m_birdBody = nullptr;
    b2Body* m_targetBody = nullptr;
    b2Body* m_groundBody = nullptr;
    b2Body* m_leftWallBody = nullptr;
    b2Body* m_rightWallBody = nullptr;
    b2Body* m_ceilingBody = nullptr;


    // Variables para el control del juego
    bool m_isDragging = false;
    bool m_isBirdLaunched = false;
    sf::Vector2f m_dragStartPos;
    GameState m_gameState;

    // Variables para texto
    sf::Font m_font;
    sf::Text m_messageText;
};

int main() {
    Game game;
    game.run();
    return 0;
}
