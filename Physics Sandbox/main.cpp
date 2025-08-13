#include <SFML/Graphics.hpp>
#include <vector>
#include <cmath>
#include <sstream>

void drawArrow(sf::RenderWindow& window, sf::Vector2f start, sf::Vector2f vec, sf::Color color) {
    sf::Vertex line[] = {
        sf::Vertex(start, color),
        sf::Vertex(start + vec, color)
    };
    window.draw(line, 2, sf::Lines);

    if (vec == sf::Vector2f(0.f, 0.f)) return;
    float size = 10.f;
    sf::Vector2f unit = vec / std::sqrt(vec.x * vec.x + vec.y * vec.y);
    sf::Vector2f perp(-unit.y, unit.x);
    sf::Vertex head[] = {
        sf::Vertex(start + vec, color),
        sf::Vertex(start + vec - unit * size + perp * size / 2.f, color),
        sf::Vertex(start + vec, color),
        sf::Vertex(start + vec - unit * size - perp * size / 2.f, color)
    };
    window.draw(head, 4, sf::Lines);
}

struct Ball {
    sf::CircleShape shape;
    sf::Vector2f velocity = {0.f, 0.f};
    sf::Vector2f acceleration = {0.f, 0.f};
    float mass = 1.f;
    float elasticity = 0.8f;
    float dragCoefficient = 0.1f;
    std::vector<sf::Vector2f> trajectoryPoints;

    sf::Vector2f gravityForce, dragForce, totalForce;
    sf::Vector2f lastThrowForce = {0.f, 0.f};

    Ball(float radius, sf::Vector2f pos) {
        shape.setRadius(radius);
        shape.setOrigin(radius, radius);
        shape.setPosition(pos);
        shape.setFillColor(sf::Color::Green);
    }

    void updateForces(sf::Vector2f gravity) {
        gravityForce = gravity * mass;
        dragForce = -dragCoefficient * velocity;
        totalForce = gravityForce + dragForce;
        acceleration = totalForce / mass;
    }

    void updatePhysics(float dt, const sf::RenderWindow& window) {
        velocity += acceleration * dt;
        shape.move(velocity * dt);

        sf::Vector2f pos = shape.getPosition();
        float r = shape.getRadius();
        if (pos.x - r < 0) {
            pos.x = r;
            velocity.x = -velocity.x * elasticity;
        } else if (pos.x + r > window.getSize().x) {
            pos.x = window.getSize().x - r;
            velocity.x = -velocity.x * elasticity;
        }
        if (pos.y - r < 0) {
            pos.y = r;
            velocity.y = -velocity.y * elasticity;
        } else if (pos.y + r > window.getSize().y) {
            pos.y = window.getSize().y - r;
            velocity.y = -velocity.y * elasticity;
        }
        shape.setPosition(pos);

        trajectoryPoints.push_back(pos);
        if (trajectoryPoints.size() > 150)
            trajectoryPoints.erase(trajectoryPoints.begin());
    }

    void drawTrajectory(sf::RenderWindow& window) const {
	if (trajectoryPoints.size() < 2) return;
	sf::VertexArray line(sf::LineStrip, trajectoryPoints.size());
	for (size_t i = 0; i < trajectoryPoints.size(); ++i) {
        	line[i].position = trajectoryPoints[i];
        	line[i].color = sf::Color(100, 255, 100, 150);
    	}
    	window.draw(line);
	}

};

void handlePlatformCollision(Ball& ball, const sf::RectangleShape& plat) {
    sf::FloatRect p = plat.getGlobalBounds();
    sf::Vector2f pos = ball.shape.getPosition();
    float r = ball.shape.getRadius();
    sf::Vector2f& v = ball.velocity;

    // Top
    if (pos.y + r > p.top && pos.y < p.top &&
        pos.x + r > p.left && pos.x - r < p.left + p.width && v.y > 0) {
        pos.y = p.top - r;
        v.y = -v.y * ball.elasticity;
    }
    // Bottom
    else if (pos.y - r < p.top + p.height && pos.y > p.top + p.height &&
             pos.x + r > p.left && pos.x - r < p.left + p.width && v.y < 0) {
        pos.y = p.top + p.height + r;
        v.y = -v.y * ball.elasticity;
    }
    // Left
    else if (pos.x + r > p.left && pos.x < p.left &&
             pos.y + r > p.top && pos.y - r < p.top + p.height && v.x > 0) {
        pos.x = p.left - r;
        v.x = -v.x * ball.elasticity;
    }
    // Right
    else if (pos.x - r < p.left + p.width && pos.x > p.left + p.width &&
             pos.y + r > p.top && pos.y - r < p.top + p.height && v.x < 0) {
        pos.x = p.left + p.width + r;
        v.x = -v.x * ball.elasticity;
    }

    ball.shape.setPosition(pos);
}
void resolveCollision(Ball& a, Ball& b) {
    sf::Vector2f delta = b.shape.getPosition() - a.shape.getPosition();
    float dist = std::sqrt(delta.x * delta.x + delta.y * delta.y);
    if (dist == 0.f) return;

    float overlap = a.shape.getRadius() + b.shape.getRadius() - dist;
    if (overlap > 0.f) {
        sf::Vector2f normal = delta / dist;
        a.shape.move(-normal * (overlap / 2.f));
        b.shape.move(normal * (overlap / 2.f));

        sf::Vector2f relVel = b.velocity - a.velocity;
        float velAlongNormal = relVel.x * normal.x + relVel.y * normal.y;
        if (velAlongNormal > 0) return;

        float restitution = (a.elasticity + b.elasticity) / 2.f;
        float impulse = -(1 + restitution) * velAlongNormal / (1 / a.mass + 1 / b.mass);
        sf::Vector2f impulseVec = impulse * normal;
        a.velocity -= impulseVec / a.mass;
        b.velocity += impulseVec / b.mass;
    }
}

std::vector<sf::Vector2f> simulateTrajectoryWithCollisions(
    const Ball& ball,
    sf::Vector2f gravity,
    float duration,
    float dt,
    const std::vector<sf::RectangleShape>& platforms,
    const sf::Vector2u& windowSize)
{
    Ball ghost = ball;
    std::vector<sf::Vector2f> points;

    for (float t = 0.f; t < duration; t += dt) {
        ghost.updateForces(gravity);
        ghost.velocity += ghost.acceleration * dt;
        ghost.shape.move(ghost.velocity * dt);

        sf::Vector2f pos = ghost.shape.getPosition();
        float r = ghost.shape.getRadius();

        if (pos.x - r < 0) {
            pos.x = r;
            ghost.velocity.x = -ghost.velocity.x * ghost.elasticity;
        } else if (pos.x + r > windowSize.x) {
            pos.x = windowSize.x - r;
            ghost.velocity.x = -ghost.velocity.x * ghost.elasticity;
        }

        if (pos.y - r < 0) {
            pos.y = r;
            ghost.velocity.y = -ghost.velocity.y * ghost.elasticity;
        } else if (pos.y + r > windowSize.y) {
            pos.y = windowSize.y - r;
            ghost.velocity.y = -ghost.velocity.y * ghost.elasticity;
        }

        ghost.shape.setPosition(pos);
        for (const auto& plat : platforms) {
            handlePlatformCollision(ghost, plat);
        }

        points.push_back(ghost.shape.getPosition());
    }

    return points;
}

int main() {
    sf::RenderWindow window(sf::VideoMode(1000, 700), "SFML Physics + Prediction with Collisions");
    window.setFramerateLimit(60);

    sf::Font font;
    font.loadFromFile("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf");

    std::vector<Ball> balls;
    std::vector<sf::RectangleShape> platforms;

    sf::RectangleShape platform(sf::Vector2f(600, 20));
    platform.setPosition(200, 600);
    platform.setFillColor(sf::Color(150, 150, 150));
    platforms.push_back(platform);

    bool dragging = false, paused = false;
    int draggingIndex = -1, selectedBall = -1;
    sf::Vector2f dragStartMouse, dragStartBallPos;
    sf::Vector2f gravity(0.f, 980.f);
    float defaultElasticity = 0.8f, defaultDrag = 0.1f;

    sf::RectangleShape menuBox(sf::Vector2f(180, 100));
    menuBox.setPosition(200, 50);

    menuBox.setFillColor(sf::Color(50, 50, 50, 200));
    sf::FloatRect menuClickArea = menuBox.getGlobalBounds();
    bool showLegend = false;
    
    bool showControls = false;
    sf::RectangleShape controlsBox(sf::Vector2f(380, 100));
    controlsBox.setPosition(700, 50);
    controlsBox.setSize(sf::Vector2f(350, 200));

    controlsBox.setFillColor(sf::Color(50, 50, 50, 200));
    sf::FloatRect controlsClickArea(controlsBox.getPosition(), sf::Vector2f(20, 20));



    sf::Clock clock;

    while (window.isOpen()) {
        float dt = clock.restart().asSeconds();
        sf::Event event;

        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed) window.close();
            if (event.type == sf::Event::KeyPressed) {
                if (event.key.code == sf::Keyboard::Space) paused = !paused;
                if (event.key.code == sf::Keyboard::A) {
                    Ball b(20.f, {500.f, 100.f});
                    b.elasticity = defaultElasticity;
                    b.dragCoefficient = defaultDrag;
                    balls.push_back(b);
                }
                if (event.key.code == sf::Keyboard::D && !balls.empty()) balls.pop_back();
                if (event.key.code == sf::Keyboard::G) gravity.y += 100.f;
                if (event.key.code == sf::Keyboard::H) gravity.y = std::max(0.f, gravity.y - 100.f);
                if (event.key.code == sf::Keyboard::R) defaultDrag += 0.05f;
                if (event.key.code == sf::Keyboard::F) defaultDrag = std::max(0.f, defaultDrag - 0.05f);
                if (event.key.code == sf::Keyboard::E) defaultElasticity = std::min(1.f, defaultElasticity + 0.05f);
                if (event.key.code == sf::Keyboard::Q) defaultElasticity = std::max(0.f, defaultElasticity - 0.05f);
                if (event.key.code == sf::Keyboard::Escape) selectedBall = -1;

            }

            if (event.type == sf::Event::MouseButtonPressed) {
                sf::Vector2i mouse = sf::Mouse::getPosition(window);
                if (event.mouseButton.button == sf::Mouse::Left) {
                    if (menuClickArea.contains((sf::Vector2f)mouse)) {
                        showLegend = !showLegend;
                    }
                    for (size_t i = 0; i < balls.size(); ++i) {
                        float dist = std::hypot(mouse.x - balls[i].shape.getPosition().x,
                                                mouse.y - balls[i].shape.getPosition().y);
                        if (dist <= balls[i].shape.getRadius()) {
                            dragging = true;
                            draggingIndex = i;
                            dragStartMouse = (sf::Vector2f)mouse;
                            dragStartBallPos = balls[i].shape.getPosition();
                            balls[i].velocity = {0.f, 0.f};
                            break;
                        }
                    }
                }
                if (event.mouseButton.button == sf::Mouse::Right) {
                    selectedBall = -1;
                    for (size_t i = 0; i < balls.size(); ++i) {
                        float dist = std::hypot(mouse.x - balls[i].shape.getPosition().x,
                                                mouse.y - balls[i].shape.getPosition().y);
                        if (dist <= balls[i].shape.getRadius()) {
                            selectedBall = i;
                            break;
                        }
                    }
                }
                if (event.mouseButton.button == sf::Mouse::Left) {
		    if (controlsClickArea.contains((sf::Vector2f)mouse)) {
			showControls = !showControls;
		    }
		}

            }

            if (event.type == sf::Event::MouseButtonReleased && event.mouseButton.button == sf::Mouse::Left) {
                if (dragging && draggingIndex >= 0) {
                    sf::Vector2f releaseMouse = (sf::Vector2f)sf::Mouse::getPosition(window);
                    sf::Vector2f throwVelocity = (releaseMouse - dragStartMouse) * 5.f;
                    balls[draggingIndex].velocity = throwVelocity;
                    balls[draggingIndex].lastThrowForce = throwVelocity;
                }
                dragging = false;
                draggingIndex = -1;
            }
        }

        if (!paused) {
	    for (size_t i = 0; i < balls.size(); ++i) {
		if (dragging && (int)i == draggingIndex) continue; // <- skip dragging ball
		balls[i].updateForces(gravity);
	    }

	    for (size_t i = 0; i < balls.size(); ++i) {
		for (size_t j = i + 1; j < balls.size(); ++j) {
		    if (dragging && ((int)i == draggingIndex || (int)j == draggingIndex)) continue;
		    resolveCollision(balls[i], balls[j]);
		}
	    }

	    for (size_t i = 0; i < balls.size(); ++i) {
		if (dragging && (int)i == draggingIndex) continue;
		balls[i].updatePhysics(dt, window);
		for (auto& plat : platforms)
		    handlePlatformCollision(balls[i], plat);
	    }
	}


        window.clear(sf::Color::Black);
        for (const auto& plat : platforms) window.draw(plat);

        for (const auto& ball : balls) {
            ball.drawTrajectory(window);
            window.draw(ball.shape);
        }

        if (selectedBall >= 0 && selectedBall < (int)balls.size()) {
            auto predictedPoints = simulateTrajectoryWithCollisions(
                balls[selectedBall], gravity, 5.f, 0.016f, platforms, window.getSize());

            sf::VertexArray prediction(sf::LineStrip, predictedPoints.size());
            for (size_t i = 0; i < predictedPoints.size(); ++i) {
                prediction[i].position = predictedPoints[i];
                prediction[i].color = sf::Color::Yellow;
            }
            window.draw(prediction);
        }

	if (paused || dragging) {  // <- include dragging
	    for (const auto& ball : balls) {
		drawArrow(window, ball.shape.getPosition(), ball.gravityForce * 0.01f, sf::Color::Blue);
		drawArrow(window, ball.shape.getPosition(), ball.dragForce * 0.5f, sf::Color::Red);
		drawArrow(window, ball.shape.getPosition(), ball.totalForce * 0.01f, sf::Color::White);
	    }

	    if (dragging && draggingIndex >= 0) {
		    sf::Vector2f currentMouse = (sf::Vector2f)sf::Mouse::getPosition(window);
		    sf::Vector2f ballPos = balls[draggingIndex].shape.getPosition();
		    sf::Vector2f dragVec = currentMouse - ballPos;

		    drawArrow(window, ballPos, dragVec, sf::Color::Green);
	    } else {
		for (const auto& ball : balls) {
		    if (ball.lastThrowForce != sf::Vector2f(0.f, 0.f))
		        drawArrow(window, ball.shape.getPosition(), ball.lastThrowForce * 0.05f, sf::Color::Green);
		}
	    }
	}


        sf::RectangleShape button(sf::Vector2f(20, 20));
        button.setPosition(menuBox.getPosition());
        button.setFillColor(showLegend ? sf::Color::White : sf::Color(100, 100, 100));
        window.draw(button);
	sf::RectangleShape controlsButton(sf::Vector2f(20, 20));
	controlsButton.setPosition(controlsBox.getPosition());
	controlsButton.setFillColor(showControls ? sf::Color::White : sf::Color(100, 100, 100));
	window.draw(controlsButton);

	if (showControls) {
	    window.draw(controlsBox);

	    sf::Text controlsText;
	    controlsText.setFont(font);
	    controlsText.setCharacterSize(14);
	    controlsText.setFillColor(sf::Color::White);

	    std::vector<std::string> controlLines = {
		"[A] - Add Ball",
		"[D] - Delete Ball",
		"[G]/[H] - Increase/Decrease Gravity",
		"[R]/[F] - Increase/Decrease Drag",
		"[E]/[Q] - Increase/Decrease Elasticity",
		"[Right Click] - Select Ball",
		"[Left Click + Drag] - Aim Throw",
		"[Space] - Pause/Resume Simulation",
		"[Esc] - Deselect Ball"
	    };

	    for (size_t i = 0; i < controlLines.size(); ++i) {
		controlsText.setString(controlLines[i]);
		controlsText.setPosition(controlsBox.getPosition().x + 10, controlsBox.getPosition().y + 10 + i * 18);
		window.draw(controlsText);
	    }
	}

        if (showLegend) {
            window.draw(menuBox);
            std::vector<std::pair<std::string, sf::Color>> legend = {
                {"Gravity", sf::Color::Blue},
                {"Drag", sf::Color::Red},
                {"Total Force", sf::Color::White},
                {"Throw", sf::Color::Green}
            };
            sf::Text label;
            label.setFont(font);
            label.setCharacterSize(14);
            for (size_t i = 0; i < legend.size(); ++i) {
                label.setString(legend[i].first);
                label.setPosition(menuBox.getPosition().x + 30, menuBox.getPosition().y + 10 + i * 20);
                label.setFillColor(sf::Color::White);
                window.draw(label);

                sf::RectangleShape colorBox(sf::Vector2f(15, 15));
                colorBox.setPosition(menuBox.getPosition().x + 10, menuBox.getPosition().y + 10 + i * 20);
                colorBox.setFillColor(legend[i].second);
                window.draw(colorBox);
            }
        }
	
        sf::Text info;
        std::ostringstream oss;
        oss << "Balls: " << balls.size()
            << " | Gravity: " << gravity.y
            << " | Drag: " << defaultDrag
            << " | Elasticity: " << defaultElasticity;
        /*
            << " | [A]dd [D]elete"
            << " | [Right Click] Select [Esc] Deselect"
            << " | [Space] Pause"
            << " | Click Box for Legend";
        */
        info.setFont(font);
        info.setCharacterSize(16);
        info.setFillColor(sf::Color::White);
        info.setPosition(10, 10);
        info.setString(oss.str());
        window.draw(info);
	
        window.display();
    }

    return 0;
}

