#define _USE_MATH_DEFINES

#include <SFML/Graphics.hpp>
#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <string>

// Constants
const float ARM_LENGTH1 = 150.f;
const float ARM_LENGTH2 = 100.f;
const float ARM_LENGTH3 = 75.f;
const int WINDOW_WIDTH = 800;
const int WINDOW_HEIGHT = 600;
const float MIN_DT = 0.0001;
constexpr double DEG2RAD = M_PI / 180.0;

// Motor Structure
struct Motor {
    double angle = 45;
    double velocity = 0;
    double integral_error = 0;
    void update(double control_signal, float dt) {
        dt = std::max(static_cast<float>(dt), MIN_DT);
        velocity = std::clamp(control_signal, -3.0, 3.0);
        angle = std::clamp(angle + velocity * dt, 10.0, 170.0);
    }
};

// Function to calculate forward kinematics
sf::Vector2f forwardKinematics(const std::vector<Motor>& motors) {
    float angle1 = motors[0].angle * DEG2RAD;
    float angle2 = motors[1].angle * DEG2RAD;
    float angle3 = motors[2].angle * DEG2RAD;

    float x = ARM_LENGTH1 * std::cos(angle1) +
        ARM_LENGTH2 * std::cos(angle1 + angle2) +
        ARM_LENGTH3 * std::cos(angle1 + angle2 + angle3);
    float y = ARM_LENGTH1 * std::sin(angle1) +
        ARM_LENGTH2 * std::sin(angle1 + angle2) +
        ARM_LENGTH3 * std::sin(angle1 + angle2 + angle3);

    return sf::Vector2f(x, y);
}

// Function to apply CCD algorithm
void applyCCD(std::vector<Motor>& motors, const sf::Vector2f& targetPosition) {
    const int maxIterations = 20;
    for (int iteration = 0; iteration < maxIterations; ++iteration) {
        for (int i = motors.size() - 1; i >= 0; --i) {
            sf::Vector2f currentPosition = forwardKinematics(motors);
            sf::Vector2f toTarget = targetPosition - currentPosition;
            float distance = std::sqrt(toTarget.x * toTarget.x + toTarget.y * toTarget.y);

            if (distance < 1e-3) {
                continue;
            }

            float targetAngle = std::atan2(toTarget.y, toTarget.x);
            float currentAngle = 0;

            for (int j = 0; j <= i; ++j) {
                currentAngle += motors[j].angle * DEG2RAD;
            }

            float angleDiff = targetAngle - currentAngle;
            motors[i].angle += angleDiff / DEG2RAD;
        }
    }
}

int main() {
    sf::RenderWindow window(sf::VideoMode({ WINDOW_WIDTH, WINDOW_HEIGHT }), "Three-Motor Manipulator with CCD");

    std::vector<Motor> motors = { Motor(), Motor(), Motor() };
    sf::Vector2f targetPosition(200.f, 150.f);

    sf::Clock clock;

    // Create shapes for the arms
    sf::RectangleShape arm1(sf::Vector2f(ARM_LENGTH1, 10));
    arm1.setFillColor(sf::Color::Red);
    arm1.setOrigin({ 0, 5 });

    sf::RectangleShape arm2(sf::Vector2f(ARM_LENGTH2, 10));
    arm2.setFillColor(sf::Color::Blue);
    arm2.setOrigin({ 0, 5 });

    sf::RectangleShape arm3(sf::Vector2f(ARM_LENGTH3, 10));
    arm3.setFillColor(sf::Color::Green);
    arm3.setOrigin({ 0, 5 });

    while (window.isOpen()) {
        sf::Time elapsed = clock.restart();
        float frameTime = std::max(elapsed.asSeconds(), MIN_DT);

        while (const std::optional event = window.pollEvent()) {
            if (event->is<sf::Event::Closed>()) {
                window.close();
            }
        }


        // Apply CCD algorithm
        applyCCD(motors, targetPosition);

        // Debug output for end effector position and motor angles
        sf::Vector2f endEffectorPosition = forwardKinematics(motors);
        std::cout << "End Effector Position: (" << endEffectorPosition.x << ", " << endEffectorPosition.y << ")" << std::endl;
        std::cout << "Motor Angles: " << motors[0].angle << ", " << motors[1].angle << ", " << motors[2].angle << std::endl;

        // Update arm positions
        sf::Vector2f basePosition(WINDOW_WIDTH / 2.f, WINDOW_HEIGHT / 2.f);
        arm1.setPosition(basePosition);
        arm1.setRotation(sf::degrees(motors[0].angle));

        sf::Vector2f arm1End(basePosition.x + ARM_LENGTH1 * std::cos(motors[0].angle * DEG2RAD),
            basePosition.y + ARM_LENGTH1 * std::sin(motors[0].angle * DEG2RAD));
        arm2.setPosition(arm1End);
        arm2.setRotation(sf::degrees(motors[0].angle + motors[1].angle));

        sf::Vector2f arm2End(arm1End.x + ARM_LENGTH2 * std::cos((motors[0].angle + motors[1].angle) * DEG2RAD),
            arm1End.y + ARM_LENGTH2 * std::sin((motors[0].angle + motors[1].angle) * DEG2RAD));
        arm3.setPosition(arm2End);
        arm3.setRotation(sf::degrees(motors[0].angle + motors[1].angle + motors[2].angle));

        // Clear the window
        window.clear();

        // Draw the arms
        window.draw(arm1);
        window.draw(arm2);
        window.draw(arm3);

        // Display the window
        window.display();
    }
    return 0;
}
