#define _USE_MATH_DEFINES

#include <SFML/Graphics.hpp>
#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <string>
#include <Eigen/Dense>  // For solving Riccati equation

// Constants
const double KP = 0.8;
const double KI = 0.03;
const double KD = 0.1;
const double LQR_Q = 1.0;
const double LQR_R = 0.5;
const double MAX_VELOCITY = 3.0;
const float MIN_DT = 0.0001;
constexpr double DEG2RAD = M_PI / 180.0;
const float ARM_LENGTH1 = 150.f;
const float ARM_LENGTH2 = 100.f;
const int WINDOW_WIDTH = 800;
const int WINDOW_HEIGHT = 600;

// Motor Structure
struct Motor {
    double angle = 45;
    double velocity = 0;
    double integral_error = 0;
    void update(double control_signal, float dt) {
        dt = std::max(static_cast<float>(dt), MIN_DT);
        velocity = std::clamp(control_signal, -MAX_VELOCITY, MAX_VELOCITY);
        angle = std::clamp(angle + velocity * dt, 10.0, 170.0);
    }
};

// Function Declarations
std::vector<double> calculateTargetTrajectory(double start_pos, double end_pos, int num_points);
double measureCurrentPosition(const Motor& motor);
double calculateError(double target_pos, double current_pos);
double applyPIDController(double error, double integral, double derivative);
double applyLQRController(double state_vector, double control_input);
void initializeSystem();

// PID Controller
class PIDController {
    double target_angle = 21;
    double last_error = 0;
    double integral = 0;
public:
    void setTarget(double target) { target_angle = target; }
    double computeControl(double current_angle, float dt) {
        dt = std::max(static_cast<float>(dt), MIN_DT);
        double error = target_angle - current_angle;
        integral += error * dt;
        double derivative = (error - last_error) / dt;
        last_error = error;
        return KP * error + KI * integral + KD * derivative;
    }
};

// LQR Контроллер
class LQRController {
    Eigen::MatrixXd K;
public:
    LQRController() {
        Eigen::MatrixXd A(1, 1), B(1, 1), Q(1, 1), R(1, 1), P(1, 1);
        A << 1.0;
        B << 1.0;
        Q << LQR_Q;
        R << LQR_R;

        // Решение уравнения Риккатти итеративным методом
        P = Q;
        for (int i = 0; i < 100; ++i) {
            P = Q + A.transpose() * P * A - A.transpose() * P * B * (R + B.transpose() * P * B).inverse() * B.transpose() * P * A;
        }

        K = (R + B.transpose() * P * B).inverse() * B.transpose() * P * A;
    }
    double computeControl(double state) {
        return -K(0, 0) * state;
    }
};

void initializeSystem() {
    std::cout << "Initializing system..." << std::endl;
    std::cout << "PID Constants: KP=" << KP << " KI=" << KI << " KD=" << KD << std::endl;
    std::cout << "LQR Constants: Q=" << LQR_Q << " R=" << LQR_R << std::endl;
    std::cout << "Motion Constraints: MAX_VELOCITY=" << MAX_VELOCITY << " MIN_DT=" << MIN_DT << std::endl;
    std::cout << "Manipulator Arm: Length1=" << ARM_LENGTH1 << " Length2=" << ARM_LENGTH2 << std::endl;
    std::cout << "Window Size: WIDTH=" << WINDOW_WIDTH << " HEIGHT=" << WINDOW_HEIGHT << std::endl;
}

std::vector<double> calculateTargetTrajectory(double start_pos, double end_pos, int num_points) {
    std::vector<double> trajectory;
    double step = (end_pos - start_pos) / (num_points - 1);
    for (int i = 0; i < num_points; ++i) {
        trajectory.push_back(start_pos + i * step);
    }
    return trajectory;
}

double measureCurrentPosition(const Motor& motor) {
    return motor.angle;
}

double calculateError(double target_pos, double current_pos) {
    return target_pos - current_pos;
}

double applyPIDController(double error, double integral, double derivative) {
    return KP * error + KI * integral + KD * derivative;
}

double applyLQRController(double state_vector, double control_input) {
    return -LQR_Q / (LQR_Q + LQR_R) * state_vector;
}

int main() {
    initializeSystem();
    sf::RenderWindow window(sf::VideoMode({ WINDOW_WIDTH, WINDOW_HEIGHT }), "Two-Motor Manipulator");

    Motor motorA, motorB;
    PIDController pidControllerA, pidControllerB;
    LQRController lqrControllerA, lqrControllerB;

    std::vector<double> trajectory = calculateTargetTrajectory(45, 21, 10);
    double dt = 0.1;
    int step = 0;
    sf::Clock clock;

    // Create shapes for the arms
    sf::RectangleShape arm1(sf::Vector2f(ARM_LENGTH1, 10));
    arm1.setFillColor(sf::Color::Red);
    arm1.setOrigin({ 0, 5 });

    sf::RectangleShape arm2(sf::Vector2f(ARM_LENGTH2, 10));
    arm2.setFillColor(sf::Color::Blue);
    arm2.setOrigin({ 0, 5 });

    while (window.isOpen()) {
        sf::Time elapsed = clock.restart();
        float frameTime = std::max(elapsed.asSeconds(), MIN_DT);

        while (const std::optional event = window.pollEvent()) {
            if (event->is<sf::Event::Closed>()) {
                window.close();
            }
        }

        double target_angle = trajectory[std::min(step, (int)trajectory.size() - 1)];
        double current_posA = measureCurrentPosition(motorA);
        double current_posB = measureCurrentPosition(motorB);
        double errorA = calculateError(target_angle, current_posA);
        double errorB = calculateError(target_angle, current_posB);

        double control_signalA = pidControllerA.computeControl(motorA.angle, frameTime) + lqrControllerA.computeControl(motorA.angle - target_angle);
        double control_signalB = pidControllerB.computeControl(motorB.angle, frameTime) + lqrControllerB.computeControl(motorB.angle - target_angle);

        motorA.update(control_signalA, frameTime);
        motorB.update(control_signalB, frameTime);

        if (step % 10 == 0) {
            std::cout << "Motor1 Angle: " << motorA.angle << " | Motor2 Angle: " << motorB.angle << std::endl;
        }
        step++;
        sf::Vector2f basePosition(WINDOW_WIDTH / 2.f, WINDOW_HEIGHT / 2.f);

        // Update arm positions
        arm1.setPosition(basePosition);
        arm1.setRotation(sf::degrees(motorA.angle));

        sf::Vector2f arm1End(WINDOW_WIDTH / 2 + ARM_LENGTH1 * std::cos(motorA.angle * DEG2RAD),
            WINDOW_HEIGHT / 2 + ARM_LENGTH1 * std::sin(motorA.angle * DEG2RAD));
        arm2.setPosition(arm1End);

        arm2.setRotation(sf::degrees(motorA.angle + motorB.angle));


        // Clear the window
        window.clear();

        // Draw the arms
        window.draw(arm1);
        window.draw(arm2);

        // Display the window
        window.display();
    }
    return 0;
}