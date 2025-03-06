#include <SFML/Graphics.hpp>
#include <cmath>
#include <iostream>

// Function to draw a grid
void drawGrid(sf::RenderWindow& window, int width, int height, int gridSize) {
    sf::VertexArray grid(sf::Lines);

    for (int x = 0; x <= width; x += gridSize) {
        grid.append(sf::Vertex(sf::Vector2f(x, 0), sf::Color(200, 200, 200)));
        grid.append(sf::Vertex(sf::Vector2f(x, height), sf::Color(200, 200, 200)));
    }
    for (int y = 0; y <= height; y += gridSize) {
        grid.append(sf::Vertex(sf::Vector2f(0, y), sf::Color(200, 200, 200)));
        grid.append(sf::Vertex(sf::Vector2f(width, y), sf::Color(200, 200, 200)));
    }
    window.draw(grid);
}

// Function for linear interpolation
float lerp(float a, float b, float t) {
    return a + (b - a) * t;
}

// Function to calculate the angles for the robotic arm
void calculateArmAngles(float px, float py, float tx, float ty, float L1, float L2, float& angle1, float& angle2) {
    float dx = tx - px;
    float dy = ty - py;
    float d = std::sqrt(dx * dx + dy * dy);

    if (d > L1 + L2) {
        std::cout << "Target out of reach!" << std::endl;
        return;
    }

    // Inverse Kinematics using Law of Cosines
    float cos_angle2 = (d * d - L1 * L1 - L2 * L2) / (2 * L1 * L2);
    angle2 = std::acos(cos_angle2);

    float cos_angle1 = (L1 * L1 + d * d - L2 * L2) / (2 * L1 * d);
    float theta1 = std::acos(cos_angle1);
    float gamma = std::atan2(dy, dx);

    angle1 = gamma - theta1;
}

// Function to draw a thicker line
void drawThickLine(sf::RenderWindow& window, float x1, float y1, float x2, float y2, sf::Color color, float thickness) {
    sf::Vector2f direction(x2 - x1, y2 - y1);
    float length = std::sqrt(direction.x * direction.x + direction.y * direction.y);

    if (length == 0) return; // Avoid division by zero

    sf::RectangleShape line(sf::Vector2f(length, thickness));
    line.setFillColor(color);
    line.setOrigin(0, thickness / 2);
    line.setPosition(x1, y1);
    line.setRotation(std::atan2(direction.y, direction.x) * 180 / 3.14159f); // Convert radians to degrees
    window.draw(line);
}

int main() {
    sf::RenderWindow window(sf::VideoMode(800, 600), "Robotic Arm Simulation");

    // Set up initial parameters
    float gridSize = 10; // Increased for better visualization
    float px = 400, py = 300; // Pivot at the center of the window
    float L1 = 100, L2 = 100; // Length of arm segments

    float tx = px; // Target starts at the pivot
    float ty = py;

    float targetAngle1 = 0, targetAngle2 = 0; // Target arm angles
    float currentAngle1 = 0, currentAngle2 = 0; // Current animated arm angles

    float thickness = 3.0f; // Thickness of the arm
    float smoothFactor = 0.0005f; // Adjust for smooth movement

    while (window.isOpen()) {
        sf::Event event;
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed)
                window.close();

            // Enter target coordinates in grid squares (relative to center)
            if (event.type == sf::Event::KeyPressed && event.key.code == sf::Keyboard::C) {
                std::cout << "Enter new target coordinates (tx ty): ";
                float x, y;
                std::cin >> x >> y;

                // Convert from grid units to pixel coordinates
                tx = px + (x * gridSize); // Move left/right from center
                ty = py - (y * gridSize); // Move up/down from center (negative Y because screen origin is top-left)

                // Check if the new target is reachable
                float d = std::sqrt((tx - px) * (tx - px) + (ty - py) * (ty - py));
                if (d > L1 + L2) {
                    std::cout << "Target is out of reach! Try again.\n";
                } else {
                    std::cout << "New target set at (" << x << ", " << y << ") in grid coordinates\n";
                }

                // Calculate the new target angles
                calculateArmAngles(px, py, tx, ty, L1, L2, targetAngle1, targetAngle2);
            }

            // Click to set target (grid-aligned)
            if (event.type == sf::Event::MouseButtonPressed && event.mouseButton.button == sf::Mouse::Left) {
                int mouseX = event.mouseButton.x;
                int mouseY = event.mouseButton.y;

                // Align to grid
                tx = std::round((mouseX - px) / gridSize) * gridSize + px;
                ty = std::round((mouseY - py) / gridSize) * gridSize + py;

                std::cout << "New target set at (" << (tx - px) / gridSize << ", " << -(ty - py) / gridSize << ") in grid coordinates\n";

                // Calculate the new target angles
                calculateArmAngles(px, py, tx, ty, L1, L2, targetAngle1, targetAngle2);
            }
        }

        // Smoothly interpolate angles towards the target angles
        currentAngle1 = lerp(currentAngle1, targetAngle1, smoothFactor);
        currentAngle2 = lerp(currentAngle2, targetAngle2, smoothFactor);

        // Compute joint positions
        float x2 = px + L1 * std::cos(currentAngle1);
        float y2 = py + L1 * std::sin(currentAngle1);
        float x3 = x2 + L2 * std::cos(currentAngle1 + currentAngle2);
        float y3 = y2 + L2 * std::sin(currentAngle1 + currentAngle2);

        window.clear(sf::Color::White);
        drawGrid(window, 800, 600, gridSize);

        // Draw robotic arm with smooth transition
        drawThickLine(window, px, py, x2, y2, sf::Color::Blue, thickness); // Upper arm
        drawThickLine(window, x2, y2, x3, y3, sf::Color::Red, thickness);  // Lower arm

        window.display();
    }

    return 0;
}