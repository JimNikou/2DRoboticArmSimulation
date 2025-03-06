#include <SFML/Graphics.hpp>
#include <iostream>
#include <cmath>

#include "RoboticArm.h"


int main() {
    sf::RenderWindow window(sf::VideoMode(800, 600), "Robotic Arm Simulation");

    // Set up initial parameters
    float gridSize = 10; // Grid size for visualization
    float px = 400, py = 300; // Pivot point at the center of the window
    float L1 = 100, L2 = 100; // Length of the arm segments

    float tx = px; // Target starts at the pivot
    float ty = py;

    float targetAngle1 = 0, targetAngle2 = 0; // Target arm angles
    float currentAngle1 = 0, currentAngle2 = 0; // Current animated arm angles

    float thickness = 3.0f; // Thickness of the arm
    float smoothFactor = 0.001f; // Factor for smooth movement


    float clawLength = 10.0f; // Length of the claw fingers
    float clawWidth = 2.5f;   // Width of the claw fingers

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

            if (event.type == sf::Event::MouseButtonPressed && event.mouseButton.button == sf::Mouse::Left) {
                int mouseX = event.mouseButton.x;
                int mouseY = event.mouseButton.y;

                // Calculate the distance from the pivot point
                float distance = std::sqrt((mouseX - px) * (mouseX - px) + (mouseY - py) * (mouseY - py));

                // Check if the click is within the minimum reach circle
                float minReach = std::max(0.0f, L1 - L2);
                if (distance < minReach) {
                    // If it's inside the minimum reachable area, set the target at the minimum distance
                    float angle = std::atan2(mouseY - py, mouseX - px);
                    tx = px + minReach * std::cos(angle);
                    ty = py + minReach * std::sin(angle);
                } else {
                    // Otherwise, set the target to the clicked position
                    tx = mouseX;
                    ty = mouseY;
                }

                std::cout << "New target set at (" << (tx - px) / gridSize << ", " << -(ty - py) / gridSize << ") in grid coordinates\n";

                // Calculate the new target angles
                calculateArmAngles(px, py, tx, ty, L1, L2, targetAngle1, targetAngle2);
            }

            if (event.type == sf::Event::KeyPressed && event.key.code == sf::Keyboard::M) {
                std::cout << "Enter new length for the upper arm (L1): ";
                std::cin >> L1; // Get new length for the upper arm
                std::cout << "Enter new length for the lower arm (L2): ";
                std::cin >> L2; // Get new length for the lower arm

                // Ensure the lengths are valid
                if (L1 <= 0 || L2 <= 0) {
                    std::cout << "Lengths must be positive numbers!" << std::endl;
                    L1 = 100; // Reset to default if invalid input
                    L2 = 100;
                } else {
                    std::cout << "Updated lengths - L1: " << L1 << ", L2: " << L2 << std::endl;
                }
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
        // Draw the claw at the end of the arm (second segment)
        drawClaw(window, x3, y3, currentAngle1 + currentAngle2, clawLength, clawWidth, sf::Color::Black);


        // Draw the minimum reach circle (radius L1 - L2)
        float minReach = std::max(0.0f, L1 - L2);  // Ensure non-negative minimum reach
        sf::CircleShape minReachCircle(minReach);
        minReachCircle.setFillColor(sf::Color::Transparent);
        minReachCircle.setOutlineColor(sf::Color::Black);
        minReachCircle.setOutlineThickness(1);
        minReachCircle.setPosition(px - minReach, py - minReach); // Center the circle at (px, py)
        window.draw(minReachCircle);

        // Draw the maximum reach circle (radius L1 + L2)
        float maxReach = L1 + L2; // Maximum reach is the sum of both arm segments
        sf::CircleShape maxReachCircle(maxReach);
        maxReachCircle.setFillColor(sf::Color::Transparent);
        maxReachCircle.setOutlineColor(sf::Color::Red);
        maxReachCircle.setOutlineThickness(1);
        maxReachCircle.setPosition(px - maxReach, py - maxReach); // Center the circle at (px, py)
        window.draw(maxReachCircle);

        window.display();
    }

    return 0;
}
