#include <SFML/Graphics.hpp>
#include <iostream>
#include <cmath>
#include <vector>
#include "RoboticArm.h"

bool elbowUp = false;
std::vector<sf::CircleShape> items; // For future use if I want to add more Items
bool itemGrabbed = false;
sf::Vector2f grabbedItemOffset(0, 0);

void drawItem(int x, int y) {
    float radius = 5.0f;  // Set a visible size
    sf::CircleShape item(radius);
    item.setFillColor(sf::Color::Black);
    item.setOutlineColor(sf::Color::Black);
    item.setOutlineThickness(1.0f);
    item.setPosition(x - radius, y - radius);  // Center the item


    items.clear();
    items.push_back(item); // Store the item so it persists
}

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

    float thickness = 4.0f; // Thickness of the arm
    float smoothFactor = 0.001f; // Factor for smooth movement


    float clawLength = 10.0f; // Length of the claw fingers
    float clawWidth = 2.5f;   // Width of the claw fingers

    float grabDistance = 10.0f;

    while (window.isOpen()) {
        sf::Event event;
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed)
                window.close();

            // Enter target coordinates in grid squares (relative to center)
            if (event.type == sf::Event::KeyPressed && event.key.code == sf::Keyboard::P) {
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
                calculateArmAngles(px, py, tx, ty, L1, L2, targetAngle1, targetAngle2, elbowUp);

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
                calculateArmAngles(px, py, tx, ty, L1, L2, targetAngle1, targetAngle2, elbowUp);

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

            if (event.type == sf::Event::KeyPressed && event.key.code == sf::Keyboard::C) {
                std::cout << "Enter new zero point X: ";
                std::cin >> px;
                std::cout << "Enter new zero point Y: ";
                std::cin >> py;

                if (px < 0 || py < 0) {
                    std::cout << "Zero point must be positive number!" << std::endl;
                    px = 400; // Reset to default if invalid input
                    py = 300;
                } else {
                    std::cout << "Updated zero point - Px: " << px << ", Py: " << py << std::endl;
                }
            }

            if (event.type == sf::Event::MouseButtonPressed && event.mouseButton.button == sf::Mouse::Right) {
                int mouseX = event.mouseButton.x;
                int mouseY = event.mouseButton.y;
                itemGrabbed = false;
                drawItem(mouseX, mouseY);
                // std::cout << items[0].getPosition().x << " " << items[0].getPosition().y << std::endl;
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


        bool holdingItem = false;
        size_t heldItemIndex = -1;
        float itemSmoothFactor = 0.1f; // Smoothing factor for item movement

        if (!items.empty()) {
            sf::Vector2f itemPos = items[0].getPosition();
            float itemX = itemPos.x + items[0].getRadius();
            float itemY = itemPos.y + items[0].getRadius();
            float distToClaw = std::sqrt((itemX - x3) * (itemX - x3) + (itemY - y3) * (itemY - y3));

            if (distToClaw < grabDistance) {
                if (!itemGrabbed) {
                    itemGrabbed = true;
                    grabbedItemOffset.x = itemX - x3;
                    grabbedItemOffset.y = itemY - y3;
                }
            }

            if (itemGrabbed) {
                // Offset the item forward so it's not directly above the claw
                float offsetDistance = clawLength * 1.0f; // Move item slightly forward
                float clawTipX = x3 + offsetDistance * std::cos(currentAngle1 + currentAngle2);
                float clawTipY = y3 + offsetDistance * std::sin(currentAngle1 + currentAngle2);

                items[0].setPosition(clawTipX - items[0].getRadius(), clawTipY - items[0].getRadius());
            }

        }



        drawJoint(window, x2, y2);

        // Draw the minimum reach circle (radius L1 - L2)
        drawMinReachCircle(window, px, py, L1, L2);

        // Draw the maximum reach circle (radius L1 + L2)
        drawMaxReachCircle(window, px, py, L1, L2);

        drawZeroPoint(window, px, py);

        for (const auto& item : items) {
            window.draw(item);
        }

        window.display();
    }

    return 0;
}
