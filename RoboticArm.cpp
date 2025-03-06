#include "RoboticArm.h"

/**
 * Function to draw the grid on the window.
 *
 * This function draws a grid using the specified grid size, width, and height.
 * It draws vertical and horizontal lines to create the grid layout.
 *
 * @param window The window where the grid will be drawn.
 * @param width The width of the grid (in pixels).
 * @param height The height of the grid (in pixels).
 * @param gridSize The size of each grid square (in pixels).
 * @return none
 */
void drawGrid(sf::RenderWindow& window, int width, int height, int gridSize) {
    sf::VertexArray grid(sf::Lines);

    // Draw vertical grid lines
    for (int x = 0; x <= width; x += gridSize) {
        grid.append(sf::Vertex(sf::Vector2f(x, 0), sf::Color(200, 200, 200)));
        grid.append(sf::Vertex(sf::Vector2f(x, height), sf::Color(200, 200, 200)));
    }

    // Draw horizontal grid lines
    for (int y = 0; y <= height; y += gridSize) {
        grid.append(sf::Vertex(sf::Vector2f(0, y), sf::Color(200, 200, 200)));
        grid.append(sf::Vertex(sf::Vector2f(width, y), sf::Color(200, 200, 200)));
    }

    // Draw the grid on the window
    window.draw(grid);
}

/**
 * Function for linear interpolation between two values.
 *
 * This function computes a linear interpolation between two values based on the interpolation factor 't'.
 *
 * @param a The start value.
 * @param b The end value.
 * @param t The interpolation factor (between 0 and 1).
 * @return The interpolated value.
 */
float lerp(float a, float b, float t) {
    return a + (b - a) * t;
}

/**
 * Function to calculate the angles for the robotic arm's joints.
 *
 * This function uses inverse kinematics and the Law of Cosines to calculate
 * the angles required for the robotic arm to reach a target point.
 *
 * @param px The x-coordinate of the arm's pivot point.
 * @param py The y-coordinate of the arm's pivot point.
 * @param tx The x-coordinate of the target point.
 * @param ty The y-coordinate of the target point.
 * @param L1 The length of the first segment of the arm.
 * @param L2 The length of the second segment of the arm.
 * @param angle1 The calculated angle for the first joint (output).
 * @param angle2 The calculated angle for the second joint (output).
 * @return none
 */
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

/**
 * Function to draw a thick line between two points.
 *
 * This function draws a line between two points with a specified color and thickness.
 * It uses a rectangle shape to simulate a thick line.
 *
 * @param window The window where the line will be drawn.
 * @param x1 The x-coordinate of the starting point.
 * @param y1 The y-coordinate of the starting point.
 * @param x2 The x-coordinate of the ending point.
 * @param y2 The y-coordinate of the ending point.
 * @param color The color of the line.
 * @param thickness The thickness of the line.
 * @return none
 */
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

/**
 * Function to draw a claw at the end of the arm.
 *
 * This function draws a simple claw mechanism at the end of the second arm segment.
 * The claw consists of two "fingers" represented as thick lines rotated to face the target.
 *
 * @param window The window where the claw will be drawn.
 * @param x The x-coordinate of the center of the claw.
 * @param y The y-coordinate of the center of the claw.
 * @param angle The angle of the arm (used to rotate the claw to align with the arm).
 * @param length The length of each claw finger.
 * @param width The width (thickness) of the claw fingers.
 * @param color The color of the claw.
 * @return none
 */
void drawClaw(sf::RenderWindow& window, float x, float y, float angle, float length, float width, sf::Color color) {
    // Create two lines for the claw fingers
    // Both fingers should have the same length and be rotated symmetrically
    sf::Vector2f claw1(x + length * std::cos(angle - M_PI_4), y + length * std::sin(angle - M_PI_4)); // First finger
    sf::Vector2f claw2(x + length * std::cos(angle + M_PI_4), y + length * std::sin(angle + M_PI_4)); // Second finger

    // Draw both fingers with the same length
    drawThickLine(window, x, y, claw1.x, claw1.y, color, width); // First "finger"
    drawThickLine(window, x, y, claw2.x, claw2.y, color, width); // Second "finger"
}


void drawJoint(sf::RenderWindow& window, float x, float y) {
    float offset = 7;
    sf::CircleShape joint(offset);
    joint.setFillColor(sf::Color::Black);
    joint.setOutlineColor(sf::Color::Black);
    joint.setPosition(x-offset, y-offset);
    window.draw(joint);
}

void drawMinReachCircle(sf::RenderWindow& window, float x, float y, float L1, float L2) {
    float minReach = std::max(0.0f, L1 - L2);  // Ensure non-negative minimum reach
    sf::CircleShape minReachCircle(minReach);
    minReachCircle.setFillColor(sf::Color::Transparent);
    minReachCircle.setOutlineColor(sf::Color::Black);
    minReachCircle.setOutlineThickness(1);
    minReachCircle.setPosition(x - minReach, y - minReach); // Center the circle at (px, py)
    window.draw(minReachCircle);
}

void drawMaxReachCircle(sf::RenderWindow& window, float x, float y, float L1, float L2) {
    float maxReach = L1 + L2; // Maximum reach is the sum of both arm segments
    sf::CircleShape maxReachCircle(maxReach);
    maxReachCircle.setFillColor(sf::Color::Transparent);
    maxReachCircle.setOutlineColor(sf::Color::Red);
    maxReachCircle.setOutlineThickness(1);
    maxReachCircle.setPosition(x - maxReach, y - maxReach); // Center the circle at (px, py)
    window.draw(maxReachCircle);
}

void drawZeroPoint(sf::RenderWindow& window, float x2, float y2) {
    float offset = 7;
    sf::CircleShape joint(offset);
    joint.setFillColor(sf::Color::Black);
    joint.setOutlineColor(sf::Color::Black);
    joint.setPosition(x2-offset, y2-offset);
    window.draw(joint);
}