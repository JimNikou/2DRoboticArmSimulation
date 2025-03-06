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
void calculateArmAngles(float px, float py, float tx, float ty, float L1, float L2, float& angle1, float& angle2, bool& elbowUp) {
    // Calculate the distance to the target
    float dx = tx - px;
    float dy = ty - py;
    float distance = std::sqrt(dx * dx + dy * dy);

    // Check if the target is within the reachable area
    if (distance > L1 + L2 || distance < std::abs(L1 - L2)) {
        std::cout << "Target is out of reach!\n";
        return;
    }

    // Calculate the cosine of angle2 using the law of cosines
    float cosAngle2 = (dx * dx + dy * dy - L1 * L1 - L2 * L2) / (2 * L1 * L2);
    if (cosAngle2 < -1 || cosAngle2 > 1) {
        std::cout << "Invalid target position\n";
        return;
    }

    // Calculate both possible angles for angle2 (elbow-up and elbow-down)
    float angle2_ElbowUp = std::acos(cosAngle2);   // Elbow-up configuration
    float angle2_ElbowDown = -std::acos(cosAngle2); // Elbow-down configuration

    // Calculate the first angle (angle1) for both configurations
    float k1_ElbowUp = L1 + L2 * std::cos(angle2_ElbowUp);
    float k2_ElbowUp = L2 * std::sin(angle2_ElbowUp);
    float angle1_ElbowUp = std::atan2(dy, dx) - std::atan2(k2_ElbowUp, k1_ElbowUp);

    float k1_ElbowDown = L1 + L2 * std::cos(angle2_ElbowDown);
    float k2_ElbowDown = L2 * std::sin(angle2_ElbowDown);
    float angle1_ElbowDown = std::atan2(dy, dx) - std::atan2(k2_ElbowDown, k1_ElbowDown);

    // Choose the configuration that minimizes the total angular movement
    float angleDiff_ElbowUp = std::abs(angle1_ElbowUp - angle1);
    float angleDiff_ElbowDown = std::abs(angle1_ElbowDown - angle1);

    // If elbow-down configuration results in a smaller total movement, use it
    if (angleDiff_ElbowDown < angleDiff_ElbowUp) {
        angle1 = angle1_ElbowDown;
        angle2 = angle2_ElbowDown;
        elbowUp = false; // Switch to elbow-down configuration
    } else {
        angle1 = angle1_ElbowUp;
        angle2 = angle2_ElbowUp;
        elbowUp = true; // Keep elbow-up configuration
    }
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

