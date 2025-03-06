#ifndef ROBOTICARM_HPP
#define ROBOTICARM_HPP

#include <SFML/Graphics.hpp>
#include <cmath>
#include <iostream>

// Function to draw the grid on the window
void drawGrid(sf::RenderWindow& window, int width, int height, int gridSize);

// Function for linear interpolation between two values
float lerp(float a, float b, float t);

// Function to calculate the angles for the robotic arm's joints
void calculateArmAngles(float px, float py, float tx, float ty, float L1, float L2, float& angle1, float& angle2);

// Function to draw a thick line between two points
void drawThickLine(sf::RenderWindow& window, float x1, float y1, float x2, float y2, sf::Color color, float thickness);

#endif // ROBOTICARM_HPP
