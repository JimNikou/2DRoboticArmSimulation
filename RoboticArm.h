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
void calculateArmAngles(float px, float py, float tx, float ty, float L1, float L2, float& angle1, float& angle2, bool& elbowUp);

// Function to draw a thick line between two points
void drawThickLine(sf::RenderWindow& window, float x1, float y1, float x2, float y2, sf::Color color, float thickness);

void drawClaw(sf::RenderWindow& window, float x, float y, float angle, float length, float width, sf::Color color);

void drawJoint(sf::RenderWindow& window, float x, float y);

void drawMinReachCircle(sf::RenderWindow& window, float x, float y, float L1, float L2);

void drawMaxReachCircle(sf::RenderWindow& window, float x, float y, float L1, float L2);

void drawZeroPoint(sf::RenderWindow& window, float x2, float y2);

void drawItem(int x, int y);

#endif // ROBOTICARM_HPP
