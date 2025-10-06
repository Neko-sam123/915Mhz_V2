#ifndef SNAKEGAME_H
#define SNAKEGAME_H

#include <Arduino.h>
#include <Adafruit_SSD1306.h>
extern Adafruit_SSD1306 display;  

#define MAX_SNAKE_LENGTH 32
#define CELL_SIZE 2
#define SCREEN_WIDTH_PIXELS 128
#define SCREEN_HEIGHT_PIXELS 64

enum Direction { UP, DOWN, LEFT, RIGHT };

struct Point { int x; int y; };

class SnakeGame {
public:
    void begin();
    void update();
    void draw();
    void changeDirection(Direction dir);
    bool isGameOver();

private:
    Point snake[MAX_SNAKE_LENGTH];
    int snakeLength;
    Direction dir;
    Point food;
    bool gameOver;

    void moveSnake();
    void spawnFood();
    bool checkCollision(Point p);
};

#endif
