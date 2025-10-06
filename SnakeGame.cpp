#include "SnakeGame.h"

void SnakeGame::begin() {
    snakeLength = 3;
    snake[0] = {SCREEN_WIDTH_PIXELS/2, SCREEN_HEIGHT_PIXELS/2};
    snake[1] = {snake[0].x - CELL_SIZE, snake[0].y};
    snake[2] = {snake[1].x - CELL_SIZE, snake[1].y};
    dir = RIGHT;
    gameOver = false;
    spawnFood();
}

void SnakeGame::update() {
    if (gameOver) return;
    moveSnake();

    if (snake[0].x == food.x && snake[0].y == food.y) {
        if (snakeLength < MAX_SNAKE_LENGTH) snakeLength++;
        spawnFood();
    }
}

void SnakeGame::draw() {
    display.clearDisplay();
    for (int i = 0; i < snakeLength; i++)
        display.fillRect(snake[i].x, snake[i].y, CELL_SIZE, CELL_SIZE, WHITE);
    display.fillRect(food.x, food.y, CELL_SIZE, CELL_SIZE, WHITE);
    display.display();
}

void SnakeGame::changeDirection(Direction newDir) {
    if ((dir == UP && newDir == DOWN) || (dir == DOWN && newDir == UP) ||
        (dir == LEFT && newDir == RIGHT) || (dir == RIGHT && newDir == LEFT)) return;
    dir = newDir;
}

bool SnakeGame::isGameOver() { return gameOver; }

void SnakeGame::moveSnake() {
    for (int i = snakeLength - 1; i > 0; i--) snake[i] = snake[i - 1];
    switch (dir) {
        case UP: snake[0].y -= CELL_SIZE; break;
        case DOWN: snake[0].y += CELL_SIZE; break;
        case LEFT: snake[0].x -= CELL_SIZE; break;
        case RIGHT: snake[0].x += CELL_SIZE; break;
    }

    if (snake[0].x < 0 || snake[0].x >= SCREEN_WIDTH_PIXELS ||
        snake[0].y < 0 || snake[0].y >= SCREEN_HEIGHT_PIXELS) gameOver = true;

    for (int i = 1; i < snakeLength; i++)
        if (snake[0].x == snake[i].x && snake[0].y == snake[i].y) gameOver = true;
}

void SnakeGame::spawnFood() {
    bool valid = false;
    while (!valid) {
        food.x = random(0, SCREEN_WIDTH_PIXELS / CELL_SIZE) * CELL_SIZE;
        food.y = random(0, SCREEN_HEIGHT_PIXELS / CELL_SIZE) * CELL_SIZE;

        valid = true;
        for (int i = 0; i < snakeLength; i++)
            if (snake[i].x == food.x && food.y == snake[i].y) valid = false;
    }
}
