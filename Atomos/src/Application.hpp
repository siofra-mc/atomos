#include "stdint.h"
#include <string>

struct GLFWwindow;

void framebufferSizeCallback(GLFWwindow* window, int width, int height);

class Application {
    GLFWwindow* m_window;
    uint16_t m_screenWidth;
    uint16_t m_screenHeight;

    void init();
    void onDisplayWrapper();
    void processInput(GLFWwindow* window);
protected:
    virtual void onCreate() = 0;
    virtual void onDisplay() = 0;
    virtual void onUpdate(float deltaTime) = 0;

public:
    Application(uint16_t width, uint16_t height);
    ~Application();
    void run();
};
