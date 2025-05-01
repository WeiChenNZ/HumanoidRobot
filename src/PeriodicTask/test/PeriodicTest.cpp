#include "PeriodicTask.h"

void taskFunction(void)
{
    static int count = 0;
    std::cout << "Tick " << ++count << "\n";

    // Simulate a random overrun
    if (count % 4 == 0)
        std::this_thread::sleep_for(std::chrono::milliseconds(700)); // Overrun
    else
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
}

int main()
{
    PeriodicTask pt(150, taskFunction);
    pt.start();

    std::this_thread::sleep_for(std::chrono::seconds(3));
    pt.stop();
    std::this_thread::sleep_for(std::chrono::seconds(3));
    pt.start();
    std::this_thread::sleep_for(std::chrono::seconds(3));

    return 0;
}