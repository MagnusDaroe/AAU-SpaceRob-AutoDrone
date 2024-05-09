#include <chrono>
#include <iostream>
#include <thread>

int main() {
    std::chrono::system_clock::time_point time_start = std::chrono::system_clock::now();
    std::this_thread::sleep_for(std::chrono::seconds(2));
    std::chrono::system_clock::time_point time_stop = std::chrono::system_clock::now();

    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(time_stop - time_start).count();

    std::cout << "Duration: " << duration << " ms" << std::endl;

    std::time_t time_start_t = std::chrono::system_clock::to_time_t(time_start);
    std::cout << "Time start: " << std::ctime(&time_start_t);

    return 0;
}

