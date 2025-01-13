#include <iostream>
#include <cstdlib>

void playAudio(const std::string& filePath) {
    std::string command = "aplay " + filePath; // Use 'aplay' for Linux-based systems
    int ret = std::system(command.c_str());
    if (ret != 0) {
        std::cerr << "Error playing audio file: " << filePath << std::endl;
    }
}
