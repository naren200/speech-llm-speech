#include <cstdlib>
#include <string>

bool synthesizeSpeech(const std::string& text, const std::string& outputFile) {
    std::string command = "python3 scripts/gtts_synthesizer.py \"" + text + "\" " + outputFile;
    return (std::system(command.c_str()) == 0);
}
