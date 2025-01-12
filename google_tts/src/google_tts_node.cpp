#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <cstdlib>
#include <string>
#include <memory>

class GoogleTTSNode : public rclcpp::Node {
private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    
    bool synthesizeSpeech(const std::string& text, const std::string& outputFile) {
        std::string command = "python3 src/speech-llm-speech/google_tts/scripts/gtts_synthesizer.py \"" + text + "\" " + outputFile;
        int ret = std::system(command.c_str());
        return ret == 0;
    }

    void textToSpeakCallback(const std_msgs::msg::String::SharedPtr msg) {
        std::string text = msg->data;
        std::string outputFile = "src/speech-llm-speech/google_tts/output/synthesized_speech.wav";

        RCLCPP_INFO(this->get_logger(), "Received text to synthesize: %s", text.c_str());

        if (synthesizeSpeech(text, outputFile)) {
            RCLCPP_INFO(this->get_logger(), "Speech synthesis successful. Saved to %s", outputFile.c_str());
            // Optionally: Play the audio file here
            // playAudio(outputFile);  // Implement in utils/audio_player.cpp
        } else {
            RCLCPP_ERROR(this->get_logger(), "Speech synthesis failed!");
        }
    }

public:
    GoogleTTSNode() : Node("google_tts_node") {
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "/text_to_speak", 10,
            std::bind(&GoogleTTSNode::textToSpeakCallback, this, std::placeholders::_1));
        
        RCLCPP_INFO(this->get_logger(), "Google TTS Node is running...");
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GoogleTTSNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
