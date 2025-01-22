#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <cstdlib>
#include <string>
#include <memory>

class GoogleTTSNode : public rclcpp::Node {
private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    
    bool synthesizeSpeech(const std::string& text, const std::string& outputFile) {
        // Remove newlines and escape quotes
        std::string sanitized = text;
        sanitized.erase(std::remove(sanitized.begin(), sanitized.end(), '\n'), sanitized.end());
        std::replace(sanitized.begin(), sanitized.end(), '"', '\'');
        
        std::string command = "python3 /root/ros2_ws/src/speech-llm-speech/google_tts/scripts/gtts_synthesizer.py \"" 
                            + sanitized + "\" \"" + outputFile + "\"";
        return (std::system(command.c_str()) == 0);
    }
    
    void textToSpeakCallback(const std_msgs::msg::String::SharedPtr msg) {
        std::string text = msg->data;
        std::string outputFile = "/root/ros2_ws/src/speech-llm-speech/google_tts/synthesized_speech.wav";

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
            "/text_to_speech", 10,
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
