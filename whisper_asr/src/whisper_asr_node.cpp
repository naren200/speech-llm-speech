#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <memory>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>
#include <sstream>
#include <algorithm>
#include <cstdlib>
#include <unistd.h>

class WhisperASRNode : public rclcpp::Node {
public:
    WhisperASRNode() : Node("whisper_asr_node") {
        publisher_ = this->create_publisher<std_msgs::msg::String>("recognized_speech", 10);
        
        // Get parameters
        model_path_ = this->declare_parameter("model_path", 
            "/root/ros2_ws/src/speech-llm-speech/whisper_asr/model/ggml-base.en.bin");
        whisper_cli_path_ = this->declare_parameter("whisper_cli_path", 
            "/root/ros2_ws/src/speech-llm-speech/whisper.cpp/build/bin/whisper-cli");
        
        // Process audio when initialized
        processAudio();
    }

private:
    std::string get_sample_rate(const std::string& filename) {
        std::string cmd = "ffprobe -v error -select_streams a:0 -show_entries stream=sample_rate -of default=noprint_wrappers=1:nokey=1 " + filename;
        char buffer[128];
        std::string result = "";
        FILE* pipe = popen(cmd.c_str(), "r");
        if (!pipe) throw std::runtime_error("Failed to execute ffprobe");
        
        while (fgets(buffer, sizeof(buffer), pipe) != nullptr) {
            result += buffer;
        }
        pclose(pipe);
        
        // Remove trailing newline
        if (!result.empty() && result.back() == '\n') {
            result.pop_back();
        }
        return result;
    }

    std::string convert_to_16k(const std::string& input_file) {
        // Create temporary output file
        char tmp_template[] = "/tmp/whisper_XXXXXX.wav";
        int fd = mkstemps(tmp_template, 4); // Create .wav file
        if (fd == -1) throw std::runtime_error("Failed to create temp file");
        close(fd);
        
        std::string output_file(tmp_template);
        
        std::string cmd = "ffmpeg -y -i " + input_file + 
                         " -acodec pcm_s16le -ac 1 -ar 16000 " + output_file + 
                         " > /dev/null 2>&1";
                         
        int result = system(cmd.c_str());
        if (result != 0) {
            unlink(output_file.c_str());
            throw std::runtime_error("Audio conversion failed");
        }
        
        return output_file;
    }

    std::string cleanTranscript(const std::string& raw_output) {
        std::stringstream ss(raw_output);
        std::string line;
        std::string cleaned;
        
        while (std::getline(ss, line)) {
            size_t end_bracket = line.find(']');
            if (end_bracket != std::string::npos && end_bracket + 2 < line.size()) {
                std::string text = line.substr(end_bracket + 2);
                text.erase(text.begin(), std::find_if(text.begin(), text.end(), [](int ch) {
                    return !std::isspace(ch);
                }));
                text.erase(std::find_if(text.rbegin(), text.rend(), [](int ch) {
                    return !std::isspace(ch);
                }).base(), text.end());
                
                if (!text.empty()) {
                    cleaned += text + " ";
                }
            }
        }
        
        if (!cleaned.empty() && cleaned.back() == ' ') {
            cleaned.pop_back();
        }
        
        return cleaned;
    }

    void processAudio() {
        const char* audio_file = std::getenv("AUDIO_FILE");
        if (!audio_file) {
            RCLCPP_ERROR(this->get_logger(), "AUDIO_FILE environment variable not set");
            return;
        }

        std::string actual_audio_file = audio_file;
        bool converted = false;

        try {
            // Check sample rate
            std::string sample_rate = get_sample_rate(audio_file);
            if (sample_rate != "16000") {
                RCLCPP_INFO(this->get_logger(), "Converting audio from %sHz to 16000Hz", sample_rate.c_str());
                actual_audio_file = convert_to_16k(audio_file);
                converted = true;
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Audio check/conversion failed: %s", e.what());
            return;
        }

        // Build command
        std::string command = whisper_cli_path_ + " -m " + model_path_ + " " + actual_audio_file;
        RCLCPP_INFO(this->get_logger(), "Executing: %s", command.c_str());

        // Execute command and capture output
        char buffer[128];
        std::string result = "";
        FILE* pipe = popen(command.c_str(), "r");
        if (!pipe) {
            RCLCPP_ERROR(this->get_logger(), "Failed to execute command");
            if (converted) unlink(actual_audio_file.c_str());
            return;
        }

        while (fgets(buffer, sizeof(buffer), pipe) != nullptr) {
            result += buffer;
        }
        pclose(pipe);

        // Clean up converted file if we created one
        if (converted) {
            unlink(actual_audio_file.c_str());
        }

        // Clean the output
        std::string cleaned_text = cleanTranscript(result);

        // Publish cleaned result
        if (!cleaned_text.empty()) {
            auto msg = std_msgs::msg::String();
            msg.data = cleaned_text;
            publisher_->publish(msg);
            RCLCPP_INFO(this->get_logger(), "Published cleaned text: '%s'", msg.data.c_str());
        } else {
            RCLCPP_WARN(this->get_logger(), "No speech recognized in audio file");
        }
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    std::string model_path_;
    std::string whisper_cli_path_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WhisperASRNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}