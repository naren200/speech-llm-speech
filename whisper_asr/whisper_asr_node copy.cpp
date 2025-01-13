#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <whisper.h>
#include <memory>
#include <string>
#include <vector>
#include <fstream>
#include <cstring>
#include <portaudio.h>

class WhisperASRNode : public rclcpp::Node {
public:
    WhisperASRNode() : Node("whisper_asr_node") {
        // Initialize publisher
        publisher_ = this->create_publisher<std_msgs::msg::String>("recognized_speech", 10);
        
        // Get audio source from environment
        const char* audio_source = std::getenv("AUDIO_SOURCE");
        const char* audio_file = std::getenv("AUDIO_FILE");
        
        // Initialize Whisper
        ctx_ = whisper_init_from_file("ggml-base.en.bin");
        if (!ctx_) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize Whisper model");
            throw std::runtime_error("Whisper initialization failed");
        }   

        // Set up processing timer
        if (audio_source && strcmp(audio_source, "microphone") == 0) {
            setupMicrophone();
            timer_ = this->create_wall_timer(
                std::chrono::milliseconds(100),
                std::bind(&WhisperASRNode::processMicrophoneAudio, this));
        } else if (audio_file) {
            timer_ = this->create_wall_timer(
                std::chrono::milliseconds(100),
                std::bind(&WhisperASRNode::processAudioFile, this));
        } else {
            RCLCPP_ERROR(this->get_logger(), "No audio source specified");
            throw std::runtime_error("Invalid audio source configuration");
        }
    }

    ~WhisperASRNode() {
        if (ctx_) {
            whisper_free(ctx_);
        }
        if (stream_) {
            Pa_CloseStream(stream_);
            Pa_Terminate();
        }
    }

private:
    void setupMicrophone() {
        PaError err = Pa_Initialize();
        if (err != paNoError) {
            RCLCPP_ERROR(this->get_logger(), "PortAudio initialization failed");
            throw std::runtime_error(Pa_GetErrorText(err));
        }

        err = Pa_OpenDefaultStream(&stream_,
            1,          // input channels
            0,          // output channels
            paFloat32,  // sample format
            16000,      // sample rate
            1024,       // frames per buffer
            nullptr, nullptr);
        
        if (err != paNoError) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open audio stream");
            throw std::runtime_error(Pa_GetErrorText(err));
        }

        err = Pa_StartStream(stream_);
        if (err != paNoError) {
            RCLCPP_ERROR(this->get_logger(), "Failed to start audio stream");
            throw std::runtime_error(Pa_GetErrorText(err));
        }
    }

    void processMicrophoneAudio() {
        try {
            std::vector<float> audio_data(16000);  // 1 second of audio at 16kHz
            Pa_ReadStream(stream_, audio_data.data(), 16000);
            processAudio(audio_data);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Microphone processing error: %s", e.what());
        }
    }

    void processAudioFile() {
        try {
            const char* audio_file = std::getenv("AUDIO_FILE");
            if (!audio_file) {
                RCLCPP_ERROR(this->get_logger(), "No audio file specified");
                return;
            }

            std::vector<float> audio_data = loadAudioFile(audio_file);
            processAudio(audio_data);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "File processing error: %s", e.what());
        }
    }

    void processAudio(const std::vector<float>& audio_data) {
        whisper_full_params params = whisper_full_default_params(WHISPER_SAMPLING_GREEDY);
        params.print_realtime = true;
        params.print_progress = false;
        params.language = "en";

        if (whisper_full(ctx_, params, audio_data.data(), audio_data.size()) != 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to process audio");
            return;
        }

        const int n_segments = whisper_full_n_segments(ctx_);
        for (int i = 0; i < n_segments; ++i) {
            const char* text = whisper_full_get_segment_text(ctx_, i);
            auto msg = std_msgs::msg::String();
            msg.data = text;
            publisher_->publish(msg);
            RCLCPP_INFO(this->get_logger(), "Published: '%s'", text);
        }
    }

    std::vector<float> loadAudioFile(const char* filename) {
        // Basic WAV file reading implementation
        std::ifstream file(filename, std::ios::binary);
        if (!file.is_open()) {
            throw std::runtime_error("Failed to open audio file");
        }

        // Read WAV header and data
        // This is a simplified implementation - you should add proper WAV parsing
        std::vector<float> audio_data(16000);  // Placeholder size
        file.read(reinterpret_cast<char*>(audio_data.data()), audio_data.size() * sizeof(float));
        return audio_data;
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    struct whisper_context* ctx_;
    PaStream* stream_ = nullptr;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WhisperASRNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
