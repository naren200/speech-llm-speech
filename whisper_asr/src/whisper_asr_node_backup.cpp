#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <whisper.h>
#include <memory>
#include <string>
#include <vector>
#include <fstream>
#include <cstring>

class WhisperASRNode : public rclcpp::Node {
public:
    WhisperASRNode() : Node("whisper_asr_node") {
        // Initialize publisher
        publisher_ = this->create_publisher<std_msgs::msg::String>("recognized_speech", 10);
        
        // Get audio file from environment
        const char* audio_file = std::getenv("AUDIO_FILE");
        
        // Initialize Whisper context parameters
        whisper_context_params context_params = whisper_context_default_params();
        
        // Initialize Whisper
        ctx_ = whisper_init_from_file_with_params("src/speech-llm-speech/whisper_asr/model/ggml-base.en.bin", context_params);
        if (!ctx_) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize Whisper model");
            throw std::runtime_error("Whisper initialization failed");
        }


        // Process audio file if specified
        if (audio_file) {
            timer_ = this->create_wall_timer(
                std::chrono::milliseconds(100),
                std::bind(&WhisperASRNode::processAudioFile, this));
        } else {
            RCLCPP_ERROR(this->get_logger(), "No audio file specified");
            throw std::runtime_error("Invalid audio file configuration");
        }
    }

    ~WhisperASRNode() {
        if (ctx_) {
            whisper_free(ctx_);
        }
    }

private:
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
        
        // Set essential parameters
        params.print_realtime = true;
        params.print_progress = false;
        params.print_timestamps = true;
        params.print_special = false;
        params.translate = false;
        params.language = "en";
        params.n_threads = 4;
        params.offset_ms = 0;
        params.duration_ms = 0;

        // Process audio
        if (whisper_full(ctx_, params, audio_data.data(), audio_data.size()) != 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to process audio");
            return;
        }

        // Get results
        const int n_segments = whisper_full_n_segments(ctx_);
        std::cout<<"n_segments: "<<n_segments<<std::endl;
        for (int i = 0; i < n_segments; ++i) {
            const char* text = whisper_full_get_segment_text(ctx_, i);
            std::cout << "Segment " << i << ": " << text << std::endl;
            if (text && strlen(text) > 0) {
                auto msg = std_msgs::msg::String();
                msg.data = text;
                publisher_->publish(msg);
                RCLCPP_INFO(this->get_logger(), "Published: '%s'", text);
            }
        }
    }


    std::vector<float> loadAudioFile(const char* filename) {
        std::ifstream file(filename, std::ios::binary);
        if (!file.is_open()) {
            throw std::runtime_error("Failed to open audio file");
        }

        // Read WAV header first
        char header[44];
        file.read(header, 44);  // Standard WAV header size

        // Get actual data size from header
        uint32_t data_size = *reinterpret_cast<uint32_t*>(&header[40]);
        uint16_t num_channels = *reinterpret_cast<uint16_t*>(&header[22]);
        uint32_t sample_rate = *reinterpret_cast<uint32_t*>(&header[24]);

        // Read audio samples
        std::vector<int16_t> pcm16;
        pcm16.resize(data_size / sizeof(int16_t));
        file.read(reinterpret_cast<char*>(pcm16.data()), data_size);

        // Convert to float32
        std::vector<float> audio_data;
        audio_data.resize(pcm16.size());
        for (size_t i = 0; i < pcm16.size(); i++) {
            audio_data[i] = static_cast<float>(pcm16[i]) / 32768.0f;
        }

        return audio_data;
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    struct whisper_context* ctx_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WhisperASRNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
