#include <ros/ros.h>
#include <audio_common_msgs/AudioData.h>
#include <std_msgs/String.h>
#include "include/whisper.h"

class WhisperASRNode {
private:
    ros::NodeHandle nh_;
    ros::Subscriber audio_sub_;
    ros::Publisher text_pub_;
    
    struct whisper_context* ctx;
    std::vector<float> pcmf32;               // audio buffer
    int max_buffer_size = 48000 * 30;        // 30 seconds buffer
    
public:
    WhisperASRNode() {
        // Initialize Whisper model
        const char* model_path = "/path/to/your/model.bin";
        ctx = whisper_init_from_file(model_path);
        
        if (ctx == nullptr) {
            ROS_ERROR("Failed to initialize Whisper model");
            ros::shutdown();
            return;
        }

        // Initialize ROS publishers and subscribers
        audio_sub_ = nh_.subscribe("audio", 1000, &WhisperASRNode::audioCallback, this);
        text_pub_ = nh_.advertise<std_msgs::String>("transcription", 1000);
        
        pcmf32.reserve(max_buffer_size);
    }

    ~WhisperASRNode() {
        if (ctx) {
            whisper_free(ctx);
        }
    }

    void audioCallback(const audio_common_msgs::AudioData::ConstPtr& msg) {
        // Convert audio data to float32
        for (const auto& sample : msg->data) {
            pcmf32.push_back(static_cast<float>(sample) / 32768.0f);
        }

        // Process when buffer is full
        if (pcmf32.size() >= max_buffer_size) {
            processAudio();
            pcmf32.clear();
        }
    }

    void processAudio() {
        whisper_full_params wparams = whisper_full_default_params(WHISPER_SAMPLING_GREEDY);
        wparams.print_realtime = true;
        wparams.print_progress = false;
        wparams.print_timestamps = true;
        wparams.translate = false;
        wparams.language = "en";
        wparams.n_threads = 4;

        if (whisper_full(ctx, wparams, pcmf32.data(), pcmf32.size()) != 0) {
            ROS_ERROR("Failed to process audio");
            return;
        }

        // Get transcription results
        const int n_segments = whisper_full_n_segments(ctx);
        for (int i = 0; i < n_segments; ++i) {
            const char* text = whisper_full_get_segment_text(ctx, i);
            
            std_msgs::String msg;
            msg.data = text;
            text_pub_.publish(msg);
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "whisper_asr_node");
    WhisperASRNode node;
    ros::spin();
    return 0;
}
