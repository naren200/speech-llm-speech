#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <thread>
#include <future>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <sstream>
#include "decision_maker/llm_api_client.hpp"


class ThreadPool {
private:
    std::vector<std::thread> workers;
    std::queue<std::function<void()>> tasks;
    std::mutex queue_mutex;
    std::condition_variable condition;
    bool stop;

public:
    ThreadPool(size_t threads) : stop(false) {
        for(size_t i = 0; i < threads; ++i) {
            workers.emplace_back([this] {
                while(true) {
                    std::function<void()> task;
                    {
                        std::unique_lock<std::mutex> lock(queue_mutex);
                        condition.wait(lock, [this] { 
                            return stop || !tasks.empty(); 
                        });
                        if(stop && tasks.empty()) return;
                        task = std::move(tasks.front());
                        tasks.pop();
                    }
                    task();
                }
            });
        }
    }

    template<class F>
    void enqueue(F&& f) {
        {
            std::unique_lock<std::mutex> lock(queue_mutex);
            tasks.emplace(std::forward<F>(f));
        }
        condition.notify_one();
    }

    ~ThreadPool() {
        {
            std::unique_lock<std::mutex> lock(queue_mutex);
            stop = true;
        }
        condition.notify_all();
        for(std::thread &worker: workers) {
            worker.join();
        }
    }
};


class DecisionMakingNode : public rclcpp::Node {
private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr speech_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr response_pub_;
    std::unique_ptr<ThreadPool> thread_pool_;
    std::mutex response_mutex_;
    std::vector<int> llm_sequence_;
    
public:
    DecisionMakingNode() : Node("decision_making_node") {
        thread_pool_ = std::make_unique<ThreadPool>(std::thread::hardware_concurrency());
        
        // Parse LLM sequence from environment variable
        parse_llm_sequence();

        speech_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/recognized_speech", 10,
            std::bind(&DecisionMakingNode::speech_callback, this, std::placeholders::_1));
            
        response_pub_ = this->create_publisher<std_msgs::msg::String>("/final_response", 10);
    }

private:
    void parse_llm_sequence() {
        const char* seq = std::getenv("LLM_SEQUENCE");
        if (!seq) {
            RCLCPP_ERROR(this->get_logger(), "LLM_SEQUENCE environment variable not set");
            throw std::runtime_error("LLM_SEQUENCE not set");
        }

        std::string sequence(seq);
        std::stringstream ss(sequence);
        std::string item;

        // Remove { and } from the string
        sequence.erase(std::remove(sequence.begin(), sequence.end(), '{'), sequence.end());
        sequence.erase(std::remove(sequence.begin(), sequence.end(), '}'), sequence.end());

        // Split by comma
        std::stringstream ss2(sequence);
        while (std::getline(ss2, item, ',')) {
            llm_sequence_.push_back(std::stoi(item));
        }
    }

    std::shared_ptr<LLMApiClient> create_llm_client(int type) {
        switch(type) {
            case 1:
                return std::make_shared<OpenAIClient>();
            case 2:
                return std::make_shared<HuggingFaceClient>();
            case 3:
                RCLCPP_INFO(this->get_logger(), "Creating Ollama client");
                return std::make_shared<OllamaClient>();
            default:
                throw std::runtime_error("Invalid LLM type");
        }
    }

    void speech_callback(const std_msgs::msg::String::SharedPtr msg) {
        std::vector<std::future<APIResponse>> futures;
        std::vector<APIResponse> responses;
        
        // Launch API calls in parallel for each LLM in sequence
        for (int llm_type : llm_sequence_) {
            auto client = create_llm_client(llm_type);
            futures.push_back(std::async(std::launch::async, 
                [client, msg]() { return client->get_response(msg->data); }));
        }

        try {
            auto timeout = std::chrono::seconds(1500);
            
            for (size_t i = 0; i < futures.size(); ++i) {
                if (futures[i].wait_for(timeout) == std::future_status::ready) {
                    auto response = futures[i].get();
                    RCLCPP_INFO(this->get_logger(), 
                        "LLM %d Score: %.3f (Latency: %.3f, Sentiment: %.3f, Relevance: %.3f)",
                        llm_sequence_[i],
                        calculate_score(response),
                        response.latency,
                        response.sentiment_score,
                        response.relevance_score);
                    responses.push_back(response);
                }
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "API call failed: %s", e.what());
        }

        if (responses.empty()) {
            RCLCPP_WARN(this->get_logger(), "No responses received within timeout");
            return;
        }

        auto best_response = select_best_response(responses);
        
        auto response_msg = std_msgs::msg::String();
        response_msg.data = best_response.text;
        response_pub_->publish(response_msg);
    }

    APIResponse select_best_response(const std::vector<APIResponse>& responses) {
        if (responses.empty()) {
            throw std::runtime_error("No responses available");
        }

        auto best_response = *std::max_element(responses.begin(), responses.end(),
            [this](const APIResponse& a, const APIResponse& b) {
                return calculate_score(a) < calculate_score(b);
            });

        RCLCPP_INFO(this->get_logger(), "Selected best response with score: %.3f and text: \n%s",
            calculate_score(best_response), best_response.text.c_str());

        // RCLCPP_INFO(this->get_logger(), "Selected best response", best_response.text.c_str());

        return best_response;
    }

    static double calculate_score(const APIResponse& response) {
        const double latency_weight = 0.3;
        const double sentiment_weight = 0.3;
        const double relevance_weight = 0.4;

        return (1.0 / response.latency) * latency_weight +
               response.sentiment_score * sentiment_weight +
               response.relevance_score * relevance_weight;
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DecisionMakingNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
