#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <thread>
#include <future>
#include <mutex>
#include <condition_variable>
#include <queue>
#include "llm_api_client.hpp"

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
    std::shared_ptr<LLMApiClient> openai_client_;
    std::shared_ptr<LLMApiClient> huggingface_client_;
    std::shared_ptr<LLMApiClient> ollama_client_;
    std::unique_ptr<ThreadPool> thread_pool_;
    std::mutex response_mutex_;
    
public:
    DecisionMakingNode() : Node("decision_making_node") {
        // Initialize thread pool with hardware concurrency
        thread_pool_ = std::make_unique<ThreadPool>(std::thread::hardware_concurrency());
        
        // Initialize API clients
        openai_client_ = std::make_shared<OpenAIClient>();
        huggingface_client_ = std::make_shared<HuggingFaceClient>();
        ollama_client_ = std::make_shared<OllamaClient>();
        
        // Create subscribers and publishers
        speech_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/recognized_speech", 10,
            std::bind(&DecisionMakingNode::speech_callback, this, std::placeholders::_1));
            
        response_pub_ = this->create_publisher<std_msgs::msg::String>("/final_response", 10);
    }

private:
    void speech_callback(const std_msgs::msg::String::SharedPtr msg) {
        std::vector<std::future<APIResponse>> futures;
        std::vector<APIResponse> responses;
        
        // Launch API calls in parallel using thread pool
        auto openai_future = std::async(std::launch::async, 
            [this, msg]() { return openai_client_->get_response(msg->data); });
        auto huggingface_future = std::async(std::launch::async, 
            [this, msg]() { return huggingface_client_->get_response(msg->data); });
        auto ollama_future = std::async(std::launch::async, 
            [this, msg]() { return ollama_client_->get_response(msg->data); });

        try {
            // Wait for all responses with timeout
            auto timeout = std::chrono::seconds(5);
            
            if (openai_future.wait_for(timeout) == std::future_status::ready) {
                responses.push_back(openai_future.get());
            }
            if (huggingface_future.wait_for(timeout) == std::future_status::ready) {
                responses.push_back(huggingface_future.get());
            }
            if (ollama_future.wait_for(timeout) == std::future_status::ready) {
                responses.push_back(ollama_future.get());
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "API call failed: %s", e.what());
        }

        if (responses.empty()) {
            RCLCPP_WARN(this->get_logger(), "No responses received within timeout");
            return;
        }

        // Select best response based on scoring
        auto best_response = select_best_response(responses);
        
        // Publish response
        auto response_msg = std_msgs::msg::String();
        response_msg.data = best_response.text;
        response_pub_->publish(response_msg);
    }

    APIResponse select_best_response(const std::vector<APIResponse>& responses) {
        if (responses.empty()) {
            throw std::runtime_error("No responses available");
        }

        return *std::max_element(responses.begin(), responses.end(),
            [](const APIResponse& a, const APIResponse& b) {
                return calculate_score(a) < calculate_score(b);
            });
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
