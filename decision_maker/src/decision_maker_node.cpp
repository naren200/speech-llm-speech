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

class SentimentAnalyzer {
public:
    struct SentimentScore {
        double polarity;      // Range: -1.0 to 1.0
        double subjectivity;  // Range: 0.0 to 1.0
        double compound;      // Overall sentiment score
    };

    static SentimentScore analyze(const std::string& text) {
        // Create a pipe to the Python script
        std::array<char, 128> buffer;
        std::string result;
        std::string cmd = "python3 src/speech-llm-speech/decision_maker/src/sentiment_analyzer.py \"" + text + "\"";
        
        FILE* pipe = popen(cmd.c_str(), "r");
        if (!pipe) {
            throw std::runtime_error("Failed to run sentiment analysis script");
        }
        
        while (fgets(buffer.data(), buffer.size(), pipe) != nullptr) {
            result += buffer.data();
        }
        
        auto returnCode = pclose(pipe);
        if (returnCode != 0) {
            throw std::runtime_error("Sentiment analysis failed");
        }
        
        // Parse JSON response
        auto json = nlohmann::json::parse(result);
        return {
            json["polarity"].get<double>(),
            json["subjectivity"].get<double>(),
            json["compound"].get<double>()
        };
    }
};

class ResponseSelector {
public:
    struct ScoredResponse {
        APIResponse response;
        SentimentAnalyzer::SentimentScore sentiment;
        double final_score;
        int llm_type;
    };

    static ScoredResponse select_best_response(
        const std::vector<APIResponse>& responses, 
        const std::vector<int>& llm_types,
        rclcpp::Logger logger) {  // Add logger parameter
            
        std::vector<ScoredResponse> scored_responses;
        
        for (size_t i = 0; i < responses.size(); i++) {
            auto sentiment = SentimentAnalyzer::analyze(responses[i].text);
            double score = calculate_response_score(responses[i], sentiment);
            
            RCLCPP_INFO(logger,
                "\nLLM %d Response Details:"
                "\n  Text Length: %zu"
                "\n  Latency: %.3f"
                "\n  Sentiment Polarity: %.3f"
                "\n  Sentiment Subjectivity: %.3f"
                "\n  Relevance Score: %.3f"
                "\n  Final Score: %.3f",
                llm_types[i],
                responses[i].text.length(),
                responses[i].latency,
                sentiment.polarity,
                sentiment.subjectivity,
                responses[i].relevance_score,
                score);
                
            scored_responses.push_back({
                responses[i], 
                sentiment, 
                score, 
                llm_types[i]
            });
        }
        
        auto best = std::max_element(scored_responses.begin(), scored_responses.end(),
            [](const ScoredResponse& a, const ScoredResponse& b) {
                return a.final_score < b.final_score;
            });
            
        RCLCPP_INFO(logger,
            "\nSelected Best Response:"
            "\n  LLM Type: %d"
            "\n  Final Score: %.3f"
            "\n  Response Text: %s",
            best->llm_type,
            best->final_score,
            best->response.text.c_str());
            
        return *best;
    }

private:
    static double calculate_response_score(
        const APIResponse& response,
        const SentimentAnalyzer::SentimentScore& sentiment) {
        
        constexpr double LATENCY_WEIGHT = 0.2;
        constexpr double SENTIMENT_WEIGHT = 0.3;
        constexpr double RELEVANCE_WEIGHT = 0.3;
        constexpr double SUBJECTIVITY_WEIGHT = 0.2;
        
        // Normalize latency (lower is better)
        double latency_score = 1.0 / (1.0 + response.latency);
        
        // Combine scores
        return latency_score * LATENCY_WEIGHT +
               std::abs(sentiment.compound) * SENTIMENT_WEIGHT +
               response.relevance_score * RELEVANCE_WEIGHT +
               (1.0 - sentiment.subjectivity) * SUBJECTIVITY_WEIGHT;
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
                RCLCPP_INFO(this->get_logger(), "Creating OpenAI client");
                return std::make_shared<OpenAIClient>();
            case 2:
                RCLCPP_INFO(this->get_logger(), "Creating HuggingFace client");
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
        
        RCLCPP_INFO(this->get_logger(), "Starting response generation for %zu LLMs", llm_sequence_.size());
        
        for (int llm_type : llm_sequence_) {
            auto client = create_llm_client(llm_type);
            RCLCPP_DEBUG(this->get_logger(), "Initiating request for LLM type %d", llm_type);
            futures.push_back(std::async(std::launch::async, 
                [client, msg]() { return client->get_response(msg->data); }));
        }

        auto start_time = std::chrono::steady_clock::now();
        auto timeout = std::chrono::seconds(120 * llm_sequence_.size()/3);

        while (responses.size() < futures.size()) {
            for (size_t i = 0; i < futures.size(); ++i) {
                if (i < responses.size()) continue;
                
                RCLCPP_DEBUG(this->get_logger(), "Checking future %zu status", i);
                
                if (futures[i].valid() && 
                    futures[i].wait_for(std::chrono::milliseconds(100)) == 
                    std::future_status::ready) {
                    try {
                        responses.push_back(futures[i].get());
                        RCLCPP_INFO(this->get_logger(), 
                            "Response %zu received. Response text length: %zu", 
                            responses.size(), 
                            responses.back().text.length());
                    } catch (const std::exception& e) {
                        RCLCPP_ERROR(this->get_logger(), 
                            "LLM %d failed with error: %s", 
                            llm_sequence_[i], 
                            e.what());
                    }
                } else if (futures[i].valid()) {
                    RCLCPP_DEBUG(this->get_logger(), 
                        "Future %zu still pending", i);
                }
            }
            
            auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
                std::chrono::steady_clock::now() - start_time);
            
            RCLCPP_DEBUG(this->get_logger(), 
                "Waiting for responses: %zu/%zu (Elapsed: %ld seconds)", 
                responses.size(), 
                futures.size(), 
                elapsed.count());
                
            if (elapsed > timeout) {
                RCLCPP_WARN(this->get_logger(), 
                    "Timeout reached with %zu/%zu responses", 
                    responses.size(), 
                    futures.size());
                break;
            }
            
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }

        if (responses.empty()) {
            RCLCPP_ERROR(this->get_logger(), "No responses received from any LLM");
            return;
        }

        try {
            if (responses.size() == 1) {
                auto response_msg = std_msgs::msg::String();
                response_msg.data = responses[0].text;
                response_pub_->publish(response_msg);
                
                RCLCPP_INFO(this->get_logger(), 
                    "Published single available response");
            } else {
                auto selected = ResponseSelector::select_best_response(responses, llm_sequence_, this->get_logger());
                
                auto response_msg = std_msgs::msg::String();
                response_msg.data = selected.response.text;
                response_pub_->publish(response_msg);
                
                RCLCPP_INFO(this->get_logger(), 
                    "Published best response with score: %.3f",
                    selected.final_score);
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), 
                "Error processing responses: %s", 
                e.what());
        }
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
