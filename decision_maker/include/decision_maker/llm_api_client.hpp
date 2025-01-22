#pragma once
#include <string>
#include <chrono>
#include <cstdlib>
#include <random>
#include <thread>
#include <nlohmann/json.hpp>
#include <curl/curl.h>
#include <iostream>
#include<decision_maker/ollama.hpp>

struct APIResponse {
    std::string text;
    double latency;
    double sentiment_score;
    double relevance_score;
};

class LLMApiClient {
public:
    virtual APIResponse get_response(const std::string& input) = 0;
    virtual ~LLMApiClient() = default;
protected:
    static size_t WriteCallback(void* contents, size_t size, size_t nmemb, std::string* userp) {
        userp->append((char*)contents, size * nmemb);
        return size * nmemb;
    }
};

class OpenAIClient : public LLMApiClient {
public:
    APIResponse get_response(const std::string& input) override {
        const char* mock_env = std::getenv("MOCK_MODE");
        bool mock_mode = mock_env != nullptr && 
                        (std::string(mock_env) != "0" && 
                        std::string(mock_env) != "false");
        
        if (mock_mode) {
            return mock_response(input);
        }
        return real_api_call(input);
    }

private:
    APIResponse real_api_call(const std::string& input) {
        CURL* curl = curl_easy_init();
        std::string response_string;
        
        if(curl) {
            std::string api_key = std::getenv("OPENAI_API_KEY");
            std::string url = "https://api.openai.com/v1/chat/completions";
            
            // Get model from environment variable
            std::string model = "gpt-3.5-turbo";  // default model
            if (const char* env_model = std::getenv("OPENAI_MODEL")) {
                model = env_model;
            }
            
            struct curl_slist* headers = nullptr;
            headers = curl_slist_append(headers, ("Authorization: Bearer " + api_key).c_str());
            headers = curl_slist_append(headers, "Content-Type: application/json");
            
            nlohmann::json payload = {
                {"model", model},
                {"messages", {{
                    {"role", "user"},
                    {"content", input}
                }}}
            };
            
            curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
            curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
            curl_easy_setopt(curl, CURLOPT_POSTFIELDS, payload.dump().c_str());
            curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
            curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response_string);
            
            auto start = std::chrono::high_resolution_clock::now();
            CURLcode res = curl_easy_perform(curl);
            auto end = std::chrono::high_resolution_clock::now();
            
            curl_slist_free_all(headers);
            curl_easy_cleanup(curl);
            
            if(res != CURLE_OK) {
                throw std::runtime_error("OpenAI API call failed");
            }
            
            double latency = std::chrono::duration<double>(end - start).count();
            std::cout << "OpenAI Response using model " << model << std::endl;
            return APIResponse{response_string, latency, 0.8, 0.9};
        }
        throw std::runtime_error("Failed to initialize CURL");
    }

    APIResponse mock_response(const std::string& input) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        std::string model = std::getenv("OPENAI_MODEL") ? std::getenv("OPENAI_MODEL") : "gpt-3.5-turbo";
        return APIResponse{
            "OpenAI mock response for: " + input + " (using " + model + ")",
            0.1,
            0.8,
            0.9
        };
    }
};

class HuggingFaceClient : public LLMApiClient {
public:
    APIResponse get_response(const std::string& input) override {
        const char* mock_env = std::getenv("MOCK_MODE");
        bool mock_mode = mock_env != nullptr && 
                        (std::string(mock_env) != "0" && 
                        std::string(mock_env) != "false");
        
        if (mock_mode) {
            return mock_response(input);
        }
        return real_api_call(input);        
    }

private:
    static size_t WriteCallback(void* contents, size_t size, size_t nmemb, std::string* userp) {
        userp->append((char*)contents, size * nmemb);
        return size * nmemb;
    }

    APIResponse real_api_call(const std::string& input) {
        CURL* curl = curl_easy_init();
        std::string response_string;
        std::stringstream stream;
        
        if(curl) {
            std::string api_key = std::getenv("HF_API_KEY");
            
            // Get model from environment variable
            std::string model = "gpt2";
            if (const char* env_model = std::getenv("HUGGINGFACE_MODEL")) {
                model = env_model;
            }
            
            std::string url = "https://api-inference.huggingface.co/models/" + model;
            
            struct curl_slist* headers = nullptr;
            headers = curl_slist_append(headers, ("Authorization: Bearer " + api_key).c_str());
            headers = curl_slist_append(headers, "Content-Type: application/json");
            headers = curl_slist_append(headers, "Accept: text/event-stream");
            
            nlohmann::json payload = {
                {"inputs", input},
                {"parameters", {
                    {"temperature", 0.5},
                    {"max_new_tokens", 2048},
                    {"top_p", 0.7}
                }}
            };
            
            curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
            curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
            curl_easy_setopt(curl, CURLOPT_POSTFIELDS, payload.dump().c_str());
            curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
            curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response_string);
            
            auto start = std::chrono::high_resolution_clock::now();
            CURLcode res = curl_easy_perform(curl);
            auto end = std::chrono::high_resolution_clock::now();
            

            curl_slist_free_all(headers);
            curl_easy_cleanup(curl);
            
            if(res != CURLE_OK) {
                throw std::runtime_error("HuggingFace API call failed");
            }
            
            double latency = std::chrono::duration<double>(end - start).count();
            
            // Parse streaming response
            std::string out;
            try {
                auto json_response = nlohmann::json::parse(response_string);
                out = json_response.value("generated_text", "");
            } catch (const std::exception& e) {
                out = "Error parsing response: " + std::string(e.what());
            }
            
            std::cout << "HuggingFace Response using model " << model << std::endl;
            return APIResponse{out, latency, 0.7, 0.8};
        }
        throw std::runtime_error("Failed to initialize CURL");
    }

    APIResponse mock_response(const std::string& input) {
        std::this_thread::sleep_for(std::chrono::milliseconds(150));
        std::string model = std::getenv("HUGGINGFACE_MODEL") ? std::getenv("HUGGINGFACE_MODEL") : "llama-3.1";
        return APIResponse{
            "HuggingFace mock response for: " + input + " (using " + model + ")",
            0.15,
            0.7,
            0.8
        };
    }
};
class OllamaClient : public LLMApiClient {
public:
    APIResponse get_response(const std::string& input) override {
        const bool mock_mode = std::getenv("MOCK_MODE") && 
                             std::string(std::getenv("MOCK_MODE")) != "0" && 
                             std::string(std::getenv("MOCK_MODE")) != "false";
        
        return mock_mode ? mock_response(input) : real_api_call(input);
    }

private:
    APIResponse real_api_call(const std::string& input) {
        double latency;
            

        std::future<ollama::response> response_future = std::async(std::launch::async, [&input, &latency]() {
            try {
                auto start = std::chrono::high_resolution_clock::now();
                auto reponse_string = ollama::generate("qwen:0.5b", input);
                auto end = std::chrono::high_resolution_clock::now();
                latency = std::chrono::duration<double>(end - start).count();
                return reponse_string;
            } catch (const std::exception& e) {
                throw; // Re-throw the exception to be handled in the main thread
            }
        });

        // Handle the response in the main thread
        try {
            ollama::response response = response_future.get();
            return APIResponse{
                response.as_simple_string(), // Use as_simple_string() instead of response field
                latency,
                0.6,
                0.7
            };
        } catch (const std::exception& e) {
            return APIResponse{
                std::string("Error: ") + e.what(),
                0.0,
                0.0,
                0.0
            };
        }
    }

    APIResponse mock_response(const std::string& input) {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        std::string model = std::getenv("OLLAMA_MODEL") ? std::getenv("OLLAMA_MODEL") : "llama2";
        return APIResponse{
            "Ollama mock response for: " + input + " (using " + model + ")",
            0.2,
            0.6,
            0.7
        };
    }
};

