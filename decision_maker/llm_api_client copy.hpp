#pragma once
#include <string>
#include <chrono>
#include <cstdlib>
#include <random>
#include <thread>
#include <nlohmann/json.hpp>
#include <curl/curl.h>

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
        if (std::getenv("MOCK_MODE")) {
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
            
            struct curl_slist* headers = nullptr;
            headers = curl_slist_append(headers, ("Authorization: Bearer " + api_key).c_str());
            headers = curl_slist_append(headers, "Content-Type: application/json");
            
            nlohmann::json payload = {
                {"model", "gpt-3.5-turbo"},
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
            return APIResponse{response_string, latency, 0.8, 0.9};
        }
        throw std::runtime_error("Failed to initialize CURL");
    }

    APIResponse mock_response(const std::string& input) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        return APIResponse{
            "OpenAI mock response for: " + input,
            0.1,
            0.8,
            0.9
        };
    }
};

class HuggingFaceClient : public LLMApiClient {
public:
    APIResponse get_response(const std::string& input) override {
        if (std::getenv("MOCK_MODE")) {
            return mock_response(input);
        }
        return real_api_call(input);
    }

private:
    APIResponse real_api_call(const std::string& input) {
        CURL* curl = curl_easy_init();
        std::string response_string;
        
        if(curl) {
            std::string api_key = std::getenv("HF_API_KEY");
            std::string url = "https://api-inference.huggingface.co/models/gpt2";
            
            struct curl_slist* headers = nullptr;
            headers = curl_slist_append(headers, ("Authorization: Bearer " + api_key).c_str());
            headers = curl_slist_append(headers, "Content-Type: application/json");
            
            nlohmann::json payload = {
                {"inputs", input}
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
            return APIResponse{response_string, latency, 0.7, 0.8};
        }
        throw std::runtime_error("Failed to initialize CURL");
    }

    APIResponse mock_response(const std::string& input) {
        std::this_thread::sleep_for(std::chrono::milliseconds(150));
        return APIResponse{
            "HuggingFace mock response for: " + input,
            0.15,
            0.7,
            0.8
        };
    }
};

class OllamaClient : public LLMApiClient {
public:
    APIResponse get_response(const std::string& input) override {
        if (std::getenv("MOCK_MODE")) {
            return mock_response(input);
        }
        return real_api_call(input);
    }

private:
    APIResponse real_api_call(const std::string& input) {
        CURL* curl = curl_easy_init();
        std::string response_string;
        
        if(curl) {
            std::string url = "http://localhost:11434/api/generate";
            
            struct curl_slist* headers = nullptr;
            headers = curl_slist_append(headers, "Content-Type: application/json");
            
            nlohmann::json payload = {
                {"model", "llama2"},
                {"prompt", input}
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
                throw std::runtime_error("Ollama API call failed");
            }
            
            double latency = std::chrono::duration<double>(end - start).count();
            return APIResponse{response_string, latency, 0.6, 0.7};
        }
        throw std::runtime_error("Failed to initialize CURL");
    }

    APIResponse mock_response(const std::string& input) {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        return APIResponse{
            "Ollama mock response for: " + input,
            0.2,
            0.6,
            0.7
        };
    }
};
