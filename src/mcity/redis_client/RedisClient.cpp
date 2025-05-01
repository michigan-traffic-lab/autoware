#include "RedisClient.h"
#include <cstdlib>
#include <iostream>

RedisClient::RedisClient() : context(nullptr) {
    loadEnvironmentVariables();
}

RedisClient::~RedisClient() {
    if (context) {
        redisFree(context);
    }
}

void RedisClient::loadEnvironmentVariables() {
    char* host_env = std::getenv("TERASIM_REDIS_HOST");
    char* port_env = std::getenv("TERASIM_REDIS_PORT");
    char* password_env = std::getenv("TERASIM_REDIS_PASSWORD");

    remote_host = host_env ? host_env : "127.0.0.1";
    remote_port = port_env ? std::stoi(port_env) : 6379;
    remote_password = password_env ? password_env : "";

    local_host = "127.0.0.1";
    local_port = 6379;
    local_password = "";
}

bool RedisClient::connect(bool local) {
    std::string password;

    if (local){
        context = redisConnect(local_host.c_str(), local_port);
        password = local_password;
    } else{
        context = redisConnect(remote_host.c_str(), remote_port);
        password = remote_password;
    }

    if (context == nullptr || context->err) {
        if (context) {
            std::cerr << "Connect redis error: " << context->err << std::endl;
            redisFree(context);
        } else {
            std::cerr << "Can't allocate redis context" << std::endl;
        }
        return false;
    }

    if (!password.empty()) {
        redisReply *reply = (redisReply *)redisCommand(context, "AUTH %s", password.c_str());
        if (reply->type == REDIS_REPLY_ERROR) {
            std::cerr << "Redis auth error: " << reply->str << std::endl;
            freeReplyObject(reply);
            return false;
        }
        freeReplyObject(reply);
    }

    return true;
}

bool RedisClient::set(const std::string &key, const std::string &value) {
    redisReply *reply = (redisReply *)redisCommand(context, "SET %s %s", key.c_str(), value.c_str());
    if (reply->type == REDIS_REPLY_ERROR) {
        std::cerr << "Redis set key error: " << key << std::endl;
        freeReplyObject(reply);
        return false;
    }
    freeReplyObject(reply);
    return true;
}

std::string RedisClient::get(const std::string &key) {
    redisReply *reply = (redisReply *)redisCommand(context, "GET %s", key.c_str());
    std::string result;
    if (reply->type == REDIS_REPLY_STRING) {
        result = reply->str;
    }
    freeReplyObject(reply);
    return result;
}