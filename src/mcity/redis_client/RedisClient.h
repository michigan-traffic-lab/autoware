#ifndef REDIS_CLIENT_H
#define REDIS_CLIENT_H

#include <hiredis/hiredis.h>
#include <string>

class RedisClient {
public:
    RedisClient();
    ~RedisClient();
    bool connect(bool local);
    bool set(const std::string &key, const std::string &value);
    std::string get(const std::string &key);

private:
    redisContext *context;

    std::string local_host;
    std::string remote_host;

    int local_port;
    int remote_port;

    std::string local_password;
    std::string remote_password;

    void loadEnvironmentVariables();
};

#endif // REDIS_CLIENT_H
