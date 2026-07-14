#pragma once
#include <string>
#include <vector>

// 定义消息结构体
struct Message {
    std::string sender;
    std::string receiver;
    std::string type;    // 如 "PASS_TARGET", "RECEIVER_READY"
    float target_x;
    float target_y;
    bool is_read = false; // 核心检查点：标记是否已被消费
};

class MockTeamBus {
private:
    std::vector<Message> message_queue;
    std::vector<Message> history; // 保留历史用于统计和日志

public:
    // Publish：写入队列
    void publish(const std::string& sender, const std::string& receiver, const std::string& type, float x, float y) {
        Message msg = {sender, receiver, type, x, y, false};
        message_queue.push_back(msg);
        history.push_back(msg);
    }

    // Consume：按接收者和消息类型取出消息（同一条不重复读取）
    bool consume(const std::string& receiver, const std::string& type, Message& out_msg) {
        for (auto& msg : message_queue) {
            if (!msg.is_read && msg.receiver == receiver && msg.type == type) {
                msg.is_read = true; // 阅后即焚标记
                out_msg = msg;
                return true;
            }
        }
        return false;
    }
};