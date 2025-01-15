#ifndef UTILS_HPP
#define UTILS_HPP

#include <iostream>
#include <sstream>
#include <string>
#include <iomanip>
#include <openssl/evp.h>  // 使用 EVP 接口
#include <ctime>
#include <optional>  // 需要包含 optional 头文件

/// @brief generate a unique token by timestamp and dev_name
/// @param timestamp 
/// @param dev_name 
/// @param optional_token (可选) 额外的标识符，可以为 nullopt
/// @return 输入相同的数据会始终产生相同的输出
std::string generate_unique_token(double timestamp, const std::string &dev_name, 
                                   const std::optional<std::string>& optional_token = std::nullopt) {
    // 提取时间戳的整数部分和纳秒部分
    time_t timestamp_int = static_cast<time_t>(timestamp);
    int nanosec = static_cast<int>((timestamp - timestamp_int) * 1e9);  // 纳秒部分
    
    // 构造字符串：时间戳（秒） + 纳秒部分 + 相机名
    std::stringstream ss;
    ss << timestamp_int << "_" << nanosec << "_" << dev_name;

    // 如果有额外的 optional_token 参数，则添加到字符串中
    if (optional_token.has_value()) {
        ss << "_" << optional_token.value();
    }

    std::string unique_string = ss.str();
    
    // 使用 EVP API 生成哈希值
    unsigned char hash[EVP_MAX_MD_SIZE];
    unsigned int hash_len = 0;

    // 创建 EVP 上下文
    EVP_MD_CTX *mdctx = EVP_MD_CTX_new();
    if (mdctx == nullptr) {
        throw std::runtime_error("Failed to create EVP_MD_CTX");
    }

    // 初始化 SHA256 消息摘要上下文
    if (EVP_DigestInit_ex(mdctx, EVP_sha256(), nullptr) != 1) {
        EVP_MD_CTX_free(mdctx);
        throw std::runtime_error("Failed to initialize EVP context");
    }

    // 更新哈希值
    if (EVP_DigestUpdate(mdctx, unique_string.c_str(), unique_string.size()) != 1) {
        EVP_MD_CTX_free(mdctx);
        throw std::runtime_error("Failed to update EVP context");
    }

    // 完成哈希计算
    if (EVP_DigestFinal_ex(mdctx, hash, &hash_len) != 1) {
        EVP_MD_CTX_free(mdctx);
        throw std::runtime_error("Failed to finalize EVP context");
    }

    // 释放 EVP 上下文
    EVP_MD_CTX_free(mdctx);

    // 将哈希值转换为十六进制字符串
    std::stringstream hash_ss;
    for (unsigned int i = 0; i < hash_len; ++i) {
        hash_ss << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(hash[i]);
    }

    return hash_ss.str();
}

void splitString(const std::string& str, char delimiter, std::string& firstPart, std::string& secondPart) {
    // 查找分隔符的位置
    size_t pos = str.find(delimiter);
    
    if (pos != std::string::npos) {
        // 获取分隔符前面的部分
        firstPart = str.substr(0, pos);
        // 获取分隔符后面的部分
        secondPart = str.substr(pos + 1);
    } else {
        // 如果没有找到分隔符，设置返回值为原始字符串
        firstPart = str;
        secondPart = "";
    }
}


#endif
