#ifndef NUSCENES_WRITER_HPP
#define NUSCENES_WRITER_HPP

#include <iostream>
#include <fstream>
#include <nlohmann/json.hpp>
#include <string>
#include <unordered_map>
#include <vector>

using json = nlohmann::json;

class NuscenesJsonWriter {
public:
    // 构造函数，初始化并加载所有 JSON 文件
    NuscenesJsonWriter(const std::vector<std::string>& file_paths) {
        load_failure_count_ = 0;
        for (const auto& path : file_paths) {
            bool success = loadJsonFromFile(path);
            if (!success) {
                load_failure_count_++;
            }
        }
    }
    
    bool isLoadError(){
        return load_failure_count_ > 0;
    }

    // 根据文件路径加载 JSON 数据（假设是一个数组格式的 JSON）
    bool loadJsonFromFile(const std::string& file_path) {
        std::ifstream in(file_path);
        if (in.is_open()) {
            json loaded_json;
            in >> loaded_json;

            // 检查加载的数据是否为对象或数组
            if (loaded_json.is_array()) {
                // 如果是数组，存储为数组
                if (loaded_json.empty()) {
                    //std::cout << "nuscenes json writer:" << "Loaded an empty JSON array from file: " << file_path << "\n";
                }
                std::string json_name = extractFileName(file_path);
                json_map[json_name] = loaded_json;
                file_path_map[json_name] = file_path; // 记录 json_name 与文件路径的映射
                //std::cout << "nuscenes json writer:" << "Loaded JSON array from file: " << file_path << "\n";
                return true;
            } else if (loaded_json.is_object()) {
                // 如果是对象，存储为对象
                std::string json_name = extractFileName(file_path);
                json_map[json_name] = loaded_json;
                file_path_map[json_name] = file_path; // 记录 json_name 与文件路径的映射
                //std::cout << "nuscenes json writer:" << "Loaded JSON object from file: " << file_path << "\n";
                return true;
            } else {
                // 如果既不是数组也不是对象，输出错误信息
                std::cerr << "nuscenes json writer:" << "nuscenes json writer:" << "Error: JSON is neither an array nor an object in file: " << file_path << "\n";
                return false;
            }
        } else {
            std::cerr << "nuscenes json writer:" << "nuscenes json writer:" << "Failed to open file: " << file_path << "\n";
            return false;
        }
    }

    // 打印所有 JSON 内容
    void printJsons() const {
        for (const auto& entry : json_map) {
            std::cout << "nuscenes json writer:" << entry.first << ": " << entry.second.dump(4) << "\n";
        }
    }

    /// @brief 写入json的key_name下的子集，没有key_name集就创建
    /// @param json_name 总 JSON 文件或对象的名称
    /// @param key_name 场景名称，用于找到对应的场景
    /// @param new_data 新的数据项，添加到指定场景的 "data" 数组中
    /// @param is_save 是否需要立即保存到文件
    void addDataUnderKey(const std::string& json_name, const std::string& key_name, const json& new_data, bool is_save) {
        // 获取整个 JSON 数据对象
        auto target_json = getJson(json_name);

        // 如果该 JSON 数据对象不存在
        if (!target_json) {
            std::cerr << "nuscenes json writer:" << "JSON object with name '" << json_name << "' not found." << std::endl;
            return;
        }

        // 检查是否包含该 key_name
        if (!target_json->contains(key_name)) {
            // 如果场景不存在，创建一个新场景，并初始化 "data" 数组
            json new_scene;
            new_scene["data"] = json::array();  // 初始化 "data" 为一个空数组

            // 将新的数据项添加到 "data" 数组中
            new_scene["data"].push_back(new_data);

            // 将新的场景添加到 JSON 中
            (*target_json)[key_name] = new_scene;
            std::cout << "nuscenes json writer:" << "Scene '" << key_name << "' added with new 'data' array." << std::endl;
        } else {
            // 如果场景已存在
            auto& scene_data = (*target_json)[key_name];

            // 确保场景内存在 "data" 数组
            if (scene_data.contains("data") && scene_data["data"].is_array()) {
                // 将新的数据项添加到 "data" 数组中
                scene_data["data"].push_back(new_data);
                std::cout << "nuscenes json writer:" << "Added new data to existing scene '" << key_name << "' under 'data'." << std::endl;
            } else {
                // 如果 "data" 不存在或不是数组，创建 "data" 数组
                std::cerr << "nuscenes json writer:" << "Scene '" << key_name << "' exists, but does not contain 'data' array. Creating it now." << std::endl;
                scene_data["data"] = json::array();
                scene_data["data"].push_back(new_data);
            }
        }

        // 如果需要保存到文件
        if (is_save) {
            auto it = file_path_map.find(json_name);
            if (it != file_path_map.end()) {
                bool save_success = saveJsonToFile(it->second, json_name);
                if (save_success) {
                    //std::cout << "nuscenes json writer:" << "Changes saved immediately to file: " << it->second << std::endl;
                } else {
                    std::cerr << "nuscenes json writer:" << "Failed to save changes to file: " << it->second << std::endl;
                }
            } else {
                std::cerr<< "nuscenes json writer:"  << "File path for JSON '" << json_name << "' not found." << std::endl;
            }
        }
    }

    // 向 JSON 数组中添加一个字典对象（键值对），并可选择是否立即保存
    void addKeyValuePairToJson(const std::string& json_name, const json& key_value_pairs, bool is_save) {
        auto target_json = getJson(json_name);
        if (target_json) {
            // 将传入的字典对象（key_value_pairs）作为新对象，添加到数组中
            target_json->push_back(key_value_pairs);
            //std::cout << "nuscenes json writer:" << "Added a new object to JSON array '" << json_name << "': " << key_value_pairs.dump() << "\n";

            if (is_save) {
                // 查找对应的文件名的文件路径
                auto it = file_path_map.find(json_name);
                if (it != file_path_map.end()) {
                    bool save_success = saveJsonToFile(it->second, json_name);
                    if (save_success) {
                        //std::cout << "nuscenes json writer:" << "Changes saved immediately to file: " << it->second << "\n";
                    } else {
                        std::cerr << "nuscenes json writer:" << "Failed to save changes to file: " << it->second << "\n";
                    }
                } else {
                    std::cerr << "nuscenes json writer:" << "File name for '" << json_name << "' not found.\n";
                }
            }
        }
    }

    // 保存 json_map 中的数据到文件
    bool saveJsonToFile(const std::string& file_path, const std::string& json_key) {
        auto target_json = getJson(json_key);
        if (!target_json) {
            std::cerr << "nuscenes json writer:" << "No JSON data found for key '" << json_key << "'\n";
            return false;
        }

        // 打开文件并写入 JSON 数据
        std::ofstream out(file_path);
        if (out.is_open()) {
            out << target_json->dump(4);  // 格式化输出 JSON 数据
            out.close();
            //std::cout << "nuscenes json writer:" << "Saved JSON data to " << file_path << "\n";
            return true;
        } else {
            std::cerr << "nuscenes json writer:" << "Failed to open file for writing: " << file_path << "\n";
            return false;
        }
    }

    // TODO:unused
    // 获取指定 JSON 数组的最后(最新)一个元素
    json* getLastJsonItem(const std::string& json_name) {
        auto target_json = getJson(json_name);
        if (target_json && !target_json->empty()) {
            // 返回 JSON 数组的最后一项
            return &(*target_json)[target_json->size() - 1];
        } else {
            std::cerr << "nuscenes json writer:" << "JSON with json_name '" << json_name << "' is empty or not found.\n";
            return nullptr;
        }
    }

private:
    // 存储 JSON 数据，键为文件名（包含扩展名）
    std::unordered_map<std::string, json> json_map;
    // 存储文件名与文件路径的映射，用于搜索文件路径
    std::unordered_map<std::string, std::string> file_path_map;
    int load_failure_count_;

    // 根据键获取 JSON 数组
    json* getJson(const std::string& json_name) {
        auto it = json_map.find(json_name);
        if (it != json_map.end()) {
            return &(it->second);  // 返回 JSON 数组的引用
        } else {
            std::cerr << "nuscenes json writer:" << "JSON with json_name '" << json_name << "' not found.\n";
            return nullptr;
        }
    }

    // 提取文件名（包含扩展名）作为 JSON 对象的键
    std::string extractFileName(const std::string& file_path) const {
        size_t last_slash_pos = file_path.find_last_of("/\\");
        std::string_view file_name_view = file_path;  // 使用 std::string_view 引用整个路径

        if (last_slash_pos != std::string::npos) {
            file_name_view = file_name_view.substr(last_slash_pos + 1);  // 提取文件名（包含扩展名）
        }

        // 查找最后一个点 (.) 的位置，用来去除扩展名
        size_t dot_pos = file_name_view.find_last_of('.');
        if (dot_pos != std::string_view::npos) {
            return std::string(file_name_view.substr(0, dot_pos));  // 去掉扩展名
        }

        return std::string(file_name_view);  // 如果没有扩展名，则直接返回文件名
    }

};

#endif // NUSCENES_WRITER_HPP

