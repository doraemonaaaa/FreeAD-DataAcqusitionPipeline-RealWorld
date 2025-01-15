#ifndef TOKEN_RULE_HPP
#define TOKEN_RULE_HPP
   
#include "rclcpp/rclcpp.hpp"
#include <nlohmann/json.hpp>
#include <filesystem>
#include <fstream>
#include <string>
#include <unordered_map>
#include <chrono>
#include <sstream>
#include <mutex> // For thread safety
#include "nuscenes_data_collector/utils.hpp"
#include "nuscenes_data_collector/nuscenes_json_writer.hpp"
    
    

std::string generate_sample_token(int sample_frame, const std::optional<std::string>& start_time){
    return generate_unique_token(sample_frame, "sample", start_time);
}

// std::tuple<std::string, std::string> generate_sample_prev_and_next(int sample_frame) {
//     // generate sample's next and prev token
//     std::string sample_prev_token, sample_next_token;
//     if(sample_frame > 1)
//         sample_prev_token = generate_unique_token(sample_frame - 1, "sample", start_system_time_token_);
//     else sample_prev_token = "";
//     if((sample_frame + 1) < target_sample_frame_)
//         sample_next_token = generate_unique_token(sample_frame + 1, "sample", start_system_time_token_);
//     else sample_next_token = "";
//     return std::make_tuple(sample_prev_token, sample_next_token);
// }
// 根据可见性百分比生成 visibility_token
std::string generate_visibility_token(double visibility_percentage) {
    if (visibility_percentage >= 0 && visibility_percentage < 0.4) {
        return "1";  // v0-40
    } else if (visibility_percentage >= 0.4 && visibility_percentage < 0.6) {
        return "2";  // v40-60
    } else if (visibility_percentage >= 0.6 && visibility_percentage < 0.8) {
        return "3";  // v60-80
    } else if (visibility_percentage >= 0.8 && visibility_percentage <= 1) {
        return "4";  // v80-100
    } else {
        return "1";  // 默认值，如果可见性超出范围
    }
}

std::string get_calibration_token(std::unordered_map<std::string, std::string> calibrated_sensor_token_map_, 
                                    const std::string device_name){
    std::string calibrated_sensor_token;
    auto calibrated_sensor_it = calibrated_sensor_token_map_.find(device_name);
        if (calibrated_sensor_it != calibrated_sensor_token_map_.end()) {
            calibrated_sensor_token = calibrated_sensor_it->second;
        } else return "";
    return calibrated_sensor_token;
}

// for sample_data or sample and instance
void generate_prev_next_and_write_json_dynamically(std::unordered_map<std::string, std::vector<json>>& prev_next_sliders, 
                                    json& target_json, 
                                    std::string key, 
                                    NuscenesJsonWriter& json_writer, 
                                    std::string write_json_name){
    // resolve next for each instance in slider map
    // get or generate a pair
    auto& this_slider = prev_next_sliders[key];  // update the instance class in prev_next_sliders_
    if(this_slider.size() < 2){
        this_slider.push_back(target_json);
        if(this_slider.size() == 2){  // solve it when slider is full
            this_slider[0]["next"] = this_slider[1]["token"];
            this_slider[1]["prev"] = this_slider[0]["token"];
            json_writer.addKeyValuePairToJson(write_json_name, this_slider[0], true);
            // pop front and slide
            this_slider[0] = this_slider[1];  
            this_slider.pop_back();
        }
    }
}

#endif