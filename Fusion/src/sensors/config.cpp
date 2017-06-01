#include "config.h"

boost::property_tree::ptree config::_tree;


template<> glm::vec3 config::get<glm::vec3>(const char* prop) {
    auto iter = _tree.get_child(prop).ordered_begin();
    float x = iter->second.get_value<float>();
    float y = (++iter)->second.get_value<float>();
    float z = (++iter)->second.get_value<float>();
    return glm::vec3(x,y,z);
}

bool config::init(const char *path) {
    boost::property_tree::read_json(path, _tree);
    return true;
}
