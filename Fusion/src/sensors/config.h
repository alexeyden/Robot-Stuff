#ifndef CONFIG_H
#define CONFIG_H

#include <glm/glm.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

class config
{
public:
    static bool init(const char* path);
    template<typename T>
    static T get(const char* prop);

private:
    static boost::property_tree::ptree _tree;
};

template<typename T> T config::get(const char* prop) {
    return _tree.get<T>(prop);
}
#endif // CONFIG_H
