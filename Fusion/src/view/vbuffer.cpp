#include "vbuffer.h"

#include <cstring>
#include <fstream>
#include <sstream>
#include <vector>
#include <glm/glm.hpp>

vbuffer::vbuffer(const float *data, size_t count, GLenum type, size_t nprim) :
    _type(type)
{
    glGenVertexArrays(1, &_vao);
    glBindVertexArray(_vao);

    glGenBuffers(1, &_id);
    glBindBuffer(GL_ARRAY_BUFFER, _id);

    _data = nullptr;

    update(data, count);

    _nb = nprim;
    _count = count;
}

vbuffer::~vbuffer()
{
    glDeleteBuffers(1, &_id);
    glDeleteVertexArrays(1, &_vao);
}

void vbuffer::bind()
{
    glBindVertexArray(_vao);
    glBindBuffer(GL_ARRAY_BUFFER, _id);
}

void vbuffer::unbind()
{
    glBindVertexArray(0);
}

void vbuffer::draw(GLint first, GLsizei count)
{
    glDrawArrays(_type, first, count);
}

void vbuffer::draw()
{
    glDrawArrays(_type, 0, _nb);
}

void vbuffer::update(const float *data, size_t count)
{
    if(_data != nullptr)
        delete[] _data;

    _data = new float[count];
    memcpy(_data, data, count * sizeof(float));
    _count = count;

    glBufferData(GL_ARRAY_BUFFER, sizeof(float) * count, data, GL_DYNAMIC_DRAW);
}

void vbuffer::setup_attrib(GLint loc, GLint n, GLint offset, GLint size)
{
    glEnableVertexAttribArray(loc);
    glVertexAttribPointer(loc, n, GL_FLOAT, GL_FALSE,
                          sizeof(float) * size,
                          (void*) (sizeof(float) * offset));
}

bool vbuffer::save(const char *p, size_t vsize)
{
    std::ofstream os(p);

    if(!os.is_open())
        return false;

    for(size_t vi = 0; vi < _count / vsize; vi++) {
        os << "v " << _data[vi * vsize + 0] << " " << _data[vi * vsize + 1] << " " << _data[vi * vsize + 2] << std::endl;
    }

    for(size_t fi = 0; fi < _count / vsize / 3; fi++) {
        size_t vi = fi * 3 + 1;
        os << "f " << vi << " " << vi + 1 << " " << vi + 2 << std::endl;
    }

    return true;
}

std::shared_ptr<vbuffer> vbuffer::from_obj(const char *p)
{
    std::ifstream ifs(p);
    std::string line;

    std::vector<glm::vec3> vertices;
    std::vector<glm::vec2> uvs;
    std::vector<float> mesh;

    while(!std::getline(ifs, line).eof()) {
        if(line.size() == 0)
            continue;

        if(line[0] == 'v' && line[1] == ' ') {
            std::istringstream ss(line.substr(2));
            float x, y, z;
            ss >> x >> y >> z;
            vertices.push_back(glm::vec3(x, y, z));
        } else if(line[0] == 'v' && line[1] == 't') {
            std::istringstream ss(line.substr(3));
            float x, y;
            ss >> x >> y;
            uvs.push_back(glm::vec2(x, y));
        } else if(line[0] == 'f' && line[1] == ' ') {
            std::istringstream ss(line.substr(2));

            int v[3], t[3];
            for(int i = 0; i < 3; i ++) {
                ss >> v[i]; ss.ignore();
                ss >> t[i]; ss.ignore();
            }

            glm::vec3 v1 = vertices[v[0] - 1];
            glm::vec3 v2 = vertices[v[1] - 1];
            glm::vec3 v3 = vertices[v[2] - 1];
            glm::vec3 n = glm::normalize(glm::cross(v2 - v1, v3 - v1));
            float triangle[] = {
                v1.x, v1.y, v1.z, uvs[t[0] - 1].x, uvs[t[0] - 1].y, n.x, n.y, n.z,
                v2.x, v2.y, v2.z, uvs[t[1] - 1].x, uvs[t[1] - 1].y, n.x, n.y, n.z,
                v3.x, v3.y, v3.z, uvs[t[2] - 1].x, uvs[t[2] - 1].y, n.x, n.y, n.z,
            };
            mesh.insert(mesh.end(), triangle, triangle + 8 * 3);
        }
    }

    return std::shared_ptr<vbuffer> { new vbuffer(mesh.data(), mesh.size(), GL_TRIANGLES, mesh.size()/8) };
}
