#pragma once

#include <vector>

#include "Mixture.h"
#include "Rectangle.h"
#include "HOGPyramid.h"

using namespace FFLD;
using namespace std;

// Обнаружение
struct Detection : public Rectangle
{
	float score;
	int x;
	int y;
	int z;
    
    int bx0;
    int by0;
    int bx1;
    int by1;
	
	Detection() : score(0), x(0), y(0), z(0), bx0(0), bx1(0), by0(0), by1(0)
	{
	}
	
	Detection(float score, int x, int y, int z, Rectangle bndbox) : Rectangle(bndbox),
	score(score), x(x), y(y), z(z),
        bx0(bndbox.x()), by0(bndbox.y()), bx1(bndbox.x() + bndbox.width()), by1(bndbox.y() + bndbox.height())
	{
	}
	
	bool operator<(const Detection & detection) const
	{
		return score > detection.score;
	}
};

class FFLDDetector {
public:
    FFLDDetector(const Mixture& mixture);
    FFLDDetector(const std::string& txt);
    
    std::vector<Detection> run(int width, int height, int depth, const uint8_t* data);
    
    int interval = 5;
    int padding = 6;
    double threshold = -1.0;
    double overlap = 0.5;
    
    Mixture mixture;
};
