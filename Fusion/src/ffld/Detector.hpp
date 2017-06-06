#pragma once

#include <vector>

#include "Mixture.h"
#include "Rectangle.h"
#include "HOGPyramid.h"

namespace FFLD {

// Обнаружение
struct Detection : public Rectangle
{
	float score;

    // позиция в пирамиде
	int x;
	int y;
	int z;

    // TODO: argmax из микстуры
	
    Detection() : score(0), x(0), y(0), z(0), Rectangle(0, 0, 0, 0)
	{
	}
	
	Detection(float score, int x, int y, int z, Rectangle bndbox) : Rectangle(bndbox),
    score(score), x(x), y(y), z(z)
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
}
