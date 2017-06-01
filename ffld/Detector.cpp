#include "Detector.hpp"

#include "JPEGImage.h"
#include "HOGPyramid.h"
#include "Patchwork.h"
#include "Rectangle.h"
#include "Intersector.h"

#include <fstream>
#include <iostream>

std::vector<Detection> FFLDDetector::run(int width, int height, int depth, const uint8_t* data) {
    std::vector<Detection> detections;
    
    const JPEGImage image(width, height, depth, data);

	HOGPyramid pyramid(image, padding, padding, interval);
    
	if (!Patchwork::InitFFTW(
		(pyramid.levels()[0].rows() - padding + 15) & ~15,
		(pyramid.levels()[0].cols() - padding + 15) & ~15)) {
			cerr << "\nCould not initialize the Patchwork class" << endl;
			return detections;
	}

	mixture.cacheFilters();

	// Compute the scores
	vector<HOGPyramid::Matrix> scores;
	vector<Mixture::Indices> argmaxes;
	vector<vector<vector<Model::Positions> > > positions;
	
	mixture.convolve(pyramid, scores, argmaxes, &positions);
	
	// Cache the size of the models
	vector<pair<int, int> > sizes(mixture.models().size());
	
	for (int i = 0; i < sizes.size(); ++i)
		sizes[i] = mixture.models()[i].rootSize();
    
    // For each scale
	for (int z = 0; z < scores.size(); ++z) {
		const double scale = pow(2.0, static_cast<double>(z) / pyramid.interval() + 2);
		
		const int rows = static_cast<int>(scores[z].rows());
		const int cols = static_cast<int>(scores[z].cols());
		
		for (int y = 0; y < rows; ++y) {
			for (int x = 0; x < cols; ++x) {
				const double score = scores[z](y, x);
				
				if (score > threshold) {
					// Non-maxima suppresion in a 3x3 neighborhood
					if (((y == 0) || (x == 0) || (score >= scores[z](y - 1, x - 1))) &&
						((y == 0) || (score >= scores[z](y - 1, x))) &&
						((y == 0) || (x == cols - 1) || (score >= scores[z](y - 1, x + 1))) &&
						((x == 0) || (score >= scores[z](y, x - 1))) &&
						((x == cols - 1) || (score >= scores[z](y, x + 1))) &&
						((y == rows - 1) || (x == 0) || (score >= scores[z](y + 1, x - 1))) &&
						((y == rows - 1) || (score >= scores[z](y + 1, x))) &&
						((y == rows - 1) || (x == cols - 1) ||
						 (score >= scores[z](y + 1, x + 1)))) {
						Rectangle bndbox((x - pyramid.padx()) * scale + 0.5,
										 (y - pyramid.pady()) * scale + 0.5,
										 sizes[argmaxes[z](y, x)].second * scale + 0.5,
										 sizes[argmaxes[z](y, x)].first * scale + 0.5);
						
						// Truncate the object
						bndbox.setX(max(bndbox.x(), 0));
						bndbox.setY(max(bndbox.y(), 0));
						bndbox.setWidth(min(bndbox.width(), width - bndbox.x()));
						bndbox.setHeight(min(bndbox.height(), height - bndbox.y()));
						
						if (!bndbox.empty())
							detections.push_back(Detection(score, x, y, z, bndbox));
					}
				}
			}
		}
	}
	
    // Non maxima suppression
	sort(detections.begin(), detections.end());
	
	for (int i = 1; i < detections.size(); ++i)
		detections.resize(remove_if(detections.begin() + i, detections.end(),
									Intersector(detections[i - 1], overlap, true)) -
						  detections.begin());
    
    return detections;
} 

FFLDDetector::FFLDDetector(const std::string& txt) {
	ifstream in(txt.c_str(), ios::binary);
	in >> mixture;
}

FFLDDetector::FFLDDetector(const Mixture& m) : mixture(m) {}
