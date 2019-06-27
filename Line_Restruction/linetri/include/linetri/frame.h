#ifndef FRAME_H
#define FRAME_H

#include "linetri/common_include.h"
#include "linetri/config.h"
#include "linetri/features.h"

namespace linetri{
class Frame
{

public:

    Frame(){};
    ~Frame(){};

    vector<KeyLine> lines;
    Mat desc;
	vector<int> data_id;
    void addframe(Mat img, int num, vector<LineFeature>& lineset);
    void initframe(Mat img, int num, vector<LineFeature>& lineset);
};
}
#endif