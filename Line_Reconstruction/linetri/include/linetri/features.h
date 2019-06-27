#ifndef FEATURES_H
#define FEATURES_H

#include "linetri/common_include.h"
#include "linetri/config.h"
namespace linetri{
class LineFeature
{
public:
    LineFeature( int idx_, Vector2d sp_, Vector2d ep_);

    vector<int> idx;
    vector<Vector2d> sp, ep;
	void output();
};
}
#endif
