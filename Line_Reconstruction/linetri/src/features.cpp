#include "linetri/features.h"

// Point feature
namespace linetri{

LineFeature::LineFeature(int idx_, Vector2d sp_, Vector2d ep_)
{
  idx.push_back(idx_);
  sp.push_back(sp_);
  ep.push_back(ep_);
}

void LineFeature::output()
{
	for(int i:idx)
		cout<< i <<" ";
	cout<<endl<<"sp:";
	for(Vector2d sp_:sp)
	{
		cout<<"["<<sp_[0]<<", "<<sp_[1]<<"] ";
	}
	cout<<endl<<"ep:";
	for(Vector2d ep_:ep)
	{
		cout<<"["<<ep_[0]<<", "<<ep_[1]<<"] ";
	}
	cout<<endl<<"------------------------------------"<<endl;
}
}
