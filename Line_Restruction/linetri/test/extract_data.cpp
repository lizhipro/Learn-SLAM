#include <iostream>
#include "linetri/common_include.h"
#include "linetri/frame.h"
#include "linetri/features.h"

using namespace std;
using namespace cv;
using namespace linetri;

int main(int argc, char** argv)
{
	if(argc!=3)
		return 1;
	string images_dir = argv[1];
	int show_id = atoi(argv[2]);
	vector<string> images;
	
	ifstream Images_txt(images_dir+"Images.txt");
	assert(Images_txt.is_open());
	
	while(!Images_txt.eof())
	{
		string s;
		Images_txt>>s;
		if(s!="")
			images.push_back(s);
	}
	Images_txt.close();

	for(string t:images)
	{
		cout<<t<<" OK"<<endl;
	}
	cout<<images.size()<<endl;
	Frame frame;
	vector<LineFeature> lineset;

	Mat image = imread(images_dir+*(images.begin()), 1);
	frame.initframe(image, 1, lineset);
	
	for(int i=1; i<images.size(); i++)
	{
		image = imread(images_dir+*(images.begin()+i), 1);
		frame.addframe(image, i+1, lineset);
	}

	for(int i=0; i<lineset.size(); i++)
	{
		if(lineset[i].idx.size()==1)
		{
			lineset.erase(lineset.begin()+i);
			i--;
		}
	}

	cout<<lineset.size();
/*	
	ofstream idx(images_dir+"idx.txt");
	ofstream sp(images_dir+"sp.txt");
	ofstream ep(images_dir+"ep.txt");
	int i=1;
	
	for(LineFeature l:lineset)
	{
		for(int i:l.idx)
		{
			idx<<i<<" ";
		}
		idx<<endl; 
		for(Vector2d sp_:l.sp)
		{
			sp<<sp_[0]<<" ";
		}
		for(Vector2d ep_:l.ep)
		{
			ep<<ep_[0]<<" ";
		}
		sp<<endl; ep<<endl;
		for(Vector2d sp_:l.sp)
		{
			sp<<sp_[1]<<" ";
		}
		for(Vector2d ep_:l.ep)
		{
			ep<<ep_[1]<<" ";
		}
		sp<<endl; ep<<endl;	
		cout<<i<<endl;
		l.output();
		i++;
	}
*/	
	/******************** Test ***********************/
	for(int i=0; i<lineset[show_id].idx.size(); i++)
	{
		Mat image = imread(images_dir+images[lineset[show_id].idx[i]-1], 1);
		Mat out = image.clone();		
		
		Point pt1 = Point2f( lineset[show_id].sp[i][0], lineset[show_id].sp[i][1] );
		Point pt2 = Point2f( lineset[show_id].ep[i][0], lineset[show_id].ep[i][1] );
		line( out, pt1, pt2, Scalar( 255, 0, 0 ), 2 );
		imshow("lines", out);
		waitKey();
	}
	/*************************************************/
}