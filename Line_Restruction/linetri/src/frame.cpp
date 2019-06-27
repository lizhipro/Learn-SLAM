#include "linetri/frame.h"

namespace linetri {
void detectlines(Mat img, vector<KeyLine> &lines, Mat &ldesc, double min_line_length);

void Frame::addframe(Mat img, int num, vector<LineFeature>& lineset)
{
	vector<KeyLine> lines_r;
	Mat desc_r;
	detectlines(img, lines_r, desc_r, 25);
	
	BFMatcher* bfm = new BFMatcher( NORM_HAMMING2, false );
	vector<vector<DMatch> > matches;
	bfm->knnMatch( desc, desc_r, matches, 2);

	float minRatio = 2.0f;
	std::vector<DMatch> good_matches;
	for ( int i = 0; i < matches.size(); i++ )
	{
		const DMatch& bestMatch = matches[i][0];
		const DMatch& betterMatch = matches[i][1];
		if(betterMatch.distance/bestMatch.distance > minRatio)
			good_matches.push_back( bestMatch );
  	}
  	
  	lines.clear();
	vector<int> data_id_;
	vector<int> inliers;
  	for(DMatch m:good_matches)
	{
		lineset[data_id[m.queryIdx]].idx.push_back(num);
		lineset[data_id[m.queryIdx]].sp.push_back(Vector2d(lines_r[m.trainIdx].startPointX, lines_r[m.trainIdx].startPointY));
		lineset[data_id[m.queryIdx]].ep.push_back(Vector2d(lines_r[m.trainIdx].endPointX, lines_r[m.trainIdx].endPointY));
		
		lines.push_back(lines_r[m.trainIdx]);
		data_id_.push_back(data_id[m.queryIdx]);
		inliers.push_back(m.trainIdx);
	}
	for(int i=0; i<lines_r.size(); i++)
	{
		vector<int>::iterator it;
		it = find(inliers.begin(), inliers.end(), i);
		if(it == inliers.end())
		{
			lines.push_back(lines_r[i]);
			data_id_.push_back(lineset.size());
			lineset.push_back(LineFeature(num, Vector2d(lines_r[i].startPointX, lines_r[i].startPointY), Vector2d(lines_r[i].endPointX, lines_r[i].endPointY)));
		}
	}
	data_id = data_id_;
	Ptr<BinaryDescriptor>   lbd = BinaryDescriptor::createBinaryDescriptor();
	lbd->compute( img, lines, desc);
}
void Frame::initframe(Mat img, int num, vector< LineFeature >& lineset)
{
	detectlines(img, lines, desc, 25);

	for(int i=0; i<lines.size(); i++)
	{
		data_id.push_back(lineset.size());
		lineset.push_back(LineFeature(num, Vector2d(lines[i].startPointX, lines[i].startPointY), Vector2d(lines[i].endPointX, lines[i].endPointY)));

	}
}


void detectlines(Mat img, vector<KeyLine> &lines, Mat &ldesc, double min_line_length)
{
	Ptr<BinaryDescriptor>   lbd = BinaryDescriptor::createBinaryDescriptor();
	Ptr<line_descriptor::LSDDetectorC> lsd = line_descriptor::LSDDetectorC::createLSDDetectorC();
	line_descriptor::LSDDetectorC::LSDOptions opts;
	opts.refine       = Config::lsdRefine();
	opts.scale        = Config::lsdScale();
	opts.sigma_scale  = Config::lsdSigmaScale();
	opts.quant        = Config::lsdQuant();
	opts.ang_th       = Config::lsdAngTh();
	opts.log_eps      = Config::lsdLogEps();
	opts.density_th   = Config::lsdDensityTh();
	opts.n_bins       = Config::lsdNBins();
	opts.min_length   = min_line_length;

	lsd->detect( img, lines, Config::lsdScale(), 1, opts);
	//lsd->detect( img, lines, Config::lsdScale(), 1);
	lbd->compute( img, lines, ldesc);
}

}