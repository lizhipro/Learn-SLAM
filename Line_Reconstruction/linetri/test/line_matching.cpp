#include <opencv/cv.h>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/core/core.hpp"
#include <opencv2/calib3d/calib3d.hpp>

#include <line_descriptor_custom.hpp>
#include <line_descriptor/descriptor_custom.hpp>
#include <iostream>
#include <linetri/config.h>
#include <linetri/auxiliar.h>

using namespace cv;
using namespace line_descriptor;
using namespace linetri;

#include <eigen3/Eigen/Core>
#include <../opencv2/include/opencv2/core/core.hpp>
using namespace Eigen;

static const char* keys =
    {
      "{ 1 | | | path to image1 }"
      "{ 2 | | | path to image2 }"
    };
    
Mat K = ( Mat_<double> ( 3,3 ) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1 );
Mat R = ( Mat_<double> (3,3) <<
  0.9985961798781875,	-0.05169917220143662,	0.01152671359827873,
  0.05139607508976055,	0.9983603445075083,	0.02520051547522442,
  -0.01281065954813571,	-0.02457271064688495,	0.9996159607036126);
Mat t = ( Mat_<double> ( 3,1 ) << 
  -0.8220841067933337,
  -0.03269742706405412,
  0.5684264241053522 );

static void help()
{
  std::cout << "\nThis example shows the functionalities of lines extraction "
            << "and descriptors computation furnished by BinaryDescriptor class\n"
            << "Please, run this sample using a command in the form\n"
            << "./example_line_descriptor_compute_descriptors <path_to_input_image 1> "
            << "<path_to_input_image 2>" << std::endl;
}

void detectlines(Mat img, vector<KeyLine> &lines, Mat &ldesc,double min_line_length)
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
    if( lines.size()>Config::lsdNFeatures() && Config::lsdNFeatures()!=0  )
	{
		// sort lines by their response
		sort( lines.begin(), lines.end(), sort_lines_by_response() );
		lines.resize(Config::lsdNFeatures());
		// reassign index
		for( int i = 0; i < Config::lsdNFeatures(); i++ )
			lines[i].class_id = i;
	}
	lbd->compute( img, lines, ldesc);
}

Point2f pixel2cam ( const Point2d& p, const Mat& K )
{
    return Point2f
    (
        ( p.x - K.at<double>(0,2) ) / K.at<double>(0,0), 
        ( p.y - K.at<double>(1,2) ) / K.at<double>(1,1) 
    );
}

Point2f cam2pixel ( const Point2d& p, const Mat& K )
{
    return Point2f
    (
        p.x * K.at<double>(0,0) + K.at<double>(0,2) , 
        p.y * K.at<double>(1,1) + K.at<double>(1,2)
    );
}

int main( int argc,const char** argv )
{
	  /* get parameters from command line */
	String image_path1 = argv[1];
	String image_path2 = argv[2];

	if( image_path1.empty() || image_path2.empty() )
	{
		help();
		return -1;
	}
	/* load image */
	cv::Mat image_l = imread( image_path1, 1 );
	cv::Mat image_r = imread( image_path2, 1 );

	vector<KeyLine> lines_l, lines_r;
	cv::Mat desc_l, desc_r;
	detectlines(image_l,lines_l,desc_l,12);
	detectlines(image_r,lines_r,desc_r,12);
	
	BFMatcher* bfm = new BFMatcher( NORM_HAMMING2, false );
	vector<vector<DMatch> > matches_lr, matches_rl;
	bfm->knnMatch( desc_l,desc_r, matches_lr, 2);
	bfm->knnMatch( desc_r,desc_l, matches_rl, 2);
	sort( matches_lr.begin(), matches_lr.end(), sort_descriptor_by_queryIdx() );
	sort( matches_rl.begin(), matches_rl.end(), sort_descriptor_by_queryIdx() );
	int n_matches = min(matches_lr.size(),matches_rl.size());
	float minRatio = 2.0f;
	std::vector<DMatch> good_matches;
	for ( int i = 0; i < n_matches; i++ )
	{
		const DMatch& bestMatch = matches_lr[i][0];
		const DMatch& betterMatch = matches_lr[i][1];
		if(betterMatch.distance/bestMatch.distance > minRatio)
      			good_matches.push_back( bestMatch );
  	}

/******************************* line match *****************************************************
	cv::Mat output;
	vector<char> mask( matches_lr.size(), 1 );
	drawLineMatches( image_l, lines_l, image_r, lines_r, good_matches, output, Scalar::all( -1 ), Scalar::all( -1 ), mask,
                     DrawLinesMatchesFlags::DEFAULT );
        imshow( "Matches", output );
	waitKey();
*************************************************************************************************/
                     
	vector<Point2f> sp_l, ep_l, sp_r, ep_r;
	Mat p4d;
	vector<Point3d> sp, ep;
	for(DMatch m:good_matches)
	{
	  sp_l.push_back(pixel2cam(Point2f(lines_l[m.queryIdx].startPointX, lines_l[m.queryIdx].startPointY),K));
	  
	  ep_l.push_back(pixel2cam(Point2f(lines_l[m.queryIdx].endPointX, lines_l[m.queryIdx].endPointY),K));
	  
	  sp_r.push_back(pixel2cam(Point2f(lines_r[m.trainIdx].startPointX, lines_r[m.trainIdx].startPointY),K));
	  
	  ep_r.push_back(pixel2cam(Point2f(lines_r[m.trainIdx].endPointX, lines_r[m.trainIdx].endPointY),K));
	}

	Mat T1 = (Mat_<float> (3,4) <<
	    1,0,0,0,
	    0,1,0,0,
	    0,0,1,0);
	Mat T2 = (Mat_<float> (3,4) <<
	    R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2), t.at<double>(0,0),
	    R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2), t.at<double>(1,0),
	    R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2), t.at<double>(2,0));
	triangulatePoints( T1, T2, sp_l, sp_r, p4d );
	for(int i=0; i<p4d.cols; i++)
	{
	  Mat x = p4d.col(i);
	  x /= x.at<float>(3,0); // 归一化
	  Point3d p (
	      x.at<float>(0,0), 
	      x.at<float>(1,0), 
	      x.at<float>(2,0) 
	  );
	  sp.push_back( p );
	}
	p4d.release();
	triangulatePoints( T1, T2, ep_l, ep_r, p4d);
	for(int i=0; i<p4d.cols; i++)
	{
	  Mat x = p4d.col(i);
	  x /= x.at<float>(3,0); // 归一化
	  Point3d p (
	      x.at<float>(0,0), 
	      x.at<float>(1,0), 
	      x.at<float>(2,0) 
	  );
	  ep.push_back( p );
	}
	
	vector<Point> sp_p, ep_p;
	for(int i=0; i<sp.size(); i++)
	{
	  Mat sp_ = R*( Mat_<double>(3,1) << sp[i].x, sp[i].y, sp[i].z ) + t;
	  sp_ /= sp_.at<double>(2,0);
	  sp_p.push_back(cam2pixel(Point2f(sp_.at<double>(0,0),sp_.at<double>(1,0)), K));
	  
	  Mat ep_ = R*( Mat_<double>(3,1) << ep[i].x, ep[i].y, ep[i].z ) + t;
	  ep_ /= ep_.at<double>(2,0);
	  ep_p.push_back(cam2pixel(Point2f(ep_.at<double>(0,0),ep_.at<double>(1,0)), K));
	}
	
	Mat output = image_r.clone();
	for(int i=0; i<sp_p.size(); i++)
	{
	  line(output, sp_p[i], ep_p[i], Scalar(0,0,255), 1.5);
	  line(output, cam2pixel(sp_r[i], K), cam2pixel(ep_r[i], K), Scalar(0,255,0), 1.5);
	}
	imshow("Reprojection", output);
	waitKey();

/************************* draw extraction ****************************
	cv::Mat output = image_l.clone();	
	if( output.channels() == 1 )
    	cvtColor( output, output, COLOR_GRAY2BGR );
  	for ( size_t i = 0; i < lines.size(); i++ )
  	{
	    	KeyLine kl = lines[i];
	    	if( kl.octave == 0)
	    	{
	      		int R = ( rand() % (int) ( 255 + 1 ) );
	      		int G = ( rand() % (int) ( 255 + 1 ) );
	      		int B = ( rand() % (int) ( 255 + 1 ) );

	      		Point pt1 = Point2f( kl.startPointX, kl.startPointY );
	      		Point pt2 = Point2f( kl.endPointX, kl.endPointY );

	      		line( output, pt1, pt2, Scalar( B, G, R ), 2 );
	    	}
	}
  imshow( "LSD lines", output );
  waitKey();
**********************************************************************/
}

