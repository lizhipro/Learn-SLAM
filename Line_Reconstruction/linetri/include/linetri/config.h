#ifndef CONFIG_H
#define CONFIG_H

#include "linetri/common_include.h" 

namespace linetri
{
class Config
{
public:

    Config();
    ~Config();

    static Config& getInstance();
    // lines detection and matching
    static int&     lsdNFeatures()       { return getInstance().lsd_nfeatures; }
    static int&     lsdRefine()          { return getInstance().lsd_refine; }
    static double&  lsdScale()           { return getInstance().lsd_scale; }
    static double&  lsdSigmaScale()      { return getInstance().lsd_sigma_scale; }
    static double&  lsdQuant()           { return getInstance().lsd_quant; }
    static double&  lsdAngTh()           { return getInstance().lsd_ang_th; }
    static double&  lsdLogEps()          { return getInstance().lsd_log_eps; }
    static double&  lsdDensityTh()       { return getInstance().lsd_density_th; }
    static int&     lsdNBins()           { return getInstance().lsd_n_bins; }
    static double&  lineHorizTh()        { return getInstance().line_horiz_th; }
    static double&  minLineLength()      { return getInstance().min_line_length; }
    static double&  descThL()            { return getInstance().desc_th_l; }
    static double&  minRatio12L()        { return getInstance().min_ratio_12_l; }
    static double&  lineCovTh()          { return getInstance().line_cov_th; }
    static double&  stereoOverlapTh()    { return getInstance().stereo_overlap_th; }

private:
    double stereo_overlap_th;
    // lines detection and matching
    int    lsd_nfeatures;
    int    lsd_refine;
    double lsd_scale;
    double lsd_sigma_scale;
    double lsd_quant;
    double lsd_ang_th;
    double lsd_log_eps;
    double lsd_density_th;
    int    lsd_n_bins;
    double line_horiz_th;
    double min_line_length;
    double desc_th_l;
    double min_ratio_12_l;
    double line_cov_th;

};
}

#endif // CONFIG_H
