#ifndef VISIONMODELS_HPP
#define VISIONMODELS_HPP
#include <vector>

namespace VisionModels {
//TODO:
//do we need to parametrize (with templates) the models?
//I think we only need float representation.

/**
   Represents a color model in two dimensions.
   r = R/(R+G+B)
   g = G/(R+G+B)
   Where R,G,B are the original values of a pixel color.
   see:

   http://en.wikipedia.org/wiki/Rg_chromaticity
   http://www.lucs.lu.se/LUCS/M012/Minor12.pdf
*/
    struct color_model_2d {
	color_model_2d(){}
	color_model_2d(float mu_r, float mu_g, float sig_00, float sig_01, float sig_10, float sig_11) :
	    mu_r(mu_r), mu_g(mu_g), sig_00(sig_00), sig_01(sig_01), sig_10(sig_10), sig_11(sig_11) {}

	//parameters:
	float mu_r;
	float mu_g;
	float sig_00;
	float sig_01;
	float sig_10;
	float sig_11;
    };

/**
   Represents a color model with a variable amount of dimensions.

   Note: assumes gaussian model. See

   http://en.wikipedia.org/wiki/Multivariate_normal_distribution
   http://en.wikipedia.org/wiki/Sample_mean_and_sample_covariance
*/
    template <class T>
    struct color_model_vardim {
	color_model_vardim() : dim(0) {}
	color_model_vardim(int dims) : dim(dims) {
	    mu = std::vector<T>(dims);
	    sigma = std::vector<std::vector<T> >(dims);
	    for(int i = 0; i < dims; ++i) {
            sigma[i] = std::vector<T>(dims);
	    }
	}

	std::vector<T> mu;
	std::vector<std::vector<T> > sigma;
	int dim;

    };

/**
   Model where all colors are independent. Old one.
*/
    struct color_model_3d_indep {
	color_model_3d_indep(){}
	color_model_3d_indep(float mu_r, float mu_g, float mu_b, float std_r, float std_g, float std_b) :
	    mu_r(mu_r), mu_g(mu_g), mu_b(mu_b), std_r(std_r),std_g(std_g),std_b(std_b){ }

	float mu_r;
	float mu_g;
	float mu_b;
	float std_r;
	float std_g;
	float std_b;
    };

    // must define templated functions inside header so that the compiler can see how to
    // define instances of the function that are called.
    template <typename T>
    void rgb2rg_chromaticity(T r, T g, T b, T& rc, T& gc) {
        T intensity = r+b+g;
        rc = r/intensity;
        gc = g/intensity;
    }
}

#endif // VISIONMODELS_HPP
