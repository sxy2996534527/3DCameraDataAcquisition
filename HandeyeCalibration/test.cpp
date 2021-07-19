#include <string>
#include <chrono>
#include <iostream>
#include <stdlib.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>

cv::Mat skew( cv::Mat res )
{
	cv::Mat result = (cv::Mat_<double>(3, 3) << 0, -res.at<double>(2), res.at<double>(1),
		res.at<double>(2), 0, -res.at<double>(0),
		-res.at<double>(1), res.at<double>(0), 0);
		
	return result;
}
void Tsai_HandEye(cv::Mat Hcg, std::vector<cv::Mat> Hgij, std::vector<cv::Mat> Hcij)
{
	CV_Assert(Hgij.size() == Hcij.size());
	int nStatus = Hgij.size();

	cv::Mat Rgij(3, 3, CV_64FC1);
	cv::Mat Rcij(3, 3, CV_64FC1);

	cv::Mat rgij(3, 1, CV_64FC1);
	cv::Mat rcij(3, 1, CV_64FC1);

	double theta_gij;
	double theta_cij;

	cv::Mat rngij(3, 1, CV_64FC1);
	cv::Mat rncij(3, 1, CV_64FC1);

	cv::Mat Pgij(3, 1, CV_64FC1);
	cv::Mat Pcij(3, 1, CV_64FC1);

	cv::Mat tempA(3, 3, CV_64FC1);
	cv::Mat tempb(3, 1, CV_64FC1);

	cv::Mat A;
	cv::Mat b;
	cv::Mat pinA;

	cv::Mat Pcg_prime(3, 1, CV_64FC1);
	cv::Mat Pcg(3, 1, CV_64FC1);
	cv::Mat PcgTrs(1, 3, CV_64FC1);

	cv::Mat Rcg(3, 3, CV_64FC1);
	cv::Mat eyeM = cv::Mat::eye(3, 3, CV_64FC1);

	cv::Mat Tgij(3, 1, CV_64FC1);
	cv::Mat Tcij(3, 1, CV_64FC1);

	cv::Mat tempAA(3, 3, CV_64FC1);
	cv::Mat tempbb(3, 1, CV_64FC1);
	cv::Mat AA;
	cv::Mat bb;
	cv::Mat pinAA;
	cv::Mat Tcg(3, 1, CV_64FC1);

	for (int i = 0; i < nStatus; i++)
	{
		Hgij[i](cv::Rect(0, 0, 3, 3)).copyTo(Rgij);
		Hcij[i](cv::Rect(0, 0, 3, 3)).copyTo(Rcij);

		Rodrigues(Rgij, rgij);
		Rodrigues(Rcij, rcij);

		theta_gij = norm(rgij);
		theta_cij = norm(rcij);
		
		rngij = rgij / theta_gij;
		rncij = rcij / theta_cij;

		Pgij = 2 * sin(theta_gij / 2) * rngij;
		Pcij = 2 * sin(theta_cij / 2) * rncij;

		tempA = skew(Pgij + Pcij);
		tempb = Pcij - Pgij;

		A.push_back(tempA);
		b.push_back(tempb);
	}

	//Compute rotation
	invert(A, pinA, cv::DECOMP_SVD);

	Pcg_prime = pinA * b;
	Pcg = 2 * Pcg_prime / sqrt(1 + norm(Pcg_prime) * norm(Pcg_prime));
	PcgTrs = Pcg.t();
	Rcg = (1 - norm(Pcg) * norm(Pcg) / 2) * eyeM + 0.5 * (Pcg * PcgTrs + sqrt(4 - norm(Pcg)*norm(Pcg))*skew(Pcg));

	//Computer Translation 
	for (int i = 0; i < nStatus; i++)
	{
		Hgij[i](cv::Rect(0, 0, 3, 3)).copyTo(Rgij);
		Hcij[i](cv::Rect(0, 0, 3, 3)).copyTo(Rcij);
		Hgij[i](cv::Rect(3, 0, 1, 3)).copyTo(Tgij);
		Hcij[i](cv::Rect(3, 0, 1, 3)).copyTo(Tcij);


		tempAA = Rgij - eyeM;
		tempbb = Rcg * Tcij - Tgij;

		AA.push_back(tempAA);
		bb.push_back(tempbb);
	}

	invert(AA, pinAA, cv::DECOMP_SVD);
	Tcg = pinAA * bb;
	std::cout<< Rcg << std::endl;

    // std::cout<<pinAA<<std::endl;
	Rcg.copyTo(Hcg(cv::Rect(0, 0, 3, 3)));
	Tcg.copyTo(Hcg(cv::Rect(3, 0, 1, 3)));
	Hcg.at<double>(3, 0) = 0.0;
	Hcg.at<double>(3, 1) = 0.0;
	Hcg.at<double>(3, 2) = 0.0;
	Hcg.at<double>(3, 3) = 1.0;

}

cv::Mat turnMat(double* t)
{
    cv::Mat T = cv::Mat::eye(4, 4, CV_64FC1);
    for (int i = 0 ; i < 4 ; i ++)
    {
        for (int j = 0 ; j < 4 ; j++)
        {
            T.at<double>(i, j) = t[i*4+j];
        }
    }
    return T;
}

int main(int argc, char** argv)
{  
    double T_standard[16] = {-0.25524255, 0.12031712, -0.95936178, -0.6760785,
                            -0.09978927, 0.9836505,   0.14991265, -0.06037279,
                            0.96171375, 0.1339981,  -0.2390631,   0.50568673,
                            0,         0,          0,         1        };
    double T_pose[14][16] =
    {
        {-0.26383388,  0.18034671, -0.94755831, -0.67968672,
        -0.08162853,  0.97466671,  0.20823448, -0.0178505,
        0.96110794,  0.1322871,  -0.24242867,  0.50596305,
        0,          0,          0,          1        },
        
        {-0.1963116,   0.21239575, -0.95726162, -0.68870174,
        -0.11550519,  0.96445191,  0.23767849, -0.04151841,
        0.9737147,   0.15722772, -0.16480029,  0.51909631,
        0,         0,         0,         1       },
        
        {-0.169679,    0.23561441, -0.95691948, -0.7170147,
        -0.10342051,  0.96138082,  0.2550512,  -0.02941691,
        0.98005777,  0.14224194, -0.13875878,  0.52580124,
        0,          0,          0,          1       },

        {-0.26520668,  0.10583734, -0.95836521, -0.6773803,
        -0.15608149,  0.97613524,  0.15099193, -0.05860629,
        0.95147464,  0.18962714, -0.24235833,  0.50599987,
        0,          0,          0,          1},

        {-0.24069052,  0.15361661, -0.95836841, -0.67738464,
        0.0294564,   0.98809724,  0.15098397, -0.05861155,
        0.97015483,  0.00811033, -0.24235063,  0.50599069,
        0,          0,          0,          1        },

        {-0.3615591,   0.19425222, -0.91188875, -0.64662315,
        -0.67335138,  0.62209175,  0.39949941,  0.11971361,
        0.64488211,  0.75846419, -0.09412294,  0.59935488,
        0,          0,          0,          1        },

        {-0.19507234,  0.07267798, -0.97809237, -0.68566007,
        -0.51631044,  0.84027536,  0.16541117,  0.05632625,
        0.83388867,  0.53726644, -0.12639009,  0.56325553,
        0,          0,          0,          1        },

        {-0.23237602,  0.15030715, -0.9609418,  -0.66938051,
        -0.09921504,  0.97917007,  0.17715065, -0.08792771,
        0.96755246,  0.13650544, -0.21262292,  0.49808188,
        0,          0,          0,          1        },
    
        {-0.08356536,  0.24551896, -0.96578324, -0.72048522,
        0.1361197,   0.9629011,   0.23300838, -0.0834392 ,
        0.98716172, -0.1119907,  -0.11388515,  0.53090047,
        0,          0,          0,          1        },
        
        {-0.04729372,  0.14458483, -0.98836154, -0.70446721,
        0.47438652,  0.87401335,  0.10515744, -0.1747646,
        0.87904535, -0.4638921,  -0.10992444,  0.53345066,
        0,          0,          0,          1        },
    
        {-0.17503298, -0.27461233, -0.9454901,  -0.69823802,
        -0.66734834, -0.67296667,  0.31900166,  0.10523705,
        -0.72388511,  0.68680706, -0.06547072,  0.88044311,
        0,          0,          0,          1        },
    
        {-0.06531844, -0.26620852, -0.96169981, -0.69689061,
        -0.44634325, -0.85417599,  0.26676034,  0.07357126,
        -0.89247477,  0.44667259, -0.06302693,  0.92860157,
        0,          0,          0,          1        },
    
        {-0.02199697,  0.27766358, -0.9604265,  -0.71570685,
        0.22554917,  0.9372706,   0.26580328, -0.09693875,
        0.97398342, -0.21077654, -0.0832439,   0.5338395,
        0,          0,          0,          1        },
        
        {-0.1473856,  -0.17992332, -0.97257652, -0.68905526,
        -0.80031314, -0.55610438,  0.22415797,  0.13696474,
        -0.58118531,  0.81140343, -0.06203322,  0.83710394,
        0,          0,          0,          1        }
    };

    double T_icp[14][16] = 
    {
        {1.000, -0.011, -0.007,  1.496, 
        0.011,  0.998,  0.055, -42.174, 
        0.006, -0.055,  0.998, -12.222, 
        0.000,  0.000,  0.000,  1.000 },

        {0.998,  0.004,  0.069, -12.265,
        -0.010,  0.996,  0.090, -11.046, 
        -0.068, -0.091,  0.994,  3.755, 
        0.000,  0.000,  0.000,  1.000},
        
        { 0.996, -0.015,  0.093, -22.542, 
        0.005,  0.994,  0.108, -20.474, 
        -0.094, -0.107,  0.990, -18.740, 
        0.000,  0.000,  0.000,  1.000}, 
        
        {0.998,  0.064, -0.001, -0.909, 
        -0.064,  0.998,  0.001, 11.387, 
        0.001, -0.001,  1.000, -1.757,
        0.000,  0.000,  0.000,  1.000}, 

        {0.992, -0.128,  0.001,  0.013,
        0.128,  0.992,  0.001, -28.970,
        -0.001, -0.001,  1.000, -1.337, 
        0.000,  0.000,  0.000,  1.000}, 

        {0.775,  0.623,  0.106, -17.595, 
        -0.625,  0.733,  0.270, -48.337, 
        0.091, -0.275,  0.957,  0.140, 
        0.000,  0.000,  0.000,  1.000}, 
    
        {0.989, -0.127,  0.069, -12.234, 
        0.122,  0.989,  0.080, -24.133,
        -0.078, -0.070,  0.994, -0.615, 
        0.000,  0.000,  0.000,  1.000}, 

        {1.000, -0.005,  0.030,  5.644,
        0.005,  1.000,  0.028, 27.656, 
        -0.030, -0.028,  0.999, 16.522,
        0.000,  0.000,  0.000,  1.000}, 

        {0.956, -0.263,  0.128, -24.772, 
        0.253,  0.963,  0.087, -20.774, 
        -0.146, -0.051,  0.988, -0.945, 
        0.000,  0.000,  0.000,  1.000}, 

        {0.808, -0.569,  0.149,  1.221, 
        0.580,  0.813, -0.040, -8.701,
        -0.098,  0.119,  0.988, 19.394, 
        0.000,  0.000,  0.000,  1.000}, 
    
        // T_icp的第11、12、14 进行过初始变换
        {0.723, -0.680,  0.125, -8.953,
        0.651,  0.730,  0.207, -36.900, 
        -0.232, -0.068,  0.970, -20.702, 
        0.000,  0.000,  0.000,  1.000}, 

        {0.870, -0.476,  0.125, -11.664, 
        0.455,  0.875,  0.168, -56.815,
        -0.189, -0.089,  0.978,  4.210, 
        0.000,  0.000,  0.000,  1.000}, 
    
        {0.920, -0.364,  0.143, -14.118, 
        0.351,  0.930,  0.113, -22.443, 
        -0.174, -0.054,  0.983, 11.453, 
        0.000,  0.000,  0.000,  1.000}, 
        
        {0.584, -0.802,  0.126,  9.211, 
        0.798,  0.596,  0.089, -39.305,
        -0.146,  0.049,  0.988, -0.394,
        0.000,  0.000,  0.000,  1.000}
    }; 
    // 初始变换的矩阵
    double T_icp_111214[16] = {-0.98768834, 0.15643447, 0.0, 0.0,
                    -0.15643447, -0.98768834, 0.0, 0.0,
                    0, 0, 1, 0,
                    0.000,  0.000,  0.000,  1.000}; 
    
    // ICP的结果，x,y,z为毫米单位，所以需要转化
    for (int i = 0; i < 14; i++)
    {
        T_icp[i][3] =  T_icp[i][3] * 0.001;
        T_icp[i][7] =  T_icp[i][7] * 0.001;
        T_icp[i][11] = T_icp[i][11] * 0.001;
    }
    
    // ICP结果由数组变为矩阵
    std::vector<cv::Mat> T_icp_mat;
    for (int i = 0; i < 14; i++)
    {
        cv::Mat X = turnMat(T_icp[i]);
        T_icp_mat.push_back(X);
    }
    // 对第11、12、14次ICP结果进行补偿
    cv::Mat T_icp_initial = turnMat(T_icp_111214);
    cv::Mat T_icp_11_afterinitial = turnMat(T_icp[10]);
    cv::Mat T_icp_12_afterinitial = turnMat(T_icp[11]);
    cv::Mat T_icp_14_afterinitial = turnMat(T_icp[13]);
    T_icp_mat[10] = T_icp_11_afterinitial * T_icp_initial;
    T_icp_mat[11] = T_icp_12_afterinitial * T_icp_initial;
    T_icp_mat[13] = T_icp_14_afterinitial * T_icp_initial;  


    std::vector<cv::Mat> A;
    std::vector<cv::Mat> B;
    cv::Mat A_ = cv::Mat::eye(4, 4, CV_64FC1);
    cv::Mat B_ = cv::Mat::eye(4, 4, CV_64FC1);

    cv::Mat T_0 = turnMat(T_standard);
    for (int i = 0 ; i < 14; i++)
    {
        cv::Mat T = turnMat(T_pose[i]);
        cv::Mat K = T_0.inv() * T;
        for (int m = 0 ; m < 4 ; m ++)
        {
            for (int n = 0 ; n < 4 ; n++)
            {
                A_.at<double>(m, n) = K.at<double>(m, n);
                B_.at<double>(m, n) = T_icp_mat[i].at<double>(m, n);
            }
        }
        A.push_back(A_);
        B.push_back(B_);
        
        std::cout<<"A: "<<i<<" "<<A_<<std::endl;
        std::cout<<"B: "<<i<<" "<<B_<<std::endl;
    }
    
    cv::Mat eyeInHand = cv::Mat::eye(4, 4, CV_64FC1);

    std::cout<<"calib result"<<std::endl;
    Tsai_HandEye(eyeInHand, A, B);
    
    std::cout<<eyeInHand<<std::endl;
    return 1;
}
