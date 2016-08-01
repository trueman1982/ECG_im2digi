#include<math.h>  
#include<stdlib.h>  
#include<stdio.h>
#include<vector>
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
   
//using namespace cv;

double calc_dist(uchar b1, uchar g1, uchar r1, uchar b2, uchar g2, uchar r2)
{
  return sqrt((b1-b2)*(b1-b2) + (g1-g2)*(g1-g2) + (r1-r2)*(r1-r2));
}
  
int main(int argc, char** argv)
{  
    int height ,width ,step ,channels;
    int same, lighter;
    float thresh;  
    uchar *dataB, *dataG, *dataR, *dataGray, *dataD;
    uchar b1, g1, r1, b2, g2, r2;
    int w = 3;
    int th = 50;
    int idx1, idx2;
    
    cv::Mat img = cv::imread("../temp/5.jpg");
    height = img.rows;
    width = img.cols;
    //height = img.cols;
    //width = img.rows;
    
    std::vector<cv::Mat> imgChannels;
    cv::split(img, imgChannels);
    cv::Mat dstSusan(height, width, CV_8UC1, cv::Scalar(0));
    cv::Mat grayImg(height, width, CV_8UC1, cv::Scalar(0));
    cv::cvtColor(img, grayImg, CV_BGR2GRAY);
    

    step = imgChannels[0].step[0];
    dataB = imgChannels[0].data;
    dataG = imgChannels[1].data;
    dataR = imgChannels[2].data;
    dataGray = grayImg.data;
    dataD= dstSusan.data;

    for (int x = w; x < width-w; x++)
      {
	for (int y = w; y < height-w; y++)
	  {
	    same = 0;
	    lighter = 0;
	    idx1 = x + y * step;
	    b1 = dataB[idx1];
	    g1 = dataG[idx1];
	    r1 = dataR[idx1];
	    for (int u = 0; u < w+1; u++)
	      {
		for (int v = 0; v < w+1; v++)
		  {
		    if (u + v == 0)
		      {
			continue;
		      }
		    idx2 = (x+u) + (y+v) * step;
		    b2 = dataB[idx2];
		    g2 = dataG[idx2];
		    r2 = dataR[idx2];
		    if (calc_dist(b1, g1, r1, b1, g2, r2) < th)
		      {
			same += 1;
		      }
		    if (dataGray[idx2] > dataGray[idx1])
		      {
			lighter += 1;
		      }
		    idx2 = (x-u) + (y+v) * step;
		    b2 = dataB[idx2];
		    g2 = dataG[idx2];
		    r2 = dataR[idx2];

		    if (u != 0 && calc_dist(b1, g1, r1, b1, g2, r2) < th)
		      {
			same += 1;
		      }
		    if (u != 0 && dataGray[idx2] > dataGray[idx1])
		      {
			lighter += 1;
		      }
		    idx2 = (x+u) + (y-v) * step;
		    b2 = dataB[idx2];
		    g2 = dataG[idx2];
		    r2 = dataR[idx2];
		    if (v != 0 && calc_dist(b1, g1, r1, b1, g2, r2) < th)
		      {
			same += 1;
		      }
		    if (v != 0 && dataGray[idx2] > dataGray[idx1])
		      {
			lighter += 1;
		      }
		    idx2 = (x-u) + (y-v) * step;
		    b2 = dataB[idx2];
		    g2 = dataG[idx2];
		    r2 = dataR[idx2];
		    if (u != 0 && v != 0 && calc_dist(b1, g1, r1, b1, g2, r2) < th)
		      {
			same += 1;
		      }
		    if (u != 0 && v!= 0 && dataGray[idx2] > dataGray[idx1])
		      {
			lighter += 1;
		      }
		  }
	      }
	    dataD[idx1] = uchar(255.0 * float(same) / ((2*w+1) * (2*w+1) - 1));
	    if (dataD[idx1] < 128)
	      {
		dataD[idx1] = 255;
	      }
	    else
	      {
		dataD[idx1] = 0;
	      }
	    /*
	    if (lighter < 4 * w * w)
	      {
		dataD[idx1] = 0;
	      }
	    else
	      {
		dataD[idx1] = uchar(255.0 * float(same) / ((2*w+1) * (2*w+1) - 1));

		if (dataD[idx1] > 100)
		  {
		    dataD[idx1] = 0;
		  }
		else
		  {
		    dataD[idx1] = 255;
		  }
	      }
	    */
	  }
      }

    cv::Mat dst(height, width, CV_8UC1, cv::Scalar(0));
    cv::Canny(dstSusan, dst, 50, 200, 3);
    /*
    for (int x = 0; x < width; x++)
      {
	for (int y = 0; y < height; y++)
	  {
	    printf("%d ", dst.data[x + y * step]);
	  }
	printf("\r\n");
	}*/
    //Hough line detection
    std::vector<cv::Vec4i> lines;
    HoughLinesP(dstSusan, lines, 1, CV_PI/180, 80, 500, 20);

    printf("%ld\r\n", lines.size());

    double thetaSum = 0.0;
    int thetaNum = 0;
    double theta;
    for(size_t i = 0; i < lines.size(); i++)
    {  
      cv::Vec4i l = lines[i];
      ///cv::line(img, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(186,88,255), 1, CV_AA);
      if (l[0] == l[2])
	{
	  theta = CV_PI / 2;
	}
      else
	{
	  theta = std::atan(-double(l[3]-l[1]) / (l[2] - l[0]));
	}
      if (theta >= -CV_PI / 4 && theta <= CV_PI / 4)
	{
	  thetaSum += theta;
	  thetaNum += 1;
	  printf("%lf\r\n", theta);
	}
    }

    theta = -thetaSum / thetaNum * 180 / CV_PI;

    printf("%lf\r\n", theta);

    cv::Mat rotateImg(height, width, CV_8UC3);
    cv::Point2f center;
    center.x = float(width / 2.0 + 0.5);
    center.y = float(height / 2.0 + 0.5);
    cv::Mat affineMat = getRotationMatrix2D(center, theta, 1);
    cv::warpAffine(img,rotateImg, affineMat, cv::Size(width, height), CV_INTER_LINEAR+CV_WARP_FILL_OUTLIERS);
    
    cv::namedWindow("Image0", cv::WINDOW_NORMAL);
    cv::imshow("Image0", rotateImg);
    cv::waitKey();
    
    return 0;  
}  
