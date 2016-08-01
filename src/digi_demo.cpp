#include <iostream>
#include <math.h>  
#include <stdlib.h>  
#include <stdio.h>
#include <vector>
#include <sstream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "SegmentLine.h"
#include "Segment.h"
#include "SegmentFactory.h"

double calc_dist(uchar b1, uchar g1, uchar r1, uchar b2, uchar g2, uchar r2)
{
  return sqrt((b1-b2)*(b1-b2) + (g1-g2)*(g1-g2) + (r1-r2)*(r1-r2));
}

bool is_gray(uchar b1, uchar g1, uchar r1)
{
  if (b1 + g1 + r1 == 0)
    {
      return false;
    }
  
  double cosine = double(b1 + g1 + r1) / sqrt(b1 * b1 + g1 * g1 + r1 * r1) / sqrt(3);
  std::cout << int(b1) << ", " << int(g1) << ", " << int(r1) << ", " << cosine << std::endl;
  if (cosine < 0.999)
    {
      return false;
    }
  else
    {
      return true;
    }
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

  cv::Mat img = cv::imread(argv[1]);
  height = img.rows;
  width = img.cols;

  cv::namedWindow("Image0", cv::WINDOW_NORMAL);
  cv::Mat textImg(1000, 1200, CV_8UC1, cv::Scalar(255));
  cv::putText(textImg, "Original Image:", cv::Point(400, 500), cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(0, 0, 0));
  cv::imshow("Image0", textImg);
  cv::waitKey();
  cv::imshow("Image0", img);
  cv::waitKey();

  textImg.setTo(cv::Scalar(255, 255, 255));
  cv::putText(textImg, "Next: Apply SUSAN algorithm to detect edge and cross.", cv::Point(200, 500), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 255));
  cv::putText(textImg, "Press any key to continue...", cv::Point(400, 600), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 255));
  cv::imshow("Image0", textImg);
  cv::waitKey();

  std::vector<cv::Mat> imgChannels;
  cv::split(img, imgChannels);
  cv::Mat dstSusan(height, width, CV_8UC1, cv::Scalar(0));
  cv::Mat grayImg(height, width, CV_8UC1, cv::Scalar(0));
    
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

		  idx2 = (x-u) + (y+v) * step;
		  b2 = dataB[idx2];
		  g2 = dataG[idx2];
		  r2 = dataR[idx2];

		  if (u != 0 && calc_dist(b1, g1, r1, b1, g2, r2) < th)
		    {
		      same += 1;
		    }

		  idx2 = (x+u) + (y-v) * step;
		  b2 = dataB[idx2];
		  g2 = dataG[idx2];
		  r2 = dataR[idx2];
		  if (v != 0 && calc_dist(b1, g1, r1, b1, g2, r2) < th)
		    {
		      same += 1;
		    }

		  idx2 = (x-u) + (y-v) * step;
		  b2 = dataB[idx2];
		  g2 = dataG[idx2];
		  r2 = dataR[idx2];
		  if (u != 0 && v != 0 && calc_dist(b1, g1, r1, b1, g2, r2) < th)
		    {
		      same += 1;
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
	}
    }

  cv::imshow("Image0", dstSusan);
  textImg.setTo(cv::Scalar(255, 255, 255));
  cv::putText(textImg, "Next: Apply Hough algorithm to detect lines.", cv::Point(300, 500), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 255));
  cv::putText(textImg, "Press any key to continue...", cv::Point(400, 600), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 255));
  cv::waitKey();
  cv::imshow("Image0", textImg);
  cv::waitKey();

  //Hough line detection
  std::vector<cv::Vec4i> lines;
  HoughLinesP(dstSusan, lines, 1, CV_PI/180, 80, 500, 20);

  double thetaSum = 0.0;
  int thetaNum = 0;
  double theta;
  for(size_t i = 0; i < lines.size(); i++)
    {  
      cv::Vec4i l = lines[i];
      cv::line(img, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(186,88,255), 1, CV_AA);
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
	}
    }

  theta = -thetaSum / thetaNum * 180 / CV_PI;
  
  cv::imshow("Image0", img);
  cv::waitKey();
  textImg.setTo(cv::Scalar(255, 255, 255));
  std::ostringstream textStr;
  textStr << "Find " << lines.size() << " lines.";
  cv::putText(textImg, textStr.str(), cv::Point(500, 400), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 255));
  textStr.str(std::string());
  textStr.clear();
  textStr << "Rotating angle is " << theta << " degree.";
  cv::putText(textImg, textStr.str(), cv::Point(350, 500), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 255));
  cv::putText(textImg, "Next: Rotating the image.", cv::Point(400, 600), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 255));
  cv::putText(textImg, "Press any key to continue...", cv::Point(400, 700), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 255));
  cv::imshow("Image0", textImg);
  cv::waitKey();

  img.release();
  img = cv::imread(argv[1]);
  imgChannels[0].release();
  imgChannels[1].release();
  imgChannels[2].release();
  imgChannels.clear();
  
  cv::Mat rotateImg(height, width, CV_8UC3);
  cv::Point2f center;
  center.x = float(width / 2.0 + 0.5);
  center.y = float(height / 2.0 + 0.5);
  cv::Mat affineMat = getRotationMatrix2D(center, theta, 1);
  cv::warpAffine(img,rotateImg, affineMat, cv::Size(width, height), CV_INTER_LINEAR+CV_WARP_FILL_OUTLIERS);

  cv::imshow("Image0", rotateImg);
  cv::waitKey();
  textImg.setTo(cv::Scalar(255, 255, 255));
  cv::putText(textImg, "Next: Transform the image to gray scale.", cv::Point(300, 500), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 255));
  cv::putText(textImg, "Press any key to continue...", cv::Point(400, 600), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 255));
  cv::imshow("Image0", textImg);
  cv::waitKey();

  cv::split(rotateImg, imgChannels);
  dataB = imgChannels[0].data;
  dataG = imgChannels[1].data;
  dataR = imgChannels[2].data;
  step = imgChannels[0].step[0];
  //imgChannels[2].setTo(cv::Scalar(0));
  for (int x = 0; x < rotateImg.cols; x++)
    {
      for (int y = 0; y < rotateImg.rows; y++)
	{
	  int idx = x + y * step;
	  if (dataB[idx] < dataG[idx] && dataB[idx] < dataR[idx])
	    {
	      dataG[idx] = dataB[idx];
	      dataR[idx] = dataB[idx];
	    }
	  if (dataG[idx] < dataB[idx] && dataG[idx] < dataR[idx])
	    {
	      dataB[idx] = dataG[idx];
	      dataR[idx] = dataG[idx];
	    }
	  if (dataR[idx] < dataB[idx] && dataR[idx] < dataG[idx])
	    {
	      dataB[idx] = dataR[idx];
	      dataG[idx] = dataR[idx];
	    }
	}
    }
  cv::Mat filterRedImg(rotateImg.rows, rotateImg.cols, CV_8UC3, cv::Scalar::all(255));
  cv::merge(imgChannels, filterRedImg);

  cv::cvtColor(filterRedImg, grayImg, CV_BGR2GRAY);

  cv::imshow("Image0", grayImg);
  cv::waitKey();
  textImg.setTo(cv::Scalar(255, 255, 255));
  cv::putText(textImg, "Next: Clean the noise.", cv::Point(450, 500), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 255));
  cv::putText(textImg, "Press any key to continue...", cv::Point(400, 600), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 255));
  cv::imshow("Image0", textImg);
  cv::waitKey();

  step = grayImg.step[0];
  for (int x = 0; x < width; x++)
    {
      for (int y = 0; y < height; y++)
	{
	  int idx = x + y * step;
	  if (grayImg.data[idx] > 100)
	  //if(!is_gray(dataB[idx], dataG[idx], dataR[idx]))
	    {
	      grayImg.data[idx] = 255;
	    }
	}
    }

  cv::imshow("Image0", grayImg);
  cv::waitKey();
  textImg.setTo(cv::Scalar(255, 255, 255));
  cv::putText(textImg, "Next: Digitizing the curves.", cv::Point(400, 500), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 255));
  cv::putText(textImg, "Press any key to continue...", cv::Point(400, 600), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 255));
  cv::imshow("Image0", textImg);
  cv::waitKey();

  cv::Mat newImg(height, width, CV_8UC3, cv::Scalar::all(255));
  
  SegmentFactory segFactory = SegmentFactory(0);
  std::vector<Segment *> segments;

  segFactory.makeSegments(grayImg, segments);

  std::vector<Segment *>::iterator itr;
  for (itr = segments.begin(); itr != segments.end(); itr++)
    {
      Segment *seg = *itr;
      std::vector<SegmentLine *>::iterator itr_l;
      for (itr_l = seg->m_lines.begin(); itr_l != seg->m_lines.end(); itr_l++)
	{
	  SegmentLine *line = *itr_l;
	  cv::line(newImg, cv::Point(line->m_x1, line->m_y1), cv::Point(line->m_x2, line->m_y2), cv::Scalar(186,88,255), 1, CV_AA);
	  std::cout << line->m_x1 << ", " << line->m_y1 << ", " << line->m_x2 << ", " << line->m_y2 << std::endl;
	}
    }

  cv::imshow("Image0", newImg);
  cv::waitKey();
  textImg.setTo(cv::Scalar(255, 255, 255));
  cv::putText(textImg, "Done.", cv::Point(550, 500), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 255));
  cv::imshow("Image0", textImg);
  cv::waitKey();

  return 0;  
}  
