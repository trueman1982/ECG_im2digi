#include <iostream>
#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <Segment.h>
#include <SegmentFactory.h>

using namespace std;

void SegmentFactory::makeSegments (const cv::Mat &imageFiltered,
                                   vector<Segment*> &segments)
{
  int madeLines = 0;
  int shortLines = 0; // Lines rejected since their segments are too short
  int foldedLines = 0; // Lines rejected since they could be into other lines

  // For each new column of pixels, loop through the runs. a run is defined as
  // one or more colored pixels that are all touching, with one uncolored pixel or the
  // image boundary at each end of the set. for each set in the current column, count
  // the number of runs it touches in the adjacent (left and right) columns. here is
  // the pseudocode:
  //   if ((L > 1) || (R > 1))
  //     "this run is at a branch point so ignore the set"
  //   else
  //     if (L == 0)
  //       "this run is the start of a new segment"
  //     else
  //       "this run is appended to the segment on the left
  int width = imageFiltered.cols;
  int height = imageFiltered.rows;

  bool* lastBool = new bool [height];
  bool* currBool = new bool [height];
  bool* nextBool = new bool [height];
  SegmentVector lastSegment (height);
  SegmentVector currSegment (height);

  loadBool(lastBool, imageFiltered, -1);
  loadBool(currBool, imageFiltered, 0);
  loadBool(nextBool, imageFiltered, 1);
  loadSegment(lastSegment, height);

  for (int x = 0; x < width; x++)
    {
      matchRunsToSegments(x,
			  width,
			  height,
			  lastBool,
			  lastSegment,
			  currBool,
			  currSegment,
			  nextBool,
			  &madeLines,
			  &foldedLines,
			  &shortLines,
			  segments);

      // Get ready for next column
      scrollBool(lastBool, currBool, height);
      scrollBool(currBool, nextBool, height);
      if (x + 1 < width) {
	loadBool(nextBool, imageFiltered, x + 1);
      }
      scrollSegment(lastSegment, currSegment, height);
    }

  removeEmptySegments (segments);

  delete[] lastBool;
  delete[] currBool;
  delete[] nextBool;
}

void SegmentFactory::loadBool (bool *columnBool,
                               const cv::Mat &image,
                               int x)
{
  for (int y = 0; y < image.rows; y++)
    {
      if (x < 0) {
	columnBool [y] = false;
      }
      else
	{
	  columnBool [y] = pixelFilteredIsOn (image, x, y);
	}
    }
}

bool SegmentFactory::pixelFilteredIsOn (const cv::Mat &image,
					int x,
					int y) const
{
  bool rtn = false;

  if ((0 <= x) &&
      (0 <= y) &&
      (x < image.cols) &&
      (y < image.rows))
    {

      // Pixel is on if it is closer to black than white in gray scale. This test must be performed
      // on little endian and big endian systems, with or without alpha bits (which are typically high bits);
      const int BLACK_WHITE_THRESHOLD = 100; // Put threshold in middle of range
      int step = image.step[0];
      int gray = image.data[x + y * step];
      //std::cout << x << ", " << y << ", " << gray << std::endl;
      rtn = (gray < BLACK_WHITE_THRESHOLD);
    }

  return rtn;
}

void SegmentFactory::loadSegment (SegmentVector &columnSegment,
                                  int height)
{
  for (int y = 0; y < height; y++)
    {
      columnSegment [y] = 0;
    }
}

void SegmentFactory::matchRunsToSegments(int x,
                                         int width,
                                         int height,
                                         bool *lastBool,
                                         SegmentVector &lastSegment,
                                         bool* currBool,
                                         SegmentVector &currSegment,
                                         bool *nextBool,
                                         int *madeLines,
                                         int *foldedLines,
                                         int *shortLines,
                                         vector<Segment*> &segments)
{
  loadSegment(currSegment,
              height);

  int yStart = 0;
  bool inRun = false;
  for (int y = 0; y < height; y++)
    {

      if (!inRun && currBool [y])
	{
	  inRun = true;
	  yStart = y;
	}

      if (inRun)
	{
	  //std::cout << x << ", " << y << std::endl;
	}
      
      if ((y + 1 >= height) || !currBool [y + 1])
	{
	  if (inRun)
	    {
	      finishRun(lastBool,
			nextBool,
			lastSegment,
			currSegment,
			x, yStart,
			y,
			height,
			madeLines);
	    }

	  inRun = false;
	}
    }

  bool lastCol;
  if (x + 1 == width) {
    lastCol = true;
  } else {
    lastCol = false;
  }

  removeUnneededLines(lastSegment,
                      currSegment,
                      height,
                      foldedLines,
                      shortLines,
                      segments,
                      lastCol);
}

void SegmentFactory::scrollBool(bool *left,
                                bool *right,
                                int height)
{
  for (int y = 0; y < height; y++)
    {
    left [y] = right [y];
  }
}

void SegmentFactory::scrollSegment(SegmentVector &left,
                                   SegmentVector &right,
                                   int height)
{
  for (int y = 0; y < height; y++)
    {
      left [y] = right [y];
    }
}

void SegmentFactory::removeEmptySegments (vector<Segment*> &segments) const
{
  for (int i = segments.size(); i > 0;)
    {
      --i;
      Segment *segment = segments.at(i);

      // False positive warning from scan-build in next line can be ignored - it is a bug in that tool regarding loop unrolling
      if (segment->lineCount () == 0)
	{
	  // Remove this Segment
	  delete segment;
	  segments.erase(segments.begin() + i);
	}
    }
}

void SegmentFactory::finishRun(bool *lastBool,
                               bool *nextBool,
                               SegmentVector &lastSegment,
                               SegmentVector &currSegment,
                               int x,
                               int yStart,
                               int yStop,
                               int height,
                               int* madeLines)
{
  // When looking at adjacent columns, include pixels that touch diagonally since
  // those may also diagonally touch nearby runs in the same column (which would indicate
  // a branch)

  // Count runs that touch on the left
  bool multi_connect = false;
  if (adjacentRuns(lastBool, yStart, yStop, height) > 1)
    {
      //return;
    }

  // Count runs that touch on the right
  if (adjacentRuns(nextBool, yStart, yStop, height) > 1)
    {
      //return;
      multi_connect = true;
    }

  Segment *seg;
  if (adjacentSegments(lastSegment, yStart, yStop, height) == 0)
    {

      // This is the start of a new segment
      //seg = new Segment((int) (0.5 + (yStart + yStop) / 2.0));
      if (yStart == yStop)
	{
	  seg = new Segment(yStart);
	}
      else
	{
	  seg = new Segment(yStart);
	  seg->appendColumn(x, yStop);
	}
    }
  else
    {
      // This is the continuation of an existing segment
      seg = adjacentSegment(lastSegment, yStart, yStop, height);

      if (seg->m_multi_connect)
	{
	  //seg = new Segment((int) (0.5 + (yStart + yStop) / 2.0));
	  if (yStart == yStop)
	    {
	      seg = new Segment(yStart);
	    }
	  else
	    {
	      seg = new Segment(yStart);
	      seg->appendColumn(x, yStop);
	    }
	}

      ++(*madeLines);
      //seg->appendColumn(x, (int) (0.5 + (yStart + yStop) / 2.0));
      if (yStart == yStop)
	{
	  seg->appendColumn(x, yStart);
	}
      else if(fabs(yStart - seg->m_yLast) < fabs(yStop - seg->m_yLast))
	{
	  seg->appendColumn(x, yStart);
	  seg->appendColumn(x, yStop);
	}
      else
	{
	  seg->appendColumn(x, yStop);
	  seg->appendColumn(x, yStart);
	}
      seg->m_multi_connect = multi_connect;
    }

  for (int y = yStart; y <= yStop; y++)
    {
      currSegment [y] = seg;
    }
}

void SegmentFactory::removeUnneededLines(SegmentVector &lastSegment,
                                         SegmentVector &currSegment,
                                         int height,
                                         int *foldedLines,
                                         int *shortLines,
                                         vector<Segment*> &segments,
                                         bool lastCol)
{
  Segment *segLast = 0;
  for (int yLast = 0; yLast < height; yLast++)
    {
      if (lastSegment [yLast] && (lastSegment [yLast] != segLast))
	{
	  segLast = lastSegment [yLast];

	  // If the segment is found in the current column then it is still in work so postpone processing
	  bool found = false;
	  for (int yCur = 0; yCur < height; yCur++)
	    {
	      if (segLast == currSegment [yCur]) {
		found = true;
		break;
	      }
	    }

	  if ((!found) || lastCol)
	    {
	      if (segLast->length() < m_min_length)
		{

		  // Remove whole segment since it is too short. Do NOT set segLast to zero since that
		  // would cause this same segment to be deleted again in the next pixel if the segment
		  // covers more than one pixel
		  *shortLines += segLast->lineCount();
		  delete segLast;
		  lastSegment [yLast] = 0;
		}
	      else
		{
		  // Keep segment, but try to fold lines
		  //segLast->removeUnneededLines(foldedLines);

		  // Add to the output array since it is done and sufficiently long
		  segments.push_back (segLast);
		}
	    }
	}
    }
}

int SegmentFactory::adjacentRuns(bool *columnBool,
                                 int yStart,
                                 int yStop,
                                 int height)
{
  int runs = 0;
  bool inRun = false;
  for (int y = yStart - 1; y <= yStop + 1; y++)
    {
      if ((0 <= y) && (y < height))
	{
	  if (!inRun && columnBool [y])
	    {
	      inRun = true;
	      ++runs;
	    } else if (inRun && !columnBool [y])
	    {
	      inRun = false;
	    }
	}
    }

  return runs;
}

Segment *SegmentFactory::adjacentSegment(SegmentVector &lastSegment,
                                         int yStart,
                                         int yStop,
                                         int height)
{
  for (int y = yStart - 1; y <= yStop + 1; y++)
    {
    if ((0 <= y) && (y < height))
      {

      if (lastSegment [y])
	{
        return lastSegment [y];
      }
    }
  }

  return 0;
}

int SegmentFactory::adjacentSegments(SegmentVector &lastSegment,
                                     int yStart,
                                     int yStop,
                                     int height)
{
  int adjacentSegments = 0;

  bool inSegment = false;
  for (int y = yStart - 1; y <= yStop + 1; y++) {
    if ((0 <= y) && (y < height)) {

      if (!inSegment && lastSegment [y]) {

       inSegment = true;
        ++adjacentSegments;
      } else if (inSegment && !lastSegment [y]) {
        inSegment = false;
      }
    }
  }

  return adjacentSegments;
}

void SegmentFactory::clearSegments (vector<Segment*> &segments)
{
  vector<Segment*>::iterator itr;
  for (itr = segments.begin(); itr != segments.end(); itr++) {

    Segment *segment = *itr;

    delete segment;
  }

  segments.clear ();
}
