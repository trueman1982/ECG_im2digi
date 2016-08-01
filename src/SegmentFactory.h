#ifndef SEGMENT_FACTORY_H
#define SEGMENT_FACTORY_H

#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>

class Segment;

typedef std::vector<Segment*> SegmentVector;

class SegmentFactory
{
public:
  /// Single constructor.
  SegmentFactory(int min_length)  {m_min_length = min_length;}
  ~SegmentFactory() {}

  /// Remove the segments created by makeSegments
  void clearSegments(std::vector<Segment*> &segments);

  /// Main entry point for creating all Segments for the filtered image.
  void makeSegments (const cv::Mat  &imageFiltered,
                     std::vector<Segment*> &segments);

  unsigned int m_min_length;

private:
  // Return the number of runs adjacent to the pixels from yStart to yStop (inclusive)
  int adjacentRuns(bool *columnBool,
                   int yStart,
                   int yStop,
                   int height);

  // Find the single segment pointer among the adjacent pixels from yStart-1 to yStop+1
  Segment *adjacentSegment(SegmentVector &lastSegment,
                           int yStart,
                           int yStop,
                           int height);

  // Return the number of segments adjacent to the pixels from yStart to yStop (inclusive)
  int adjacentSegments(SegmentVector &lastSegment,
                       int yStart,
                       int yStop,
                       int height);

  // Process a run of pixels. If there are fewer than two adjacent pixel runs on
  // either side, this run will be added to an existing segment, or the start of
  // a new segment
  void finishRun(bool *lastBool,
                 bool *nextBool,
                 SegmentVector &lastSegment,
                 SegmentVector &currSegment,
                 int x,
                 int yStart,
                 int yStop,
                 int height,
                 int* madeLines);

  // Initialize one column of boolean flags using the pixels of the specified column
  void loadBool (bool *columnBool,
                 const cv::Mat &image,
                 int x);

  // Initialize one column of segment pointers
  void loadSegment (SegmentVector &columnSegment,
                    int height);

  // Identify the runs in a column, and connect them to segments
  void matchRunsToSegments (int x,
                            int width,
                            int height,
                            bool *lastBool,
                            SegmentVector &lastSegment,
                            bool *currBool,
                            SegmentVector &currSegment,
                            bool *nextBool,
                            int *madeLines,
                            int *foldedLines,
                            int *shortLine,
                            std::vector<Segment*> &segments);

  /// Remove any Segment with no lines. This prevents crashes in Segment::firstPoint which requires at least one line in each Segment
  void removeEmptySegments (std::vector<Segment*> &segments) const;

  // Remove unneeded lines belonging to segments that just finished in the previous column.
  // The results of this function are displayed in the debug spew of makeSegments
  void removeUnneededLines(SegmentVector &lastSegment,
                           SegmentVector &currSegment,
                           int height,
                           int *foldedLines,
                           int *shortLines,
                           std::vector<Segment*> &segments,
                           bool lastCol);

  // Scroll the boolean flags of the right column into the left column
  void scrollBool(bool *left,
                  bool *right,
                  int height);

  // Scroll the segment pointers of the right column into the left column
  void scrollSegment(SegmentVector &left,
                     SegmentVector &right,
                     int height);

  bool pixelFilteredIsOn (const cv::Mat &image,
					int x,
					  int y) const;
};

#endif // SEGMENT_FACTORY_H
