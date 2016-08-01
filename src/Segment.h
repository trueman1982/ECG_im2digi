#ifndef SEGMENT_H
#define SEGMENT_H

#include <vector>

class SegmentLine;

class Segment
{ 
public:
  /// Single constructor.
  Segment(int yLast);
  ~Segment();

  /// Add some more pixels in a new column to an active segment
  void appendColumn(int x, int y);

  /// Get method for number of lines
  int lineCount() const;

  /// Try to compress a segment that was just completed, by folding together line from
  /// point i to point i+1, with the line from i+1 to i+2, then the line from i+2 to i+3,
  /// until one of the points is more than a half pixel from the folded line. this should
  /// save memory and improve user interface responsiveness
  void removeUnneededLines(int *foldedLines);

  double length() const;

  // Y value of last point which is in previous column
  int m_yLast;

  // Total length of lines owned by this segment, as floating point to allow fractional increments
  double m_length;

  // This segment is drawn as a series of line segments
  std::vector<SegmentLine*> m_lines;

  bool m_multi_connect;
};

#endif // SEGMENT_H
