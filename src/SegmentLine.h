#ifndef SEGMENT_LINE_H
#define SEGMENT_LINE_H

class SegmentLine
{
public:
  /// Single constructor.
  SegmentLine();
  ~SegmentLine();

  void setLine(int x1, int y1, int x2, int y2);
  
  int m_x1;
  int m_y1;
  int m_x2;
  int m_y2;
};

#endif // SEGMENT_LINE_H
