#include <math.h>
#include "SegmentLine.h"
#include "Segment.h"

using namespace std;

Segment::Segment(int y):
  m_yLast (y)
{
  m_multi_connect = false;
  m_length = 0;
}

Segment::~Segment()
{
  vector<SegmentLine*>::iterator itr;
  for (itr = m_lines.begin(); itr != m_lines.end(); itr++) {

    SegmentLine *segmentLine = *itr;
    delete segmentLine;
  }
}

void Segment::appendColumn(int x,
                           int y)
{
  int xOld = x - 1;
  int yOld = m_yLast;
  int xNew = x;
  int yNew = y;

  SegmentLine* line = new SegmentLine();
  line->setLine(xOld,
		yOld,
		xNew,
		yNew);

  // Do not show this line or its segment. this is handled later

  m_lines.push_back(line);

  // Update total length using distance formula
  m_length += sqrt((1.0) * (1.0) + (y - m_yLast) * (y - m_yLast));

  m_yLast = y;
}

int Segment::lineCount() const
{
  return m_lines.size();
}

double Segment::length() const
{
  return m_length;
}
